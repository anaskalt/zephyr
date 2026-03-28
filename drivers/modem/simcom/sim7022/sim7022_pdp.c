/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_pdp, CONFIG_MODEM_LOG_LEVEL);

#include "sim7022.h"

/*
 * Band setup commands.
 *
 * The SIM7022 is an NB-IoT-only module.  Band selection uses AT+CNBP
 * with a 64-bit hexadecimal bitmask, where bit N (0-based) enables
 * LTE band (N+1).  For example, bit 7 → band 8 (880–915 MHz).
 *
 * The default value 0x0002000000400180 is the example from the SIM7022
 * AT Command Manual §3.2.54.  Adjust CONFIG_MODEM_SIMCOM_SIM7022_LTE_BANDS
 * for your region; use AT+CNBP=? to query supported bands on the hardware.
 *
 * No CNMP or CMNB commands are needed — the SIM7022 supports NB-IoT only.
 */

/*
 * Helper macros to stringify a preprocessor token into a C string literal.
 * Example: _STR(0x0080) → "0x0080"
 *
 * Note: Kconfig `hex` values are already defined with the 0x prefix,
 * e.g. #define CONFIG_MODEM_SIMCOM_SIM7022_LTE_BANDS 0x0002000000400180
 * so the stringified form is "0x0002000000400180" — no extra "0x" needed.
 */
#define _XSTR(x) #x
#define _STR(x)  _XSTR(x)

static const struct setup_cmd band_setup_cmds[] = {
	/*
	 * AT+CNBP=<mask>
	 * 64-bit hex bitmask where bit N enables LTE band (N+1).
	 * The CONFIG value already carries the 0x prefix, so _STR()
	 * expands to e.g. "AT+CNBP=0x0002000000400180".
	 */
	SETUP_CMD_NOHANDLE("AT+CNBP=" _STR(CONFIG_MODEM_SIMCOM_SIM7022_LTE_BANDS)),
};

/* ------------------------------------------------------------------ */
/* AT+CSQ — RSSI query                                                 */
/* ------------------------------------------------------------------ */

/*
 * +CSQ: <rssi>,<ber>
 *
 * RSSI encoding per SIM7022 AT Command Manual §3.2.8:
 *   0      → -113 dBm or less
 *   1      → -111 dBm
 *   2..30  → -113 + 2*rssi dBm  (i.e. -109 .. -53 dBm)
 *   31     → -51 dBm or greater
 *   99     → not known or not detectable
 */
MODEM_CMD_DEFINE(on_cmd_csq)
{
	int rssi = atoi(argv[0]);

	if (rssi == 0) {
		mdata.mdm_rssi = -113;
	} else if (rssi == 1) {
		mdata.mdm_rssi = -111;
	} else if (rssi >= 2 && rssi <= 30) {
		mdata.mdm_rssi = -113 + 2 * rssi;
	} else if (rssi == 31) {
		mdata.mdm_rssi = -51;
	} else {
		/* rssi == 99 or any other value: unknown */
		mdata.mdm_rssi = -1000;
	}

	LOG_INF("RSSI: %d dBm (raw=%d)", mdata.mdm_rssi, rssi);
	return 0;
}

/*
 * Periodic RSSI query work handler.
 *
 * When called with a non-NULL work pointer the query is rescheduled
 * after completion.  When called with NULL it runs once.
 */
void sim7022_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd[] = {
		MODEM_CMD("+CSQ: ", on_cmd_csq, 2U, ",")
	};
	int ret;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmd, ARRAY_SIZE(cmd),
			     "AT+CSQ",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CSQ ret: %d", ret);
	}

	if (work) {
		k_work_reschedule_for_queue(&modem_workq,
					    &mdata.rssi_query_work,
					    K_SECONDS(RSSI_TIMEOUT_SECS));
	}
}

/* ------------------------------------------------------------------ */
/* AT+CEREG / AT+CGATT handlers                                        */
/* ------------------------------------------------------------------ */

/*
 * Response to AT+CEREG?:
 *   +CEREG: <n>,<stat>[,...]
 * argv[0] = <n>, argv[1] = <stat>
 */
MODEM_CMD_DEFINE(on_cmd_cereg)
{
	mdata.mdm_registration = atoi(argv[1]);
	LOG_INF("CEREG stat: %u", mdata.mdm_registration);
	return 0;
}

/*
 * Response to AT+CGATT?:
 *   +CGATT: <state>
 */
MODEM_CMD_DEFINE(on_cmd_cgatt)
{
	int cgatt = atoi(argv[0]);

	if (cgatt) {
		mdata.status_flags |= SIM7022_STATUS_FLAG_ATTACHED;
	} else {
		mdata.status_flags &= ~SIM7022_STATUS_FLAG_ATTACHED;
	}
	LOG_INF("CGATT: %d", cgatt);
	return 0;
}

/* ------------------------------------------------------------------ */
/* PDP activate                                                         */
/* ------------------------------------------------------------------ */

int sim7022_pdp_activate(void)
{
	int counter;
	int ret = 0;

	const char *cereg_cmd = "AT+CEREG?";
	struct modem_cmd cereg_cmds[] = {
		MODEM_CMD("+CEREG: ", on_cmd_cereg, 2U, ",")
	};
	struct modem_cmd cgatt_cmd[] = {
		MODEM_CMD("+CGATT: ", on_cmd_cgatt, 1U, "")
	};


	/* Step 1 – configure NB-IoT bands */
	LOG_INF("Setting NB-IoT bands...");
	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   band_setup_cmds,
					   ARRAY_SIZE(band_setup_cmds),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("Failed to send band setup commands");
		goto error;
	}
	LOG_INF("Bands set");

	/* Step 2 – wait for a valid RSSI value */

	sim7022_rssi_query_work(NULL);
	counter = 0;
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000)) {
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
		sim7022_rssi_query_work(NULL);
	}

	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		LOG_ERR("No valid RSSI reached");
		ret = -ENETUNREACH;
		goto error;
	}
	LOG_INF("Signal: %d dBm", mdata.mdm_rssi);

	/* Step 3 – wait for GPRS attachment */

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cgatt_cmd, ARRAY_SIZE(cgatt_cmd),
			     "AT+CGATT?",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to query CGATT");
		goto error;
	}

	counter = 0;
	while (counter++ < MDM_MAX_CGATT_WAITS &&
	       (mdata.status_flags & SIM7022_STATUS_FLAG_ATTACHED) == 0) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     cgatt_cmd, ARRAY_SIZE(cgatt_cmd),
				     "AT+CGATT?",
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Failed to query CGATT");
			goto error;
		}
		k_sleep(K_SECONDS(1));
	}

	if ((mdata.status_flags & SIM7022_STATUS_FLAG_CPIN_READY) == 0 ||
	    (mdata.status_flags & SIM7022_STATUS_FLAG_ATTACHED)   == 0) {
		LOG_ERR("Modem is not attached to NB-IoT network");
		ret = -ENETUNREACH;
		goto error;
	}
	LOG_INF("GPRS attached");

	/*
	 * Step 4 – wait for EPS network registration (stat 1 or 5).
	 */

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U,
			     "AT+CEREG=2",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to set AT+CEREG=2");
		goto error;
	}

	LOG_INF("Waiting for network registration...");

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cereg_cmds, ARRAY_SIZE(cereg_cmds),
			     cereg_cmd,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to query CEREG");
		goto error;
	}

	counter = 0;
	while (counter++ < MDM_MAX_CEREG_WAITS &&
	       mdata.mdm_registration != 1 &&
	       mdata.mdm_registration != 5) {
		k_sleep(K_SECONDS(1));
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     cereg_cmds, ARRAY_SIZE(cereg_cmds),
				     cereg_cmd,
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Failed to query CEREG");
			goto error;
		}
	}

	if (mdata.mdm_registration != 1 && mdata.mdm_registration != 5) {
		LOG_ERR("Network registration failed!");
		ret = -ENETUNREACH;
		goto error;
	}
	LOG_INF("Registered to NB-IoT network (stat=%u)", mdata.mdm_registration);

	/*
	 * Step 6 – activate PDP context / open socket service.
	 */

	{
		char cgdcont_buf[sizeof("AT+CGDCONT=1,\"IP\",\"\"") +
				 CONFIG_MODEM_SIMCOM_SIM7022_APN_MAX_LEN];

		ret = snprintk(cgdcont_buf, sizeof(cgdcont_buf),
			       "AT+CGDCONT=1,\"IP\",\"%s\"", MDM_APN);
		if (ret < 0) {
			LOG_ERR("Failed to build CGDCONT command");
			goto error;
		}

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0U, cgdcont_buf,
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("AT+CGDCONT failed: %d", ret);
			goto error;
		}
	}
	LOG_INF("APN set");



	/*
	 * If the STM32 reset while the SIM7022 stayed powered (e.g. during
	 * flash/debug), the modem may still have an active NETOPEN session.
	 * AT+NETOPEN would then return ERROR.  Send AT+NETCLOSE first to
	 * ensure a clean state; ignore the result (fails gracefully if
	 * already closed).
	 */
	modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
		       NULL, 0U, "AT+NETCLOSE",
		       &mdata.sem_response, K_SECONDS(5));
	k_sleep(K_MSEC(500));

	k_sem_reset(&mdata.pdp_sem);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U,
			     "AT+NETOPEN",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+NETOPEN send failed: %d", ret);
		goto error;
	}


	ret = k_sem_take(&mdata.pdp_sem, MDM_PDP_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for +NETOPEN URC");
		ret = -ETIMEDOUT;
		goto error;
	}

	if ((mdata.status_flags & SIM7022_STATUS_FLAG_NET_OPEN) == 0) {
		LOG_ERR("Network open failed");
		ret = -ENETUNREACH;
		goto error;
	}
	LOG_INF("Network opened");


	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U,
			     "AT+CIPRXGET=1",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to set CIPRXGET=1 (manual receive mode)");
		goto error;
	}


	LOG_INF("PDP context active, socket service open");

	/* Start periodic RSSI query */
	k_work_reschedule_for_queue(&modem_workq, &mdata.rssi_query_work,
				    K_SECONDS(RSSI_TIMEOUT_SECS));

	return 0;

error:
	return ret;
}

/* ------------------------------------------------------------------ */
/* PDP deactivate                                                       */
/* ------------------------------------------------------------------ */

int sim7022_pdp_deactivate(void)
{
	int ret;

	k_work_cancel_delayable(&mdata.rssi_query_work);

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_WRN("Not in networking state, nothing to deactivate");
		return -EALREADY;
	}

	/*
	 * AT+NETCLOSE:
	 *   Response sequence:
	 *     OK
	 *     +NETCLOSE: <err>
	 *
	 * on_urc_netclose() handles the URC and gives pdp_sem.
	 */
	k_sem_reset(&mdata.pdp_sem);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U,
			     "AT+NETCLOSE",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+NETCLOSE send failed: %d", ret);
		return ret;
	}

	ret = k_sem_take(&mdata.pdp_sem, MDM_PDP_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for +NETCLOSE URC");
		return -ETIMEDOUT;
	}

	LOG_INF("PDP context deactivated");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Public wrappers (called from sim7022_sock.c and user code)          */
/* ------------------------------------------------------------------ */

int mdm_sim7022_start_network(void)
{
	int ret = -EALREADY;

	if (sim7022_get_state() == SIM7022_STATE_NETWORKING) {
		LOG_WRN("Network already active");
		goto out;
	} else if (sim7022_get_state() != SIM7022_STATE_IDLE) {
		LOG_WRN("Can only activate networking from idle state");
		ret = -EINVAL;
		goto out;
	}

	ret = sim7022_pdp_activate();
out:
	return ret;
}

int mdm_sim7022_stop_network(void)
{
	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_WRN("Modem not in networking state");
		return -EINVAL;
	}
	return sim7022_pdp_deactivate();
}
