/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_sim7022

#include <zephyr/logging/log.h>
#include <zephyr/net/offloaded_netdev.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/drivers/modem/simcom-sim7022.h>
#include "sim7022.h"

/* ------------------------------------------------------------------ */
/* Global driver state (one instance only)                             */
/* ------------------------------------------------------------------ */

struct sim7022_data    mdata;
struct modem_context   mctx;

static struct k_thread modem_rx_thread;
struct k_work_q        modem_workq;

static K_KERNEL_STACK_DEFINE(modem_rx_stack,
			     CONFIG_MODEM_SIMCOM_SIM7022_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(modem_workq_stack,
			     CONFIG_MODEM_SIMCOM_SIM7022_RX_WORKQ_STACK_SIZE);

NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

/* PWRKEY GPIO from Device Tree */
static const struct gpio_dt_spec power_gpio =
	GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER 37
	uint32_t h = 0;

	for (int i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}
	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct sim7022_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* Derive last 4 bytes of MAC from IMEI hash */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));
	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/* ------------------------------------------------------------------ */
/* Network interface init                                              */
/* ------------------------------------------------------------------ */

static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct sim7022_data *data = dev->data;

	net_if_set_link_addr(iface, modem_get_mac(dev),
			     sizeof(data->mac_addr), NET_LINK_ETHERNET);

	data->netif = iface;

	socket_offload_dns_register(&offload_dns_ops);
	net_if_socket_offload_set(iface, sim7022_offload_socket);
}

/* ------------------------------------------------------------------ */
/* State machine                                                        */
/* ------------------------------------------------------------------ */

void sim7022_change_state(enum sim7022_state state)
{
	LOG_DBG("Changing state to (%d)", state);
	mdata.state = state;
}

enum sim7022_state sim7022_get_state(void)
{
	return mdata.state;
}

enum sim7022_state mdm_sim7022_get_state(void)
{
	return sim7022_get_state();
}

/* ------------------------------------------------------------------ */
/* Offload API function table                                           */
/* ------------------------------------------------------------------ */

static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET && family != AF_INET6) {
		return false;
	}
	if (type != SOCK_DGRAM && type != SOCK_STREAM) {
		return false;
	}
	if (proto != IPPROTO_TCP && proto != IPPROTO_UDP) {
		return false;
	}
	return true;
}

/* ------------------------------------------------------------------ */
/* AT command response / setup handlers                                */
/* ------------------------------------------------------------------ */

MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_exterror)
{
	/* +CME ERROR: <err> */
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * "SEND OK" — not sent by SIM7022 (kept for safety).
 * The real completion signal is "+CIPSEND: <link>,<req>,<cnf>" below.
 */
MODEM_CMD_DEFINE(on_cmd_send_ok)
{
	LOG_DBG("SEND OK received");
	k_sem_give(&mdata.sem_send_ok);
	return 0;
}

/*
 * "SEND FAIL" — transmission failed.
 */
MODEM_CMD_DEFINE(on_cmd_send_fail)
{
	LOG_ERR("SEND FAIL received");
	k_sem_give(&mdata.sem_send_ok);
	return 0;
}

/*
 * "+CIPSEND: <link_num>,<reqSendLength>,<cnfSendLength>"
 *
 * This is the authoritative send-completion URC from the SIM7022.
 * It arrives in unsolicited_cmds context (after data bytes are written,
 * there is no pending AT command), so it is registered there.
 * It unblocks offload_sendto which waits on sem_send_ok.
 */
MODEM_CMD_DEFINE(on_urc_cipsend_confirm)
{
	int req = atoi(argv[1]);
	int cnf = atoi(argv[2]);

	if (cnf != req) {
		LOG_ERR("+CIPSEND: link=%s req=%d cnf=%d — partial send!",
			argv[0], req, cnf);
	} else {
		LOG_INF("+CIPSEND: link=%s %d bytes confirmed", argv[0], cnf);
	}
	k_sem_give(&mdata.sem_send_ok);
	return 0;
}

MODEM_CMD_DEFINE(on_urc_ciperror)
{
	LOG_ERR("+CIPERROR: %s — TCP/IP stack error during send", argv[0]);
	k_sem_give(&mdata.sem_send_ok);
	return 0;
}

/*
 * '>' prompt received — the modem is ready for us to write data.
 * Used by offload_sendto in sim7022_sock.c.
 */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

/* ------------------------------------------------------------------ */
/* Setup command handlers (AT+CGMI, AT+CGMM, …)                       */
/* ------------------------------------------------------------------ */

MODEM_CMD_DEFINE(on_cmd_cgmi)
{
	size_t out_len = net_buf_linearize(mdata.mdm_manufacturer,
					   sizeof(mdata.mdm_manufacturer) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cgmm)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model,
					   sizeof(mdata.mdm_model) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", mdata.mdm_model);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cgmr)
{
	size_t out_len = net_buf_linearize(mdata.mdm_revision,
					   sizeof(mdata.mdm_revision) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", mdata.mdm_revision);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cgsn)
{
	size_t out_len = net_buf_linearize(mdata.mdm_imei,
					   sizeof(mdata.mdm_imei) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", mdata.mdm_imei);
	return 0;
}

#if defined(CONFIG_MODEM_SIM_NUMBERS)
MODEM_CMD_DEFINE(on_cmd_cimi)
{
	size_t out_len = net_buf_linearize(mdata.mdm_imsi,
					   sizeof(mdata.mdm_imsi) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';
	LOG_INF("IMSI: %s", mdata.mdm_imsi);
	return 0;
}

/*
 * AT+CICCID response on SIM7022:
 *   +ICCID: <20-digit number>
 * The handler is registered for "+ICCID: " so argv[0] is the number.
 */
MODEM_CMD_DEFINE(on_cmd_ciccid)
{
	size_t out_len = net_buf_linearize(mdata.mdm_iccid,
					   sizeof(mdata.mdm_iccid) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_iccid[out_len] = '\0';
	LOG_INF("ICCID: %s", mdata.mdm_iccid);
	return 0;
}
#endif /* CONFIG_MODEM_SIM_NUMBERS */

/* ------------------------------------------------------------------ */
/* Unsolicited Result Code (URC) handlers                              */
/* ------------------------------------------------------------------ */

/*
 * "RDY" — modem has finished booting and is ready.
 */
MODEM_CMD_DIRECT_DEFINE(on_urc_rdy)
{
	LOG_DBG("RDY received");
	mdata.status_flags |= SIM7022_STATUS_FLAG_POWER_ON;
	k_sem_give(&mdata.boot_sem);
	return 0;
}

/*
 * "NORMAL POWER DOWN" — modem has shut down.
 */
MODEM_CMD_DIRECT_DEFINE(on_urc_pwr_down)
{
	LOG_DBG("NORMAL POWER DOWN received");
	mdata.status_flags &= ~SIM7022_STATUS_FLAG_POWER_ON;
	k_sem_give(&mdata.boot_sem);
	return 0;
}

/*
 * "+CPIN: READY" or "+CPIN: <other>" — SIM card status.
 */
MODEM_CMD_DEFINE(on_urc_cpin)
{
	if (strcmp(argv[0], "READY") == 0) {
		mdata.status_flags |= SIM7022_STATUS_FLAG_CPIN_READY;
	} else {
		mdata.status_flags &= ~SIM7022_STATUS_FLAG_CPIN_READY;
	}
	k_sem_give(&mdata.boot_sem);
	LOG_INF("CPIN: %s", argv[0]);
	return 0;
}

/*
 * "+NETOPEN: <err>" — PDP context activation result.
 * err == 0: success.
 */
MODEM_CMD_DEFINE(on_urc_netopen)
{
	int err = atoi(argv[0]);

	if (err == 0) {
		LOG_INF("Network opened successfully");
		mdata.status_flags |= SIM7022_STATUS_FLAG_NET_OPEN;
		sim7022_change_state(SIM7022_STATE_NETWORKING);
	} else {
		LOG_ERR("Network open failed with error: %d", err);
		mdata.status_flags &= ~SIM7022_STATUS_FLAG_NET_OPEN;
	}
	k_sem_give(&mdata.pdp_sem);
	return 0;
}

/*
 * "+NETCLOSE: <err>" — PDP context was closed (by us or network).
 */
MODEM_CMD_DEFINE(on_urc_netclose)
{
	int err = atoi(argv[0]);

	LOG_INF("Network closed (err=%d)", err);
	mdata.status_flags &= ~SIM7022_STATUS_FLAG_NET_OPEN;
	sim7022_change_state(SIM7022_STATE_IDLE);
	k_sem_give(&mdata.pdp_sem);
	return 0;
}

/*
 * "+CIPRXGET: 1,<link_num>" — data is available on a socket.
 *
 * The SIM7022 uses manual receive mode (AT+CIPRXGET=1).  When data
 * arrives the module sends this URC.  We call the socket indication
 * helper which sets a dummy packet size and unblocks any waiting
 * recv() call.
 *
 * argv[0] = link_num (0 or 1) — the modem's socket identifier.
 */
MODEM_CMD_DEFINE(on_urc_ciprxget)
{
	int link_num = atoi(argv[0]);

	sim7022_handle_sock_data_indication(link_num);
	return 0;
}

/*
 * "+CIPCLOSE: <link_num>,<err>" — a socket has been closed.
 *
 * This URC is sent when the remote side closes the connection, or
 * when AT+NETCLOSE is called while a socket is still open.
 *
 * argv[0] = link_num (0 or 1) — the modem's socket identifier.
 * argv[1] = err (0 = success, nonzero = error).
 */
MODEM_CMD_DEFINE(on_urc_cipclose)
{
	int link_num = atoi(argv[0]);
	int err      = atoi(argv[1]);

	LOG_INF("+CIPCLOSE: link=%d err=%d", link_num, err);
	sim7022_handle_sock_state(link_num, 0);
	return 0;
}

/*
 * "+CIPOPEN: <link_num>,<err>" — socket connection result.
 *
 * The SIM7022 sends this in different positions relative to OK/ERROR
 * depending on protocol:
 *
 *   TCP success:  OK\r\n+CIPOPEN: <link>,0\r\n
 *   UDP success:  +CIPOPEN: <link>,0\r\nOK\r\n
 *   Error (both): +CIPOPEN: <link>,<err>\r\nERROR\r\n
 *
 * By handling +CIPOPEN: as a URC (not a cmd[] argument) and using a
 * dedicated semaphore, offload_connect() can safely wait for the
 * result regardless of ordering.
 *
 * argv[0] = link_num, argv[1] = err  (0 = success)
 */
MODEM_CMD_DEFINE(on_urc_cipopen)
{
	int err = atoi(argv[1]);

	LOG_INF("+CIPOPEN: link=%s err=%d", argv[0], err);
	mdata.socket_open_rc = (uint8_t)err;
	k_sem_give(&mdata.sem_cipopen);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Static command tables                                               */
/* ------------------------------------------------------------------ */

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK",        on_cmd_ok,        0U, ""),
	MODEM_CMD("ERROR",     on_cmd_error,     0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
	MODEM_CMD("SEND OK",   on_cmd_send_ok,   0U, ""),
	MODEM_CMD("SEND FAIL", on_cmd_send_fail, 0U, ""),
};

static const struct modem_cmd unsolicited_cmds[] = {
	MODEM_CMD("RDY",              on_urc_rdy,             0U, ""),
	MODEM_CMD("NORMAL POWER DOWN",on_urc_pwr_down,        0U, ""),
	MODEM_CMD("+CPIN: ",          on_urc_cpin,            1U, ","),
	MODEM_CMD("+NETOPEN: ",       on_urc_netopen,         1U, ""),
	MODEM_CMD("+NETCLOSE: ",      on_urc_netclose,        1U, ""),
	/*
	 * "+CIPSEND: <link>,<req>,<cnf>" — send completion URC.
	 * This is the authoritative signal that the modem accepted (or
	 * rejected) the data bytes written after the '>' prompt.
	 * It arrives when there is no pending AT command, so it MUST be
	 * in unsolicited_cmds.  offload_sendto waits on sem_send_ok.
	 */
	MODEM_CMD("+CIPSEND: ",       on_urc_cipsend_confirm, 3U, ","),
	/* Safety handlers — SIM7022 sends OK/+CIPSEND, not "SEND OK" */
	MODEM_CMD("SEND OK",          on_cmd_send_ok,         0U, ""),
	MODEM_CMD("SEND FAIL",        on_cmd_send_fail,       0U, ""),
	/*
	 * +CIPERROR: <err> — TCP/IP stack error (e.g. failed to send data).
	 * Unblocks sem_send_ok with an error indication.
	 */
	MODEM_CMD("+CIPERROR: ",      on_urc_ciperror,        1U, ""),
	/*
	 * '>' — TX-ready prompt sent by the modem after AT+CIPSEND=<id>,<len>.
	 */
	MODEM_CMD_DIRECT(">",         on_cmd_tx_ready),
	/*
	 * +CIPOPEN: <link_num>,<err>
	 * Handled as a URC (not a per-command response) so that both TCP
	 * (OK before +CIPOPEN) and UDP (+CIPOPEN before OK) orderings are
	 * handled correctly via sem_cipopen.  See on_urc_cipopen().
	 */
	MODEM_CMD("+CIPOPEN: ",       on_urc_cipopen,   2U, ","),
	/*
	 * +CIPRXGET: 1,<link_num>  — data available on socket.
	 * We match the literal prefix "+CIPRXGET: 1," so argv[0] is
	 * the link_num (0 or 1) directly.
	 */
	MODEM_CMD("+CIPRXGET: 1,",   on_urc_ciprxget,  1U, ""),
	MODEM_CMD("+CIPCLOSE: ",      on_urc_cipclose,  2U, ","),
};

/* ------------------------------------------------------------------ */
/* PWRKEY toggle                                                       */
/* ------------------------------------------------------------------ */

/*
 * Per SIM7022 hardware notes: PWRKEY must be held HIGH for at least
 * 1 second to trigger a power-on or power-off event (same timing as
 * SIM7080).
 */
static void modem_pwrkey(void)
{
	LOG_DBG("Pulling PWRKEY");
	gpio_pin_set_dt(&power_gpio, 1);
	k_sleep(K_MSEC(1500));
	gpio_pin_set_dt(&power_gpio, 0);
}

/* ------------------------------------------------------------------ */
/* Autobaud                                                             */
/* ------------------------------------------------------------------ */

static int modem_autobaud(void)
{
	int counter = 0;
	int ret = -1;

	/*
	 * The SIM7022 supports autobaud on first startup.
	 * Send "AT" repeatedly until the modem responds with "OK".
	 */
	while (counter++ <= MDM_MAX_AUTOBAUD) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
				     "AT", &mdata.sem_response, K_MSEC(500));
		if (ret == 0) {
			break;
		}
		LOG_DBG("No response to autobaud AT (%d)", counter);
	}
	return ret;
}

/* ------------------------------------------------------------------ */
/* Boot sequence                                                        */
/* ------------------------------------------------------------------ */

static int modem_boot(bool allow_autobaud)
{
	uint8_t boot_tries = 0;
	int ret = -1;
	bool autobaud_path = false;

	mdata.status_flags = 0;


	while (boot_tries++ <= MDM_BOOT_TRIES) {
		k_sem_reset(&mdata.boot_sem);

			modem_pwrkey();

		/* Wait up to 10s for RDY URC */
		ret = k_sem_take(&mdata.boot_sem, K_SECONDS(10));
		if (ret == 0) {
			if (mdata.status_flags & SIM7022_STATUS_FLAG_POWER_ON) {
				LOG_INF("Modem booted");
				break;
			}
			LOG_INF("Modem turned off");
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_WRN("No RDY after PWRKEY (module may auto-power-on)");

		if (!allow_autobaud) {
			continue;
		}

		LOG_INF("Trying autobaud");
		ret = modem_autobaud();
		if (ret != 0) {
			LOG_WRN("Autobaud failed");
			continue;
		}

		/*
		 * Autobaud succeeded — modem is alive at 115200.
		 *
		 * When the module auto-powers-on (PWRKEY pulled HIGH by a
		 * board resistor) it sends RDY and +CPIN: READY before the
		 * driver starts listening.  We skip AT+QCRST and poll
		 * SIM status directly with AT+CPIN?.
		 */
		LOG_INF("Autobaud OK — treating modem as booted");
		mdata.status_flags |= SIM7022_STATUS_FLAG_POWER_ON;
		autobaud_path = true;
		ret = 0;
		break;
	}

	if (ret != 0) {
		LOG_ERR("Modem boot failed!");
		goto out;
	}

	/* ── SIM card check ─────────────────────────────────────── */
	if (autobaud_path) {
		/*
		 * Modem was already running → +CPIN: READY was sent before
		 * we started listening.  Poll AT+CPIN? directly.
		 */


		/* Give the modem a moment to settle if it just powered on */
		k_sleep(K_MSEC(500));

		struct modem_cmd cpin_cmds[] = {
			MODEM_CMD("+CPIN: ", on_urc_cpin, 1U, ",")
		};
		int cpin_tries = 0;

		while (cpin_tries++ < 10) {
			k_sem_reset(&mdata.boot_sem);
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
					     cpin_cmds, ARRAY_SIZE(cpin_cmds),
					     "AT+CPIN?",
					     &mdata.sem_response,
					     K_SECONDS(3));
			if (ret == 0 &&
			    (mdata.status_flags & SIM7022_STATUS_FLAG_CPIN_READY)) {
				LOG_INF("SIM card ready");
				break;
			}

			k_sleep(K_SECONDS(2));
		}

		if (!(mdata.status_flags & SIM7022_STATUS_FLAG_CPIN_READY)) {
			LOG_ERR("SIM card not ready!");
			ret = -EIO;
			goto out;
		}
	} else {
		/* Normal path: wait for +CPIN: URC */

		ret = k_sem_take(&mdata.boot_sem, K_SECONDS(15));
		if (ret != 0) {
			LOG_ERR("Timeout waiting for SIM status");
			goto out;
		}

		if ((mdata.status_flags & SIM7022_STATUS_FLAG_CPIN_READY) == 0) {
			LOG_ERR("SIM card not ready!");
			goto out;
		}
		}

	/* Disable command echo */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
			     "ATE0", &mdata.sem_response, K_MSEC(500));
	if (ret != 0) {
		LOG_ERR("Disabling echo failed");
		goto out;
	}

	sim7022_change_state(SIM7022_STATE_IDLE);

out:
	return ret;
}

/* ------------------------------------------------------------------ */
/* Setup commands (sent once after boot)                               */
/* ------------------------------------------------------------------ */

static const struct setup_cmd setup_cmds[] = {
	SETUP_CMD("AT+CGMI", "",       on_cmd_cgmi, 0U, ""),
	SETUP_CMD("AT+CGMM", "",       on_cmd_cgmm, 0U, ""),
	SETUP_CMD("AT+CGMR", "",       on_cmd_cgmr, 0U, ""),
	SETUP_CMD("AT+CGSN", "",       on_cmd_cgsn, 0U, ""),
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	SETUP_CMD("AT+CIMI",   "",     on_cmd_cimi,   0U, ""),
	/*
	 * AT+CICCID on SIM7022 responds with:
	 *   +ICCID: <number>
	 * so we need to match "+ICCID: " as the prefix.
	 */
	SETUP_CMD("AT+CICCID", "+ICCID: ", on_cmd_ciccid, 0U, ""),
#endif
};

/* ------------------------------------------------------------------ */
/* Full modem setup: boot → identify → bring up network               */
/* ------------------------------------------------------------------ */

static int modem_setup(void)
{
	int ret;

	k_work_cancel_delayable(&mdata.rssi_query_work);

	ret = modem_boot(true);
	if (ret < 0) {
		LOG_ERR("Booting modem failed!");
		return ret;
	}

	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds, ARRAY_SIZE(setup_cmds),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to send setup commands!");
		return ret;
	}

	/*
	 * Verify we are talking to the right module.
	 * AT+CGMM on the SIM7022 returns "SIM7022" (without the "SIMCOM_"
	 * prefix that the SIM7080 uses).
	 */
	if (strncmp(mdata.mdm_model, "SIM7022", 7) != 0) {
		LOG_ERR("Wrong modem model: %s", mdata.mdm_model);
		return -EINVAL;
	}

	/* Activate PDP context and open socket service */
	ret = sim7022_pdp_activate();

	return ret;
}

/* ------------------------------------------------------------------ */
/* Public API: power on / off / force reset                            */
/* ------------------------------------------------------------------ */

int mdm_sim7022_power_on(void)
{
	return modem_boot(false);
}

int mdm_sim7022_power_off(void)
{
	int ret = -EALREADY;

	k_work_cancel_delayable(&mdata.rssi_query_work);

	if ((mdata.status_flags & SIM7022_STATUS_FLAG_POWER_ON) == 0) {
		LOG_WRN("Modem already off");
		goto out;
	}

	k_sem_reset(&mdata.boot_sem);
	modem_pwrkey();

	ret = k_sem_take(&mdata.boot_sem, K_SECONDS(5));
	if (ret != 0) {
		LOG_ERR("No power down indication");
		goto out;
	}

	if ((mdata.status_flags & SIM7022_STATUS_FLAG_POWER_ON) != 0) {
		LOG_ERR("Modem not powered down!");
		ret = -1;
		goto out;
	}

	LOG_DBG("Modem turned off");
	mdata.status_flags = 0;
	sim7022_change_state(SIM7022_STATE_OFF);

out:
	return ret;
}

void mdm_sim7022_force_reset(void)
{
	LOG_DBG("Forcefully resetting modem");
	gpio_pin_set_dt(&power_gpio, 1);
	k_sleep(K_SECONDS(15));
	gpio_pin_set_dt(&power_gpio, 0);
}

/* ------------------------------------------------------------------ */
/* Public API: identification getters                                  */
/* ------------------------------------------------------------------ */

const char *mdm_sim7022_get_manufacturer(void) { return mdata.mdm_manufacturer; }
const char *mdm_sim7022_get_model(void)        { return mdata.mdm_model;        }
const char *mdm_sim7022_get_revision(void)     { return mdata.mdm_revision;     }
const char *mdm_sim7022_get_imei(void)         { return mdata.mdm_imei;         }

#if defined(CONFIG_MODEM_SIM_NUMBERS)
const char *mdm_sim7022_get_iccid(void) { return mdata.mdm_iccid; }
#else
const char *mdm_sim7022_get_iccid(void) { return NULL; }
#endif

/* ------------------------------------------------------------------ */
/* RX thread: feeds incoming UART bytes to the command handler        */
/* ------------------------------------------------------------------ */

static void modem_rx(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);
		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

/* ------------------------------------------------------------------ */
/* Driver init: called by Zephyr device model at boot                 */
/* ------------------------------------------------------------------ */

static int modem_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	/* Initialise all semaphores */
	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_tx_ready, 0, 1);
	k_sem_init(&mdata.sem_send_ok,  0, 1);
	k_sem_init(&mdata.sem_dns,      0, 1);
	k_sem_init(&mdata.boot_sem,     0, 1);
	k_sem_init(&mdata.pdp_sem,      0, 1);
	k_sem_init(&mdata.sem_cipopen,  0, 1);

	/* Start the work queue used for periodic RSSI queries */
	k_work_queue_start(&modem_workq, modem_workq_stack,
			   K_KERNEL_STACK_SIZEOF(modem_workq_stack),
			   K_PRIO_COOP(7), NULL);

	mdata.mdm_registration = 0;
	mdata.status_flags     = 0;

	mdata.dns.recount = CONFIG_MODEM_SIMCOM_SIM7022_DNS_DEFAULT_RECOUNT;
	mdata.dns.timeout = CONFIG_MODEM_SIMCOM_SIM7022_DNS_DEFAULT_TIMEOUT;

	/* Initialise the socket pool */
	ret = modem_socket_init(&mdata.socket_config,
				&mdata.sockets[0],
				ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM,
				true,
				&offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	sim7022_change_state(SIM7022_STATE_OFF);

	/* AT command handler */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf      = &mdata.cmd_match_buf[0],
		.match_buf_len  = sizeof(mdata.cmd_match_buf),
		.buf_pool       = &mdm_recv_pool,
		.alloc_timeout  = BUF_ALLOC_TIMEOUT,
		.eol            = "\r\n",
		.user_data      = NULL,
		.response_cmds      = response_cmds,
		.response_cmds_len  = ARRAY_SIZE(response_cmds),
		.unsol_cmds         = unsolicited_cmds,
		.unsol_cmds_len     = ARRAY_SIZE(unsolicited_cmds),
	};

	ret = modem_cmd_handler_init(&mctx.cmd_handler,
				     &mdata.cmd_handler_data,
				     &cmd_handler_config);
	if (ret < 0) {
		goto error;
	}

	/* UART interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf     = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev           = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};

	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data,
				    &uart_config);
	if (ret < 0) {
		goto error;
	}

	mdata.current_sock_fd      = -1;
	mdata.current_sock_written = 0;

	/* Register modem data with the context */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model        = mdata.mdm_model;
	mctx.data_revision     = mdata.mdm_revision;
	mctx.data_imei         = mdata.mdm_imei;
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	mctx.data_imsi         = mdata.mdm_imsi;
	mctx.data_iccid        = mdata.mdm_iccid;
#endif
	mctx.data_rssi         = &mdata.mdm_rssi;

	ret = gpio_pin_configure_dt(&power_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure power GPIO");
		goto error;
	}

	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* Start the RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
			K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			modem_rx, NULL, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Initialise the periodic RSSI work item */
	k_work_init_delayable(&mdata.rssi_query_work, sim7022_rssi_query_work);

	return modem_setup();

error:
	return ret;
}

/* ------------------------------------------------------------------ */
/* Zephyr device model registration                                    */
/* ------------------------------------------------------------------ */

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_SIMCOM_SIM7022_INIT_PRIORITY,
				  &api_funcs,
				  MDM_MAX_DATA_LENGTH);

NET_SOCKET_OFFLOAD_REGISTER(simcom_sim7022,
			    CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC,
			    offload_is_supported,
			    sim7022_offload_socket);
