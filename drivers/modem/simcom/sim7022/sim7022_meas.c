/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NOTE: The SIM7022 does NOT implement AT+CBC (battery charge).
 * The AT+QCBCINFO command returns serving cell information, not battery data.
 * No battery voltage/charge API is provided for this module.
 */

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L   /* Required for strtok_r() */

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_meas, CONFIG_MODEM_LOG_LEVEL);

#include "sim7022.h"

/* ------------------------------------------------------------------ */
/* AT+CPSI — UE system information                                     */
/* ------------------------------------------------------------------ */

/*
 * CPSI field indices (comma-separated fields after "+CPSI: "):
 *
 *   Field  Idx  Value (from datasheet §3.2.55 example)
 *   -----  ---  -------------------------------------------
 *   System Mode      0   "NO SERVICE" | "GSM" | "LTE" | "NB"
 *   Operation Mode   1   "Unknown" | "Online" | "Offline" |
 *                        "Factory Test Mode" | "Reset" |
 *                        "Low Power Mode" | "Flight Mode"
 *   MCC-MNC          2   e.g. "460-01"
 *   TAC              3   hex string, e.g. "0x230A"
 *   SCellID          4   decimal, e.g. "175499523"
 *   PCellID          5   decimal, e.g. "318"
 *   Freq Band        6   string, e.g. "EUTRAN-BAND3"
 *   earfcn           7   decimal
 *   dlbw             8   decimal (0-6, bandwidth code)
 *   ulbw             9   decimal (0-6, bandwidth code)
 *   RSRQ            10   signed decimal (tenths of dB)
 *   RSRP            11   signed decimal (tenths of dBm)
 *   RSSI            12   signed decimal (tenths of dBm)
 *   RSSNR           13   signed decimal (tenths of dB)
 *
 * Reference: AT Command Manual §3.2.55, example:
 *   +CPSI: LTE,Online,460-01,0x230A,175499523,318,EUTRAN-BAND3,
 *          1650,5,0,21,67,255,19
 */
#define CPSI_SYS_MODE_IDX    0U
#define CPSI_OP_MODE_IDX     1U
#define CPSI_MCC_MNC_IDX     2U
#define CPSI_LTE_TAC_IDX     3U
#define CPSI_LTE_SCI_IDX     4U
#define CPSI_LTE_PCI_IDX     5U
#define CPSI_LTE_BAND_IDX    6U
#define CPSI_LTE_EARFCN_IDX  7U
#define CPSI_LTE_DLBW_IDX    8U
#define CPSI_LTE_ULBW_IDX    9U
#define CPSI_LTE_RSRQ_IDX   10U
#define CPSI_LTE_RSRP_IDX   11U
#define CPSI_LTE_RSSI_IDX   12U
#define CPSI_LTE_RSSNR_IDX  13U

#define CPSI_LTE_ARG_COUNT  14U

/*
 * System Mode strings as returned by the SIM7022.
 * From AT Command Manual §3.2.55:
 *   "NO SERVICE", "GSM", "WCDMA", "LTE", "NB"
 *
 * Note: "NB" is the NB-IoT mode string on SIM7022, NOT "LTE NB-IOT".
 * "LTE" appears for regular LTE bearers; both map to
 * SIM7022_UE_SYS_MODE_LTE_NB_IOT in our enum since the hardware is
 * NB-IoT only.
 */
static const char *ue_sys_mode_lut[] = {
	"NO SERVICE",   /* SIM7022_UE_SYS_MODE_NO_SERVICE */
	"NB",           /* SIM7022_UE_SYS_MODE_LTE_NB_IOT — NB-IoT bearer */
	"LTE",          /* SIM7022_UE_SYS_MODE_LTE_NB_IOT — LTE bearer    */
};

/*
 * Operation Mode strings as returned by the SIM7022.
 * From AT Command Manual §3.2.55:
 *   "Unknown", "Online", "Offline", "Factory Test Mode",
 *   "Reset", "Low Power Mode", "Flight Mode"
 */
static const char *ue_op_mode_lut[] = {
	"Unknown",           /* SIM7022_UE_OP_MODE_ONLINE (fallback) */
	"Online",            /* SIM7022_UE_OP_MODE_ONLINE            */
	"Offline",           /* SIM7022_UE_OP_MODE_OFFLINE           */
	"Factory Test Mode", /* SIM7022_UE_OP_MODE_FACTORY_TEST      */
	"Reset",             /* SIM7022_UE_OP_MODE_RESET             */
	"Low Power Mode",    /* SIM7022_UE_OP_MODE_LOW_POWER         */
	"Flight Mode",       /* SIM7022_UE_OP_MODE_LOW_POWER (alias) */
};

/* Maps ue_sys_mode_lut index → sim7022_ue_sys_mode enum value */
static const enum sim7022_ue_sys_mode sys_mode_map[] = {
	SIM7022_UE_SYS_MODE_NO_SERVICE,
	SIM7022_UE_SYS_MODE_LTE_NB_IOT,
	SIM7022_UE_SYS_MODE_LTE_NB_IOT,
};

/* Maps ue_op_mode_lut index → sim7022_ue_op_mode enum value */
static const enum sim7022_ue_op_mode op_mode_map[] = {
	SIM7022_UE_OP_MODE_ONLINE,        /* "Unknown"           */
	SIM7022_UE_OP_MODE_ONLINE,        /* "Online"            */
	SIM7022_UE_OP_MODE_OFFLINE,       /* "Offline"           */
	SIM7022_UE_OP_MODE_FACTORY_TEST,  /* "Factory Test Mode" */
	SIM7022_UE_OP_MODE_RESET,         /* "Reset"             */
	SIM7022_UE_OP_MODE_LOW_POWER,     /* "Low Power Mode"    */
	SIM7022_UE_OP_MODE_LOW_POWER,     /* "Flight Mode"       */
};

static int lut_match(const char *s, const char **lut, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		if (strcmp(s, lut[i]) == 0) {
			return (int)i;
		}
	}
	return -1;
}

/*
 * Parse the numeric band number from a string like "EUTRAN-BAND3".
 * Finds the last run of digits at the end of the string.
 * Returns 0 if no digits found.
 */
static uint8_t parse_band_number(const char *s)
{
	const char *p = s + strlen(s);

	/* Walk backwards to find end of digit run */
	while (p > s && isdigit((unsigned char)*(p - 1))) {
		p--;
	}
	if (*p == '\0') {
		return 0;
	}
	return (uint8_t)strtoul(p, NULL, 10);
}

static struct sim7022_ue_sys_info *ue_sys_info;

/*
 * Parse the LTE/NB-IoT-specific fields from CPSI response.
 */
static int cpsi_parse_lte(struct sim7022_ue_sys_info *info,
			  uint8_t **argv, uint16_t argc)
{
	if (argc < CPSI_LTE_ARG_COUNT) {
		LOG_ERR("CPSI: not enough LTE fields: %u", argc);
		return -EBADMSG;
	}

	strncpy(info->lte.tac, argv[CPSI_LTE_TAC_IDX],
		sizeof(info->lte.tac) - 1);
	info->lte.tac[sizeof(info->lte.tac) - 1] = '\0';

	strncpy(info->lte.sci, argv[CPSI_LTE_SCI_IDX],
		sizeof(info->lte.sci) - 1);
	info->lte.sci[sizeof(info->lte.sci) - 1] = '\0';

	info->lte.pci    = (uint16_t)strtoul(argv[CPSI_LTE_PCI_IDX],
					     NULL, 10);
	/*
	 * Frequency band field is "EUTRAN-BANDx" (e.g. "EUTRAN-BAND3").
	 * Extract the numeric part from the end of the string.
	 */
	info->lte.band   = parse_band_number(argv[CPSI_LTE_BAND_IDX]);
	info->lte.earfcn = (uint32_t)strtoul(argv[CPSI_LTE_EARFCN_IDX],
					     NULL, 10);
	info->lte.dl_bw  = (uint16_t)strtoul(argv[CPSI_LTE_DLBW_IDX],
					     NULL, 10);
	info->lte.ul_bw  = (uint16_t)strtoul(argv[CPSI_LTE_ULBW_IDX],
					     NULL, 10);

	/* Signal quality fields are signed integers */
	info->lte.rsrq  = (int16_t)strtol(argv[CPSI_LTE_RSRQ_IDX], NULL, 10);
	info->lte.rsrp  = (int16_t)strtol(argv[CPSI_LTE_RSRP_IDX], NULL, 10);
	info->lte.rssi  = (int16_t)strtol(argv[CPSI_LTE_RSSI_IDX], NULL, 10);
	info->lte.rssnr = (int16_t)strtol(argv[CPSI_LTE_RSSNR_IDX], NULL, 10);

	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cpsi)
{
	int idx;

	if (!ue_sys_info) {
		return -EINVAL;
	}

	memset(ue_sys_info, 0, sizeof(*ue_sys_info));

	if (argc < 2) {
		LOG_ERR("CPSI: insufficient arguments: %u", argc);
		return -EBADMSG;
	}

	idx = lut_match(argv[CPSI_SYS_MODE_IDX],
			ue_sys_mode_lut, ARRAY_SIZE(ue_sys_mode_lut));
	if (idx < 0) {
		LOG_ERR("CPSI: unknown sys mode: '%s'",
			argv[CPSI_SYS_MODE_IDX]);
		return -EBADMSG;
	}
	ue_sys_info->sys_mode = sys_mode_map[idx];

	idx = lut_match(argv[CPSI_OP_MODE_IDX],
			ue_op_mode_lut, ARRAY_SIZE(ue_op_mode_lut));
	if (idx < 0) {
		LOG_ERR("CPSI: unknown op mode: '%s'",
			argv[CPSI_OP_MODE_IDX]);
		return -EBADMSG;
	}
	ue_sys_info->op_mode = op_mode_map[idx];

	if (argc >= 3) {
		strncpy(ue_sys_info->mcc_mnc, argv[CPSI_MCC_MNC_IDX],
			sizeof(ue_sys_info->mcc_mnc) - 1);
		ue_sys_info->mcc_mnc[sizeof(ue_sys_info->mcc_mnc) - 1] = '\0';
	}

	if (ue_sys_info->sys_mode != SIM7022_UE_SYS_MODE_NO_SERVICE) {
		return cpsi_parse_lte(ue_sys_info, argv, argc);
	}

	return 0;
}

int mdm_sim7022_get_ue_sys_info(struct sim7022_ue_sys_info *info)
{
	int ret = -EINVAL;
	struct modem_cmd cmds[] = {
		MODEM_CMD("+CPSI: ", on_cmd_cpsi, 14U, ",")
	};

	if (!info) {
		return -EINVAL;
	}

	if (sim7022_get_state() == SIM7022_STATE_OFF) {
		LOG_ERR("SIM7022 is not powered on");
		return -ENODEV;
	}

	ue_sys_info = info;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmds, ARRAY_SIZE(cmds),
			     "AT+CPSI?",
			     &mdata.sem_response, K_SECONDS(9));

	ue_sys_info = NULL;
	return ret;
}

/* ------------------------------------------------------------------ */
/* AT+CCLK? — local time query                                         */
/* ------------------------------------------------------------------ */

static struct tm *local_tm;

/*
 * +CCLK: "<yyyy/MM/dd,hh:mm:ss±zz>"
 *
 * The full datetime including its internal comma is enclosed in quotes.
 * We use MODEM_CMD with argc=1U and delim="" (no splitting) so that
 * argv[0] receives the entire quoted string:
 *   "yyyy/MM/dd,hh:mm:ss±zz"
 *
 * Then we split manually on ',' to separate the date from the time.
 */
MODEM_CMD_DEFINE(on_cmd_cclk)
{
	char *saveptr;
	int ret = -EBADMSG;

	if (!local_tm) {
		return -EINVAL;
	}

	/*
	 * argv[0] = "yyyy/MM/dd,hh:mm:ss±zz"  (including surrounding quotes)
	 * Skip the leading '"' with +1.
	 */
	char *date = strtok_r(argv[0] + 1, ",", &saveptr);

	if (!date) {
		LOG_WRN("CCLK: failed to parse date field");
		return ret;
	}

	/*
	 * After the comma: "hh:mm:ss±zz"
	 * Split on '"' to discard the trailing closing quote.
	 */
	char *time_str = strtok_r(NULL, "\"", &saveptr);

	if (!time_str) {
		LOG_WRN("CCLK: failed to parse time field");
		return ret;
	}

	ret = sim7022_utils_parse_time((uint8_t *)date,
				       (uint8_t *)time_str,
				       local_tm);
	return ret;
}

int mdm_sim7022_get_local_time(struct tm *t)
{
	int ret;
	struct modem_cmd cmds[] = {
		/*
		 * +CCLK: "yyyy/MM/dd,hh:mm:ss±zz"
		 *
		 * Use delim="" (no splitting) with 1U arg so that argv[0]
		 * receives the entire quoted datetime string including its
		 * internal comma.  If we used delim="," argv[0] would be
		 * truncated at the comma between date and time.
		 */
		MODEM_CMD("+CCLK: ", on_cmd_cclk, 1U, "")
	};

	if (!t) {
		return -EINVAL;
	}

	if (sim7022_get_state() == SIM7022_STATE_OFF) {
		return -ENODEV;
	}

	local_tm = t;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmds, ARRAY_SIZE(cmds),
			     "AT+CCLK?",
			     &mdata.sem_response, K_SECONDS(2));

	local_tm = NULL;
	return ret;
}
