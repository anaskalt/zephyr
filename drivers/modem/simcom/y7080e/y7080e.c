/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SIMCom Y7080E NB-IoT modem: power, boot, configuration, attach, PSM.
 *
 * The module boots by itself when VBAT is applied and has no PWRKEY. It
 * is switched with a load switch (mdm-power-gpios) and woken from PSM with
 * a short pulse on RESET (mdm-reset-gpios). Everything else is AT commands
 * over UART1 through modem_chat; the payload of the socket commands is hex
 * encoded, so no raw pipe access is needed anywhere.
 *
 * Reference: Y70XX Series AT Command Manual V1.07, Y7080E Hardware Design
 * V1.00.
 */

#define DT_DRV_COMPAT simcom_y7080e

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/offloaded_netdev.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

#include "y7080e.h"

LOG_MODULE_REGISTER(modem_simcom_y7080e, CONFIG_MODEM_LOG_LEVEL);

#define Y7080E_STR_(x) #x
#define Y7080E_STR(x)  Y7080E_STR_(x)

struct y7080e_data mdata;

static const struct gpio_dt_spec power_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
static const struct gpio_dt_spec status_gpio = GPIO_DT_SPEC_INST_GET_OR(0, mdm_status_gpios, {0});
static const struct gpio_dt_spec ring_gpio = GPIO_DT_SPEC_INST_GET_OR(0, mdm_ring_gpios, {0});

/* ------------------------------------------------------------------ */
/* Shared matches                                                       */
/* ------------------------------------------------------------------ */

const struct modem_chat_match y7080e_ok_match = MODEM_CHAT_MATCH("OK", "", NULL);

const struct modem_chat_match y7080e_abort_matches[2] = {
	MODEM_CHAT_MATCH("ERROR", "", NULL),
	MODEM_CHAT_MATCH("+CME ERROR", "", NULL),
};

static const struct modem_chat_match ok_or_error_matches[] = {
	MODEM_CHAT_MATCH("OK", "", NULL),
	MODEM_CHAT_MATCH("ERROR", "", NULL),
	MODEM_CHAT_MATCH("+CME ERROR", "", NULL),
};

/* ------------------------------------------------------------------ */
/* Small helpers                                                        */
/* ------------------------------------------------------------------ */

static const char *skip_spaces(const char *s)
{
	while (*s == ' ') {
		s++;
	}
	return s;
}

/* Copy a possibly quoted, possibly space-padded field. */
static void copy_field(char *dst, size_t dst_len, const char *src)
{
	size_t len;

	src = skip_spaces(src);
	if (*src == '"') {
		src++;
	}
	len = strcspn(src, "\"");
	len = MIN(len, dst_len - 1);
	memcpy(dst, src, len);
	dst[len] = '\0';
}

static int field_int(const char *s)
{
	return (int)strtol(skip_spaces(s), NULL, 10);
}

/* ------------------------------------------------------------------ */
/* PM guard                                                             */
/* ------------------------------------------------------------------ */

static atomic_t pm_lock_count;

void y7080e_pm_lock(void)
{
	if (atomic_inc(&pm_lock_count) == 0) {
#if defined(CONFIG_PM)
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#endif
	}
}

void y7080e_pm_unlock(void)
{
	if (atomic_dec(&pm_lock_count) == 1) {
#if defined(CONFIG_PM)
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#endif
	}
}

void mdm_y7080e_pm_lock(void)
{
	y7080e_pm_lock();
}

void mdm_y7080e_pm_unlock(void)
{
	y7080e_pm_unlock();
}

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

void y7080e_change_state(enum y7080e_state state)
{
	if (mdata.state != state) {
		LOG_DBG("state %d -> %d", mdata.state, state);
	}
	mdata.state = state;
}

enum y7080e_state y7080e_get_state(void)
{
	return mdata.state;
}

enum y7080e_state mdm_y7080e_get_state(void)
{
	return mdata.state;
}

/* ------------------------------------------------------------------ */
/* URC handlers                                                         */
/* ------------------------------------------------------------------ */

static void on_urc_poweron(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	mdata.poweron_cause = (argc >= 2) ? field_int(argv[1]) : -1;
	LOG_INF("+POWERON: cause %d", mdata.poweron_cause);
	y7080e_flag_set(Y7080E_FLAG_POWERON);
	y7080e_flag_clear(Y7080E_FLAG_PSM_SLEEP);
	y7080e_flag_clear(Y7080E_FLAG_POWERDOWN);
	k_sem_give(&mdata.sem_poweron);
}

static void on_urc_simst(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	int st = (argc >= 2) ? field_int(argv[1]) : 0;

	if (st == 1 || st == 2) {
		y7080e_flag_set(Y7080E_FLAG_SIM_READY);
	} else {
		y7080e_flag_clear(Y7080E_FLAG_SIM_READY);
	}
	LOG_INF("^SIMST: %d", st);
}

static void on_urc_rebooting(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	LOG_WRN("module rebooting");
	y7080e_flag_set(Y7080E_FLAG_REBOOTING);
	y7080e_flag_clear(Y7080E_FLAG_POWERON);
	y7080e_flag_clear(Y7080E_FLAG_ATTACHED);
	y7080e_flag_clear(Y7080E_FLAG_PDN_ACTIVE);
	k_sem_reset(&mdata.sem_poweron);
}

static void on_urc_powerdown(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	LOG_INF("+POWERDOWN: tau offset %s s, rtc wake %s s", argc >= 2 ? argv[1] : "?",
		argc >= 3 ? argv[2] : "?");
	y7080e_flag_set(Y7080E_FLAG_POWERDOWN);
	y7080e_flag_set(Y7080E_FLAG_PSM_SLEEP);
	k_sem_give(&mdata.sem_powerdown);
}

static void on_urc_npsmr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* URC: +NPSMR:<mode>, read: +NPSMR:<n>,<mode>. The mode is last. */
	int mode = (argc >= 2) ? field_int(argv[argc - 1]) : 0;

	if (mode == 1) {
		y7080e_flag_set(Y7080E_FLAG_PSM_SLEEP);
		LOG_INF("+NPSMR: entering PSM");
	} else {
		y7080e_flag_clear(Y7080E_FLAG_PSM_SLEEP);
	}
}

static void on_urc_mnbiotevent(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc < 2) {
		return;
	}
	if (strstr(argv[1], "ENTER") != NULL) {
		y7080e_flag_set(Y7080E_FLAG_PSM_SLEEP);
		LOG_INF("PSM entered");
	} else if (strstr(argv[1], "EXIT") != NULL) {
		y7080e_flag_clear(Y7080E_FLAG_PSM_SLEEP);
		LOG_INF("PSM exited");
	}
}

static const char *const cereg_stat_str[] = {
	"not registered", "registered (home)", "searching", "denied", "unknown",
	"registered (roaming)",
};

static void on_urc_cereg(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	uint8_t stat;
	int d;

	if (argc < 2) {
		return;
	}

	/*
	 * Read form: +CEREG:<n>,<stat>[,...]  (argv[2] is a bare number)
	 * URC form:  +CEREG:<stat>[,<tac>,...] (argv[2], if any, is quoted)
	 */
	if (argc >= 3 && skip_spaces(argv[2])[0] != '"') {
		stat = (uint8_t)field_int(argv[2]);
		d = 3;
	} else {
		stat = (uint8_t)field_int(argv[1]);
		d = 2;
	}

	mdata.registration = stat;
	LOG_INF("+CEREG: %u (%s)", stat,
		stat < ARRAY_SIZE(cereg_stat_str) ? cereg_stat_str[stat] : "?");

	if (stat == 1U || stat == 5U) {
		y7080e_flag_set(Y7080E_FLAG_ATTACHED);
	} else {
		y7080e_flag_clear(Y7080E_FLAG_ATTACHED);
	}

	if (y7080e_cereg_parse_granted(argv, argc, d, &mdata.granted_active_sec,
				       &mdata.granted_tau_sec) == 0) {
		LOG_INF("PSM granted: T3324 %ld s, T3412 %ld s (-1 = deactivated)",
			mdata.granted_active_sec, mdata.granted_tau_sec);
	}
}

static void on_urc_cgev(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	const char *ev = (argc >= 2) ? skip_spaces(argv[1]) : "";

	LOG_INF("+CGEV: %s", ev);

	if (strstr(ev, "PDN ACT") != NULL || strstr(ev, "ME ACT") != NULL) {
		y7080e_flag_set(Y7080E_FLAG_PDN_ACTIVE);
	} else if (strstr(ev, "DEACT") != NULL || strstr(ev, "DETACH") != NULL) {
		y7080e_flag_clear(Y7080E_FLAG_PDN_ACTIVE);
		if (strstr(ev, "DETACH") != NULL) {
			y7080e_flag_clear(Y7080E_FLAG_ATTACHED);
		}
		/* The module drops every socket when the PDN goes away. */
		y7080e_sock_invalidate_all();
	}
}

static void on_urc_xyipdns(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +XYIPDNS:<cid_num>,<cid>,<PDP_type>[,<PDP_address>,"",<dns1>,<dns2>] */
	if (argc >= 5) {
		copy_field(mdata.ip_addr, sizeof(mdata.ip_addr), argv[4]);
		LOG_INF("PDP address %s", mdata.ip_addr);
		y7080e_flag_set(Y7080E_FLAG_PDN_ACTIVE);
	}
}

static void on_urc_cscon(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* URC: +CSCON:<mode>[,<state>...], read: +CSCON:<n>,<mode>[,...].
	 * <state> is 7 for E-UTRAN, so a second field of 0/1 is the read form.
	 */
	int mode;
	int second;

	if (argc < 2) {
		return;
	}
	second = (argc >= 3) ? field_int(argv[2]) : -1;
	mode = (second == 0 || second == 1) ? second : field_int(argv[1]);
	if (mode == 1) {
		y7080e_flag_set(Y7080E_FLAG_RRC_CONNECTED);
	} else {
		y7080e_flag_clear(Y7080E_FLAG_RRC_CONNECTED);
	}
	LOG_DBG("+CSCON: %d", mode);
}

static void on_urc_cgatt(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	int att = (argc >= 2) ? field_int(argv[1]) : 0;

	if (att == 1) {
		y7080e_flag_set(Y7080E_FLAG_ATTACHED);
	} else {
		y7080e_flag_clear(Y7080E_FLAG_ATTACHED);
	}
	LOG_DBG("+CGATT: %d", att);
}

static void on_urc_cgpaddr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +CGPADDR:<cid>[,<addr>] */
	if (argc >= 3 && field_int(argv[1]) == 0) {
		copy_field(mdata.ip_addr, sizeof(mdata.ip_addr), argv[2]);
		if (mdata.ip_addr[0] != '\0' && strcmp(mdata.ip_addr, "0.0.0.0") != 0) {
			y7080e_flag_set(Y7080E_FLAG_PDN_ACTIVE);
		}
	}
}

static void on_urc_cfun(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.cfun = (uint8_t)field_int(argv[1]);
	}
}

static void on_urc_nband(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +NBAND:<n>[,<n>...] with "," separators: rebuild the list. */
	size_t off = 0;

	mdata.nband[0] = '\0';
	for (uint16_t i = 1; i < argc; i++) {
		int n = snprintk(&mdata.nband[off], sizeof(mdata.nband) - off, "%s%d",
				 i > 1 ? "," : "", field_int(argv[i]));

		if (n < 0 || (size_t)n >= sizeof(mdata.nband) - off) {
			break;
		}
		off += (size_t)n;
	}
}

static void on_urc_cgdcont(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +CGDCONT:<cid>,<type>,<apn>,... keep the default context only. */
	if (argc >= 4 && field_int(argv[1]) == 0) {
		copy_field(mdata.apn, sizeof(mdata.apn), argv[3]);
	}
}

static void on_urc_cpsms(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +CPSMS:<mode>,[<rau>],[<ready>],[<tau>],[<active>] */
	if (argc >= 2) {
		mdata.cpsms_mode = (uint8_t)field_int(argv[1]);
	}
	mdata.cpsms_tau[0] = '\0';
	mdata.cpsms_active[0] = '\0';
	if (argc >= 5) {
		copy_field(mdata.cpsms_tau, sizeof(mdata.cpsms_tau), argv[4]);
	}
	if (argc >= 6) {
		copy_field(mdata.cpsms_active, sizeof(mdata.cpsms_active), argv[5]);
	}
}

static void on_urc_resetctl(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.resetctl_mode = (uint8_t)field_int(argv[1]);
	}
}

static void on_urc_csq(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	mdata.csq_rssi = (argc >= 2) ? field_int(argv[1]) : 99;
}

static void on_urc_cgsn(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		copy_field(mdata.imei, sizeof(mdata.imei), argv[1]);
	}
}

static void on_urc_cimi(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		copy_field(mdata.imsi, sizeof(mdata.imsi), argv[1]);
	}
}

static void on_urc_nccid(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		copy_field(mdata.iccid, sizeof(mdata.iccid), argv[1]);
	}
}

static void on_urc_vbat(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.vbat_mv = field_int(argv[1]);
	}
}

static void on_urc_lowvbat(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	LOG_WRN("+LOWVBAT: %s", argc >= 2 ? argv[1] : "");
}

static void on_urc_cclk(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		copy_field(mdata.cclk, sizeof(mdata.cclk), argv[1]);
	}
}

static void on_urc_qdns(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc < 2) {
		return;
	}
	copy_field(mdata.dns_ip, sizeof(mdata.dns_ip), argv[1]);
	mdata.dns_ok = (strncmp(mdata.dns_ip, "QUERY", 5) != 0);
	k_sem_give(&mdata.sem_dns);
}

static void on_urc_nsocr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.nsocr_id = field_int(argv[1]);
	}
}

static void on_urc_nsost(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* +NSOST:<id>,<len> and +NSOSTF:<id>,<len> */
	if (argc >= 3) {
		mdata.nsost_id = field_int(argv[1]);
		mdata.nsost_len = field_int(argv[2]);
	}
}

static void on_urc_nsostr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 4) {
		LOG_INF("+NSOSTR: socket %s seq %s %s", argv[1], argv[2],
			field_int(argv[3]) == 1 ? "sent" : "FAILED");
	}
}

static void on_urc_nsonmi(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	/* Mode 1: +NSONMI:<socket>,<length>. "+NSONMI:drop N bytes pkt" is logged. */
	if (argc >= 3) {
		y7080e_sock_on_nsonmi(field_int(argv[1]), field_int(argv[2]));
	} else if (argc >= 2) {
		LOG_WRN("+NSONMI: %s", argv[1]);
	}
}

static void on_urc_nsorf(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	y7080e_sock_on_nsorf(argv, argc);
}

static void on_urc_nsocli(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		y7080e_sock_on_nsocli(field_int(argv[1]));
	}
}

/* AT+NUESTATS=RADIO lines. Values are in 0.1 dB(m) unless stated. */
static void on_urc_stat_signal_power(struct modem_chat *chat, char **argv, uint16_t argc,
				     void *user_data)
{
	if (argc >= 2) {
		mdata.stats.rsrp_dbm = field_int(argv[1]) / 10;
	}
}

static void on_urc_stat_total_power(struct modem_chat *chat, char **argv, uint16_t argc,
				    void *user_data)
{
	if (argc >= 2) {
		mdata.stats.rssi_dbm = field_int(argv[1]) / 10;
	}
}

static void on_urc_stat_tx_power(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	if (argc >= 2) {
		mdata.stats.tx_power_dbm = field_int(argv[1]) / 10;
	}
}

static void on_urc_stat_cell_id(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data)
{
	if (argc >= 2) {
		mdata.stats.cell_id = (uint32_t)strtoul(skip_spaces(argv[1]), NULL, 0);
	}
}

static void on_urc_stat_ecl(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.stats.ecl = field_int(argv[1]);
	}
}

static void on_urc_stat_snr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.stats.snr_db = field_int(argv[1]) / 10;
	}
}

static void on_urc_stat_earfcn(struct modem_chat *chat, char **argv, uint16_t argc,
			       void *user_data)
{
	if (argc >= 2) {
		mdata.stats.earfcn = (uint32_t)strtoul(skip_spaces(argv[1]), NULL, 0);
	}
}

static void on_urc_stat_pci(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.stats.pci = (uint16_t)field_int(argv[1]);
	}
}

static void on_urc_stat_rsrq(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.stats.rsrq_db = field_int(argv[1]) / 10;
	}
}

static void on_urc_stat_plmn(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		copy_field(mdata.stats.plmn, sizeof(mdata.stats.plmn), argv[1]);
	}
}

static void on_urc_stat_sband(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	if (argc >= 2) {
		mdata.stats.band = (uint16_t)field_int(argv[1]);
	}
}

/*
 * Catch-all for lines without a prefix: the identification replies
 * (AT+CGMI/CGMM/CGMR) and the "<id>,<len>" answer of AT+NSOSD. Only active
 * while a caller has armed the capture under at_lock.
 */
static void on_urc_any(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data)
{
	const char *line;

	if (!y7080e_flag(Y7080E_FLAG_CAPTURE) || argc < 2) {
		return;
	}

	line = skip_spaces(argv[1]);
	if (line[0] == '\0' || strcmp(line, "OK") == 0 || strncmp(line, "AT", 2) == 0) {
		return;
	}

	strncpy(mdata.capture, line, sizeof(mdata.capture) - 1);
	mdata.capture[sizeof(mdata.capture) - 1] = '\0';
	y7080e_flag_clear(Y7080E_FLAG_CAPTURE);
}

/*
 * Every URC and query response the driver understands. The module is
 * configured with AT header space 0 ("+XXX:YY"); atoi() tolerates a space
 * in case the NV setting differs.
 */
static const struct modem_chat_match unsol_matches[] = {
	MODEM_CHAT_MATCH("+POWERON:", ",", on_urc_poweron),
	MODEM_CHAT_MATCH("^SIMST:", ",", on_urc_simst),
	MODEM_CHAT_MATCH("REBOOTING", "", on_urc_rebooting),
	MODEM_CHAT_MATCH("RESETING", "", on_urc_rebooting),
	MODEM_CHAT_MATCH("+POWERDOWN:", ",", on_urc_powerdown),
	MODEM_CHAT_MATCH("+NPSMR:", ",", on_urc_npsmr),
	MODEM_CHAT_MATCH("+MNBIOTEVENT:", ",", on_urc_mnbiotevent),
	MODEM_CHAT_MATCH("+CEREG:", ",", on_urc_cereg),
	MODEM_CHAT_MATCH("+CGEV:", "", on_urc_cgev),
	MODEM_CHAT_MATCH("+XYIPDNS:", ",", on_urc_xyipdns),
	MODEM_CHAT_MATCH("+CSCON:", ",", on_urc_cscon),
	MODEM_CHAT_MATCH("+CGATT:", ",", on_urc_cgatt),
	MODEM_CHAT_MATCH("+CGPADDR:", ",", on_urc_cgpaddr),
	MODEM_CHAT_MATCH("+CFUN:", ",", on_urc_cfun),
	MODEM_CHAT_MATCH("+NBAND:", ",", on_urc_nband),
	MODEM_CHAT_MATCH("+CGDCONT:", ",", on_urc_cgdcont),
	MODEM_CHAT_MATCH("+CPSMS:", ",", on_urc_cpsms),
	MODEM_CHAT_MATCH("+RESETCTL:", ",", on_urc_resetctl),
	MODEM_CHAT_MATCH("+CSQ:", ",", on_urc_csq),
	MODEM_CHAT_MATCH("+CGSN:", ",", on_urc_cgsn),
	MODEM_CHAT_MATCH("+CIMI:", ",", on_urc_cimi),
	MODEM_CHAT_MATCH("+NCCID:", ",", on_urc_nccid),
	MODEM_CHAT_MATCH("+VBAT:", ",", on_urc_vbat),
	MODEM_CHAT_MATCH("+LOWVBAT:", ",", on_urc_lowvbat),
	MODEM_CHAT_MATCH("+CCLK:", "", on_urc_cclk),
	MODEM_CHAT_MATCH("+QDNS:", ",", on_urc_qdns),
	MODEM_CHAT_MATCH("+NSOCR:", ",", on_urc_nsocr),
	MODEM_CHAT_MATCH("+NSOSTF:", ",", on_urc_nsost),
	MODEM_CHAT_MATCH("+NSOST:", ",", on_urc_nsost),
	MODEM_CHAT_MATCH("+NSOSTR:", ",", on_urc_nsostr),
	MODEM_CHAT_MATCH("+NSONMI:", ",", on_urc_nsonmi),
	MODEM_CHAT_MATCH("+NSORF:", ",", on_urc_nsorf),
	MODEM_CHAT_MATCH("+NSOCLI:", ",", on_urc_nsocli),
	MODEM_CHAT_MATCH("+Signal power:", ",", on_urc_stat_signal_power),
	MODEM_CHAT_MATCH("+Total power:", ",", on_urc_stat_total_power),
	MODEM_CHAT_MATCH("+TX power:", ",", on_urc_stat_tx_power),
	MODEM_CHAT_MATCH("+Cell ID:", ",", on_urc_stat_cell_id),
	MODEM_CHAT_MATCH("+ECL:", ",", on_urc_stat_ecl),
	MODEM_CHAT_MATCH("+SNR:", ",", on_urc_stat_snr),
	MODEM_CHAT_MATCH("+EARFCN:", ",", on_urc_stat_earfcn),
	MODEM_CHAT_MATCH("+PCI:", ",", on_urc_stat_pci),
	MODEM_CHAT_MATCH("+RSRQ:", ",", on_urc_stat_rsrq),
	MODEM_CHAT_MATCH("+PLMN:", ",", on_urc_stat_plmn),
	MODEM_CHAT_MATCH("+SBAND:", ",", on_urc_stat_sband),
	/* Catch-all, must be last. */
	MODEM_CHAT_MATCH("", "", on_urc_any),
};

/* ------------------------------------------------------------------ */
/* Script execution                                                     */
/* ------------------------------------------------------------------ */

int y7080e_run_script(const struct modem_chat_script *script, int retries)
{
	int ret;

	y7080e_pm_lock();
	ret = modem_chat_run_script(&mdata.chat, script);
	for (int i = 0; i < retries && ret != 0; i++) {
		k_sleep(K_MSEC(200));
		ret = modem_chat_run_script(&mdata.chat, script);
	}
	y7080e_pm_unlock();

	return ret;
}

static int cmd_run_locked(const struct modem_chat_match *resp, uint16_t nresp,
			  const struct modem_chat_match *abort, uint16_t nabort, uint32_t timeout_s)
{
	int ret;

	if (!mdata.pipe_open) {
		return -ENOTCONN;
	}

	modem_chat_script_chat_init(&mdata.dyn_chat);
	modem_chat_script_chat_set_timeout(&mdata.dyn_chat, 0);
	ret = modem_chat_script_chat_set_request(&mdata.dyn_chat, mdata.cmd_buf);
	ret |= modem_chat_script_chat_set_response_matches(&mdata.dyn_chat, resp, nresp);

	modem_chat_script_init(&mdata.dyn_script);
	modem_chat_script_set_name(&mdata.dyn_script, "at");
	ret |= modem_chat_script_set_script_chats(&mdata.dyn_script, &mdata.dyn_chat, 1);
	ret |= modem_chat_script_set_abort_matches(&mdata.dyn_script, abort, nabort);
	modem_chat_script_set_callback(&mdata.dyn_script, NULL);
	modem_chat_script_set_timeout(&mdata.dyn_script, timeout_s ? timeout_s : 1);
	if (ret < 0) {
		return -EINVAL;
	}

	y7080e_pm_lock();
	ret = modem_chat_run_script(&mdata.chat, &mdata.dyn_script);
	y7080e_pm_unlock();

	if (ret < 0) {
		LOG_DBG("'%.32s' failed (%d)", mdata.cmd_buf, ret);
	}

	return ret;
}

int y7080e_cmd_run_buf(const struct modem_chat_match *resp, uint16_t nresp, uint32_t timeout_s)
{
	int ret;

	if (resp == NULL) {
		resp = &y7080e_ok_match;
		nresp = 1;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	ret = cmd_run_locked(resp, nresp, y7080e_abort_matches, ARRAY_SIZE(y7080e_abort_matches),
			     timeout_s);
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int y7080e_cmd(const struct modem_chat_match *resp, uint16_t nresp, uint32_t timeout_s,
	       const char *fmt, ...)
{
	va_list ap;
	int ret;

	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	va_start(ap, fmt);
	ret = vsnprintk(mdata.cmd_buf, sizeof(mdata.cmd_buf), fmt, ap);
	va_end(ap);
	if (ret < 0 || ret >= (int)sizeof(mdata.cmd_buf)) {
		k_mutex_unlock(&mdata.at_lock);
		return -EMSGSIZE;
	}

	ret = y7080e_cmd_run_buf(resp, nresp, timeout_s);

	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int y7080e_cmd_tolerant(uint32_t timeout_s, const char *fmt, ...)
{
	va_list ap;
	int ret;

	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	va_start(ap, fmt);
	ret = vsnprintk(mdata.cmd_buf, sizeof(mdata.cmd_buf), fmt, ap);
	va_end(ap);
	if (ret < 0 || ret >= (int)sizeof(mdata.cmd_buf)) {
		k_mutex_unlock(&mdata.at_lock);
		return -EMSGSIZE;
	}

	ret = cmd_run_locked(ok_or_error_matches, ARRAY_SIZE(ok_or_error_matches), NULL, 0,
			     timeout_s);

	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

/* Run a command whose interesting reply is a bare line (no prefix). */
static int cmd_capture(char *dst, size_t dst_len, uint32_t timeout_s, const char *cmd)
{
	int ret;

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	mdata.capture[0] = '\0';
	y7080e_flag_set(Y7080E_FLAG_CAPTURE);
	ret = y7080e_cmd(NULL, 0, timeout_s, "%s", cmd);
	y7080e_flag_clear(Y7080E_FLAG_CAPTURE);
	if (ret == 0) {
		strncpy(dst, mdata.capture, dst_len - 1);
		dst[dst_len - 1] = '\0';
	}
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

/* ------------------------------------------------------------------ */
/* UART pipe                                                            */
/* ------------------------------------------------------------------ */

static int pipe_open(void)
{
	int ret;

	if (mdata.pipe_open) {
		return 0;
	}

	ret = modem_pipe_open(mdata.uart_pipe, K_SECONDS(2));
	if (ret < 0) {
		LOG_ERR("UART pipe open failed (%d)", ret);
		return ret;
	}

	modem_chat_attach(&mdata.chat, mdata.uart_pipe);
	mdata.pipe_open = true;

	return 0;
}

static void pipe_close(void)
{
	if (!mdata.pipe_open) {
		return;
	}

	modem_chat_release(&mdata.chat);
	modem_pipe_close(mdata.uart_pipe, K_SECONDS(2));
	mdata.pipe_open = false;
}

static int uart_set_baud(uint32_t baud)
{
	struct uart_config cfg;
	int ret;

	ret = uart_config_get(MDM_UART_DEV, &cfg);
	if (ret < 0) {
		return ret;
	}
	if (cfg.baudrate == baud) {
		return 0;
	}

	pipe_close();
	cfg.baudrate = baud;
	ret = uart_configure(MDM_UART_DEV, &cfg);
	if (ret < 0) {
		LOG_ERR("UART reconfigure to %u failed (%d)", baud, ret);
		return ret;
	}

	return pipe_open();
}

/* ------------------------------------------------------------------ */
/* Hardware control                                                     */
/* ------------------------------------------------------------------ */

static void modem_supply(bool on)
{
	gpio_pin_set_dt(&power_gpio, on ? 1 : 0);
}

static bool modem_status_ready(void)
{
	if (status_gpio.port == NULL) {
		return true;
	}
	return gpio_pin_get_dt(&status_gpio) == 1;
}

static int wait_status(bool ready, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	if (status_gpio.port == NULL) {
		k_sleep(K_MSEC(ready ? 300 : 100));
		return 0;
	}

	while (modem_status_ready() != ready) {
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(20));
	}

	return 0;
}

/* Wake pulse: shorter than the reset threshold of the configured mode. */
static void reset_pulse_wake(void)
{
	gpio_pin_set_dt(&reset_gpio, 1);
	if (mdata.resetctl_mode == 0U) {
		k_busy_wait(MDM_WAKE_PULSE_MODE0_US);
	} else {
		k_sleep(K_MSEC(MDM_WAKE_PULSE_MODE1_MS));
	}
	gpio_pin_set_dt(&reset_gpio, 0);
}

/* ------------------------------------------------------------------ */
/* Boot                                                                 */
/* ------------------------------------------------------------------ */

static int at_probe(int tries)
{
	int ret = -EAGAIN;

	for (int i = 0; i < tries && ret != 0; i++) {
		ret = y7080e_cmd(NULL, 0, MDM_PROBE_TIMEOUT_S, "AT");
		if (ret != 0) {
			k_sleep(K_MSEC(300));
		}
	}

	return ret;
}

/*
 * Establish the AT link. The module remembers its baud rate in NV (factory
 * 9600); the board expects MDM_UART_BAUD. Try the expected rate first, fall
 * back to 9600 and move the module to the expected rate for good.
 */
static int establish_at_link(void)
{
	int ret;

	ret = at_probe(MDM_PROBE_TRIES);
	if (ret == 0) {
		return 0;
	}

	if (MDM_UART_BAUD == MDM_FACTORY_BAUD) {
		return ret;
	}

	LOG_WRN("no answer at %u baud, trying %u", (unsigned int)MDM_UART_BAUD,
		(unsigned int)MDM_FACTORY_BAUD);

	ret = uart_set_baud(MDM_FACTORY_BAUD);
	if (ret < 0) {
		return ret;
	}

	ret = at_probe(MDM_PROBE_TRIES);
	if (ret != 0) {
		(void)uart_set_baud(MDM_UART_BAUD);
		return ret;
	}

	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+IPR=%u", (unsigned int)MDM_UART_BAUD);
	if (ret != 0) {
		LOG_ERR("AT+IPR failed (%d)", ret);
		(void)uart_set_baud(MDM_UART_BAUD);
		return ret;
	}

	k_sleep(K_MSEC(100));
	ret = uart_set_baud(MDM_UART_BAUD);
	if (ret < 0) {
		return ret;
	}

	ret = at_probe(MDM_PROBE_TRIES);
	if (ret == 0) {
		LOG_INF("module moved to %u baud", (unsigned int)MDM_UART_BAUD);
	}

	return ret;
}

static int modem_boot(void)
{
	int ret;

	atomic_clear(&mdata.flags);
	mdata.registration = 0;
	mdata.granted_active_sec = -1;
	mdata.granted_tau_sec = -1;
	mdata.ip_addr[0] = '\0';
	k_sem_reset(&mdata.sem_poweron);

	/* RESET must be low before the supply arrives. */
	gpio_pin_set_dt(&reset_gpio, 0);
	modem_supply(true);

	ret = wait_status(true, MDM_STATUS_TIMEOUT_MS);
	if (ret < 0) {
		LOG_ERR("STATUS never went high");
		return ret;
	}

	ret = pipe_open();
	if (ret < 0) {
		return ret;
	}

	/* +POWERON and ^SIMST arrive at the module's NV baud rate. */
	(void)k_sem_take(&mdata.sem_poweron, K_MSEC(MDM_POWERON_TIMEOUT_MS));

	ret = establish_at_link();
	if (ret != 0) {
		LOG_ERR("module does not answer AT (%d)", ret);
		return ret;
	}

	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "ATE0");
	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CMEE=1");

	return 0;
}

/* ------------------------------------------------------------------ */
/* Configuration                                                        */
/* ------------------------------------------------------------------ */

static void modem_identify(void)
{
	(void)cmd_capture(mdata.manufacturer, sizeof(mdata.manufacturer), MDM_CMD_TIMEOUT_S,
			  "AT+CGMI");
	(void)cmd_capture(mdata.model, sizeof(mdata.model), MDM_CMD_TIMEOUT_S, "AT+CGMM");
	(void)cmd_capture(mdata.revision, sizeof(mdata.revision), MDM_CMD_TIMEOUT_S, "AT+CGMR");
	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CGSN=1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CIMI");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NCCID");

	LOG_INF("%s %s fw %s IMEI %s", mdata.manufacturer, mdata.model, mdata.revision,
		mdata.imei);
	if (mdata.iccid[0] != '\0') {
		LOG_INF("SIM ICCID %s IMSI %s", mdata.iccid, mdata.imsi);
	} else {
		LOG_WRN("no SIM identification");
	}
}

/* Volatile settings, lost on every module reboot. */
static void modem_configure_urc(void)
{
	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CEREG=5");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CGEREP=1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CSCON=1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NPSMR=1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+MNBIOTEVENT=1,1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NSONMI=1");
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CEDRXS=0");
	if (ring_gpio.port != NULL) {
		(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CMSRI=1");
	}
}

/*
 * AT+NV=SAVE persists the factory NV; some firmware versions reboot the
 * module afterwards. Returns 0 when the module stayed up, 1 when it
 * rebooted (and is back), negative errno otherwise.
 */
static int modem_nv_save(void)
{
	int ret;

	y7080e_flag_clear(Y7080E_FLAG_REBOOTING);
	k_sem_reset(&mdata.sem_poweron);

	ret = y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NV=SAVE");
	if (ret != 0) {
		/* No reply: it may have rebooted before answering. */
		if (k_sem_take(&mdata.sem_poweron, K_SECONDS(10)) != 0) {
			return ret;
		}
	} else {
		k_sleep(K_MSEC(200));
		if (!y7080e_flag(Y7080E_FLAG_REBOOTING)) {
			return 0;
		}
		if (k_sem_take(&mdata.sem_poweron, K_SECONDS(10)) != 0) {
			LOG_WRN("reboot after NV save not confirmed");
		}
	}

	ret = at_probe(MDM_PROBE_TRIES);
	if (ret != 0) {
		return ret;
	}
	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "ATE0");
	(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CMEE=1");

	return 1;
}

static int radio_off(void)
{
	return y7080e_cmd(NULL, 0, MDM_CFUN_TIMEOUT_S, "AT+CFUN=0");
}

static int radio_on(void)
{
	return y7080e_cmd(NULL, 0, MDM_CFUN_TIMEOUT_S, "AT+CFUN=1");
}

static bool band_list_equal(const char *a, const char *b)
{
	/* Both are comma separated decimal lists; compare as sets of ints. */
	uint64_t ma = 0, mb = 0;
	const char *p;

	for (p = a; *p != '\0'; p++) {
		if (*p >= '0' && *p <= '9') {
			int v = (int)strtol(p, (char **)&p, 10);

			ma |= (v < 64) ? BIT64(v) : 0;
			if (*p == '\0') {
				break;
			}
		}
	}
	for (p = b; *p != '\0'; p++) {
		if (*p >= '0' && *p <= '9') {
			int v = (int)strtol(p, (char **)&p, 10);

			mb |= (v < 64) ? BIT64(v) : 0;
			if (*p == '\0') {
				break;
			}
		}
	}

	return ma == mb;
}

/*
 * Persistent settings: compared first and written only on a difference so
 * a normal boot never touches the module's NV flash. Returns 1 when the
 * module rebooted during the process.
 */
static int modem_configure_persistent(void)
{
	bool rebooted = false;
	int ret;

	/* RESET pin mode drives our wake pulse width. */
	mdata.resetctl_mode = 0xFF;
	if (y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+RESETCTL?") == 0 &&
	    mdata.resetctl_mode != 0xFF &&
	    mdata.resetctl_mode != CONFIG_MODEM_SIMCOM_Y7080E_RESETCTL_MODE) {
		LOG_INF("RESETCTL %u -> %u", mdata.resetctl_mode,
			CONFIG_MODEM_SIMCOM_Y7080E_RESETCTL_MODE);
		(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+RESETCTL=%u",
					  CONFIG_MODEM_SIMCOM_Y7080E_RESETCTL_MODE);
	}
	mdata.resetctl_mode = CONFIG_MODEM_SIMCOM_Y7080E_RESETCTL_MODE;

	/* Bands: only settable with the radio off. */
	if (CONFIG_MODEM_SIMCOM_Y7080E_BAND_LIST[0] != '\0') {
		mdata.nband[0] = '\0';
		if (y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NBAND?") == 0 &&
		    !band_list_equal(mdata.nband, CONFIG_MODEM_SIMCOM_Y7080E_BAND_LIST)) {
			LOG_INF("bands %s -> %s", mdata.nband, CONFIG_MODEM_SIMCOM_Y7080E_BAND_LIST);
			if (radio_off() == 0) {
				ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+NBAND=%s",
						 CONFIG_MODEM_SIMCOM_Y7080E_BAND_LIST);
				if (ret != 0) {
					LOG_WRN("AT+NBAND rejected (%d)", ret);
				}
				(void)radio_on();
			}
		}
	}

	/* APN of the default context. */
	if (CONFIG_MODEM_SIMCOM_Y7080E_APN[0] != '\0') {
		mdata.apn[0] = '\0';
		if (y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CGDCONT?") == 0 &&
		    strcmp(mdata.apn, CONFIG_MODEM_SIMCOM_Y7080E_APN) != 0) {
			LOG_INF("APN '%s' -> '%s'", mdata.apn, CONFIG_MODEM_SIMCOM_Y7080E_APN);
			if (radio_off() == 0) {
				ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S,
						 "AT+CGDCONT=0,\"IP\",\"%s\"",
						 CONFIG_MODEM_SIMCOM_Y7080E_APN);
				if (ret == 0) {
					ret = modem_nv_save();
					rebooted = (ret > 0);
				} else {
					LOG_WRN("AT+CGDCONT rejected (%d)", ret);
				}
				(void)radio_on();
			}
		}
	}

	/* PSM request. */
#if defined(CONFIG_MODEM_SIMCOM_Y7080E_PSM)
	mdata.cpsms_mode = 0xFF;
	if (y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CPSMS?") == 0 &&
	    (mdata.cpsms_mode != 1U ||
	     strcmp(mdata.cpsms_tau, CONFIG_MODEM_SIMCOM_Y7080E_PSM_TAU) != 0 ||
	     strcmp(mdata.cpsms_active, CONFIG_MODEM_SIMCOM_Y7080E_PSM_ACTIVE_TIME) != 0)) {
		LOG_INF("PSM request TAU %s active %s", CONFIG_MODEM_SIMCOM_Y7080E_PSM_TAU,
			CONFIG_MODEM_SIMCOM_Y7080E_PSM_ACTIVE_TIME);
		ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CPSMS=1,,,\"%s\",\"%s\"",
				 CONFIG_MODEM_SIMCOM_Y7080E_PSM_TAU,
				 CONFIG_MODEM_SIMCOM_Y7080E_PSM_ACTIVE_TIME);
		if (ret == 0) {
			ret = modem_nv_save();
			rebooted = rebooted || (ret > 0);
		} else {
			LOG_WRN("AT+CPSMS rejected (%d)", ret);
		}
	}
#else
	mdata.cpsms_mode = 0xFF;
	if (y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CPSMS?") == 0 && mdata.cpsms_mode == 1U) {
		(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CPSMS=0");
		(void)modem_nv_save();
	}
#endif

	return rebooted ? 1 : 0;
}

static int modem_setup(void)
{
	int ret;

	y7080e_pm_lock();
	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	y7080e_change_state(Y7080E_STATE_OFF);

	ret = modem_boot();
	if (ret < 0) {
		goto out;
	}

	modem_identify();
	modem_configure_urc();
	ret = modem_configure_persistent();
	if (ret > 0) {
		/* The module rebooted: the volatile settings are gone. */
		modem_configure_urc();
	}
	ret = 0;

	/* Any AT command holds the work lock; make it explicit. */
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+WORKLOCK=1");

	y7080e_change_state(Y7080E_STATE_IDLE);

out:
	if (ret < 0) {
		pipe_close();
		modem_supply(false);
		y7080e_change_state(Y7080E_STATE_OFF);
	}
	k_mutex_unlock(&mdata.at_lock);
	y7080e_pm_unlock();

	return ret;
}

/* ------------------------------------------------------------------ */
/* Public API: power                                                    */
/* ------------------------------------------------------------------ */

int mdm_y7080e_power_on(void)
{
	if (mdata.state == Y7080E_STATE_IDLE || mdata.state == Y7080E_STATE_NETWORKING) {
		return 0;
	}
	if (mdata.state == Y7080E_STATE_SLEEPING) {
		return mdm_y7080e_wake();
	}

	return modem_setup();
}

int mdm_y7080e_power_off(void)
{
	int ret = 0;

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	y7080e_pm_lock();

	if (mdata.state == Y7080E_STATE_OFF) {
		goto out;
	}

	if (mdata.state == Y7080E_STATE_SLEEPING) {
		/* Nothing to say to a sleeping module: cut the supply. */
		goto cut;
	}

	y7080e_flag_clear(Y7080E_FLAG_POWERDOWN);
	k_sem_reset(&mdata.sem_powerdown);
	ret = y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+FASTOFF=1");
	if (ret == 0) {
		(void)k_sem_take(&mdata.sem_powerdown, K_SECONDS(5));
	}
	(void)wait_status(false, 2000);

cut:
	pipe_close();
	modem_supply(false);
	y7080e_sock_invalidate_all();
	atomic_clear(&mdata.flags);
	y7080e_change_state(Y7080E_STATE_OFF);
	/* Let the load switch decay (R37 x C50 = 350 ms) before a re-power. */
	k_sleep(K_MSEC(500));

out:
	y7080e_pm_unlock();
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int mdm_y7080e_force_reset(void)
{
	LOG_WRN("force reset");
	(void)mdm_y7080e_power_off();

	return modem_setup();
}

/* ------------------------------------------------------------------ */
/* Public API: network                                                  */
/* ------------------------------------------------------------------ */

static int wait_registered(int64_t deadline)
{
	while (true) {
		(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CEREG?");
		if (mdata.registration == 1U || mdata.registration == 5U) {
			return 0;
		}
		if (mdata.registration == 3U) {
			LOG_WRN("registration denied");
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(MDM_REG_POLL_MS));
	}
}

static int wait_pdn(int64_t deadline)
{
	while (true) {
		(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CGATT?");
		if (y7080e_flag(Y7080E_FLAG_ATTACHED)) {
			mdata.ip_addr[0] = '\0';
			(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CGPADDR=0");
			if (y7080e_flag(Y7080E_FLAG_PDN_ACTIVE) || mdata.ip_addr[0] != '\0') {
				return 0;
			}
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(MDM_REG_POLL_MS));
	}
}

int mdm_y7080e_start_network(void)
{
	int64_t deadline;
	int ret;

	if (mdata.state == Y7080E_STATE_OFF) {
		ret = modem_setup();
		if (ret < 0) {
			return ret;
		}
	} else if (mdata.state == Y7080E_STATE_SLEEPING) {
		ret = mdm_y7080e_wake();
		if (ret < 0) {
			return ret;
		}
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	y7080e_pm_lock();

	mdata.cfun = 0xFF;
	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CFUN?");
	if (mdata.cfun != 1U) {
		ret = radio_on();
		if (ret != 0) {
			LOG_ERR("AT+CFUN=1 failed (%d)", ret);
			goto out;
		}
	}

	deadline = k_uptime_get() + (int64_t)CONFIG_MODEM_SIMCOM_Y7080E_ATTACH_TIMEOUT_S * 1000;

	ret = wait_registered(deadline);
	if (ret < 0) {
		LOG_ERR("not registered after %d s", CONFIG_MODEM_SIMCOM_Y7080E_ATTACH_TIMEOUT_S);
		goto out;
	}

	ret = wait_pdn(MAX(deadline, k_uptime_get() + MDM_PDN_TIMEOUT_S * 1000));
	if (ret < 0) {
		LOG_ERR("PDN not active");
		goto out;
	}

	LOG_INF("attached, IP %s", mdata.ip_addr);
	y7080e_change_state(Y7080E_STATE_NETWORKING);
	ret = 0;

out:
	y7080e_pm_unlock();
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

/* ------------------------------------------------------------------ */
/* Public API: sleep / wake                                             */
/* ------------------------------------------------------------------ */

int mdm_y7080e_sleep(void)
{
	int ret;

	if (mdata.state == Y7080E_STATE_OFF || mdata.state == Y7080E_STATE_SLEEPING) {
		return 0;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	y7080e_pm_lock();

	y7080e_flag_clear(Y7080E_FLAG_POWERDOWN);
	k_sem_reset(&mdata.sem_powerdown);

	ret = y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+WORKLOCK=0");
	if (ret != 0) {
		LOG_WRN("AT+WORKLOCK=0 failed (%d)", ret);
	}

	/*
	 * The module needs the RRC connection released and T3324 expired
	 * before it powers down. With a PSM grant this takes a few seconds;
	 * without one +POWERDOWN never comes.
	 */
	if (k_sem_take(&mdata.sem_powerdown, K_MSEC(CONFIG_MODEM_SIMCOM_Y7080E_SLEEP_WAIT_MS)) !=
	    0) {
		LOG_WRN("no +POWERDOWN within %d ms (PSM not granted?)",
			CONFIG_MODEM_SIMCOM_Y7080E_SLEEP_WAIT_MS);
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	/* Sockets do not survive deep sleep. */
	y7080e_sock_invalidate_all();

	pipe_close();
	y7080e_change_state(Y7080E_STATE_SLEEPING);

	y7080e_pm_unlock();
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int mdm_y7080e_wake(void)
{
	int ret;

	if (mdata.state == Y7080E_STATE_OFF) {
		return modem_setup();
	}
	if (mdata.state != Y7080E_STATE_SLEEPING) {
		return 0;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	y7080e_pm_lock();

	ret = pipe_open();
	if (ret < 0) {
		goto out;
	}

	y7080e_flag_clear(Y7080E_FLAG_POWERON);
	k_sem_reset(&mdata.sem_poweron);

	reset_pulse_wake();

	/* A module that really slept announces itself; one that did not
	 * ignores the pulse and simply answers the probe.
	 */
	(void)k_sem_take(&mdata.sem_poweron, K_MSEC(MDM_POWERON_TIMEOUT_MS));

	ret = at_probe(MDM_PROBE_TRIES);
	if (ret != 0) {
		LOG_ERR("module did not wake (%d)", ret);
		goto out;
	}

	if (y7080e_flag(Y7080E_FLAG_REBOOTING) ||
	    (mdata.poweron_cause >= 0 && mdata.poweron_cause != 3 && mdata.poweron_cause != 4 &&
	     mdata.poweron_cause != 10)) {
		/* A reset rather than a wake: volatile settings are gone. */
		LOG_WRN("module restarted (cause %d), reconfiguring", mdata.poweron_cause);
		(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "ATE0");
		(void)y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CMEE=1");
		modem_configure_urc();
		y7080e_flag_clear(Y7080E_FLAG_REBOOTING);
	}

	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+WORKLOCK=1");
	y7080e_flag_clear(Y7080E_FLAG_PSM_SLEEP);

	(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CEREG?");
	if (mdata.registration == 1U || mdata.registration == 5U) {
		y7080e_change_state(Y7080E_STATE_NETWORKING);
	} else {
		y7080e_change_state(Y7080E_STATE_IDLE);
	}
	ret = 0;

out:
	if (ret < 0) {
		pipe_close();
	}
	y7080e_pm_unlock();
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

/* ------------------------------------------------------------------ */
/* Public API: queries                                                  */
/* ------------------------------------------------------------------ */

const char *mdm_y7080e_get_manufacturer(void)
{
	return mdata.manufacturer;
}

const char *mdm_y7080e_get_model(void)
{
	return mdata.model;
}

const char *mdm_y7080e_get_revision(void)
{
	return mdata.revision;
}

const char *mdm_y7080e_get_imei(void)
{
	return mdata.imei;
}

const char *mdm_y7080e_get_imsi(void)
{
	return mdata.imsi;
}

const char *mdm_y7080e_get_iccid(void)
{
	return mdata.iccid;
}

uint8_t mdm_y7080e_get_registration(void)
{
	return mdata.registration;
}

int mdm_y7080e_get_psm_timers(int *active_time_sec, int *tau_sec)
{
	if (mdata.granted_active_sec < 0 && mdata.granted_tau_sec < 0) {
		return -EAGAIN;
	}
	if (active_time_sec != NULL) {
		*active_time_sec = (int)mdata.granted_active_sec;
	}
	if (tau_sec != NULL) {
		*tau_sec = (int)mdata.granted_tau_sec;
	}

	return 0;
}

static int require_awake(void)
{
	if (mdata.state == Y7080E_STATE_OFF) {
		return -ENODEV;
	}
	if (mdata.state == Y7080E_STATE_SLEEPING) {
		return -EHOSTDOWN;
	}

	return 0;
}

int mdm_y7080e_get_rssi(int *rssi_dbm)
{
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	mdata.csq_rssi = 99;
	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CSQ");
	if (ret == 0 && rssi_dbm != NULL) {
		*rssi_dbm = y7080e_csq_to_dbm(mdata.csq_rssi);
	}

	return ret;
}

int mdm_y7080e_get_radio_stats(struct y7080e_radio_stats *stats)
{
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	memset(&mdata.stats, 0, sizeof(mdata.stats));
	mdata.stats.ecl = -1;
	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+NUESTATS=RADIO");
	if (ret == 0 && stats != NULL) {
		*stats = mdata.stats;
	}
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int mdm_y7080e_get_ip(char *buf, size_t len)
{
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	mdata.ip_addr[0] = '\0';
	ret = y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CGPADDR=0");
	if (ret == 0) {
		if (mdata.ip_addr[0] == '\0') {
			ret = -ENODATA;
		} else {
			strncpy(buf, mdata.ip_addr, len - 1);
			buf[len - 1] = '\0';
		}
	}
	k_mutex_unlock(&mdata.at_lock);

	return ret;
}

int mdm_y7080e_get_time(struct tm *t)
{
	int year, mon, day, hour, min, sec;
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	mdata.cclk[0] = '\0';
	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+CCLK?");
	k_mutex_unlock(&mdata.at_lock);
	if (ret != 0) {
		return ret;
	}

	/* "yy/MM/dd,hh:mm:ss+zz" or "yyyy/MM/dd,..." */
	if (sscanf(mdata.cclk, "%d/%d/%d,%d:%d:%d", &year, &mon, &day, &hour, &min, &sec) != 6) {
		return -EBADMSG;
	}
	if (year < 100) {
		year += 2000;
	}

	memset(t, 0, sizeof(*t));
	t->tm_year = year - 1900;
	t->tm_mon = mon - 1;
	t->tm_mday = day;
	t->tm_hour = hour;
	t->tm_min = min;
	t->tm_sec = sec;

	return 0;
}

int mdm_y7080e_get_vbat_mv(int *mv)
{
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	mdata.vbat_mv = 0;
	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+VBAT?");
	if (ret == 0 && mv != NULL) {
		*mv = mdata.vbat_mv;
	}

	return ret;
}

int mdm_y7080e_release_rrc(void)
{
	int ret;

	ret = require_awake();
	if (ret < 0) {
		return ret;
	}

	return y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+CNMPSD");
}

/* ------------------------------------------------------------------ */
/* Network interface                                                    */
/* ------------------------------------------------------------------ */

static uint32_t hash32(const char *str, size_t len)
{
	uint32_t h = 0;

	for (size_t i = 0; i < len; i++) {
		h = (h * 37U) + (uint8_t)str[i];
	}

	return h;
}

static void modem_net_iface_init(struct net_if *iface)
{
	uint32_t h = hash32(mdata.imei, strlen(mdata.imei));

	mdata.iface = iface;
	mdata.mac_addr[0] = 0x00;
	mdata.mac_addr[1] = 0x10;
	UNALIGNED_PUT(h, (uint32_t *)&mdata.mac_addr[2]);

	net_if_set_link_addr(iface, mdata.mac_addr, sizeof(mdata.mac_addr), NET_LINK_ETHERNET);
	socket_offload_dns_register(&y7080e_dns_ops);
	net_if_socket_offload_set(iface, y7080e_offload_socket);
}

static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
};

static bool offload_is_supported(int family, int type, int proto)
{
	return family == NET_AF_INET && (type == NET_SOCK_DGRAM || type == NET_SOCK_STREAM) &&
	       (proto == NET_IPPROTO_UDP || proto == NET_IPPROTO_TCP);
}

/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */

static int modem_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	k_mutex_init(&mdata.at_lock);
	k_sem_init(&mdata.sem_poweron, 0, 1);
	k_sem_init(&mdata.sem_powerdown, 0, 1);
	k_sem_init(&mdata.sem_dns, 0, 1);
	mdata.state = Y7080E_STATE_OFF;
	mdata.granted_active_sec = -1;
	mdata.granted_tau_sec = -1;
	mdata.resetctl_mode = CONFIG_MODEM_SIMCOM_Y7080E_RESETCTL_MODE;

	ret = modem_socket_init(&mdata.socket_config, mdata.sockets, ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM, false, &y7080e_socket_fd_op_vtable);
	if (ret < 0) {
		return ret;
	}

	/* Safe electrical state: supply off, RESET released. */
	ret = gpio_pin_configure_dt(&power_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}
	if (status_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&status_gpio, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}
	if (ring_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&ring_gpio, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}

	{
		const struct modem_backend_uart_config uart_cfg = {
			.uart = MDM_UART_DEV,
			.dtr_gpio = NULL,
			.receive_buf = mdata.uart_rx,
			.receive_buf_size = sizeof(mdata.uart_rx),
			.transmit_buf = mdata.uart_tx,
			.transmit_buf_size = sizeof(mdata.uart_tx),
		};

		mdata.uart_pipe = modem_backend_uart_init(&mdata.uart_backend, &uart_cfg);
		if (mdata.uart_pipe == NULL) {
			LOG_ERR("UART backend init failed");
			return -ENODEV;
		}
	}

	{
		mdata.chat_delimiter[0] = '\r';
		mdata.chat_delimiter[1] = '\n';

		const struct modem_chat_config chat_cfg = {
			.user_data = &mdata,
			.receive_buf = mdata.chat_rx,
			.receive_buf_size = sizeof(mdata.chat_rx),
			.delimiter = mdata.chat_delimiter,
			.delimiter_size = 2,
			.filter = NULL,
			.filter_size = 0,
			.argv = mdata.chat_argv,
			.argv_size = ARRAY_SIZE(mdata.chat_argv),
			.unsol_matches = unsol_matches,
			.unsol_matches_size = ARRAY_SIZE(unsol_matches),
		};

		ret = modem_chat_init(&mdata.chat, &chat_cfg);
		if (ret < 0) {
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_MODEM_SIMCOM_Y7080E_AUTOSTART)) {
		ret = modem_setup();
		if (ret < 0) {
			LOG_ERR("autostart failed (%d)", ret);
		}
	}

	return 0;
}

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_SIMCOM_Y7080E_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);

NET_SOCKET_OFFLOAD_REGISTER(simcom_y7080e, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, NET_AF_UNSPEC,
			    offload_is_supported, y7080e_offload_socket);
