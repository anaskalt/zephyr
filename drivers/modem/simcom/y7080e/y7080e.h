/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_H_
#define ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/modem/simcom-y7080e.h>
#include <zephyr/kernel.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/sys/atomic.h>

#include "modem_socket.h"
#include "y7080e_lpm.h"

/* ------------------------------------------------------------------ */
/* Hardware                                                            */
/* ------------------------------------------------------------------ */

#define MDM_UART_NODE DT_INST_BUS(0)
#define MDM_UART_DEV  DEVICE_DT_GET(MDM_UART_NODE)
#define MDM_UART_BAUD DT_PROP(MDM_UART_NODE, current_speed)

/* Baud rate of a module whose NV still holds the factory setting. */
#define MDM_FACTORY_BAUD 9600U

/* ------------------------------------------------------------------ */
/* Sizes                                                               */
/* ------------------------------------------------------------------ */

#define MDM_MAX_DATA_LENGTH CONFIG_MODEM_SIMCOM_Y7080E_MAX_DATA_LENGTH
#define MDM_CMD_BUF_SIZE    (2 * MDM_MAX_DATA_LENGTH + 96)
#define MDM_UART_BUF_SIZE   CONFIG_MODEM_SIMCOM_Y7080E_UART_BUF_SIZE
#define MDM_CHAT_BUF_SIZE   CONFIG_MODEM_SIMCOM_Y7080E_CHAT_BUF_SIZE
#define MDM_CHAT_ARGV_COUNT 24

/* The module supports two sockets, link ids 0 and 1. */
#define MDM_MAX_SOCKETS     2
#define MDM_BASE_SOCKET_NUM 0

#define MDM_MANUFACTURER_LENGTH 16
#define MDM_MODEL_LENGTH        16
#define MDM_REVISION_LENGTH     32
#define MDM_IMEI_LENGTH         16
#define MDM_IMSI_LENGTH         16
#define MDM_ICCID_LENGTH        24
#define MDM_APN_LENGTH          64

/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */

#define MDM_CMD_TIMEOUT_S      10
#define MDM_CFUN_TIMEOUT_S     20
#define MDM_CONNECT_TIMEOUT_S  30
#define MDM_STATUS_TIMEOUT_MS  3000
#define MDM_POWERON_TIMEOUT_MS 5000
#define MDM_PROBE_TRIES        5
#define MDM_PROBE_TIMEOUT_S    2
#define MDM_REG_POLL_MS        2000
#define MDM_PDN_TIMEOUT_S      60

/* RESET pulse widths for the two AT+RESETCTL modes. */
#define MDM_WAKE_PULSE_MODE1_MS 100
#define MDM_WAKE_PULSE_MODE0_US 5000
#define MDM_RESET_HOLD_MODE1_MS 6500
#define MDM_RESET_HOLD_MODE0_MS 50

/* ------------------------------------------------------------------ */
/* Status flags                                                        */
/* ------------------------------------------------------------------ */

enum y7080e_flag {
	Y7080E_FLAG_POWERON = 0,
	Y7080E_FLAG_SIM_READY,
	Y7080E_FLAG_POWERDOWN,
	Y7080E_FLAG_REBOOTING,
	Y7080E_FLAG_PSM_SLEEP,
	Y7080E_FLAG_ATTACHED,
	Y7080E_FLAG_PDN_ACTIVE,
	Y7080E_FLAG_RRC_CONNECTED,
	Y7080E_FLAG_CAPTURE,
};

/* ------------------------------------------------------------------ */
/* Driver data                                                          */
/* ------------------------------------------------------------------ */

struct y7080e_data {
	/* UART backend + chat */
	struct modem_backend_uart uart_backend;
	struct modem_pipe *uart_pipe;
	uint8_t uart_rx[MDM_UART_BUF_SIZE];
	uint8_t uart_tx[MDM_UART_BUF_SIZE];
	struct modem_chat chat;
	uint8_t chat_rx[MDM_CHAT_BUF_SIZE];
	uint8_t *chat_argv[MDM_CHAT_ARGV_COUNT];
	uint8_t chat_delimiter[2];
	bool pipe_open;

	/* Dynamic one-shot script, serialised by at_lock. */
	struct k_mutex at_lock;
	char cmd_buf[MDM_CMD_BUF_SIZE];
	struct modem_chat_script_chat dyn_chat;
	struct modem_chat_script dyn_script;

	/* Bare-line capture for prefix-less responses. */
	char capture[MDM_REVISION_LENGTH];

	/* Sockets */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];
	struct net_if *iface;
	uint8_t mac_addr[6];

	/* State */
	enum y7080e_state state;
	atomic_t flags;
	struct k_sem sem_poweron;
	struct k_sem sem_powerdown;
	struct k_sem sem_dns;
	int poweron_cause;
	uint8_t registration;
	uint8_t cfun;
	uint8_t resetctl_mode;
	long granted_active_sec;
	long granted_tau_sec;
	char ip_addr[NET_IPV4_ADDR_LEN];

	/* Identification */
	char manufacturer[MDM_MANUFACTURER_LENGTH];
	char model[MDM_MODEL_LENGTH];
	char revision[MDM_REVISION_LENGTH];
	char imei[MDM_IMEI_LENGTH];
	char imsi[MDM_IMSI_LENGTH];
	char iccid[MDM_ICCID_LENGTH];

	/* Query results */
	int csq_rssi;
	int vbat_mv;
	char cclk[24];
	char nband[32];
	char apn[MDM_APN_LENGTH];
	uint8_t cpsms_mode;
	char cpsms_tau[12];
	char cpsms_active[12];
	struct y7080e_radio_stats stats;

	/* Socket command results (valid while at_lock is held) */
	int nsocr_id;
	int nsost_id;
	int nsost_len;
	int nsosd_id;
	int nsosd_len;
	uint8_t *nsorf_dst;
	size_t nsorf_max;
	int nsorf_len;
	uint16_t nsorf_remaining;
	char nsorf_ip[NET_IPV4_ADDR_LEN];
	uint16_t nsorf_port;

	/* DNS */
	char dns_ip[NET_IPV4_ADDR_LEN];
	bool dns_ok;
};

extern struct y7080e_data mdata;

extern const struct modem_chat_match y7080e_ok_match;
extern const struct modem_chat_match y7080e_abort_matches[2];
extern const struct socket_op_vtable y7080e_socket_fd_op_vtable;
extern const struct socket_dns_offload y7080e_dns_ops;

/* ------------------------------------------------------------------ */
/* Internal API                                                         */
/* ------------------------------------------------------------------ */

/* Stop2 / UART clock guard, refcounted. */
void y7080e_pm_lock(void);
void y7080e_pm_unlock(void);

void y7080e_change_state(enum y7080e_state state);
enum y7080e_state y7080e_get_state(void);

static inline bool y7080e_flag(enum y7080e_flag f)
{
	return atomic_test_bit(&mdata.flags, f);
}

static inline void y7080e_flag_set(enum y7080e_flag f)
{
	atomic_set_bit(&mdata.flags, f);
}

static inline void y7080e_flag_clear(enum y7080e_flag f)
{
	atomic_clear_bit(&mdata.flags, f);
}

/*
 * Run a static script with the PM lock held. Retries the whole script
 * @p retries extra times on failure.
 */
int y7080e_run_script(const struct modem_chat_script *script, int retries);

/*
 * Format one AT command into mdata.cmd_buf and run it as a one-step script
 * that completes on any of @p resp (OK by default) and aborts on ERROR.
 * Takes at_lock (recursively) and the PM lock. Returns 0, -EAGAIN on
 * ERROR/timeout, other negative errno on failure.
 */
int y7080e_cmd(const struct modem_chat_match *resp, uint16_t nresp, uint32_t timeout_s,
	       const char *fmt, ...);

/* Same, but ERROR is accepted as a normal (logged) completion. */
int y7080e_cmd_tolerant(uint32_t timeout_s, const char *fmt, ...);

/* Run whatever the caller has already placed in mdata.cmd_buf. */
int y7080e_cmd_run_buf(const struct modem_chat_match *resp, uint16_t nresp, uint32_t timeout_s);

/* Socket helpers implemented in y7080e_sock.c. */
int y7080e_offload_socket(int family, int type, int proto);
void y7080e_sock_on_nsonmi(int id, int len);
void y7080e_sock_on_nsocli(int id);
void y7080e_sock_on_nsorf(char **argv, uint16_t argc);
void y7080e_sock_invalidate_all(void);

#endif /* ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_H_ */
