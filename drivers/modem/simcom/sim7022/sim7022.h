/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SIMCOM_SIM7022_H
#define SIMCOM_SIM7022_H

#include <zephyr/kernel.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/modem/simcom-sim7022.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <string.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "modem_context.h"
#include "modem_cmd_handler.h"
#include "modem_iface_uart.h"
#include "modem_socket.h"

/* ------------------------------------------------------------------ */
/* Hardware / UART macros                                               */
/* ------------------------------------------------------------------ */

#define MDM_UART_NODE    DT_INST_BUS(0)
#define MDM_UART_DEV     DEVICE_DT_GET(MDM_UART_NODE)

/* ------------------------------------------------------------------ */
/* Buffer / pool sizes                                                  */
/* ------------------------------------------------------------------ */

#define MDM_MAX_DATA_LENGTH  1024
#define MDM_RECV_BUF_SIZE    1024
#define MDM_RECV_MAX_BUF     30

/*
 * The SIM7022 supports a maximum of 2 simultaneous socket connections
 * (link_num 0-1).  This is hard-wired in the module firmware.
 */
#define MDM_MAX_SOCKETS      2
#define MDM_BASE_SOCKET_NUM  0

/* ------------------------------------------------------------------ */
/* Timing constants                                                     */
/* ------------------------------------------------------------------ */

#define BUF_ALLOC_TIMEOUT          K_SECONDS(1)
#define MDM_CMD_TIMEOUT            K_SECONDS(10)
#define MDM_REGISTRATION_TIMEOUT   K_SECONDS(180)
#define MDM_CONNECT_TIMEOUT        K_SECONDS(90)
#define MDM_PDP_TIMEOUT            K_SECONDS(120)
#define MDM_DNS_TIMEOUT            K_SECONDS(210)
#define MDM_WAIT_FOR_RSSI_DELAY    K_SECONDS(2)
#define MDM_WAIT_FOR_RSSI_COUNT    30
#define MDM_MAX_AUTOBAUD           5
#define MDM_MAX_CEREG_WAITS        40
#define MDM_MAX_CGATT_WAITS        40
#define MDM_BOOT_TRIES             2
#define RSSI_TIMEOUT_SECS          30

/* ------------------------------------------------------------------ */
/* Config shortcuts                                                     */
/* ------------------------------------------------------------------ */

#define MDM_APN       CONFIG_MODEM_SIMCOM_SIM7022_APN

/* ------------------------------------------------------------------ */
/* String buffer lengths                                                */
/* ------------------------------------------------------------------ */

#define MDM_MANUFACTURER_LENGTH  12
#define MDM_MODEL_LENGTH         16
#define MDM_REVISION_LENGTH      64
#define MDM_IMEI_LENGTH          16
#define MDM_IMSI_LENGTH          16
#define MDM_ICCID_LENGTH         32

/* ------------------------------------------------------------------ */
/* Status flags (bit field stored in status_flags)                     */
/* ------------------------------------------------------------------ */

enum sim7022_status_flags {
	/** Modem has sent "RDY" — it is powered on. */
	SIM7022_STATUS_FLAG_POWER_ON   = 0x01,
	/** +CPIN: READY has been received — SIM card is usable. */
	SIM7022_STATUS_FLAG_CPIN_READY = 0x02,
	/** +CGATT: 1 — modem is attached to GPRS/LTE. */
	SIM7022_STATUS_FLAG_ATTACHED   = 0x04,
	/** +NETOPEN: 0 — PDP context is active and socket service open. */
	SIM7022_STATUS_FLAG_NET_OPEN   = 0x08,
};

/* ------------------------------------------------------------------ */
/* Driver data structure                                                */
/* ------------------------------------------------------------------ */

struct sim7022_data {
	/* ------- Zephyr network interface ------- */
	struct net_if *netif;
	uint8_t mac_addr[6];

	/* ------- UART interface ------- */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* ------- AT command handler ------- */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];

	/* ------- Socket pool ------- */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* ------- Driver state machine ------- */
	enum sim7022_state state;

	/* ------- Periodic RSSI query ------- */
	struct k_work_delayable rssi_query_work;

	/* ------- Modem identification strings ------- */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	char mdm_imsi[MDM_IMSI_LENGTH];
	char mdm_iccid[MDM_ICCID_LENGTH];
#endif
	int mdm_rssi;

	/* ------- Active socket tracking ------- */
	int current_sock_fd;
	int current_sock_written;
	/** Result code of the last AT+CIPOPEN command (0 = success). */
	uint8_t socket_open_rc;

	/* ------- Network registration ------- */
	uint8_t mdm_registration;

	/* ------- Status bit flags ------- */
	uint32_t status_flags;

	/* ------- DNS retry/timeout config ------- */
	struct {
		uint8_t  recount;
		uint16_t timeout;
	} dns;

	/* ------- Semaphores ------- */
	struct k_sem sem_response;   /* generic AT command response gate */
	struct k_sem sem_tx_ready;   /* '>' prompt received              */
	struct k_sem sem_send_ok;    /* SEND OK / SEND FAIL received     */
	struct k_sem sem_dns;        /* +CDNSGIP response received       */
	struct k_sem boot_sem;       /* RDY / POWER DOWN received        */
	struct k_sem pdp_sem;        /* +NETOPEN / +NETCLOSE received    */
	/*
	 * Given by on_urc_cipopen() when +CIPOPEN: <link>,<err> is received.
	 *
	 * For TCP the module sends:  OK\r\n+CIPOPEN: <link>,<err>\r\n
	 * For UDP the module sends:  +CIPOPEN: <link>,<err>\r\nOK\r\n
	 *
	 * Using a dedicated semaphore decouples the connection result from
	 * the synchronous OK/ERROR response and avoids a race condition
	 * where modem_cmd_send() could return before +CIPOPEN: is processed.
	 */
	struct k_sem sem_cipopen;    /* +CIPOPEN: result received        */
};

/* ------------------------------------------------------------------ */
/* Per-socket read context (passed through sock->data)                 */
/* ------------------------------------------------------------------ */

struct socket_read_data {
	char                 *recv_buf;
	size_t                recv_buf_len;
	struct sockaddr      *recv_addr;
	uint16_t              recv_read_len;
};

/* ------------------------------------------------------------------ */
/* Cross-file globals (defined in sim7022.c)                           */
/* ------------------------------------------------------------------ */

extern struct sim7022_data           mdata;
extern struct modem_context          mctx;
extern const struct socket_op_vtable offload_socket_fd_op_vtable;
extern const struct socket_dns_offload offload_dns_ops;
extern struct k_work_q               modem_workq;

/* ------------------------------------------------------------------ */
/* Internal function prototypes                                         */
/* ------------------------------------------------------------------ */

/* sim7022.c */
enum sim7022_state sim7022_get_state(void);
void               sim7022_change_state(enum sim7022_state state);

/* sim7022_pdp.c */
void sim7022_rssi_query_work(struct k_work *work);
int  sim7022_pdp_activate(void);
int  sim7022_pdp_deactivate(void);

/* sim7022_sock.c */
int  sim7022_offload_socket(int family, int type, int proto);
void sim7022_handle_sock_data_indication(int id);
void sim7022_handle_sock_state(int id, uint8_t state);

/* sim7022_utils.c */
int sim7022_utils_parse_time(uint8_t *date, uint8_t *time_str, struct tm *t);

#endif /* SIMCOM_SIM7022_H */
