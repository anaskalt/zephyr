/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API of the SIMCom Y7080E NB-IoT modem driver.
 *
 * Sockets and DNS go through the regular Zephyr socket API (offloaded to
 * the module). This header covers power, network attach, PSM sleep/wake
 * and status queries.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_Y7080E_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_Y7080E_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Driver state. */
enum y7080e_state {
	/** Unpowered. */
	Y7080E_STATE_OFF = 0,
	/** Powered, identified and configured, not attached. */
	Y7080E_STATE_IDLE,
	/** Registered with an active PDN: sockets usable. */
	Y7080E_STATE_NETWORKING,
	/** In deep sleep (PSM) after mdm_y7080e_sleep(); UART closed. */
	Y7080E_STATE_SLEEPING,
};

/** Radio statistics from AT+NUESTATS=RADIO. */
struct y7080e_radio_stats {
	/** Reference signal received power, dBm. */
	int rsrp_dbm;
	/** Received signal strength, dBm. */
	int rssi_dbm;
	/** Signal to noise ratio, dB. */
	int snr_db;
	/** Last transmit power, dBm. */
	int tx_power_dbm;
	/** Reference signal received quality, dB. */
	int rsrq_db;
	/** Coverage enhancement level 0..2, -1 unknown. */
	int ecl;
	/** Serving cell identity. */
	uint32_t cell_id;
	/** E-UTRA absolute radio frequency channel number. */
	uint32_t earfcn;
	/** Physical cell identity. */
	uint16_t pci;
	/** Serving band. */
	uint16_t band;
	/** Operator PLMN, e.g. "20201". */
	char plmn[8];
};

/**
 * @brief Power the module on, identify it and apply the configuration.
 *
 * Blocks for the boot (about two seconds) plus the AT exchanges. Safe to
 * call when the module is already on. Leaves the driver in IDLE.
 *
 * @return 0 on success, negative errno otherwise.
 */
int mdm_y7080e_power_on(void);

/**
 * @brief Shut the module down and remove its supply.
 *
 * Runs AT+FASTOFF, waits for the module to report +POWERDOWN, then opens
 * the load switch. Leaves the driver in OFF.
 */
int mdm_y7080e_power_off(void);

/**
 * @brief Wait until the module is registered and the PDN is active.
 *
 * Enables the radio (AT+CFUN=1) when needed and polls the registration
 * for up to CONFIG_MODEM_SIMCOM_Y7080E_ATTACH_TIMEOUT_S. Requests PSM when
 * CONFIG_MODEM_SIMCOM_Y7080E_PSM is set. Leaves the driver in NETWORKING.
 *
 * @return 0 on success, -ETIMEDOUT when the network never answered,
 *         negative errno otherwise.
 */
int mdm_y7080e_start_network(void);

/**
 * @brief Let the module drop into PSM and close the UART.
 *
 * Releases the module work lock (AT+WORKLOCK=0) and waits for
 * +POWERDOWN for up to CONFIG_MODEM_SIMCOM_Y7080E_SLEEP_WAIT_MS. Without
 * a network PSM grant the module never reports +POWERDOWN and the call
 * returns -ETIMEDOUT with the UART still closed: the module then idles at
 * a few hundred microamps until the next wake.
 *
 * @return 0 when the module entered deep sleep, negative errno otherwise.
 */
int mdm_y7080e_sleep(void);

/**
 * @brief Wake the module from PSM.
 *
 * Reopens the UART, pulses RESET (short pulse = wake), waits for the
 * module to answer and re-takes the work lock. Verifies the registration
 * and leaves the driver in NETWORKING or IDLE accordingly.
 *
 * @return 0 when the module answers, negative errno otherwise.
 */
int mdm_y7080e_wake(void);

/**
 * @brief Power cycle the module and run the boot sequence again.
 */
int mdm_y7080e_force_reset(void);

/** @brief Current driver state. */
enum y7080e_state mdm_y7080e_get_state(void);

/** @brief Manufacturer string (AT+CGMI), valid after power on. */
const char *mdm_y7080e_get_manufacturer(void);
/** @brief Model string (AT+CGMM). */
const char *mdm_y7080e_get_model(void);
/** @brief Firmware revision (AT+CGMR). */
const char *mdm_y7080e_get_revision(void);
/** @brief IMEI (AT+CGSN=1). */
const char *mdm_y7080e_get_imei(void);
/** @brief IMSI (AT+CIMI), empty when no SIM. */
const char *mdm_y7080e_get_imsi(void);
/** @brief SIM ICCID (AT+NCCID), empty when no SIM. */
const char *mdm_y7080e_get_iccid(void);

/**
 * @brief Last EPS registration status (+CEREG <stat>).
 *
 * 0 not registered, 1 home, 2 searching, 3 denied, 4 unknown, 5 roaming.
 */
uint8_t mdm_y7080e_get_registration(void);

/**
 * @brief Network-granted PSM timers decoded from +CEREG.
 *
 * @param active_time_sec Out, may be NULL. -1 when deactivated.
 * @param tau_sec Out, may be NULL. -1 when deactivated.
 * @return 0 on success, -EAGAIN when no grant was seen yet.
 */
int mdm_y7080e_get_psm_timers(int *active_time_sec, int *tau_sec);

/**
 * @brief Query the signal strength (AT+CSQ).
 *
 * @param rssi_dbm Out: RSSI in dBm, Y7080E_RSSI_UNKNOWN when not known.
 * @return 0 on success, negative errno otherwise.
 */
int mdm_y7080e_get_rssi(int *rssi_dbm);

/** @brief Query the radio statistics (AT+NUESTATS=RADIO). */
int mdm_y7080e_get_radio_stats(struct y7080e_radio_stats *stats);

/**
 * @brief Copy the PDP address of the default context (AT+CGPADDR=0).
 *
 * @return 0 on success, -ENODATA when no address is assigned.
 */
int mdm_y7080e_get_ip(char *buf, size_t len);

/**
 * @brief Read the module clock (AT+CCLK?), UTC.
 *
 * @return 0 on success, negative errno otherwise.
 */
int mdm_y7080e_get_time(struct tm *t);

/**
 * @brief Module supply voltage as measured by the module (AT+VBAT?).
 */
int mdm_y7080e_get_vbat_mv(int *mv);

/**
 * @brief Ask the network to release the RRC connection now (AT+CNMPSD).
 */
int mdm_y7080e_release_rrc(void);

/**
 * @brief Block or allow the SoC low-power states while a multi-command
 *        modem transaction is in flight.
 *
 * The driver takes the lock around each of its own AT exchanges; use
 * these to bracket a sequence that must not be interrupted by STOP2.
 */
void mdm_y7080e_pm_lock(void);
void mdm_y7080e_pm_unlock(void);

/** Unknown RSSI value returned by mdm_y7080e_get_rssi(). */
#define Y7080E_RSSI_UNKNOWN (-1000)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_Y7080E_H_ */
