/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_SIM7022_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_SIM7022_H_

#include <stdint.h>
#include <stddef.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Possible operating states of the SIM7022 modem.
 */
enum sim7022_state {
	/** Modem is powered off. */
	SIM7022_STATE_OFF = 0,
	/** Modem is powered on and idle (not connected to network). */
	SIM7022_STATE_IDLE,
	/** Modem has an active PDP context and can send/receive data. */
	SIM7022_STATE_NETWORKING,
};

/*
 * SMS capability note:
 * The SIM7022 does NOT support standard 3GPP SMS read/list/delete commands
 * (AT+CMGL, AT+CMGR, AT+CMGD, AT+CMGF).  Only SMS send is available via
 * the proprietary AT+QCSMSSEND command (TEXT mode only).
 * See AT Command Manual §4.2.11.
 *
 * Battery charge note:
 * The SIM7022 does NOT support AT+CBC.  No battery voltage or charge level
 * API is provided.
 */

/**
 * @brief UE system mode as reported by AT+CPSI.
 *
 * The SIM7022 reports "NB" for NB-IoT and "LTE" for LTE bearers.
 * Both map to SIM7022_UE_SYS_MODE_LTE_NB_IOT since the hardware is
 * NB-IoT only (AT Command Manual §3.2.55).
 */
enum sim7022_ue_sys_mode {
	SIM7022_UE_SYS_MODE_NO_SERVICE = 0,
	SIM7022_UE_SYS_MODE_LTE_NB_IOT,
};

/**
 * @brief UE operation mode as reported by AT+CPSI.
 *
 * Matches the strings from AT Command Manual §3.2.55:
 *   "Unknown", "Online", "Offline", "Factory Test Mode",
 *   "Reset", "Low Power Mode", "Flight Mode"
 */
enum sim7022_ue_op_mode {
	SIM7022_UE_OP_MODE_ONLINE = 0,
	SIM7022_UE_OP_MODE_OFFLINE,
	SIM7022_UE_OP_MODE_FACTORY_TEST,
	SIM7022_UE_OP_MODE_RESET,
	SIM7022_UE_OP_MODE_LOW_POWER,
};

/**
 * @brief LTE/NB-IoT radio measurements from AT+CPSI.
 *
 * Signal quality fields are in the raw units returned by the modem
 * (integers, tenths of dB/dBm for RSRQ/RSRP/RSSI/RSSNR).
 * Reference: AT Command Manual §3.2.55.
 */
struct sim7022_lte_info {
	/** Tracking Area Code (hex string, e.g. "0x230A"). */
	char tac[8];
	/** Serving Cell ID (decimal string). */
	char sci[12];
	/** Physical Cell ID (PCellID). */
	uint16_t pci;
	/**
	 * LTE band number extracted from the "EUTRAN-BANDx" field
	 * (e.g. 3 from "EUTRAN-BAND3").
	 */
	uint8_t band;
	/** E-UTRA Absolute Radio Frequency Channel Number. */
	uint32_t earfcn;
	/** DL bandwidth configuration code (0-6). */
	uint16_t dl_bw;
	/** UL bandwidth configuration code (0-6). */
	uint16_t ul_bw;
	/** RSRQ (raw modem value, tenths of dB). */
	int16_t rsrq;
	/** RSRP (raw modem value, tenths of dBm). */
	int16_t rsrp;
	/** RSSI (raw modem value, tenths of dBm). */
	int16_t rssi;
	/** RS-SNR (raw modem value, tenths of dB). */
	int16_t rssnr;
};

/**
 * @brief UE system information from AT+CPSI.
 */
struct sim7022_ue_sys_info {
	/** System mode. */
	enum sim7022_ue_sys_mode sys_mode;
	/** Operational mode. */
	enum sim7022_ue_op_mode op_mode;
	/** MCC-MNC string as returned by modem (e.g. "460-01"). */
	char mcc_mnc[8];
	/** Radio measurements (valid when sys_mode is LTE_NB_IOT). */
	struct sim7022_lte_info lte;
};

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/** @brief Get the current operating state of the modem. */
enum sim7022_state mdm_sim7022_get_state(void);

/**
 * @brief Power on the modem (toggles PWRKEY, waits for RDY URC).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_power_on(void);

/**
 * @brief Power off the modem gracefully (waits for NORMAL POWER DOWN URC).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_power_off(void);

/** @brief Forcefully reset the modem by holding PWRKEY for 15 seconds. */
void mdm_sim7022_force_reset(void);

/**
 * @brief Bring up the NB-IoT data connection (AT+NETOPEN).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_start_network(void);

/**
 * @brief Tear down the NB-IoT data connection (AT+NETCLOSE).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_stop_network(void);

/**
 * @brief Send one SMS in text mode (AT+QCSMSSEND).
 *
 * The SIM7022 only supports sending SMS; reading and deleting are not
 * available (no AT+CMGL/CMGR/CMGD/CMGF support).
 *
 * @param number  Destination number (null-terminated, E.164 format).
 * @param message Message text (null-terminated ASCII, max ~160 chars).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_send_sms(const char *number, const char *message);

/**
 * @brief Query UE system information (AT+CPSI).
 * @param info Output structure.
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_get_ue_sys_info(struct sim7022_ue_sys_info *info);

/**
 * @brief Query the modem real-time clock (AT+CCLK?).
 *
 * The year field in struct tm is years-since-1900 (standard C convention).
 * The SIM7022 provides a 4-digit year.
 *
 * @param t Output struct tm.
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_get_local_time(struct tm *t);

/** @brief Return the manufacturer identification string (AT+CGMI). */
const char *mdm_sim7022_get_manufacturer(void);

/** @brief Return the model identification string (AT+CGMM). */
const char *mdm_sim7022_get_model(void);

/** @brief Return the firmware revision string (AT+CGMR). */
const char *mdm_sim7022_get_revision(void);

/** @brief Return the IMEI string (AT+CGSN). */
const char *mdm_sim7022_get_imei(void);

#if defined(CONFIG_MODEM_SIM_NUMBERS)
/** @brief Return the ICCID string (AT+CICCID, requires CONFIG_MODEM_SIM_NUMBERS). */
const char *mdm_sim7022_get_iccid(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_SIM7022_H_ */
