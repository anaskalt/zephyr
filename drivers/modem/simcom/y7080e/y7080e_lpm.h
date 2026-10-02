/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure helpers for the 3GPP GPRS timer octets used by AT+CPSMS and the
 * granted-timer fields of +CEREG, plus the AT+CSQ conversion. No Zephyr
 * dependencies so they can be unit tested on native_sim.
 */

#ifndef ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_LPM_H_
#define ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_LPM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Sentinel returned by y7080e_csq_to_dbm() for an unknown RSSI. */
#define Y7080E_RSSI_UNKNOWN (-1000)

/** Granted-timer value for a timer field the network did not send. */
#define Y7080E_TIMER_ABSENT (-2)

/**
 * @brief Decode an 8-digit binary GPRS timer string to seconds.
 *
 * @param s      "00100001" with or without surrounding quotes.
 * @param is_tau true for T3412 (GPRS timer 3), false for T3324 (timer 2).
 * @return seconds, or -1 when deactivated, reserved or malformed.
 */
long y7080e_timer_decode(const char *s, bool is_tau);

/** @brief True when @p s is an 8-digit binary octet (any T3324 unit). */
bool y7080e_is_active_timer(const char *s);

/** @brief True when @p s is an 8-digit binary octet (any T3412 unit). */
bool y7080e_is_tau_timer(const char *s);

/**
 * @brief Tell the AT+CEREG? reply from the unsolicited +CEREG and get <stat>.
 *
 * Read form <n>,<stat>,<tac>,... and URC <stat>,<tac>,... differ in the
 * second field: a bare one- or two-digit <stat> in the read form, and
 * four hex digits, a quoted string or nothing in the URC. Firmware
 * 2212B07Y7080E sends <tac> unquoted, so quoting cannot decide it.
 *
 * @param argv     modem_chat argv, argv[0] is the match prefix.
 * @param argc     modem_chat argc.
 * @param data_idx Out: 2 for the URC, 3 for the read form.
 * @return <stat>, or -1 for a line that is neither.
 */
int y7080e_cereg_parse_stat(char **argv, uint16_t argc, int *data_idx);

/**
 * @brief Find and decode the granted timers in a +CEREG field array.
 *
 * @param argv     modem_chat argv, argv[0] is the match prefix.
 * @param argc     modem_chat argc.
 * @param data_idx Index of the first field after <stat>: 2 for the URC,
 *                 3 for the AT+CEREG? read form.
 * @param active_sec Out: granted T3324 in seconds, -1 when deactivated,
 *                   Y7080E_TIMER_ABSENT when the field was not sent.
 * @param tau_sec    Out: granted extended T3412 in seconds, -1 when
 *                   deactivated, Y7080E_TIMER_ABSENT when the network sent
 *                   none (its standard T3412 then applies).
 *
 * With no timer field at all the outputs are left untouched.
 *
 * @return 0 when a T3324 was granted (that is what PSM needs), -1 otherwise.
 */
int y7080e_cereg_parse_granted(char **argv, uint16_t argc, int data_idx, long *active_sec,
			       long *tau_sec);

/** @brief Convert an AT+CSQ <rssi> code to dBm. */
int y7080e_csq_to_dbm(int rssi);

#endif /* ZEPHYR_DRIVERS_MODEM_SIMCOM_Y7080E_LPM_H_ */
