/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L   /* Required for strtok_r() */

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_utils, CONFIG_MODEM_LOG_LEVEL);

#include "sim7022.h"

/*
 * Parse date and time strings from the AT+CCLK? response into struct tm.
 *
 * The SIM7022 AT+CCLK format (§3.2.18):
 *   "yyyy/MM/dd,hh:mm:ss±zz"
 *
 * where:
 *   yyyy = 4-digit year (e.g. 2024)
 *   MM   = month 01-12
 *   dd   = day 01-31
 *   hh   = hour 00-23
 *   mm   = minute 00-59
 *   ss   = second 00-59
 *   ±zz  = timezone offset in quarters of an hour (-96..+96), optional
 *
 * Example: "2024/05/06,14:30:00+08"
 *
 * By the time this function is called the MODEM_CMD handler has already
 * stripped the outer quotes and split on the first comma, so:
 *   date     → "yyyy/MM/dd"  (null-terminated)
 *   time_str → "hh:mm:ss±zz" (null-terminated, ±zz optional)
 *
 * @param date     Pointer to date string "yyyy/MM/dd".
 * @param time_str Pointer to time string "hh:mm:ss[+/-]zz".
 * @param t        Output struct tm (filled in).
 * @return 0 on success, -EINVAL on bad input, -EBADMSG on parse error.
 */
int sim7022_utils_parse_time(uint8_t *date, uint8_t *time_str, struct tm *t)
{
	char *saveptr;
	char *tmp;

	if (!date || !time_str || !t) {
		return -EINVAL;
	}

	memset(t, 0, sizeof(*t));

	/* --- Parse date: yyyy/MM/dd --- */

	tmp = strtok_r((char *)date, "/", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse year");
		return -EBADMSG;
	}
	/*
	 * SIM7022 sends a 4-digit year (e.g. 2024).
	 * struct tm counts years from 1900, so subtract 1900.
	 */
	t->tm_year = (int)strtol(tmp, NULL, 10) - 1900;

	tmp = strtok_r(NULL, "/", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse month");
		return -EBADMSG;
	}
	t->tm_mon = (int)strtol(tmp, NULL, 10) - 1; /* struct tm: 0-based */

	tmp = strtok_r(NULL, "", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse day");
		return -EBADMSG;
	}
	t->tm_mday = (int)strtol(tmp, NULL, 10);

	/* --- Parse time: hh:mm:ss[+/-]zz --- */

	tmp = strtok_r((char *)time_str, ":", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse hour");
		return -EBADMSG;
	}
	t->tm_hour = (int)strtol(tmp, NULL, 10);

	tmp = strtok_r(NULL, ":", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse minute");
		return -EBADMSG;
	}
	t->tm_min = (int)strtol(tmp, NULL, 10);

	/*
	 * Seconds field is followed by the optional timezone offset
	 * (+ or - then decimal digits).  Split on '+' or '-'.
	 */
	tmp = strtok_r(NULL, "+-", &saveptr);
	if (!tmp) {
		LOG_WRN("utils: failed to parse second");
		return -EBADMSG;
	}
	t->tm_sec = (int)strtol(tmp, NULL, 10);

	/* DST status is not available from the module */
	t->tm_isdst = -1;

	return 0;
}
