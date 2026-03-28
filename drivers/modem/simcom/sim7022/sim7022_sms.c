/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SIM7022 SMS support
 * ===================
 * The SIM7022 does NOT implement the standard 3GPP TS 27.005 SMS AT commands
 * (AT+CMGF, AT+CMGL, AT+CMGR, AT+CMGD, AT+CMGS).
 *
 * The only SMS capability available is AT+QCSMSSEND, which sends a single SMS
 * in TEXT mode (not PDU mode).  There is no support for reading, listing or
 * deleting stored SMS messages from the SIM card.
 *
 * Reference: SIM7022 Series AT Command Manual V1.05, §4.2.11
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_sms, CONFIG_MODEM_LOG_LEVEL);

#include "sim7022.h"

/* ------------------------------------------------------------------ */
/* AT+QCSMSSEND — send one SMS in text mode                           */
/* ------------------------------------------------------------------ */

/*
 * Send a single SMS message.
 *
 * Format (SIM7022 AT Command Manual §4.2.11):
 *   AT+QCSMSSEND=<mode>,<da>[,<toda>,<test_sms>]
 *
 *   <mode>     = 1 (TEXT mode — only supported mode)
 *   <da>       = destination address (phone number string)
 *   <toda>     = type of destination address:
 *                  129 = national numbering plan
 *                  145 = international (+country code) numbering plan
 *   <test_sms> = message content (ASCII text)
 *
 * Response: OK  or  +CME ERROR: <err>
 *
 * @param number  Destination phone number (null-terminated).
 *                Use international format (+...) or national digits.
 * @param message Message text (null-terminated ASCII, max ~160 chars).
 * @return 0 on success, negative errno on failure.
 */
int mdm_sim7022_send_sms(const char *number, const char *message)
{
	/*
	 * Buffer: "AT+QCSMSSEND=1," (15) + '"' (1) + number (20 max) + '"' (1)
	 *         + ",145," (5) + '"' (1) + message (160 max) + '"' (1) + NUL (1)
	 * = 15 + 1 + 20 + 1 + 5 + 1 + 160 + 1 + 1 = 205 bytes
	 */
	char buf[210];
	int ret;
	int toda;

	if (!number || !message) {
		return -EINVAL;
	}

	if (sim7022_get_state() == SIM7022_STATE_OFF) {
		LOG_ERR("SIM7022 is not powered on");
		return -ENODEV;
	}

	/*
	 * Select type-of-address based on number format:
	 *   145 = international (number starts with '+')
	 *   129 = national / unknown
	 */
	toda = (number[0] == '+') ? 145 : 129;

	ret = snprintk(buf, sizeof(buf),
		       "AT+QCSMSSEND=1,\"%s\",%d,\"%s\"",
		       number, toda, message);
	if (ret < 0 || ret >= (int)sizeof(buf)) {
		LOG_ERR("SMS command too long");
		return -ENOMEM;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U, buf,
			     &mdata.sem_response, K_SECONDS(30));
	if (ret < 0) {
		LOG_ERR("AT+QCSMSSEND failed: %d", ret);
	}

	return ret;
}
