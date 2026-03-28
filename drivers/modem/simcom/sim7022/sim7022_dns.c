/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_sim7022

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_dns, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/drivers/modem/simcom-sim7022.h>
#include "sim7022.h"

static struct zsock_addrinfo    dns_result;
static struct sockaddr      dns_result_addr;
static char dns_result_canonname[DNS_MAX_NAME_SIZE + 1];

/* ------------------------------------------------------------------ */
/* AT+CDNSGIP response handler                                         */
/* ------------------------------------------------------------------ */

/*
 * Response on success:
 *   +CDNSGIP: 1,"<domain>","<IPv4>"
 *
 * Response on failure:
 *   +CDNSGIP: 0,<dns_error_code>
 *
 * MODEM_CMD("+CDNSGIP: ", handler, 2U, ",") delivers:
 *   argv[0] = "1"                   (state flag)
 *   argv[1] = "\"<domain>\""        (domain name with surrounding quotes)
 *
 * The Zephyr modem_cmd_handler advances rx_buf past both consumed args
 * and their delimiting commas.  What remains in rx_buf is:
 *   "<IPv4>"                        (opening quote, IP, closing quote)
 *
 * We read from rx_buf with offset=1 to skip the opening '"', giving:
 *   <IPv4>"
 * then we null-terminate at the closing '"'.
 *
 * Example: +CDNSGIP: 1,"www.baidu.com","61.135.169.121"
 *   argv[0] = "1"
 *   argv[1] = "\"www.baidu.com\""
 *   rx_buf  = "\"61.135.169.121\""
 *   offset=1 → ips = "61.135.169.121\""
 *   after strstr/terminate: ips = "61.135.169.121"
 */
MODEM_CMD_DEFINE(on_cmd_cdnsgip)
{
	int state;
	char ips[NET_IPV4_ADDR_LEN + 2]; /* IP + closing '"' + NUL */
	size_t out_len;
	int ret = -1;

	state = atoi(argv[0]);
	if (state == 0) {
		LOG_ERR("DNS lookup failed (error code %s)", argv[1]);
		goto exit;
	}

	/*
	 * rx_buf = "\"<IPv4>\""
	 * offset=1 skips the opening '"', leaving "<IPv4>\""
	 */
	out_len = net_buf_linearize(ips, sizeof(ips) - 1,
				    data->rx_buf, 1, len);
	ips[out_len] = '\0';

	/* Null-terminate at the closing '"' */
	char *ipv4_end = strstr(ips, "\"");

	if (!ipv4_end) {
		LOG_ERR("Malformed DNS response — no closing quote");
		goto exit;
	}
	*ipv4_end = '\0';

	if (net_addr_pton(dns_result.ai_family, ips,
			  &((struct sockaddr_in *)
			    &dns_result_addr)->sin_addr) < 0) {
		LOG_ERR("DNS: invalid IP address '%s'", ips);
		goto exit;
	}

	LOG_INF("DNS resolved: %s", ips);
	ret = 0;

exit:
	k_sem_give(&mdata.sem_dns);
	return ret;
}

/* ------------------------------------------------------------------ */
/* getaddrinfo offload                                                  */
/* ------------------------------------------------------------------ */

static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints,
			       struct zsock_addrinfo **res)
{
	struct modem_cmd cmd[] = {
		MODEM_CMD("+CDNSGIP: ", on_cmd_cdnsgip, 2U, ",")
	};
	/* "AT+CDNSGIP=\"" + up to 254-char domain + "\"" */
	char sendbuf[sizeof("AT+CDNSGIP=\"\"") + 254];
	uint32_t port = 0;
	int ret;
	uint8_t retry = 0;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state for DNS");
		return DNS_EAI_AGAIN;
	}

	/* Initialise result structures */
	(void)memset(&dns_result,      0, sizeof(dns_result));
	(void)memset(&dns_result_addr, 0, sizeof(dns_result_addr));

	/* Currently only IPv4 is supported */
	dns_result.ai_family      = AF_INET;
	dns_result_addr.sa_family = AF_INET;
	dns_result.ai_addr        = &dns_result_addr;
	dns_result.ai_addrlen     = sizeof(dns_result_addr);
	dns_result.ai_canonname   = dns_result_canonname;
	dns_result_canonname[0]   = '\0';

	if (service) {
		port = atoi(service);
		if (port < 1 || port > USHRT_MAX) {
			return DNS_EAI_SERVICE;
		}
	}

	if (port > 0U) {
		net_sin(&dns_result_addr)->sin_port = htons(port);
	}

	/* If node is already a numeric IP address, skip the DNS query */
	if (net_addr_pton(dns_result.ai_family, node,
			  &((struct sockaddr_in *)
			    &dns_result_addr)->sin_addr) == 0) {
		*res = &dns_result;
		return 0;
	}

	/* Honour AI_NUMERICHOST — refuse if it fails above */
	if (hints && hints->ai_flags & AI_NUMERICHOST) {
		return DNS_EAI_NONAME;
	}

	snprintk(sendbuf, sizeof(sendbuf),
		 "AT+CDNSGIP=\"%s\"", node);

	/*
	 * Retry loop mirrors the SIM7080 approach:
	 * CONFIG_MODEM_SIMCOM_SIM7022_DNS_DEFAULT_RECOUNT retries with
	 * CONFIG_MODEM_SIMCOM_SIM7022_DNS_DEFAULT_TIMEOUT ms per attempt.
	 */
	do {
		k_sem_reset(&mdata.sem_dns);

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     cmd, ARRAY_SIZE(cmd),
				     sendbuf,
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("AT+CDNSGIP failed: %d", ret);
			continue;
		}

		ret = k_sem_take(&mdata.sem_dns,
				 K_MSEC(mdata.dns.timeout));
		if (ret == 0) {
			/* Success — check that an IP was actually stored */
			if (net_sin(&dns_result_addr)->sin_addr.s_addr != 0) {
				*res = &dns_result;
				return 0;
			}
		}

		LOG_WRN("DNS attempt %u/%u failed", retry + 1,
			mdata.dns.recount + 1);
	} while (++retry <= mdata.dns.recount);

	return DNS_EAI_AGAIN;
}

static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	/* Static buffer — nothing to free */
	ARG_UNUSED(res);
}

const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo  = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};
