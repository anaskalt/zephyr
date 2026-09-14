/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Y7080E DNS offload: AT+QDNS=0,<name> answers OK at once and reports
 * "+QDNS:<ip>" (or "+QDNS:QUERY_DNS_FAILED") later as a URC.
 */

#define DT_DRV_COMPAT simcom_y7080e

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/socket.h>

#include "y7080e.h"

LOG_MODULE_REGISTER(modem_simcom_y7080e_dns, CONFIG_MODEM_LOG_LEVEL);

static struct zsock_addrinfo dns_result;
static struct net_sockaddr dns_result_addr;
static char dns_result_canonname[DNS_MAX_NAME_SIZE + 1];

static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	uint32_t port = 0;
	int ret;

	if (node == NULL) {
		return DNS_EAI_NONAME;
	}

	memset(&dns_result, 0, sizeof(dns_result));
	memset(&dns_result_addr, 0, sizeof(dns_result_addr));
	dns_result.ai_family = NET_AF_INET;
	dns_result.ai_socktype = hints ? hints->ai_socktype : 0;
	dns_result.ai_protocol = hints ? hints->ai_protocol : 0;
	dns_result_addr.sa_family = NET_AF_INET;
	dns_result.ai_addr = &dns_result_addr;
	dns_result.ai_addrlen = sizeof(struct net_sockaddr_in);
	dns_result.ai_canonname = dns_result_canonname;
	dns_result_canonname[0] = '\0';

	if (service != NULL) {
		port = strtoul(service, NULL, 10);
		if (port < 1 || port > UINT16_MAX) {
			return DNS_EAI_SERVICE;
		}
		net_sin(&dns_result_addr)->sin_port = net_htons((uint16_t)port);
	}

	/* Numeric literal: no modem round trip. */
	if (net_addr_pton(NET_AF_INET, node, &net_sin(&dns_result_addr)->sin_addr) == 0) {
		*res = &dns_result;
		return 0;
	}

	if (hints != NULL && (hints->ai_flags & ZSOCK_AI_NUMERICHOST)) {
		return DNS_EAI_NONAME;
	}

	if (y7080e_get_state() != Y7080E_STATE_NETWORKING) {
		return DNS_EAI_AGAIN;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);
	y7080e_pm_lock();

	mdata.dns_ok = false;
	mdata.dns_ip[0] = '\0';
	k_sem_reset(&mdata.sem_dns);

	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+QDNS=0,%s", node);
	if (ret == 0) {
		ret = k_sem_take(&mdata.sem_dns, K_MSEC(CONFIG_MODEM_SIMCOM_Y7080E_DNS_TIMEOUT_MS));
	}

	y7080e_pm_unlock();
	k_mutex_unlock(&mdata.at_lock);

	if (ret != 0 || !mdata.dns_ok) {
		LOG_WRN("DNS lookup of %s failed (%d, %s)", node, ret, mdata.dns_ip);
		return DNS_EAI_AGAIN;
	}

	if (net_addr_pton(NET_AF_INET, mdata.dns_ip, &net_sin(&dns_result_addr)->sin_addr) < 0) {
		LOG_WRN("DNS answer '%s' is not an IPv4 address", mdata.dns_ip);
		return DNS_EAI_FAIL;
	}

	LOG_INF("%s -> %s", node, mdata.dns_ip);
	*res = &dns_result;

	return 0;
}

static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	ARG_UNUSED(res);
}

const struct socket_dns_offload y7080e_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};
