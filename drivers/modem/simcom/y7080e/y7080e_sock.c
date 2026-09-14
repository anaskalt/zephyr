/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Y7080E socket offload.
 *
 *   AT+NSOCR=<type>,<proto>,<lport>,1     -> +NSOCR:<id>
 *   AT+NSOCO=<id>,<ip>,<port>             (TCP connect)
 *   AT+NSOSTF=<id>,<ip>,<port>,<flag>,<len>,<hex>  -> +NSOSTF:<id>,<len>
 *   AT+NSOSD=<id>,<len>,<hex>[,<flag>]    -> <id>,<len>
 *   +NSONMI:<id>,<len>                    (data pending)
 *   AT+NSORF=<id>,<len>                   -> +NSORF:<id>,<ip>,<port>,<len>,<hex>,<rem>
 *   AT+NSOCL=<id>                         (+NSOCLI:<id>)
 *
 * The module owns the link ids, so the socket pool runs with
 * assign_id = false and the id is bound after +NSOCR.
 */

#define DT_DRV_COMPAT simcom_y7080e

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/sys/util.h>

#include "sockets_internal.h"
#include "y7080e.h"

LOG_MODULE_REGISTER(modem_simcom_y7080e_sock, CONFIG_MODEM_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static const char *skip_spaces(const char *s)
{
	while (*s == ' ') {
		s++;
	}
	return s;
}

static int addr_to_ip_port(const struct net_sockaddr *addr, char *ip, size_t ip_len,
			   uint16_t *port)
{
	if (addr->sa_family != NET_AF_INET) {
		return -EAFNOSUPPORT;
	}
	if (net_addr_ntop(NET_AF_INET, &net_sin(addr)->sin_addr, ip, ip_len) == NULL) {
		return -EINVAL;
	}
	*port = net_ntohs(net_sin(addr)->sin_port);

	return 0;
}

static uint16_t pending_total(struct modem_socket *sock)
{
	uint32_t total = 0;

	for (uint16_t i = 0; i < sock->packet_count; i++) {
		total += sock->packet_sizes[i];
	}

	return (uint16_t)MIN(total, UINT16_MAX);
}

/* Reset the module-side identity of a socket (the fd stays valid). */
static void sock_forget_link(struct modem_socket *sock)
{
	sock->is_connected = false;
	sock->id = MDM_BASE_SOCKET_NUM + MDM_MAX_SOCKETS;
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock, 0);
	modem_socket_data_ready(&mdata.socket_config, sock);
}

void y7080e_sock_invalidate_all(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(mdata.sockets); i++) {
		struct modem_socket *sock = &mdata.sockets[i];

		if (modem_socket_is_allocated(&mdata.socket_config, sock)) {
			sock_forget_link(sock);
		}
	}
}

/* ------------------------------------------------------------------ */
/* URC hooks (called from the chat work queue)                          */
/* ------------------------------------------------------------------ */

void y7080e_sock_on_nsonmi(int id, int len)
{
	struct modem_socket *sock = modem_socket_from_id(&mdata.socket_config, id);

	if (sock == NULL || len <= 0) {
		return;
	}

	(void)modem_socket_packet_size_update(&mdata.socket_config, sock,
					      pending_total(sock) + len);
	modem_socket_data_ready(&mdata.socket_config, sock);
	LOG_DBG("socket %d: %d bytes pending", id, len);
}

void y7080e_sock_on_nsocli(int id)
{
	struct modem_socket *sock = modem_socket_from_id(&mdata.socket_config, id);

	if (sock == NULL) {
		return;
	}

	LOG_INF("socket %d closed by the module", id);
	sock_forget_link(sock);
}

void y7080e_sock_on_nsorf(char **argv, uint16_t argc)
{
	/* argv: [0]"+NSORF:" [1]id [2]ip [3]port [4]len [5]hex [6]remaining */
	int len;
	size_t hex_len;
	size_t decoded;

	if (argc < 7 || mdata.nsorf_dst == NULL) {
		return;
	}

	len = (int)strtol(skip_spaces(argv[4]), NULL, 10);
	hex_len = strlen(skip_spaces(argv[5]));
	if (len < 0 || (size_t)len > mdata.nsorf_max || hex_len != (size_t)len * 2U) {
		LOG_WRN("+NSORF length mismatch (%d bytes, %u hex chars)", len,
			(unsigned int)hex_len);
		mdata.nsorf_len = -EBADMSG;
		return;
	}

	decoded = hex2bin(skip_spaces(argv[5]), hex_len, mdata.nsorf_dst, mdata.nsorf_max);
	if (decoded != (size_t)len) {
		mdata.nsorf_len = -EBADMSG;
		return;
	}

	strncpy(mdata.nsorf_ip, skip_spaces(argv[2]), sizeof(mdata.nsorf_ip) - 1);
	mdata.nsorf_ip[sizeof(mdata.nsorf_ip) - 1] = '\0';
	mdata.nsorf_port = (uint16_t)strtoul(skip_spaces(argv[3]), NULL, 10);
	mdata.nsorf_remaining = (uint16_t)strtoul(skip_spaces(argv[6]), NULL, 10);
	mdata.nsorf_len = len;
}

/* ------------------------------------------------------------------ */
/* Link creation                                                        */
/* ------------------------------------------------------------------ */

/* Create the module socket for @p sock. Caller holds at_lock. */
static int sock_ensure_link(struct modem_socket *sock)
{
	uint16_t lport = 0;
	int ret;

	if (modem_socket_id_is_assigned(&mdata.socket_config, sock)) {
		return 0;
	}

	if (y7080e_get_state() != Y7080E_STATE_NETWORKING) {
		return -ENETDOWN;
	}

	if (sock->src.sa_family == NET_AF_INET) {
		lport = net_ntohs(net_sin(&sock->src)->sin_port);
	}

	mdata.nsocr_id = -1;
	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+NSOCR=%s,%d,%u,1",
			 sock->type == NET_SOCK_STREAM ? "STREAM" : "DGRAM",
			 sock->type == NET_SOCK_STREAM ? 6 : 17, lport);
	if (ret != 0) {
		LOG_ERR("AT+NSOCR failed (%d)", ret);
		return ret == -EAGAIN ? -EIO : ret;
	}
	if (mdata.nsocr_id < 0) {
		LOG_ERR("AT+NSOCR gave no link id");
		return -EIO;
	}

	ret = modem_socket_id_assign(&mdata.socket_config, sock, mdata.nsocr_id);
	if (ret < 0) {
		LOG_ERR("link id %d unusable (%d)", mdata.nsocr_id, ret);
		(void)y7080e_cmd_tolerant(MDM_CMD_TIMEOUT_S, "AT+NSOCL=%d", mdata.nsocr_id);
		return ret;
	}

	LOG_DBG("socket fd %d -> link %d", sock->sock_fd, sock->id);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Socket operations                                                    */
/* ------------------------------------------------------------------ */

int y7080e_offload_socket(int family, int type, int proto)
{
	int fd;

	fd = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (fd < 0) {
		errno = -fd;
		return -1;
	}

	errno = 0;
	return fd;
}

static int offload_bind(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	struct modem_socket *sock = obj;

	if (addr->sa_family != NET_AF_INET) {
		errno = EAFNOSUPPORT;
		return -1;
	}

	memcpy(&sock->src, addr, MIN(addrlen, sizeof(sock->src)));
	errno = 0;

	return 0;
}

static int offload_connect(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	struct modem_socket *sock = obj;
	char ip[NET_IPV4_ADDR_LEN];
	uint16_t port;
	int ret;

	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		errno = EINVAL;
		return -1;
	}
	if (sock->is_connected) {
		errno = EISCONN;
		return -1;
	}

	ret = addr_to_ip_port(addr, ip, sizeof(ip), &port);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	memcpy(&sock->dst, addr, MIN(addrlen, sizeof(sock->dst)));

	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	ret = sock_ensure_link(sock);
	if (ret < 0) {
		goto out;
	}

	if (sock->type == NET_SOCK_STREAM) {
		ret = y7080e_cmd(NULL, 0, MDM_CONNECT_TIMEOUT_S, "AT+NSOCO=%d,%s,%u", sock->id, ip,
				 port);
		if (ret != 0) {
			LOG_ERR("AT+NSOCO failed (%d)", ret);
			ret = (ret == -EAGAIN) ? -ECONNREFUSED : ret;
			goto out;
		}
	}

	sock->is_connected = true;
	ret = 0;

out:
	k_mutex_unlock(&mdata.at_lock);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return 0;
}

static ssize_t offload_sendto(void *obj, const void *buf, size_t len, int flags,
			      const struct net_sockaddr *dest_addr, net_socklen_t addrlen)
{
	struct modem_socket *sock = obj;
	const struct net_sockaddr *dst;
	char ip[NET_IPV4_ADDR_LEN];
	uint16_t port = 0;
	int n;
	int ret;

	ARG_UNUSED(flags);
	ARG_UNUSED(addrlen);

	if (buf == NULL || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (len > MDM_MAX_DATA_LENGTH) {
		len = MDM_MAX_DATA_LENGTH;
	}

	if (sock->type == NET_SOCK_DGRAM) {
		dst = (dest_addr != NULL) ? dest_addr : &sock->dst;
		if (dst->sa_family != NET_AF_INET) {
			errno = EDESTADDRREQ;
			return -1;
		}
		ret = addr_to_ip_port(dst, ip, sizeof(ip), &port);
		if (ret < 0) {
			errno = -ret;
			return -1;
		}
	} else if (!sock->is_connected) {
		errno = ENOTCONN;
		return -1;
	}

	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	ret = sock_ensure_link(sock);
	if (ret < 0) {
		goto out;
	}

	if (sock->type == NET_SOCK_DGRAM) {
		n = snprintk(mdata.cmd_buf, sizeof(mdata.cmd_buf), "AT+NSOSTF=%d,%s,%u,0x%X,%u,",
			     sock->id, ip, port, CONFIG_MODEM_SIMCOM_Y7080E_UDP_SEND_FLAG,
			     (unsigned int)len);
	} else {
		n = snprintk(mdata.cmd_buf, sizeof(mdata.cmd_buf), "AT+NSOSD=%d,%u,", sock->id,
			     (unsigned int)len);
	}
	if (n < 0 || (size_t)n + 2 * len + 8 >= sizeof(mdata.cmd_buf)) {
		ret = -EMSGSIZE;
		goto out;
	}

	if (bin2hex(buf, len, &mdata.cmd_buf[n], sizeof(mdata.cmd_buf) - n) != 2 * len) {
		ret = -EMSGSIZE;
		goto out;
	}
	/* The manual shows upper-case hex; make sure the module gets it. */
	for (char *p = &mdata.cmd_buf[n]; *p != '\0'; p++) {
		if (*p >= 'a' && *p <= 'f') {
			*p = (char)(*p - 'a' + 'A');
		}
	}

	if (sock->type == NET_SOCK_DGRAM) {
		mdata.nsost_id = -1;
		mdata.nsost_len = -1;
		ret = y7080e_cmd_run_buf(NULL, 0, MDM_CMD_TIMEOUT_S);
		if (ret == 0 && (mdata.nsost_id != sock->id || mdata.nsost_len != (int)len)) {
			LOG_WRN("AT+NSOSTF confirmed %d/%u bytes", mdata.nsost_len,
				(unsigned int)len);
			ret = -EIO;
		}
	} else {
		size_t used = strlen(mdata.cmd_buf);

		snprintk(&mdata.cmd_buf[used], sizeof(mdata.cmd_buf) - used, ",0x%X",
			 CONFIG_MODEM_SIMCOM_Y7080E_UDP_SEND_FLAG);
		mdata.capture[0] = '\0';
		y7080e_flag_set(Y7080E_FLAG_CAPTURE);
		ret = y7080e_cmd_run_buf(NULL, 0, MDM_CMD_TIMEOUT_S);
		y7080e_flag_clear(Y7080E_FLAG_CAPTURE);
		if (ret == 0) {
			int rid = -1, rlen = -1;

			if (sscanf(mdata.capture, "%d,%d", &rid, &rlen) != 2 || rid != sock->id ||
			    rlen != (int)len) {
				LOG_WRN("AT+NSOSD confirmed '%s'", mdata.capture);
				ret = -EIO;
			}
		}
	}

	if (ret == -EAGAIN) {
		ret = -EIO;
	}

out:
	k_mutex_unlock(&mdata.at_lock);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return (ssize_t)len;
}

static ssize_t offload_recvfrom(void *obj, void *buf, size_t max_len, int flags,
				struct net_sockaddr *src_addr, net_socklen_t *addrlen)
{
	struct modem_socket *sock = obj;
	int pending;
	int received;
	int ret;

	if (buf == NULL || max_len == 0) {
		errno = EINVAL;
		return -1;
	}
	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	pending = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (pending == 0) {
		if (!modem_socket_id_is_assigned(&mdata.socket_config, sock)) {
			errno = ENOTCONN;
			return -1;
		}
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}
		modem_socket_wait_data(&mdata.socket_config, sock);
		pending = modem_socket_next_packet_size(&mdata.socket_config, sock);
		if (pending == 0) {
			/* Woken by a close indication. */
			errno = 0;
			return 0;
		}
	}

	max_len = MIN(max_len, MDM_MAX_DATA_LENGTH);

	k_mutex_lock(&mdata.at_lock, K_FOREVER);

	mdata.nsorf_dst = buf;
	mdata.nsorf_max = max_len;
	mdata.nsorf_len = -ENODATA;
	mdata.nsorf_remaining = 0;

	ret = y7080e_cmd(NULL, 0, MDM_CMD_TIMEOUT_S, "AT+NSORF=%d,%u", sock->id,
			 (unsigned int)max_len);

	mdata.nsorf_dst = NULL;

	if (ret == 0) {
		ret = mdata.nsorf_len;
	} else if (ret == -EAGAIN) {
		ret = -EIO;
	}

	if (ret >= 0) {
		received = ret;
		/* The module reports what is left of the current node. */
		(void)modem_socket_packet_size_update(&mdata.socket_config, sock,
						      mdata.nsorf_remaining);
		if (src_addr != NULL && addrlen != NULL) {
			struct net_sockaddr_in from = {
				.sin_family = NET_AF_INET,
				.sin_port = net_htons(mdata.nsorf_port),
			};

			(void)net_addr_pton(NET_AF_INET, mdata.nsorf_ip, &from.sin_addr);
			*addrlen = MIN(*addrlen, sizeof(from));
			memcpy(src_addr, &from, *addrlen);
		}
	} else {
		received = 0;
		/* Drop the indication so a broken read does not spin. */
		(void)modem_socket_packet_size_update(&mdata.socket_config, sock, 0);
	}

	k_mutex_unlock(&mdata.at_lock);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return (ssize_t)received;
}

static ssize_t offload_sendmsg(void *obj, const struct net_msghdr *msg, int flags)
{
	struct modem_socket *sock = obj;
	ssize_t sent = 0;

	if (sock->type == NET_SOCK_DGRAM) {
		size_t total = 0;

		for (size_t i = 0; i < msg->msg_iovlen; i++) {
			total += msg->msg_iov[i].iov_len;
		}
		if (msghdr_non_empty_iov_count(msg) > 1 || total > MDM_MAX_DATA_LENGTH) {
			errno = EMSGSIZE;
			return -1;
		}
	}

	for (size_t i = 0; i < msg->msg_iovlen; i++) {
		const uint8_t *p = msg->msg_iov[i].iov_base;
		size_t l = msg->msg_iov[i].iov_len;

		while (l > 0) {
			ssize_t r = offload_sendto(obj, p, l, flags, msg->msg_name, msg->msg_namelen);

			if (r < 0) {
				return r;
			}
			sent += r;
			p += r;
			l -= (size_t)r;
		}
	}

	return sent;
}

static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, NULL);
}

static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static int offload_close(void *obj)
{
	struct modem_socket *sock = obj;

	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		return 0;
	}

	if (modem_socket_id_is_assigned(&mdata.socket_config, sock) &&
	    y7080e_get_state() == Y7080E_STATE_NETWORKING) {
		k_mutex_lock(&mdata.at_lock, K_FOREVER);
		(void)y7080e_cmd_tolerant(5, "AT+NSOCL=%d", sock->id);
		k_mutex_unlock(&mdata.at_lock);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);

	return 0;
}

static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	struct modem_socket *sock = obj;

	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

		if (pfd->events & ZSOCK_POLLIN) {
			if (*pev == pev_end) {
				return -ENOMEM;
			}
			k_poll_event_init(*pev, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
					  &sock->sig_data_ready);
			(*pev)++;
		}
		if (pfd->events & ZSOCK_POLLOUT) {
			/* Always writable: make poll() return at once. */
			return -EALREADY;
		}
		return 0;
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);

		if (pfd->events & ZSOCK_POLLIN) {
			if ((*pev)->state != K_POLL_STATE_NOT_READY) {
				pfd->revents |= ZSOCK_POLLIN;
			}
			(*pev)++;
		}
		if (pfd->events & ZSOCK_POLLOUT) {
			pfd->revents |= ZSOCK_POLLOUT;
		}
		if (sock->type == NET_SOCK_STREAM && !sock->is_connected) {
			pfd->revents |= ZSOCK_POLLHUP;
		}
		return 0;
	}
	case ZFD_IOCTL_FIONREAD: {
		int *avail = va_arg(args, int *);

		*avail = modem_socket_next_packet_size(&mdata.socket_config, sock);
		return 0;
	}
	case ZVFS_F_GETFL:
	case ZVFS_F_SETFL:
		return 0;
	default:
		errno = EINVAL;
		return -1;
	}
}

const struct socket_op_vtable y7080e_socket_fd_op_vtable = {
	.fd_vtable = {
		.read = offload_read,
		.write = offload_write,
		.close = offload_close,
		.ioctl = offload_ioctl,
	},
	.shutdown = NULL,
	.bind = offload_bind,
	.connect = offload_connect,
	.listen = NULL,
	.accept = NULL,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.getsockopt = NULL,
	.setsockopt = NULL,
	.sendmsg = offload_sendmsg,
	.recvmsg = NULL,
	.getpeername = NULL,
	.getsockname = NULL,
};
