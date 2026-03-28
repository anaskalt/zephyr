/*
 * Copyright (c) 2025 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_sim7022

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_sim7022_sock, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/drivers/modem/simcom-sim7022.h>
#include "sim7022.h"

static void socket_close(struct modem_socket *sock);

/* ------------------------------------------------------------------ */
/* AT+CIPOPEN — establish a TCP or UDP connection                      */
/* ------------------------------------------------------------------ */

/*
 * NOTE: +CIPOPEN: is handled as a URC in sim7022.c (on_urc_cipopen),
 * not here as a cmd[] argument to modem_cmd_send.  This is because
 * the SIM7022 sends the response in different orders for TCP and UDP:
 *
 *   TCP success:  OK\r\n+CIPOPEN: <link>,0\r\n
 *   UDP success:  +CIPOPEN: <link>,0\r\nOK\r\n
 *   Error cases:  +CIPOPEN: <link>,<err>\r\nERROR\r\n
 *
 * offload_connect() therefore:
 *   1. Resets sem_cipopen
 *   2. Calls modem_cmd_send (waits for OK or ERROR)
 *   3. Waits for sem_cipopen (given by on_urc_cipopen regardless of order)
 *   4. Checks socket_open_rc
 */
static int offload_connect(void *obj, const struct sockaddr *addr,
			   socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	uint16_t dst_port = 0;
	const char *protocol;
	/*
	 * Largest possible command:
	 * AT+CIPOPEN=1,"UDP","xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxx.xxx.xxx.xxx",65535
	 */
	char buf[sizeof("AT+CIPOPEN=#,\"UDP\","
			"\"xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxx.xxx.xxx.xxx\","
			"#####")];
	char ip_str[NET_IPV6_ADDR_LEN];
	int ret;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		LOG_ERR("Invalid socket id %d from fd %d",
			sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (sock->is_connected) {
		LOG_ERR("Socket already connected! id=%d fd=%d",
			sock->id, sock->sock_fd);
		errno = EISCONN;
		return -1;
	}

	/* Extract destination port */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	/* Save destination address so offload_sendto can use it for UDP */
	memcpy(&sock->dst, addr,
	       (addrlen < sizeof(sock->dst)) ? addrlen : sizeof(sock->dst));

	protocol = (sock->type == SOCK_STREAM) ? "TCP" : "UDP";

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Failed to format IP address");
		errno = ENOMEM;
		return -1;
	}

	/*
	 * AT+CIPOPEN format (SIM7022):
	 *   TCP: AT+CIPOPEN=<link>,"TCP","<ip>",<port>
	 *   UDP: AT+CIPOPEN=<link>,"UDP","<ip>",<port>,<local_port>
	 *
	 * For UDP, the local_port parameter is REQUIRED — omitting it
	 * causes the modem to return +CIPOPEN: <link>,3 (wrong parameter).
	 * Use a fixed local port derived from the socket id.
	 */
	if (sock->type == SOCK_DGRAM) {
		/* UDP — include local port */
		uint16_t local_port = 5000 + sock->id;

		ret = snprintk(buf, sizeof(buf),
			       "AT+CIPOPEN=%d,\"%s\",\"%s\",%d,%u",
			       sock->id, protocol, ip_str, dst_port, local_port);
	} else {
		/* TCP — no local port needed */
		ret = snprintk(buf, sizeof(buf),
			       "AT+CIPOPEN=%d,\"%s\",\"%s\",%d",
			       sock->id, protocol, ip_str, dst_port);
	}
	if (ret < 0) {
		LOG_ERR("Failed to build CIPOPEN command");
		errno = ENOMEM;
		return -1;
	}
	LOG_DBG("Sending: %s", buf);

	/* Prepare for the +CIPOPEN: URC (handled by on_urc_cipopen) */
	mdata.socket_open_rc = 1;  /* pre-set to error */
	k_sem_reset(&mdata.sem_cipopen);

	/*
	 * Send AT+CIPOPEN and wait for OK (TCP) or ERROR.
	 * For UDP, +CIPOPEN: arrives before OK so sem_cipopen may already
	 * be given before this call returns — that is fine.
	 */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U, buf,
			     &mdata.sem_response, MDM_CONNECT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
		goto error;
	}

	/*
	 * Wait for the +CIPOPEN: URC result.
	 * For TCP this arrives AFTER OK so we must wait here.
	 * For UDP this arrived BEFORE OK, so sem_cipopen is already
	 * given and k_sem_take returns immediately.
	 */
	ret = k_sem_take(&mdata.sem_cipopen, MDM_CONNECT_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for +CIPOPEN result");
		ret = -ETIMEDOUT;
		goto error;
	}

	if (mdata.socket_open_rc != 0) {
		LOG_ERR("CIPOPEN failed with error: %u", mdata.socket_open_rc);
		ret = -ENOTCONN;
		goto error;
	}

	sock->is_connected = true;
	errno = 0;
	return 0;

error:
	errno = -ret;
	return -1;
}

/* ------------------------------------------------------------------ */
/* AT+CIPSEND — send data over a socket                               */
/* ------------------------------------------------------------------ */

/*
 * The SIM7022 send flow with fixed-length mode:
 *
 *   Host:   AT+CIPSEND=<link_num>,<length>\r\n
 *   Module: > (unterminated prompt, no CRLF)
 *   Host:   <exactly <length> bytes of data>
 *   Module: OK\r\n
 *           +CIPSEND: <link_num>,<reqLen>,<cnfLen>\r\n
 *
 * This is essentially the same ">" prompt mechanism as the SIM7080,
 * so we reuse the sem_tx_ready semaphore (given by on_cmd_tx_ready in
 * sim7022.c).
 *
 * Unlike the SIM7080, we do NOT need to pre-query the available TX
 * buffer space — just clip to the 1500-byte hardware limit.
 */

static ssize_t offload_sendto(void *obj, const void *buf, size_t len,
			      int flags,
			      const struct sockaddr *dest_addr,
			      socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	/*
	 * AT+CIPSEND format:
	 *   TCP: AT+CIPSEND=<link>,<len>
	 *   UDP: AT+CIPSEND=<link>,<len>,"<ip>",<port>
	 *
	 * For UDP the SIM7022 always requires the destination IP and port in
	 * AT+CIPSEND even if the socket was "connected" via AT+CIPOPEN.
	 * Without the IP/port the modem returns +CIPERROR: 11 (DNS failed).
	 */
	char send_buf[sizeof("AT+CIPSEND=#,####,\"xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxx.xxx.xxx.xxx\",#####")] = {0};
	char ip_str[NET_IPV6_ADDR_LEN];
	int ret;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (!sock->is_connected) {
		errno = ENOTCONN;
		return -1;
	}

	/*
	 * Clip to our RX/TX buffer size.  The SIM7022 hardware supports up
	 * to 1500 bytes per AT+CIPSEND, but our MDM_MAX_DATA_LENGTH buffer
	 * is 1024.  Callers that need larger transfers should loop.
	 */
	if (len > MDM_MAX_DATA_LENGTH) {
		len = MDM_MAX_DATA_LENGTH;
	}

	ret = modem_context_sprint_ip_addr((struct sockaddr *)&sock->dst,
					   ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Failed to format destination IP address");
		errno = EINVAL;
		return -1;
	}
	uint16_t dst_port = ntohs(net_sin((struct sockaddr *)&sock->dst)->sin_port);

	if (sock->type == SOCK_DGRAM) {
		/* UDP — destination IP and port required in AT+CIPSEND */
		ret = snprintk(send_buf, sizeof(send_buf),
			       "AT+CIPSEND=%d,%zu,\"%s\",%u",
			       sock->id, len, ip_str, dst_port);
	} else {
		/* TCP — already connected, no IP/port needed */
		ret = snprintk(send_buf, sizeof(send_buf),
			       "AT+CIPSEND=%d,%zu",
			       sock->id, len);
	}
	if (ret < 0) {
		LOG_ERR("Failed to build CIPSEND command");
		errno = ENOMEM;
		return -1;
	}


	/* Grab TX lock so only one send happens at a time */
	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	mdata.current_sock_written = len;

	/* Send AT+CIPSEND=<id>,<len> */
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler,
				    NULL, 0U, send_buf, NULL, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to send CIPSEND command");
		goto exit;
	}

	k_sleep(K_MSEC(300));

	k_sem_reset(&mdata.sem_send_ok);
	modem_cmd_send_data_nolock(&mctx.iface, buf, len);

	ret = k_sem_take(&mdata.sem_send_ok, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for send OK");
	}

exit:
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return mdata.current_sock_written;
}

/* ------------------------------------------------------------------ */
/* AT+CIPRXGET — read data from a socket                              */
/* ------------------------------------------------------------------ */

/*
 * Response format for AT+CIPRXGET=2,<link_num>,<len>:
 *
 *   +CIPRXGET: 2,<link_num>,<read_len>,<rest_len>\r\n
 *   <read_len bytes of data>
 *   OK
 *
 * The command handler on_cmd_ciprxget_data is registered for the literal
 * prefix "+CIPRXGET: 2," so the three remaining comma-separated fields
 * become:
 *   argv[0] = link_num
 *   argv[1] = read_len   ← actual byte count of the payload that follows
 *   argv[2] = rest_len   ← bytes still buffered in the module
 *
 * Using the "2," prefix (not the generic "+CIPRXGET: ") prevents this
 * handler from accidentally matching the data-available URC
 * "+CIPRXGET: 1,<link_num>".
 *
 * We use the same sockread_common() pattern as the SIM7080 driver.
 */

static int sockread_common(int sockfd,
			   struct modem_cmd_handler_data *data,
			   int socket_data_length,
			   uint16_t len)
{
	struct modem_socket *sock;
	struct socket_read_data *sock_data;
	int ret, packet_size;

	if (!len) {
		LOG_ERR("Zero length in sockread_common");
		return -EAGAIN;
	}

	if (!data->rx_buf) {
		LOG_ERR("No rx_buf in sockread_common");
		return -EINVAL;
	}

	if (socket_data_length <= 0) {
		LOG_ERR("Negative or zero data length: %d",
			socket_data_length);
		return -EAGAIN;
	}

	if (net_buf_frags_len(data->rx_buf) < socket_data_length) {
		LOG_DBG("Not enough data yet — wait");
		return -EAGAIN;
	}

	sock = modem_socket_from_fd(&mdata.socket_config, sockfd);
	if (!sock) {
		LOG_ERR("Socket not found for fd %d", sockfd);
		return -EINVAL;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("No socket read data for fd %d", sockfd);
		return -EINVAL;
	}

	ret = net_buf_linearize(sock_data->recv_buf,
				sock_data->recv_buf_len,
				data->rx_buf, 0,
				(uint16_t)socket_data_length);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;

	if (ret != socket_data_length) {
		LOG_ERR("Copied %d bytes but expected %d",
			ret, socket_data_length);
		ret = -EINVAL;
		goto exit;
	}

exit:
	/* Clear the dummy "1 byte" packet size set by the data indication */
	packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	modem_socket_packet_size_update(&mdata.socket_config, sock,
					-packet_size);
	return ret;
}

/*
 * Handler for the data read response:
 *   +CIPRXGET: 2,<link_num>,<read_len>,<rest_len>\r\n
 *   <read_len bytes of data>
 *   OK
 *
 * Registered for "+CIPRXGET: 2," so argv[0]=link_num, argv[1]=read_len,
 * argv[2]=rest_len.  This avoids accidentally matching the data-available
 * URC "+CIPRXGET: 1,<link_num>" which only has one argument.
 */
MODEM_CMD_DEFINE(on_cmd_ciprxget_data)
{
	int read_len = atoi(argv[1]);

	return sockread_common(mdata.current_sock_fd, data, read_len, len);
}

static ssize_t offload_recvfrom(void *obj, void *buf, size_t max_len,
				int flags,
				struct sockaddr *src_addr,
				socklen_t *addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char sendbuf[sizeof("AT+CIPRXGET=2,#,####")];
	int ret, packet_size;
	struct socket_read_data sock_data;

	/*
	 * Match "+CIPRXGET: 2," specifically (not the generic "+CIPRXGET: ")
	 * to avoid confusing the data response with the data-available URC.
	 * Three comma-separated args follow: link_num, read_len, rest_len.
	 */
	struct modem_cmd data_cmd[] = {
		MODEM_CMD("+CIPRXGET: 2,", on_cmd_ciprxget_data, 3U, ",")
	};

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	if (!buf || max_len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (!packet_size) {
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}
		modem_socket_wait_data(&mdata.socket_config, sock);
		packet_size = modem_socket_next_packet_size(&mdata.socket_config,
							    sock);
	}

	/* Cap at the hardware per-read limit of 1500 bytes */
	if (max_len > MDM_MAX_DATA_LENGTH) {
		max_len = MDM_MAX_DATA_LENGTH;
	}

	/*
	 * AT+CIPRXGET=2,<link_num>,<max_len>
	 * Read max_len bytes from the modem's receive buffer.
	 */
	snprintk(sendbuf, sizeof(sendbuf),
		 "AT+CIPRXGET=2,%d,%zd", sock->id, max_len);

	memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf     = buf;
	sock_data.recv_buf_len = max_len;
	sock_data.recv_addr    = src_addr;
	sock->data             = &sock_data;
	mdata.current_sock_fd  = sock->sock_fd;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     data_cmd, ARRAY_SIZE(data_cmd),
			     sendbuf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto exit;
	}

	/* Use destination address as source (same trick as SIM7080) */
	if (src_addr && addrlen) {
		*addrlen = sizeof(sock->dst);
		memcpy(src_addr, &sock->dst, *addrlen);
	}

	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	mdata.current_sock_fd = -1;
	sock->data = NULL;
	return ret;
}

/* ------------------------------------------------------------------ */
/* AT+CIPCLOSE — close a socket                                        */
/* ------------------------------------------------------------------ */

static void socket_close(struct modem_socket *sock)
{
	char buf[sizeof("AT+CIPCLOSE=##")];
	int ret;

	snprintk(buf, sizeof(buf), "AT+CIPCLOSE=%d", sock->id);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U, buf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
}

/* ------------------------------------------------------------------ */
/* sendmsg wrapper                                                      */
/* ------------------------------------------------------------------ */

static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg,
			       int flags)
{
	struct modem_socket *sock = obj;
	ssize_t sent = 0;
	const char *buf;
	size_t len;
	int ret;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	if (sock->type == SOCK_DGRAM) {
		/* Only support a single contiguous datagram at a time */
		if (msghdr_non_empty_iov_count(msg) > 1) {
			errno = EMSGSIZE;
			return -1;
		}
	}

	for (int i = 0; i < msg->msg_iovlen; i++) {
		buf = msg->msg_iov[i].iov_base;
		len = msg->msg_iov[i].iov_len;

		while (len > 0) {
			ret = offload_sendto(obj, buf, len, flags,
					     msg->msg_name,
					     msg->msg_namelen);
			if (ret < 0) {
				return ret;
			}
			sent += ret;
			buf  += ret;
			len  -= ret;
		}
	}

	return sent;
}

/* ------------------------------------------------------------------ */
/* Thin wrappers for read/write/close                                  */
/* ------------------------------------------------------------------ */

static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	if (!modem_socket_is_allocated(&mdata.socket_config, sock)) {
		return 0;
	}

	socket_close(sock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* poll                                                                 */
/* ------------------------------------------------------------------ */

static int offload_poll(struct zsock_pollfd *fds, int nfds, int msecs)
{
	void *obj;

	if (sim7022_get_state() != SIM7022_STATE_NETWORKING) {
		LOG_ERR("Modem not in networking state");
		return -EINVAL;
	}

	for (int i = 0; i < nfds; i++) {
		if (fds[i].fd < 0) {
			continue;
		}
		obj = zvfs_get_fd_obj(
			fds[i].fd,
			(const struct fd_op_vtable *)
				&offload_socket_fd_op_vtable,
			EINVAL);
		if (obj == NULL) {
			return -1;
		}
	}

	return modem_socket_poll(&mdata.socket_config, fds, nfds, msecs);
}

static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE:
		return -EXDEV;

	case ZFD_IOCTL_POLL_UPDATE:
		return -EOPNOTSUPP;

	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zsock_pollfd *fds;
		int nfds, timeout;

		fds     = va_arg(args, struct zsock_pollfd *);
		nfds    = va_arg(args, int);
		timeout = va_arg(args, int);
		return offload_poll(fds, nfds, timeout);
	}

	default:
		errno = EINVAL;
		return -1;
	}
}

/* ------------------------------------------------------------------ */
/* vtable registration                                                  */
/* ------------------------------------------------------------------ */

const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read  = offload_read,
		.write = offload_write,
		.close = offload_close,
		.ioctl = offload_ioctl,
	},
	.bind      = NULL,
	.connect   = offload_connect,
	.sendto    = offload_sendto,
	.recvfrom  = offload_recvfrom,
	.listen    = NULL,
	.accept    = NULL,
	.sendmsg   = offload_sendmsg,
	.getsockopt = NULL,
	.setsockopt = NULL,
};

/* ------------------------------------------------------------------ */
/* Data-available indication (called from URC handler in sim7022.c)   */
/* ------------------------------------------------------------------ */

/*
 * Called when "+CIPRXGET: 1,<link_num>" URC is received.
 *
 * @param id  The modem socket ID (link_num, 0 or 1), NOT a POSIX fd.
 *            modem_socket_from_id() must be used because the link_num
 *            is the modem's internal socket identifier, while sock_fd
 *            is the POSIX file descriptor (a larger OS-assigned number).
 */
void sim7022_handle_sock_data_indication(int id)
{
	struct modem_socket *sock =
		modem_socket_from_id(&mdata.socket_config, id);

	if (!sock) {
		LOG_INF("No socket with id %d for data indication", id);
		return;
	}

	/*
	 * The module does not tell us how many bytes are available.
	 * Set a dummy size of 1 to unblock any waiting recv() call;
	 * the actual data will be fetched with AT+CIPRXGET=2.
	 */
	modem_socket_packet_size_update(&mdata.socket_config, sock, 1);
	LOG_INF("Data available on socket id=%d fd=%d", id, sock->sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);
}

/* ------------------------------------------------------------------ */
/* Socket-state change (called from URC handler in sim7022.c)         */
/* ------------------------------------------------------------------ */

/*
 * Called when "+CIPCLOSE: <link_num>,<err>" URC is received.
 *
 * @param id     The modem socket ID (link_num, 0 or 1), NOT a POSIX fd.
 * @param state  0 = closed.
 */
void sim7022_handle_sock_state(int id, uint8_t state)
{
	struct modem_socket *sock =
		modem_socket_from_id(&mdata.socket_config, id);

	if (!sock) {
		LOG_INF("No socket with id %d for state change", id);
		return;
	}

	/* state == 0 means closed */
	if (state != 0) {
		return;
	}

	LOG_INF("Socket close indication for id=%d fd=%d", id, sock->sock_fd);
	sock->is_connected = false;

	/* Unblock any waiting recv() */
	modem_socket_packet_size_update(&mdata.socket_config, sock, 0);
	modem_socket_data_ready(&mdata.socket_config, sock);
}

/* ------------------------------------------------------------------ */
/* Socket allocation (called by offload_is_supported in sim7022.c)    */
/* ------------------------------------------------------------------ */

int sim7022_offload_socket(int family, int type, int proto)
{
	int ret;

	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}
