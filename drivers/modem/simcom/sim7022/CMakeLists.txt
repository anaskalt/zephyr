# Copyright (c) 2025 Metratec GmbH.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/modem)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/ip)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/lib/sockets)
zephyr_library_sources(
    sim7022.c
    sim7022_pdp.c
    sim7022_dns.c
    sim7022_sock.c
    sim7022_sms.c
    sim7022_meas.c
    sim7022_utils.c)
