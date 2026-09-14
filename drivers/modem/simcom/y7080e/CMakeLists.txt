# Copyright (c) 2026 Anastasios Kaltakis
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/modem)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/ip)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/lib/sockets)
zephyr_library_sources(
    y7080e.c
    y7080e_sock.c
    y7080e_dns.c
    y7080e_lpm.c)
