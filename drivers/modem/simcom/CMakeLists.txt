# Copyright (c) 2025 Metratec GmbH.
# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_MODEM_SIM7080 sim7080)
add_subdirectory_ifdef(CONFIG_MODEM_Y7080E y7080e)
# zephyr-keep-sorted-stop
