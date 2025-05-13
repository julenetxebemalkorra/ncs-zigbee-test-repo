#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dual_wireless_test_uart)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/zigbee_bdb.c
  src/Tcu_Uart.c
  src/zigbee_aps.c
  src/zigbee_configuration.c
  src/Digi_node_discovery.c
  src/Digi_At_commands.c
  src/Digi_wireless_at_commands.c
  src/nvram.c
  src/Digi_fota.c
  src/OTA_dfu_target.c
)

target_include_directories(app PRIVATE include)
# NORDIC SDK APP END
