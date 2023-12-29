#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Manually add the app_uart module to the sample
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/app_uart
  )

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dual_wireless_test_uart)

target_include_directories(app PRIVATE src)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)

target_include_directories(app PRIVATE include)
# NORDIC SDK APP END
