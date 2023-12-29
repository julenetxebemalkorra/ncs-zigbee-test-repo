# SPDX-License-Identifier: Apache-2.0

if(CONFIG_APP_UART)
	zephyr_include_directories(../../include/app_uart)
	zephyr_library()
	zephyr_library_sources(
	  uart_handler.c
	  )
  endif()