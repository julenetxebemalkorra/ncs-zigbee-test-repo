# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_APP_UART lib)
target_include_directories(app PRIVATE include/app_uart)