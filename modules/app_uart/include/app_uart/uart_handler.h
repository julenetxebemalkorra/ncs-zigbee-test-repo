#ifndef __UART_HANDLER_H
#define __UART_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

enum app_uart_evt_types_t {APP_UART_EVT_RX, APP_UART_EVT_ERROR, APP_UART_EVT_QUEUE_OVERFLOW};

struct app_uart_evt_t {
	enum app_uart_evt_types_t type;
	union {
		struct {
			uint8_t *bytes;
			uint32_t length;
			uint8_t *_source_buf;
		} rx;
		struct {
			enum uart_rx_stop_reason reason;
		} error;
	} data;
};

typedef void (*app_uart_event_handler_t)(struct app_uart_evt_t *evt);

int app_uart_init(app_uart_event_handler_t evt_handler);

int app_uart_send(const uint8_t * data_ptr, uint32_t data_len, k_timeout_t timeout);

#endif