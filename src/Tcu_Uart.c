/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of the UART0, which is connected to the TCU.
 */

#include <zephyr/kernel.h>

// UART async includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "tcu_Uart.h"

LOG_MODULE_REGISTER(uart_app, LOG_LEVEL_DBG);


static uint8_t tcu_transmission_buffer[SIZE_TRANSMISSION_BUFFER] = {0};
static uint8_t tcu_transmission_buffer_index = 0;
static uint8_t tcu_transmission_size = 0;
static bool    tcu_transmission_running = false;
static bool    b_tcu_uart_in_command_mode = false;
static uint8_t tcuTransmittedFramesCounter = 0;
static volatile bool b_tcu_uart_receiving_frame = false;
static volatile uint16_t tcu_uart_ticks_since_last_byte;

/* Get the device pointer of the UART hardware */
static const struct device *dev_tcu_uart= DEVICE_DT_GET(DT_NODELABEL(uart0));
	

/* UART configuration variables*/
static volatile bool b_UART_overflow;

// Create a configuration structure for the UART used to communicate with the TCU
static struct uart_config tcu_uart_config = {
    .baudrate = 19200,
    .parity = UART_CFG_PARITY_EVEN,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
 };


/**@brief Initialization of the TCU UART FW module
 *
 * @retval -1 Error
 * @retval 0 OK
 */
int8_t tcu_uart_init(void)
{
    int8_t ret = tcu_uart_configuration();
    b_tcu_uart_in_command_mode = false;
    b_tcu_uart_receiving_frame = false;
    tcu_uart_ticks_since_last_byte = 0;
	b_UART_overflow = false;
	UART_rx_buffer_index = 0;
	UART_rx_buffer_index_max = 0;
    return ret;
}

/**@brief Configuration and initialization of the UART used to communicate with the TCU
 *
 * @retval -1 Error
 * @retval 0 OK
 */
int8_t tcu_uart_configuration(void)
{
	if (!device_is_ready(dev_tcu_uart)) {
		LOG_ERR("UART device not found!");
		return -1;
	}

	// Call uart_configure to apply the configuration
    int result = uart_configure(dev_tcu_uart, &tcu_uart_config);

	// Check the result
    if (result == 0) {
        LOG_DBG("TCU UART configuration successful!\n");
    } else {
        LOG_ERR("TCU UART configuration failed with error code: %d\n", result);
        return -1;
    }

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(dev_tcu_uart, tcu_uart_isr, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API\n");
		} else {
			LOG_ERR("Error setting UART callback: %d\n", ret);
		}
		return -1;
	}
	else
	{
        LOG_DBG("UART Interrupt configuration successful!\n");
	}

	uart_irq_rx_enable(dev_tcu_uart);
    return 0;
}

/**@brief This function evaluates the time elapsed since the reception of the
 *        last character through the TCU UART.
 *
 * After 10 ms without receiving a character, it is considered that the frame is completed.
 * This function should be executed at 10 kHz.
 *
 */
void tcu_uart_end_of_frame_time_control_10kHz(void)
{
    if( !b_tcu_uart_in_command_mode )
    {
        if( b_tcu_uart_receiving_frame )
        {
            tcu_uart_ticks_since_last_byte++;
            if( tcu_uart_ticks_since_last_byte > TICKS_TO_CONSIDER_FRAME_COMPLETED )
            {
                b_tcu_uart_receiving_frame = false;
                tcu_uart_ticks_since_last_byte = 0;
                b_Modbus_Ready_to_send = true;
            }
        }
        else
        {
            tcu_uart_ticks_since_last_byte = 0;
        }
    }
}

/**@brief Interrupt Service Routine for the UART used to communicate with the TCU
 *
 *
 */
void tcu_uart_isr(const struct device *dev, void *user_data)
{
	uint8_t uart_rx_hw_fifo[SIZE_OF_RX_FIFO_OF_NRF52840_UART];
    uint8_t uart_rx_hw_fifo_index = 0;

	if (!uart_irq_update(dev_tcu_uart)) {
		return;
	}

	if ( uart_irq_rx_ready(dev_tcu_uart)) // Check if the UART interruption was triggered because a new character was received
    {
    /*  read until FIFO empty  */
        int number_of_bytes_available;
        number_of_bytes_available = uart_fifo_read(dev_tcu_uart, &uart_rx_hw_fifo, SIZE_OF_RX_FIFO_OF_NRF52840_UART);
        while(number_of_bytes_available > 0)
        {
            number_of_bytes_available--;
            tcu_uart_ticks_since_last_byte = 0; //Reset 3.5T Modbus timer
            if( b_tcu_uart_receiving_frame )
            {
                if( !b_UART_overflow )
                {
                    if( UART_rx_buffer_index >= UART_RX_BUFFER_SIZE )
                    {
                        b_UART_overflow = true;
                    }
                    else
                    {
                        UART_rx_buffer[UART_rx_buffer_index] = uart_rx_hw_fifo[uart_rx_hw_fifo_index];
                        uart_rx_hw_fifo_index++;
                        UART_rx_buffer_index_max = UART_rx_buffer_index;
                        UART_rx_buffer_index++;
                    }
                }
            }
            else
            {
                b_tcu_uart_receiving_frame = true;
                b_UART_overflow = false;
                UART_rx_buffer[0] = uart_rx_hw_fifo[uart_rx_hw_fifo_index];
                uart_rx_hw_fifo_index++;
                UART_rx_buffer_index = 1;
                UART_rx_buffer_index_max = UART_rx_buffer_index;
            }
        }		
	}
	if (uart_irq_tx_ready(dev_tcu_uart)) // Check if the UART interruption was triggered because the transmission buffer got empty
    {
        int ret;
        if(tcu_transmission_running)
        {
            if(tcu_transmission_buffer_index < tcu_transmission_size) // Any pending byte to be transmitted?
            {
                ret = uart_fifo_fill(dev_tcu_uart, (uint8_t *)&tcu_transmission_buffer[tcu_transmission_buffer_index], 1);
                if(ret > 0)
                {
                    tcu_transmission_buffer_index = tcu_transmission_buffer_index + ret;
                }
            }
            else
            {
                tcu_transmission_running = false;
                uart_irq_tx_disable(dev_tcu_uart);
            }
        }
        else
        {
            uart_irq_tx_disable(dev_tcu_uart);
        }
    }
}

/**@brief Transmit frame using the TCU uart
 *
 *
 */
void sendFrameToTcu(uint8_t *input_data, uint16_t size_input_data)
{
    if(!tcu_transmission_running)
    {
        for(uint16_t i = 0; i <size_input_data; i++)
        {
            tcu_transmission_buffer[i] = input_data[i];
        }
        tcu_transmission_size = size_input_data;
        tcu_transmission_running = true;
        uart_poll_out(dev_tcu_uart, tcu_transmission_buffer[0]); // Place the first byte in the UART transmission buffer
        tcu_transmission_buffer_index = 1;      // Index of next byte to be transmitted
        uart_irq_tx_enable(dev_tcu_uart);
    }
}

/**@brief Switch the TCU uart to command mode
 *
 *
 */
void switch_tcu_uart_to_command_mode(void)
{
    b_tcu_uart_in_command_mode = true;
}

/**@brief Switch the TCU uart out of command mode
 *
 *
 */
void switch_tcu_uart_out_of_command_mode(void)
{
    tcu_uart_ticks_since_last_byte = 0;
    b_tcu_uart_in_command_mode = false;
}
