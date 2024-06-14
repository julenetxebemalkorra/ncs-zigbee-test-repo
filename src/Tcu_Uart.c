/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of the UART0, which is connected to the TCU.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zboss_api.h>

// UART async includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_At_commands.h"

#include "tcu_Uart.h"

extern uint16_t tcu_uart_frames_received_counter;

LOG_MODULE_REGISTER(uart_app, LOG_LEVEL_DBG);

/* Local variables used to manage the command mode of the zigbee module */
static bool b_zigbee_module_in_command_mode = false;
static uint8_t enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST;
static uint16_t pre_silence_timer_ms = 0;
static uint16_t post_silence_timer_ms = 0;
static uint16_t leave_cmd_mode_silence_timer_ms = 0;
/**/

static uint8_t tcu_transmission_buffer[SIZE_TRANSMISSION_BUFFER] = {0};
static uint8_t tcu_transmission_buffer_index = 0;
static uint8_t tcu_transmission_size = 0;
static bool    tcu_transmission_running = false;

static volatile uint8_t tcu_uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};
static volatile uint16_t tcu_uart_rx_buffer_index = 0;
static volatile uint16_t tcu_uart_rx_buffer_frame_size = 0;
static volatile bool b_tcu_uart_rx_buffer_overflow = false;
static volatile bool b_tcu_uart_rx_buffer_busy = false;
static volatile bool b_tcu_uart_rx_receiving_frame = false;
static volatile bool b_tcu_uart_rx_complete_frame_received = false;
static volatile bool b_tcu_uart_rx_corrupted_frame = false;
static volatile uint16_t tcu_uart_rx_time_since_last_byte_ms = 0;

/* Get the device pointer of the UART hardware */
static const struct device *dev_tcu_uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

// Create a configuration structure for the UART used to communicate with the TCU
static struct uart_config tcu_uart_config = {
    .baudrate = 19200,
    .parity = UART_CFG_PARITY_NONE,
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
    int8_t ret = 0;//tcu_uart_configuration();
    b_zigbee_module_in_command_mode = false;
    tcu_uart_rx_buffer_init();
    return ret;
}

/**@brief Initialization of the TCU UART RX buffer
 *
 */
void tcu_uart_rx_buffer_init(void)
{
    tcu_uart_rx_buffer_index = 0;
    tcu_uart_rx_buffer_frame_size = 0;
    b_tcu_uart_rx_buffer_overflow = false;
    b_tcu_uart_rx_buffer_busy = false;
    b_tcu_uart_rx_receiving_frame = false;
    b_tcu_uart_rx_complete_frame_received = false;
    b_tcu_uart_rx_corrupted_frame = false;
    tcu_uart_rx_time_since_last_byte_ms = 0;
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

/**@brief This function updates the timers used in the Tcu UART FW module.
 * @details The 10 ms silence to consider that a RX frame is complete [10ms]
 *        The 0.5 s silence before and after the sequence "+++", which make the Zigbee module
 *        to enter in command mode
 *        The 10 s silence which makes the which make the Zigbee module to exit command mode
 *
 * This function should be executed at 10 kHz.
 *
 */
void tcu_uart_timers_10kHz(void)
{
    static uint8_t one_ms_counter = 0;

    if( one_ms_counter < 10 ) one_ms_counter++;
    else
    {
        one_ms_counter = 0;
/*      Verify the 10 ms silence needed to consider that the RX frame is complete    */
        if( !b_zigbee_module_in_command_mode )
        {
            if( b_tcu_uart_rx_receiving_frame )
            {
                tcu_uart_rx_time_since_last_byte_ms++;
                if( tcu_uart_rx_time_since_last_byte_ms > TICKS_TO_CONSIDER_FRAME_COMPLETED )
                {
                    if( b_tcu_uart_rx_buffer_overflow || b_tcu_uart_rx_corrupted_frame ) // Discard frame if rx buffer overflow or frame corrupted
                    {
                        b_tcu_uart_rx_buffer_overflow = false;
                        b_tcu_uart_rx_corrupted_frame = false;
                    }
                    else
                    {
                        b_tcu_uart_rx_complete_frame_received = true;
                        tcu_uart_rx_buffer_frame_size = tcu_uart_rx_buffer_index;
                        b_tcu_uart_rx_buffer_busy = true; // Do not accept new characters until received frame is processed
                    }
                    b_tcu_uart_rx_receiving_frame = false;
                    tcu_uart_rx_time_since_last_byte_ms = 0;
                    tcu_uart_rx_buffer_index = 0;
                }
            }
            else
            {
                tcu_uart_rx_time_since_last_byte_ms = 0;
            }
        }
/*      Verify the 500 ms silences needed to accept the "+++" sequence        */
        if( enter_cmd_mode_sequence_st == ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST )
        {
            if( pre_silence_timer_ms < 500 ) pre_silence_timer_ms++;
            else
            {
                pre_silence_timer_ms = 0;
                enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_FIRST_CHAR_ST;
            }
        }
        else if( enter_cmd_mode_sequence_st == ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_END_SILENCE_ST )
        {
            if( post_silence_timer_ms < 500 ) post_silence_timer_ms++;
            else
            {
                post_silence_timer_ms = 0;
                switch_tcu_uart_to_command_mode();
                digi_at_reply_ok();
                enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST;
            }
        }
/*      Verify the 10 s silence which makes the zigbee module to leave automatically the command mode */
        if( b_zigbee_module_in_command_mode )
        {
            if( leave_cmd_mode_silence_timer_ms < 10000 ) leave_cmd_mode_silence_timer_ms++;
            else
            {
                switch_tcu_uart_out_of_command_mode();
            }
        }
    }
}

/**@brief This function processes the last byte received throught the
 *        TCU's UART when the Zigbee module is in command mode
 *
 */
void tcu_uart_process_byte_received_in_command_mode(uint8_t input_byte)
{
    leave_cmd_mode_silence_timer_ms = 0; //Reset the timer every time a character is received
    check_input_sequence_for_entering_in_command_mode(input_byte); // It also has to be checked when we are already in command mode.

    if(input_byte == '\r') // End of frame
    {
        if( tcu_uart_rx_buffer_index == 0 ) return; //Ignore empty commands

        if( b_tcu_uart_rx_buffer_overflow )
        {
            digi_at_reply_error();
        }
        else
        {
            int8_t command_analysis_result = digi_at_analyze_and_reply_to_command(tcu_uart_rx_buffer, tcu_uart_rx_buffer_index);
            if( command_analysis_result == AT_CMD_OK_LEAVE_CMD_MODE ) //As a result of the command, we should leave command mode
            {
                switch_tcu_uart_out_of_command_mode();
            }
            else if( command_analysis_result < 0 ) //Command not accepted
            {
                LOG_ERR("Wrong AT command. Error code: %d", command_analysis_result);
                for(uint8_t kk=0; kk<tcu_uart_rx_buffer_index; kk++)
                {
                    LOG_WRN("%d", tcu_uart_rx_buffer[kk]);
                }
            }
        }
        tcu_uart_rx_buffer_index = 0;
        b_tcu_uart_rx_buffer_overflow = false;
    }
    else if( !b_tcu_uart_rx_buffer_overflow )
    {
        if( tcu_uart_rx_buffer_index >= UART_RX_BUFFER_SIZE )
        {
            b_tcu_uart_rx_buffer_overflow = true;
        }
        else if( tcu_uart_rx_buffer_index == 0 )
        {
            if( (input_byte == 'A') || (input_byte == 'a') ) //Ignore received characters until the first 'A' or 'a' is received
            {
                tcu_uart_rx_buffer[tcu_uart_rx_buffer_index] = input_byte;
                tcu_uart_rx_buffer_index++;
            }
        }
        else
        {
            tcu_uart_rx_buffer[tcu_uart_rx_buffer_index] = input_byte;
            tcu_uart_rx_buffer_index++;
        }
    }
}

/**@brief This function processes the last byte received throught the
 *        TCU's UART when the Zigbee module is in transparent mode
 *
 */
void tcu_uart_process_byte_received_in_transparent_mode(uint8_t input_byte)
{
    tcu_uart_rx_time_since_last_byte_ms = 0; //Reset timer used to detect end of frame

    check_input_sequence_for_entering_in_command_mode(input_byte);

    if( b_tcu_uart_rx_receiving_frame )
    {
        if( !b_tcu_uart_rx_corrupted_frame )
        {
            if( !b_tcu_uart_rx_buffer_overflow )
            {
                if( tcu_uart_rx_buffer_index >= UART_RX_BUFFER_SIZE )
                {
                    b_tcu_uart_rx_buffer_overflow = true;
                }
                else
                {
                    tcu_uart_rx_buffer[tcu_uart_rx_buffer_index] = input_byte;
                    tcu_uart_rx_buffer_index++;
                }
            }
        }
    }
    else
    {
        if(input_byte != '+') // Do not consider input frames starting by '+'
        {            
            b_tcu_uart_rx_receiving_frame = true;
            b_tcu_uart_rx_buffer_overflow = false;
            if( b_tcu_uart_rx_buffer_busy ) // Ignore new characters if we are still processing the last received frame.
            {
                b_tcu_uart_rx_corrupted_frame  = true;
            }
            else
            {
                tcu_uart_rx_buffer[0] = input_byte;
                tcu_uart_rx_buffer_index = 1;
            }
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

	if( !uart_irq_update(dev_tcu_uart) )
    {
		return;
	}

	if( uart_irq_rx_ready(dev_tcu_uart) ) // Check if the UART interruption was triggered because a new character was received
    {
    /*  read until FIFO empty  */
        int number_of_bytes_available;
        number_of_bytes_available = uart_fifo_read(dev_tcu_uart, uart_rx_hw_fifo, SIZE_OF_RX_FIFO_OF_NRF52840_UART);
        while( number_of_bytes_available > 0 )
        {
            number_of_bytes_available--;
            if( b_zigbee_module_in_command_mode )
            {
                tcu_uart_process_byte_received_in_command_mode(uart_rx_hw_fifo[uart_rx_hw_fifo_index]);
            }
            else
            {
                tcu_uart_process_byte_received_in_transparent_mode(uart_rx_hw_fifo[uart_rx_hw_fifo_index]);
            }
            uart_rx_hw_fifo_index++;
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
    tcu_uart_rx_buffer_index = 0;
    leave_cmd_mode_silence_timer_ms = 0;
    if( !b_zigbee_module_in_command_mode )
    {
        b_zigbee_module_in_command_mode = true;
        LOG_DBG("Enter in command mode");
    }
}

/**@brief Switch the TCU uart out of command mode
 *
 *
 */
void switch_tcu_uart_out_of_command_mode(void)
{
    tcu_uart_rx_buffer_init();
    b_zigbee_module_in_command_mode = false;
    LOG_DBG("Leave command mode");
}

/**@brief Indicate if the TCU UART is in command mode
 *
 * @retval true In command mode
 * @retval false Not in command mode (i.e. transparent mode)
 */
bool is_tcu_uart_in_command_mode(void)
{
    return b_zigbee_module_in_command_mode;
}

/**@brief Check if the TCU has sent the "+++" sequence
 *
 * @param[in]   input_byte   Last byte received through the TCU UART
 *
 */
void check_input_sequence_for_entering_in_command_mode(uint8_t input_byte)
{
    if( input_byte == '+' )
    {
        if( enter_cmd_mode_sequence_st == ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_FIRST_CHAR_ST )
        {
            enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_SECOND_CHAR_ST;
        }
        else if( enter_cmd_mode_sequence_st == ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_SECOND_CHAR_ST )
        {
            enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_THIRD_CHAR_ST;
        }
        else if( enter_cmd_mode_sequence_st == ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_THIRD_CHAR_ST )
        {
            enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_END_SILENCE_ST;
        }
        else
        {
            pre_silence_timer_ms = 0;
            enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST;
        }
    }
    else
    {
        pre_silence_timer_ms = 0;
        enter_cmd_mode_sequence_st = ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST;
    }
}

/**@brief This function places in the APS output frame queue a frame received through
*         the TCU UART when the zigbee module is in transparent mode.
*/
bool tcu_uart_send_received_frame_through_zigbee(void)
{
    bool b_return = false;

    if( zigbee_aps_get_output_frame_buffer_free_space() )
    {
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.cluster_id = DIGI_BINARY_VALUE_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        element.payload_size = tcu_uart_rx_buffer_frame_size;

        if( element.payload_size > APS_UNENCRYPTED_PAYLOAD_MAX)
        {
            // TODO: send the payload in several frames
            LOG_WRN("Payload size too big to be sent in a single frame %d", element.payload_size);
            // Split the payload into multiple frames and send them sequentially
            uint8_t remaining_payload_size = element.payload_size;
            uint8_t offset = 0;
            while (remaining_payload_size > 0)
            {
                // Calculate the payload size for this frame
                if (remaining_payload_size > APS_UNENCRYPTED_PAYLOAD_MAX) 
                {
                    element.payload_size = APS_UNENCRYPTED_PAYLOAD_MAX;            
                } 
                else 
                {
                    element.payload_size = remaining_payload_size;            
                }
                // Copy the portion of the payload to the new frame
                for( uint8_t i = 0; i < element.payload_size; i++ )
                {
                    element.payload[i] = tcu_uart_rx_buffer[offset + i];
                }

                LOG_WRN("Added new frame to buffer. Remaining payload size: %d", remaining_payload_size);
                b_return = enqueue_aps_frame(&element);

                // Update remaining_payload_size and offset
                remaining_payload_size -= element.payload_size;
                offset += element.payload_size;
            }
            if (b_return) 
            {
                b_return = true;
            }
        }
        else
        {
            memcpy(element.payload, &tcu_uart_rx_buffer[0], element.payload_size);
            if( enqueue_aps_frame(&element) ) b_return = true;
        }
    }

    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}

/**@brief If a complete frame has been received from the TCU UART when the module is
 *        i transparente mode, place it in the APS output frame queue.
 *
 */
void tcu_uart_transparent_mode_manager(void)
{
    if( b_tcu_uart_rx_complete_frame_received )
    {
        tcu_uart_frames_received_counter++;
        tcu_uart_send_received_frame_through_zigbee();
        b_tcu_uart_rx_complete_frame_received = false;
        b_tcu_uart_rx_buffer_busy = false;
    }   
}