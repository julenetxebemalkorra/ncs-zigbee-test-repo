/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef TCU_UART_H_
#define TCU_UART_H_

/* Default tick to consider a modbus frame completed*/
#define TICKS_TO_CONSIDER_FRAME_COMPLETED 10 // 10ms = approximately time for transmitting 20 chars at 19200

/*UART Modbus and zigbee buffer size definitions*/
#define UART_RX_BUFFER_SIZE              255 //253 bytes + CRC (2 bytes) = 255


#define MAXIMUM_SIZE_MODBUS_RTU_FRAME 256
#define SIZE_TRANSMISSION_BUFFER MAXIMUM_SIZE_MODBUS_RTU_FRAME
#define SIZE_OF_RX_FIFO_OF_NRF52840_UART 6

/* States used to validate the "+++" sequence to enter in command mode        */
enum
{
    ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_INITIAL_SILENCE_ST,
    ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_FIRST_CHAR_ST,
    ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_SECOND_CHAR_ST,
    ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_THIRD_CHAR_ST,
    ENTER_CMD_MODE_SEQUENCE_WAITING_FOR_END_SILENCE_ST
};

/* Function prototypes                                                        */
int8_t tcu_uart_init(void);
void tcu_uart_rx_buffer_init(void);
int8_t tcu_uart_configuration(void);
void tcu_uart_timers_10kHz(void);
void tcu_uart_process_byte_received_in_command_mode(uint8_t input_byte);
void tcu_uart_process_byte_received_in_transparent_mode(uint8_t input_byte);
void tcu_uart_isr(const struct device *dev, void *user_data);
void sendFrameToTcu(uint8_t *input_data, uint16_t size_input_data);
void switch_tcu_uart_to_command_mode(void);
void switch_tcu_uart_out_of_command_mode(void);
bool is_tcu_uart_in_command_mode(void);
void check_input_sequence_for_entering_in_command_mode(uint8_t input_byte);
bool tcu_uart_send_received_frame_through_zigbee(void);
void tcu_uart_transparent_mode_manager(void);

extern uint8_t tcu_transmitted_frames_counter;
#endif /* TCU_UART_H_ */







