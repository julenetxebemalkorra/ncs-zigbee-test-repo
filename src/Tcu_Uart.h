/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef TCU_UART_H_
#define TCU_UART_H_

/* Default tick to consider a modbus frame completed*/
#define TICKS_TO_CONSIDER_FRAME_COMPLETED 100 // 100 ticks of 100 us = 10ms = approximately time for transmitting 20 chars at 19200

/*UART Modbus and zigbee buffer size definitions*/
#define UART_RX_BUFFER_SIZE              255 //253 bytes + CRC (2 bytes) = 255


#define MAXIMUM_SIZE_MODBUS_RTU_FRAME 256
#define SIZE_TRANSMISSION_BUFFER MAXIMUM_SIZE_MODBUS_RTU_FRAME
#define SIZE_OF_RX_FIFO_OF_NRF52840_UART 6

int8_t tcu_uart_init(void);
int8_t tcu_uart_configuration(void);
void tcu_uart_end_of_frame_time_control_10kHz(void);
void tcu_uart_isr(const struct device *dev, void *user_data);
void sendFrameToTcu(uint8_t *input_data, uint16_t size_input_data);

extern volatile char UART_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint16_t UART_rx_buffer_index;
extern volatile uint16_t UART_rx_buffer_index_max;
extern uint8_t tcu_transmitted_frames_counter;
extern volatile bool b_Modbus_Ready_to_send;
#endif /* TCU_UART_H_ */







