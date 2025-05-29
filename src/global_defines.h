/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef GLOBAL_DEFINES_H_
#define GLOBAL_DEFINES_H_

#define MAXIMUM_SIZE_NODE_IDENTIFIER 20
#define SIZE_LINK_KEY 16
#define APS_ACK_REQUIRED 1

#define COORDINATOR_SHORT_ADDRESS 0x0000
#define COORDINATOR_SHORT_ADDRESS_LOWER_BYTE 0x00
#define COORDINATOR_SHORT_ADDRESS_HIGHER_BYTE 0x00

//Indicadores de alarma     
extern bool g_b_flash_error;
extern bool g_b_flash_write_cmd;
extern bool g_b_reset_cmd;
extern bool g_b_reset_zigbee_cmd;
extern bool g_b_reset_mcu_after_leaving_network;
extern bool tcu_transmission_running;

#endif /* GLOBAL_DEFINES_H_ */
