/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef GLOBAL_DEFINES_H_
#define GLOBAL_DEFINES_H_

#define MAXIMUM_SIZE_NODE_IDENTIFIER 20
#define MAXIMUM_SIZE_LINK_KEY 32

//Indicadores de alarma     
extern bool g_b_flash_error;
extern bool g_b_flash_write_cmd;
extern bool g_b_reset_cmd;
extern bool g_b_reset_zigbee_cmd;
extern bool tcu_transmission_running;
extern bool g_b_nvram_write_done;

#endif /* GLOBAL_DEFINES_H_ */
