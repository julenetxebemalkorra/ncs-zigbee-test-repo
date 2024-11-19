/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef ZIGBEE_CONFIGURATION_H_
#define ZIGBEE_CONFIGURATION_H_

#include "global_defines.h"

enum nvram_status_t {
    NVRAM_UNKNOWN_ERR = -4,
    NVRAM_ERROR_READING = -3,
    NVRAM_WRONG_DATA = -2,
    NVRAM_NOT_WRITTEN = -1,
    SUCCESS = 0
};

struct zb_user_conf_t {
    uint64_t extended_pan_id; // Extended pan id
    uint8_t at_ni[MAXIMUM_SIZE_NODE_IDENTIFIER + 1];   // Node identifier string parameter (plus one to include the '\0')
};

/* Function prototypes (used only internally)                                 */

/* Function prototypes (used externally)                                      */
int8_t zb_nvram_check_usage(void);
uint8_t zb_conf_read_from_nvram (void);
void zb_conf_write_to_nvram (void);
void zb_conf_update (void);
uint64_t zb_conf_get_extended_pan_id (void);
void zb_conf_get_extended_node_identifier (uint8_t *ni);
uint32_t calculate_checksum(char* data, int size);
void nvram_manager(void);
void zigbee_thread_manager(void);
uint32_t zb_get_mac_addr_high (void);
uint32_t zb_get_mac_addr_low (void);


#endif /* ZIGBEE_CONFIGURATION_H_ */







