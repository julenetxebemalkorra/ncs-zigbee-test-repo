/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef ZIGBEE_CONFIGURATION_H_
#define ZIGBEE_CONFIGURATION_H_

#include "global_defines.h"

struct zb_user_conf_t {
    uint64_t extended_pan_id; // Extended pan id
    uint8_t at_ni[MAXIMUM_SIZE_NODE_IDENTIFIER + 1];   // Node identifier string parameter (plus one to include the '\0')
};

/* Function prototypes (used only internally)                                 */

/* Function prototypes (used externally)                                      */
void zb_conf_read_from_nvram (void);
void zb_conf_copy_to_nvram (void);
uint64_t zb_conf_get_extended_pan_id (void);
void zb_conf_get_extended_node_identifier (uint8_t *ni);

#endif /* ZIGBEE_CONFIGURATION_H_ */







