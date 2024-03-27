/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef DIGI_NODE_DISCOVERY_H_
#define DIGI_NODE_DISCOVERY_H_

#define DIGI_NODE_DISCOVERY_REPLY_PAYLOAD_SIZE_MAX 60 //Considering that maximum size of node identifier is 32 chars

struct node_discovery_reply_t {
    bool    b_pending_request; // There is a pending node discovery request
    uint8_t first_character; // First character of the last node discovery request
    uint16_t max_reply_time_ms; // Maximum allowed time for sending response to the discovery request
    uint64_t time_request_ms; // Time at which the node discovery request was received
    uint64_t time_reply_ms; // Time at which the node discovery request will be replied.
    uint8_t at_ni[MAXIMUM_SIZE_NODE_IDENTIFIER+1];   // Node identifier string parameter
};

/* Function prototypes                                                        */
void digi_node_discovery_init(void);
bool is_a_digi_node_discovery_request(uint8_t* input_data, int16_t size_of_input_data);
bool digi_node_discovery_reply(void);
void digi_node_discovery_request_manager(void);

#endif /* DIGI_NODE_DISCOVERY_H_ */







