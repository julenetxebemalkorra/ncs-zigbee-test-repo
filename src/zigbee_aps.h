/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef ZIGBEE_APS_H_
#define ZIGBEE_APS_H_

#define APS_UNENCRYPTED_PAYLOAD_MAX 82
#define APS_PAYLOAD_MAX 255
#define APS_OUTPUT_FRAME_BUFFER_SIZE 8

typedef struct {
    zb_addr_u dst_addr;
    zb_uint16_t cluster_id;
    zb_uint8_t dst_endpoint;
    zb_uint8_t src_endpoint;
    zb_uint8_t payload[APS_PAYLOAD_MAX];
    zb_uint8_t payload_size;
} aps_output_frame_t;

typedef struct {
    aps_output_frame_t data[APS_OUTPUT_FRAME_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t free_space;
    uint16_t used_space;
} aps_output_frame_circular_buffer_t;

/* Function prototypes (used only internally)                                 */
void init_aps_output_frame_buffer(void);
bool enqueue_aps_frame(aps_output_frame_t *element);
bool dequeue_aps_frame(aps_output_frame_t *element);
void zigbee_aps_frame_scheduling_cb(zb_uint8_t param);

/* Function prototypes (used externally)                                      */
void zigbee_aps_init(void);
void zigbee_aps_user_data_tx_cb(zb_bufid_t bufid);
uint16_t zigbee_aps_get_output_frame_buffer_free_space(void);
void zigbee_aps_manager(void);

#endif /* ZIGBEE_APS_H_ */







