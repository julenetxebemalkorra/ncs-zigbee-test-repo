/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef NVRAM_H_
#define NVRAM_H_

// NVRAM partition where application data is stored in flash memory 
#define NVS_PARTITION		    storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

enum nvram_id_t {
    ZB_NVRAM_CHECK_ID,
    RBT_CNT_ID,
    RBT_CNT_REASON,
    ZB_EXT_PANID,
    ZB_NODE_IDENTIFIER,
    ZB_NETWORK_ENCRYPTION_KEY,
    ZB_CHECKSUM,
};

uint8_t init_nvram(void);
uint8_t read_nvram(uint16_t id, uint8_t *data, size_t len);
void write_nvram(uint16_t id, uint8_t *data, size_t len);

#endif /* NVRAM_H_ */
