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

#define ZB_NVRAM_CHECK_ID 1
#define RBT_CNT_ID 2
#define RBT_CNT_REASON 3
#define ZB_EXT_PANID 4
#define ZB_NODE_IDENTIFIER 5
#define ZB_CHECKSUM 6


uint8_t init_nvram(void);
uint8_t read_nvram(uint16_t id, uint8_t *data, size_t len);
void write_nvram(uint16_t id, uint8_t *data, size_t len);
size_t calc_free_space(void);

#endif /* NVRAM_H_ */
