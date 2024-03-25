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
#define ZB_EXT_PANID 2
#define ZB_NODE_IDENTIFIER 3

uint8_t init_nvram(void);
uint8_t read_nvram_first_id(uint8_t *nvram_first_id, size_t nvram_first_id_size);
uint8_t read_nvram_PAN_ID(uint64_t *panid, size_t size);
uint8_t read_nvram_NI(uint8_t *ni, size_t size);
void write_nvram_PAN_ID(uint64_t *panid, size_t size);
void write_nvram_NI(uint8_t *ni, size_t size);

#endif /* NVRAM_H_ */
