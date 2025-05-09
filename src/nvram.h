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
    DUFOTA_STATUS,
    DUFOTA_FW_VERSION,
    DUFOTA_FW_SIZE,
    ZB_CHECKSUM,
};
/**
 * The NVS_SECTOR_COUNT is set to 2 because we expect to write a maximum of once per day.
 * The total data written per day is approximately 115 bytes, which includes:
 * extended_pan_id: 16 bytes
 * at_ni: 29 bytes
 * network_link_key: 24 bytes
 * nvram_first_id: 14 bytes
 * nvram_first_id_expected: 14 bytes
 * number_restarts: 9 bytes
 * reset_reason: 9 bytes
 * This ensures sufficient storage capacity and longevity of the flash memory.
 * Calculates the expected device life in minutes based on the following parameters:
 *
 * @param NS The number of storage requests per minute.
 * @param DS The data size in bytes.
 * @param SECTOR_SIZE The size of a sector in bytes.
 * @param PAGE_ERASES The number of times a page can be erased.
 * 
 * The formula used for calculation is:
 * 
 * Expected device life (in minutes) = 
 * (SECTOR_COUNT * SECTOR_SIZE * PAGE_ERASES) / (NS * (DS + 8))
 * 
 * Example:
 * For the given values, the expected device life is approximately:
 * 2,056,712,348 minutes ≈ 3,912 years.
 */
#define NVS_SECTOR_COUNT          2U

uint8_t init_nvram(void);
uint8_t read_nvram(uint16_t id, uint8_t *data, size_t len);
void write_nvram(uint16_t id, uint8_t *data, size_t len);

#endif /* NVRAM_H_ */
