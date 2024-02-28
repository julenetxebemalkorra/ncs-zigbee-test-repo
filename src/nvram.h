/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef NVRAM_H_
#define NVRAM_H_

uint8_t xbee_nvram_write_app_data(uint8_t page, uint32_t pos);
void xbee_nvram_read_app_data(uint8_t page, uint32_t pos, uint16_t payload_length);
uint16_t xbee_get_nvram_data_size();

#endif /* NVRAM_H_ */







