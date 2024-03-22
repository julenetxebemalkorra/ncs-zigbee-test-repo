/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef NVRAM_H_
#define NVRAM_H_

uint8_t init_nvram(void);
uint8_t read_nvram_PAN_ID(uint64_t *panid, size_t size);
uint8_t read_nvram_NI(uint8_t *ni, size_t size);
void write_nvram_PAN_ID(uint64_t *panid, size_t size);
void write_nvram_NI(uint8_t *ni, size_t size);

#endif /* NVRAM_H_ */
