/**
  ******************************************************************************
  * @file    loader_flash.h
  * @brief   Checked flash erase/program/readback and APP record storage.
  *
  * Unlike the legacy Bsp/flash.c, every HAL call result is checked and every
  * programmed region is read back and compared before it is reported as written.
  ******************************************************************************
  */
#ifndef LOADER_FLASH_H
#define LOADER_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#include "loader_cfg.h"

typedef struct
{
	uint32_t magic; /* LDR_RECORD_MAGIC */
	uint32_t size; /* image size in bytes */
	uint32_t crc32; /* whole-image CRC over [LDR_APP_BASE, +size) */
	uint32_t version; /* host supplied monotonic version */
	uint32_t image_type; /* 1 = product */
	uint32_t record_crc; /* CRC32 over the first 20 bytes of this record */
} ldr_app_record_t;

/** Erase the APP record page (invalidates the stored boot record). */
bool loader_flash_erase_record(void);

/** Erase the whole APP region (48 pages). */
bool loader_flash_erase_app(void);

/** Program `length` bytes at LDR_APP_BASE + offset (length multiple of 8) and read back. */
bool loader_flash_program(uint32_t offset, const uint8_t *data, uint32_t length);

/** Read the stored record; validates magic and record CRC. */
bool loader_record_read(ldr_app_record_t *out);

/** Erase record page then write the record; verified by read-back. */
bool loader_record_write(const ldr_app_record_t *record);

#endif /* LOADER_FLASH_H */
