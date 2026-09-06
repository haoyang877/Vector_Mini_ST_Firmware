#ifndef PLATFORM_STM32G431_PARAMETER_STORE_FLASH_H
#define PLATFORM_STM32G431_PARAMETER_STORE_FLASH_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_system.h"

typedef struct
{
	uint32_t flash_base_address;
	uint32_t flash_capacity_bytes;
	uint32_t storage_base_address;
	uint32_t capacity_bytes;
	uint32_t erase_size_bytes;
	uint32_t program_alignment_bytes;
} ParameterStoreFlashResourceConfig;

typedef struct
{
	const ParameterStoreFlashResourceConfig *resources;
} ParameterStoreFlashContext;

bool ParameterStoreFlash_CreatePort(ParameterStoreFlashContext *context,
	const ParameterStoreFlashResourceConfig *resources,
	BspNonvolatileStoragePort *port);

#endif
