#include "parameter_store_flash.h"

#include <string.h>
#include "main.h"

#define PARAMETER_FLASH_PAGE_SIZE_BYTES 2048U
#define PARAMETER_FLASH_SLOT_SIZE_BYTES (4U * PARAMETER_FLASH_PAGE_SIZE_BYTES)
#define PARAMETER_FLASH_SLOT_0_ADDRESS  0x0801C000UL
#define PARAMETER_FLASH_SLOT_1_ADDRESS  0x0801E000UL

static const uint32_t ParameterSlotAddresses[PARAMETER_STORE_SLOT_COUNT] =
{
	PARAMETER_FLASH_SLOT_0_ADDRESS,
	PARAMETER_FLASH_SLOT_1_ADDRESS
};

static bool ParameterStoreFlash_ReadPreviousFormat(void *context,
	void *destination, uint32_t size_bytes);

static bool ParameterStoreFlash_Read(void *context, uint8_t slot,
	uint32_t offset, void *destination, uint32_t size_bytes)
{
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT || destination == 0 ||
		offset > PARAMETER_FLASH_SLOT_SIZE_BYTES ||
		size_bytes > PARAMETER_FLASH_SLOT_SIZE_BYTES - offset)
		return false;
	memcpy(destination, (const void *)(ParameterSlotAddresses[slot] + offset), size_bytes);
	return true;
}

static bool ParameterStoreFlash_Erase(void *context, uint8_t slot)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t page_error = 0U;
	bool result;
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT)
		return false;
	erase.Banks = FLASH_BANK_1;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page = (ParameterSlotAddresses[slot] - FLASH_BASE) / PARAMETER_FLASH_PAGE_SIZE_BYTES;
	erase.NbPages = PARAMETER_FLASH_SLOT_SIZE_BYTES / PARAMETER_FLASH_PAGE_SIZE_BYTES;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	result = HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
	HAL_FLASH_Lock();
	return result;
}

static bool ParameterStoreFlash_Program(void *context, uint8_t slot,
	uint32_t offset, const void *source, uint32_t size_bytes)
{
	const uint8_t *bytes = (const uint8_t *)source;
	uint32_t count;
	uint32_t index;
	bool result = true;
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT || source == 0 ||
		(offset & 7U) != 0U || offset > PARAMETER_FLASH_SLOT_SIZE_BYTES ||
		size_bytes > PARAMETER_FLASH_SLOT_SIZE_BYTES - offset)
		return false;
	count = (size_bytes + 7U) / 8U;
	HAL_FLASH_Unlock();
	for (index = 0U; index < count; ++index)
	{
		uint64_t value = UINT64_MAX;
		uint32_t byte_offset = index * 8U;
		uint32_t copy_size = size_bytes - byte_offset;
		if (copy_size > 8U)
			copy_size = 8U;
		memcpy(&value, &bytes[byte_offset], copy_size);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
			ParameterSlotAddresses[slot] + offset + byte_offset, value) != HAL_OK)
		{
			result = false;
			break;
		}
	}
	HAL_FLASH_Lock();
	return result;
}

ParameterStorePort ParameterStoreFlash_CreatePort(void)
{
	ParameterStorePort port = {0};
	port.context = 0;
	port.read = ParameterStoreFlash_Read;
	port.erase = ParameterStoreFlash_Erase;
	port.program = ParameterStoreFlash_Program;
	port.read_previous_format = ParameterStoreFlash_ReadPreviousFormat;
	return port;
}

static bool ParameterStoreFlash_ReadPreviousFormat(void *context,
	void *destination, uint32_t size_bytes)
{
	(void)context;
	if (destination == 0 || size_bytes > PARAMETER_FLASH_SLOT_SIZE_BYTES)
		return false;
	memcpy(destination, (const void *)PARAMETER_FLASH_SLOT_0_ADDRESS, size_bytes);
	return true;
}
