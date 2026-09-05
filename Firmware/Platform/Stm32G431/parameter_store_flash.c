#include "parameter_store_flash.h"

#include <string.h>
#include "main.h"
#include "memory_layout_profile.h"

static uint32_t ParameterStoreFlash_GetSlotAddress(uint8_t slot)
{
	const MemoryLayoutProfile *layout = MemoryLayoutProfile_GetActive();
	return slot == 0U ? layout->parameter_slot_0_address :
		layout->parameter_slot_1_address;
}

static bool ParameterStoreFlash_ReadPreviousFormat(void *context,
	void *destination, uint32_t size_bytes);

static bool ParameterStoreFlash_Read(void *context, uint8_t slot,
	uint32_t offset, void *destination, uint32_t size_bytes)
{
	const MemoryLayoutProfile *layout = MemoryLayoutProfile_GetActive();
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT || destination == 0 ||
		offset > layout->parameter_slot_size_bytes ||
		size_bytes > layout->parameter_slot_size_bytes - offset)
		return false;
	memcpy(destination, (const void *)(ParameterStoreFlash_GetSlotAddress(slot) +
		offset), size_bytes);
	return true;
}

static bool ParameterStoreFlash_Erase(void *context, uint8_t slot)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t page_error = 0U;
	bool result;
	const MemoryLayoutProfile *layout = MemoryLayoutProfile_GetActive();
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT)
		return false;
	erase.Banks = FLASH_BANK_1;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page = (ParameterStoreFlash_GetSlotAddress(slot) -
		layout->flash_base_address) / layout->flash_page_size_bytes;
	erase.NbPages = layout->parameter_slot_size_bytes /
		layout->flash_page_size_bytes;
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
	const MemoryLayoutProfile *layout = MemoryLayoutProfile_GetActive();
	(void)context;
	if (slot >= PARAMETER_STORE_SLOT_COUNT || source == 0 ||
		(offset & 7U) != 0U || offset > layout->parameter_slot_size_bytes ||
		size_bytes > layout->parameter_slot_size_bytes - offset)
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
			ParameterStoreFlash_GetSlotAddress(slot) + offset + byte_offset,
			value) != HAL_OK)
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
	const MemoryLayoutProfile *layout = MemoryLayoutProfile_GetActive();
	(void)context;
	if (destination == 0 || size_bytes > layout->parameter_slot_size_bytes)
		return false;
	memcpy(destination, (const void *)layout->parameter_slot_0_address,
		size_bytes);
	return true;
}
