#include "parameter_store_flash.h"

#include <limits.h>
#include <string.h>

#include "main.h"

static bool ParameterStoreFlash_RangeIsValid(
	const ParameterStoreFlashResourceConfig *resources, uint32_t offset,
	size_t length)
{
	return resources != 0 && length <= UINT32_MAX &&
		offset <= resources->capacity_bytes &&
		(uint32_t)length <= resources->capacity_bytes - offset;
}

static bool ParameterStoreFlash_ResourcesAreValid(
	const ParameterStoreFlashResourceConfig *resources)
{
	uint32_t storage_end;

	if (resources == 0 || resources->flash_base_address == 0U ||
		resources->flash_capacity_bytes == 0U ||
		resources->storage_base_address < resources->flash_base_address ||
		resources->capacity_bytes == 0U || resources->erase_size_bytes == 0U ||
		resources->program_alignment_bytes != 8U ||
		(resources->storage_base_address - resources->flash_base_address) %
			resources->erase_size_bytes != 0U ||
		resources->capacity_bytes % resources->erase_size_bytes != 0U)
	{
		return false;
	}
	storage_end = resources->storage_base_address + resources->capacity_bytes;
	return storage_end >= resources->storage_base_address &&
		resources->storage_base_address - resources->flash_base_address <=
			resources->flash_capacity_bytes &&
		resources->capacity_bytes <= resources->flash_capacity_bytes -
			(resources->storage_base_address - resources->flash_base_address);
}

static const ParameterStoreFlashResourceConfig *ParameterStoreFlash_Resources(
	void *context)
{
	ParameterStoreFlashContext *adapter = (ParameterStoreFlashContext *)context;
	return adapter != 0 ? adapter->resources : 0;
}

static BspResult ParameterStoreFlash_Read(void *context, uint32_t offset,
	void *destination, size_t length)
{
	const ParameterStoreFlashResourceConfig *resources =
		ParameterStoreFlash_Resources(context);

	if (destination == 0 ||
		!ParameterStoreFlash_RangeIsValid(resources, offset, length))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	memcpy(destination, (const void *)(resources->storage_base_address + offset),
		length);
	return BSP_RESULT_OK;
}

static BspResult ParameterStoreFlash_Erase(void *context, uint32_t offset,
	size_t length)
{
	const ParameterStoreFlashResourceConfig *resources =
		ParameterStoreFlash_Resources(context);
	FLASH_EraseInitTypeDef erase;
	uint32_t page_error = 0U;
	bool erased;

	if (!ParameterStoreFlash_RangeIsValid(resources, offset, length) ||
		length == 0U || offset % resources->erase_size_bytes != 0U ||
		(uint32_t)length % resources->erase_size_bytes != 0U)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	erase.Banks = FLASH_BANK_1;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page = (resources->storage_base_address + offset -
		resources->flash_base_address) / resources->erase_size_bytes;
	erase.NbPages = (uint32_t)length / resources->erase_size_bytes;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	erased = HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
	HAL_FLASH_Lock();
	return erased ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

static BspResult ParameterStoreFlash_Program(void *context, uint32_t offset,
	const void *source, size_t length)
{
	const ParameterStoreFlashResourceConfig *resources =
		ParameterStoreFlash_Resources(context);
	const uint8_t *bytes = (const uint8_t *)source;
	uint32_t count;
	uint32_t index;
	bool programmed = true;

	if (source == 0 ||
		!ParameterStoreFlash_RangeIsValid(resources, offset, length) ||
		(offset % resources->program_alignment_bytes) != 0U ||
		length > UINT32_MAX - (resources->program_alignment_bytes - 1U))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	count = ((uint32_t)length + resources->program_alignment_bytes - 1U) /
		resources->program_alignment_bytes;
	if (count > (resources->capacity_bytes - offset) /
		resources->program_alignment_bytes)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	HAL_FLASH_Unlock();
	for (index = 0U; index < count; ++index)
	{
		uint64_t value = UINT64_MAX;
		uint32_t byte_offset = index * resources->program_alignment_bytes;
		uint32_t copy_size = (uint32_t)length - byte_offset;

		if (copy_size > resources->program_alignment_bytes)
			copy_size = resources->program_alignment_bytes;
		memcpy(&value, &bytes[byte_offset], copy_size);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
			resources->storage_base_address + offset + byte_offset,
			value) != HAL_OK)
		{
			programmed = false;
			break;
		}
	}
	HAL_FLASH_Lock();
	return programmed ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

bool ParameterStoreFlash_CreatePort(ParameterStoreFlashContext *context,
	const ParameterStoreFlashResourceConfig *resources,
	BspNonvolatileStoragePort *port)
{
	if (context == 0 || port == 0 ||
		!ParameterStoreFlash_ResourcesAreValid(resources))
	{
		return false;
	}
	context->resources = resources;
	port->context = context;
	port->geometry.capacity_bytes = resources->capacity_bytes;
	port->geometry.erase_size_bytes = resources->erase_size_bytes;
	port->geometry.program_alignment_bytes =
		resources->program_alignment_bytes;
	port->read = ParameterStoreFlash_Read;
	port->erase = ParameterStoreFlash_Erase;
	port->program = ParameterStoreFlash_Program;
	return true;
}
