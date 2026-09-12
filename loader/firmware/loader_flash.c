/**
  ******************************************************************************
  * @file    loader_flash.c
  * @brief   Checked flash operations for the resident Loader.
  ******************************************************************************
  */
#include "loader_flash.h"

#include <string.h>

#include "loader_crc.h"
#include "stm32g4xx_hal.h"

static bool loader_flash_erase_pages(uint32_t first_page, uint32_t page_count)
{
	FLASH_EraseInitTypeDef erase_init;
	uint32_t page_error = 0U;
	bool ok;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	erase_init.Banks = FLASH_BANK_1;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Page = first_page;
	erase_init.NbPages = page_count;
	ok = (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK);
	HAL_FLASH_Lock();
	return ok;
}

bool loader_flash_erase_record(void)
{
	return loader_flash_erase_pages(LDR_RECORD_PAGE, 1U);
}

bool loader_flash_erase_app(void)
{
	return loader_flash_erase_pages(LDR_APP_FIRST_PAGE, LDR_APP_PAGES);
}

bool loader_flash_program(uint32_t offset, const uint8_t *data, uint32_t length)
{
	uint32_t i;
	uint32_t doublewords;
	bool ok = true;
	const uint8_t *verify;

	if ((length == 0U) || ((length & 7U) != 0U))
	{
		return false;
	}
	if (offset > (LDR_APP_SIZE - length))
	{
		return false;
	}

	doublewords = length / 8U;
	HAL_FLASH_Unlock();
	for (i = 0U; i < doublewords; ++i)
	{
		uint64_t value = 0xFFFFFFFFFFFFFFFFULL;
		memcpy(&value, &data[i * 8U], 8U);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							  LDR_APP_BASE + offset + (i * 8U), value) != HAL_OK)
		{
			ok = false;
			break;
		}
	}
	HAL_FLASH_Lock();

	if (!ok)
	{
		return false;
	}

	/* Read-back verification: flash is memory mapped. */
	verify = (const uint8_t *)(LDR_APP_BASE + offset);
	return (memcmp(verify, data, length) == 0);
}

bool loader_record_read(ldr_app_record_t *out)
{
	ldr_app_record_t tmp;
	uint32_t stored_crc;
	uint32_t computed_crc;

	memcpy(&tmp, (const void *)LDR_RECORD_ADDR, sizeof(tmp));
	if (tmp.magic != LDR_RECORD_MAGIC)
	{
		return false;
	}

	stored_crc = tmp.record_crc;
	tmp.record_crc = 0U;
	computed_crc = loader_crc32((const uint8_t *)&tmp, 20U);
	tmp.record_crc = stored_crc;
	if (stored_crc != computed_crc)
	{
		return false;
	}

	*out = tmp;
	return true;
}

bool loader_record_write(const ldr_app_record_t *record)
{
	ldr_app_record_t local = *record;
	uint32_t i;
	uint64_t value;

	local.record_crc = 0U;
	local.record_crc = loader_crc32((const uint8_t *)&local, 20U);

	if (!loader_flash_erase_record())
	{
		return false;
	}

	HAL_FLASH_Unlock();
	for (i = 0U; i < 3U; ++i)
	{
		memcpy(&value, ((const uint8_t *)&local) + (i * 8U), 8U);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							  LDR_RECORD_ADDR + (i * 8U), value) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return false;
		}
	}
	HAL_FLASH_Lock();

	return (memcmp((const void *)LDR_RECORD_ADDR, &local, sizeof(local)) == 0);
}
