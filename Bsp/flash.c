#include "flash.h"

#include <string.h>
#include "foc_param.h"
#include "heap.h"

#define PARAM_FLASH_ADDR ADDR_FLASH_PAGE_56

/**
	* @brief  Get flash page number from address
	* @param  addr: flash address
	* @retval flash page number
 **/
static uint32_t get_page(uint32_t addr)
{
	return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/**
	* @brief  Erase one flash page
	* @param  page: page number
	* @retval erase result, true if success
 **/
bool flash_erase_page(uint8_t page)
{
	uint32_t page_error = 0U;
	FLASH_EraseInitTypeDef erase_init;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	erase_init.Banks = FLASH_BANK_1;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Page = page;
	erase_init.NbPages = 1U;
	bool result = HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK;
	HAL_FLASH_Lock();
	return result;
}

/**
	* @brief  Erase flash pages between start and end addresses
	* @param  start_addr: start flash address
	* @param  end_addr: end flash address
	* @retval erase result, true if success
 **/
bool flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
{
	uint32_t page_error = 0U;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t first_page = get_page(start_addr);
	uint32_t page_count = get_page(end_addr) - first_page + 1U;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	erase_init.Banks = FLASH_BANK_1;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Page = first_page;
	erase_init.NbPages = page_count;
	bool result = HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK;
	HAL_FLASH_Lock();
	return result;
}

/**
	* @brief  Write data to flash
	* @param  addr: flash address
	* @param  data: data buffer pointer
	* @param  size_bytes: byte count to write
 **/
static void flash_write_data(uint32_t addr, const void *data, uint32_t size_bytes)
{
	const uint8_t *bytes = (const uint8_t *)data;
	uint32_t doubleword_count = (size_bytes + 7U) / 8U;

	HAL_FLASH_Unlock();
	for (uint32_t i = 0U; i < doubleword_count; ++i)
	{
		uint64_t value = UINT64_MAX;
		uint32_t offset = i * 8U;
		uint32_t copy_size = size_bytes - offset;
		if (copy_size > 8U)
			copy_size = 8U;
		memcpy(&value, &bytes[offset], copy_size);
		(void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, value);
	}
	HAL_FLASH_Lock();
}

/**
	* @brief  Write interface parameters to flash
 **/
void flash_write_param(void)
{
	InterfaceParam_TypeDef *param = HEAP_malloc(sizeof(*param));
	if (param == NULL)
		return;

	Param_Upload(param);
	param->magic_word = MAGIC_WORD;
	if (flash_erase_pages(PARAM_FLASH_ADDR, PARAM_FLASH_ADDR + sizeof(*param) - 1U))
		flash_write_data(PARAM_FLASH_ADDR, param, sizeof(*param));

	HEAP_free(param);
}

/**
	* @brief  Read interface parameters from flash
 **/
void flash_read_param(void)
{
	InterfaceParam_TypeDef *param = HEAP_malloc(sizeof(*param));
	if (param == NULL)
	{
		Param_Return_Default();
		return;
	}

	memcpy(param, (const void *)PARAM_FLASH_ADDR, sizeof(*param));
	Param_Download(param);
	HEAP_free(param);
}
