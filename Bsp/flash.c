#include "flash.h"

#include <string.h>
#include "utils.h"
#include "foc_calibration.h"
#include "foc_param.h"
#include "heap.h"

#define PARAM_NUM 	sizeof(InterfaceParam_TypeDef) / sizeof(uint32_t)
/*double-word count for flash programming*/
#define PARAM_DW_NUM ((PARAM_NUM + 1) / 2) 

int num;

extern MotorControl_TypeDef MotorControl;
extern InterfaceParam_TypeDef InterfaceParam;

__ALIGNED(8) uint32_t p_param[PARAM_NUM];

float* struct_ptr = (float *)(&InterfaceParam);

/**
	* @brief  Get flash page number from address
	* @param  addr: flash address
	* @retval flash page number
 **/
static uint32_t get_page(uint32_t addr)
{
	return (addr / FLASH_BASE) / FLASH_PAGE_SIZE;
}

/**
	* @brief  Erase one flash page
	* @param  page: page number to erase
	* @retval erase result, true if success
 **/
bool flash_erase_page(uint8_t page)
{
	uint32_t page_error = 0;
	
	FLASH_EraseInitTypeDef EraseInit;
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	
	/*fill EraseInit structure*/
	EraseInit.Banks		  = FLASH_BANK_1;
	EraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInit.Page        = page;
	EraseInit.NbPages     = 1;
	
	if(HAL_FLASHEx_Erase(&EraseInit, &page_error) != HAL_OK) 
		return 0;
	
	HAL_FLASH_Lock();
	
	return 1;
}

/**
	* @brief  Erase flash pages between start and end addresses
	* @param  start_addr: start flash address
	* @param  end_addr: end flash address
	* @retval erase result, true if success
 **/
bool flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
{
	uint32_t page_error = 0;
	
	FLASH_EraseInitTypeDef EraseInit;
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	
	uint32_t FirstPage = get_page(start_addr);
	uint32_t NbofPages = get_page(end_addr) - FirstPage + 1;
	
	/*fill EraseInit structure*/
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.Page 		= FirstPage;
	EraseInit.NbPages	= NbofPages;
	
	if(HAL_FLASHEx_Erase(&EraseInit, &page_error) != HAL_OK)
		return 0;
	
	HAL_FLASH_Lock();
	
	return 1;
}

/**
	* @brief  Write data to flash
	* @param  addr: flash address
	* @param  *data: data buffer pointer
	* @param  size: double-word count to write
 **/
void flash_write_data(uint32_t addr, void *data, uint32_t size)
{
	uint64_t *buffer = (uint64_t *)data;
	uint32_t temp_addr = addr;
	
	HAL_FLASH_Unlock();
	
	for(uint32_t i = 0; i < size; i++)
	{
		/*64 bits once operation*/
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, temp_addr + i * 8, *buffer);
		buffer++;
	}
	
	HAL_FLASH_Lock();	
}

/**
	* @brief  Read data from flash
	* @param  addr: flash address
	* @param  *data: data buffer pointer
	* @param  size: byte count to read
 **/
void flash_read_data(uint32_t addr ,uint32_t *data, uint32_t size)
{
	memcpy(data, (uint32_t *)addr, size);
}

/**
	* @brief  Write interface parameters to flash
 **/
void flash_write_param(void)
{
	uint32_t flash_addr = ADDR_FLASH_PAGE_54;
	uint8_t flash_page = 54;
	
	Param_Upload();
	
//	if(p_param == NULL) 
//		p_param = HEAP_malloc(PARAM_NUM * sizeof(int));
	
	num = PARAM_NUM;
	
	memcpy(p_param, struct_ptr, (PARAM_NUM - 1) * sizeof(uint32_t));
	
	p_param[PARAM_NUM - 1] = (uint32_t)MAGIC_WORD;
	
	//flash_erase_pages(flash_addr, flash_addr + PARAM_NUM * 8);
	
	flash_erase_page(flash_page);
	flash_erase_page(flash_page + 1);
	flash_erase_page(flash_page + 2);
	flash_write_data(flash_addr, (uint64_t *)p_param, PARAM_DW_NUM);
	
//	if(p_param != NULL)
//	{
//		HEAP_free(p_param);
//		p_param = NULL;
//	}
}

/**
	* @brief  Read interface parameters from flash
 **/
void flash_read_param(void)
{
	uint32_t flash_addr = ADDR_FLASH_PAGE_54;
	
	//flash_read_data(flash_addr, (uint32_t *)param_read, PARAM_NUM * 8);

//	if(p_param == NULL) 
//		p_param = HEAP_malloc(PARAM_NUM * sizeof(int));
	
	memcpy(p_param, (uint32_t *)flash_addr, PARAM_NUM * sizeof(int));
	
	for(uint32_t i = 0; i < PARAM_NUM; i++)
	{
		struct_ptr[i] = IntBitToFloat(p_param[i]);
	}
	
	InterfaceParam.magic_word = (uint32_t)p_param[PARAM_NUM - 1];
	
//	if(p_param != NULL)
//	{
//		HEAP_free(p_param);
//		p_param = NULL;
//	}
	
	Param_Download();
}
