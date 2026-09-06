#include "device_identity_stm32g431.h"

#include "stm32g4xx_hal.h"

static BspResult DeviceIdentityStm32G431_Read(void *context, uint8_t *buffer,
	size_t capacity, size_t *length)
{
	uint32_t words[3];
	uint8_t word_index;

	(void)context;
	if (buffer == 0 || length == 0 || capacity < sizeof(words))
		return BSP_RESULT_INVALID_ARGUMENT;
	words[0] = HAL_GetUIDw0();
	words[1] = HAL_GetUIDw1();
	words[2] = HAL_GetUIDw2();
	/* Preserve the deployed STM32 little-endian byte sequence explicitly. */
	for (word_index = 0U; word_index < 3U; ++word_index)
	{
		uint32_t word = words[word_index];
		buffer[(uint32_t)word_index * 4U] = (uint8_t)word;
		buffer[(uint32_t)word_index * 4U + 1U] = (uint8_t)(word >> 8U);
		buffer[(uint32_t)word_index * 4U + 2U] = (uint8_t)(word >> 16U);
		buffer[(uint32_t)word_index * 4U + 3U] = (uint8_t)(word >> 24U);
	}
	*length = sizeof(words);
	return BSP_RESULT_OK;
}

BspUniqueIdPort DeviceIdentityStm32G431_CreatePort(void)
{
	BspUniqueIdPort port;
	port.context = 0;
	port.read = DeviceIdentityStm32G431_Read;
	return port;
}
