#include "device_identity_stm32g431.h"

#include "stm32g4xx_hal.h"

static bool DeviceIdentityStm32G431_ReadWords(void *context,
	uint32_t words[DEVICE_IDENTITY_WORD_COUNT])
{
	(void)context;
	if (words == 0)
		return false;
	words[0] = HAL_GetUIDw0();
	words[1] = HAL_GetUIDw1();
	words[2] = HAL_GetUIDw2();
	return true;
}

DeviceIdentityPort DeviceIdentityStm32G431_CreatePort(void)
{
	DeviceIdentityPort port;
	port.context = 0;
	port.read_words = DeviceIdentityStm32G431_ReadWords;
	return port;
}
