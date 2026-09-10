#include "../../../api/time_hw.h"
#include "stm32g4xx_hal.h"
uint32_t time_hw_now_ms(void) { return HAL_GetTick(); }
