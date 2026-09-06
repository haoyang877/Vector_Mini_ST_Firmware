#ifndef PLATFORM_STM32G431_BOARD_RUNTIME_H
#define PLATFORM_STM32G431_BOARD_RUNTIME_H

#include <stdbool.h>
#include "bsp_system.h"

bool BoardRuntimeStm32G431_Start(void);
BspCriticalSectionPort BoardRuntimeStm32G431_CreateCriticalSectionPort(void);

#endif
