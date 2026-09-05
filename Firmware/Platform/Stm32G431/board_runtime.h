#ifndef PLATFORM_STM32G431_BOARD_RUNTIME_H
#define PLATFORM_STM32G431_BOARD_RUNTIME_H

#include <stdbool.h>
#include "critical_section_port.h"

bool BoardRuntimeStm32G431_Start(void);
CriticalSectionPort BoardRuntimeStm32G431_CreateCriticalSectionPort(void);

#endif
