#ifndef PLATFORM_STM32G431_PARAMETER_STORE_FLASH_H
#define PLATFORM_STM32G431_PARAMETER_STORE_FLASH_H

#include <stdbool.h>
#include <stdint.h>
#include "parameter_store_port.h"

ParameterStorePort ParameterStoreFlash_CreatePort(void);

#endif
