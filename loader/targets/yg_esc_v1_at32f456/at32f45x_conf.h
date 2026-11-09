#ifndef LOADER_AT32F45X_CONF_H
#define LOADER_AT32F45X_CONF_H
/* 独立Loader只装配时钟、GPIO、CAN、计时和Flash，不包含APP配置。 */
#define HEXT_VALUE 8000000U
#define HEXT_STARTUP_TIMEOUT 0x3000U
#define HICK_VALUE 8000000U
#define LEXT_VALUE 32768U
#define CRM_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define CAN_MODULE_ENABLED
#include "at32f45x_crm.h"
#include "at32f45x_flash.h"
#include "at32f45x_gpio.h"
#include "at32f45x_pwc.h"
#include "at32f45x_tmr.h"
#include "at32f45x_can.h"
#endif
