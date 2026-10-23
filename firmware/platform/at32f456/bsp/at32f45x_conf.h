#ifndef R40_AT32F45X_CONF_H
#define R40_AT32F45X_CONF_H

/* 仅启用本阶段使用的厂商外设模块；不复制基础工程的自动启相初始化。 */
#define HEXT_VALUE 8000000U
#define HEXT_STARTUP_TIMEOUT 0x3000U
#define HICK_VALUE 8000000U
#define LEXT_VALUE 32768U
#define CRM_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define ADC_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define CAN_MODULE_ENABLED
#define SPI_MODULE_ENABLED

#include "at32f45x_crm.h"
#include "at32f45x_flash.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_pwc.h"
#include "at32f45x_tmr.h"
#include "at32f45x_adc.h"
#include "at32f45x_dma.h"
#include "at32f45x_can.h"
#include "at32f45x_spi.h"

#endif
