#ifndef R40_ENCODER_FAKE_H
#define R40_ENCODER_FAKE_H

#include "../r40_fake/at32f45x.h"

/* 沿用真实SPI声明；仅将寄存器地址与周期计数替换为离线对象。 */
#define __IO volatile
#include "at32f45x_spi.h"
#undef SPI2
#define SPI2 (&sim_spi)
#define GPIO_PINS_13 (1U << 13)
#define GPIO_PINS_14 (1U << 14)
#define GPIO_PINS_15 (1U << 15)
#define GPIO_PINS_SOURCE13 13U
#define GPIO_PINS_SOURCE14 14U
#define GPIO_PINS_SOURCE15 15U
#define GPIO_MUX_5 5U
#define CRM_SPI2_PERIPH_CLOCK 6U
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U

typedef struct
{
    uint32_t CTRL, CYCCNT;
} SimDwt;
typedef struct
{
    uint32_t DEMCR;
} SimCoreDebug;
extern spi_type sim_spi;
extern SimDwt sim_dwt;
extern SimCoreDebug sim_core_debug;
#define DWT (&sim_dwt)
#define CoreDebug (&sim_core_debug)
/** @brief 推进离线CPU周期，不操作实际寄存器。 */
void __NOP(void);
/** @brief 捕获本测试不允许的致命等待。 */
void __WFI(void);

#endif
