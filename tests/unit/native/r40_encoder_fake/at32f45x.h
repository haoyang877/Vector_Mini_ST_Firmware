#ifndef R40_ENCODER_FAKE_H
#define R40_ENCODER_FAKE_H

#include "../r40_fake/at32f45x.h"

/* GPIO枚举的主机替身；取值由板配置提供。 */
typedef unsigned gpio_pins_source_type;
typedef unsigned gpio_mux_sel_type;

/* 沿用真实SPI声明；仅将寄存器地址与周期计数替换为离线对象。 */
#define __IO volatile
#include "at32f45x_spi.h"
#include "at32f45x_dma.h"
#undef SPI2
#define SPI2 (&sim_spi)
#undef DMA2
#undef DMA2_CHANNEL1
#undef DMA2_CHANNEL2
#undef DMA2MUX_CHANNEL1
#undef DMA2MUX_CHANNEL2
#define DMA2 (&sim_dma)
#define DMA2_CHANNEL1 (&sim_dma_rx)
#define DMA2_CHANNEL2 (&sim_dma_tx)
#define DMA2MUX_CHANNEL1 (&sim_rx_mux)
#define DMA2MUX_CHANNEL2 (&sim_tx_mux)
#define CRM_DMA2_PERIPH_CLOCK 7U
#define __DMB() ((void)0)
extern dma_type sim_dma;
extern dma_channel_type sim_dma_rx, sim_dma_tx;
extern dmamux_channel_type sim_rx_mux, sim_tx_mux;
#define GPIO_PINS_13 (1U << 13)
#define GPIO_PINS_14 (1U << 14)
#define GPIO_PINS_15 (1U << 15)
#define GPIO_PINS_SOURCE13 13U
#define GPIO_PINS_SOURCE14 14U
#define GPIO_PINS_SOURCE15 15U
#define GPIO_MUX_5 5U
#define CRM_SPI2_PERIPH_CLOCK 6U
#undef CoreDebug_DEMCR_TRCENA_Msk
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
#undef DWT
#undef CoreDebug
#define DWT (&sim_dwt)
#define CoreDebug (&sim_core_debug)
/** @brief 推进离线CPU周期，不操作实际寄存器。 */
void __NOP(void);
/** @brief 捕获本测试不允许的致命等待。 */
void __WFI(void);

#endif
