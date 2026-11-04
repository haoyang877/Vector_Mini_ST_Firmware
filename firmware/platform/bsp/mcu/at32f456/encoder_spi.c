/**
 * @file encoder_spi.c
 * @brief AT32编码器初始化与SPI/DMA直接采样。
 */
#include "../../bsp_internal.h"
#include "bsp_encoder.h"
#include "at32f45x.h"
#include "mcu_resources.h"
#include <stddef.h>

#define MT6835_FRAME_WORD_COUNT 3U

/* 位域按AT32小端半字的低位到高位声明；SPI仍从每个半字的最高位发送。 */
/** @brief 发送命令、寄存器地址，再发送两个空半字以读回数据。 */
typedef struct
{
    uint16_t address : 12; /**< 首半字[11:0]，角度起始地址0x003。 */
    uint16_t command : 4;  /**< 首半字[15:12]，连续读命令0xA。 */
    uint16_t dummy[2];     /**< 提供后续32个读取时钟。 */
} Mt6835TxFrame;

/** @brief 接收角度高低位、芯片状态和CRC，与三个DMA半字直接重合。 */
typedef struct
{
    uint16_t ignored;       /**< 命令阶段读回，丢弃。 */
    uint16_t angle_high;    /**< ANGLE[20:5]，每圈65536计数。 */
    uint16_t crc : 8;       /**< 末半字[7:0]。 */
    uint16_t status : 3;    /**< bit0超速，bit1弱磁场，bit2欠压。 */
    uint16_t angle_low : 5; /**< 末半字[15:11]，ANGLE[4:0]。 */
} Mt6835RxFrame;

/* DMA按半字连续读写，编译期拒绝填充或字段偏移变化。 */
typedef char Mt6835TxFrameLayoutCheck
    [(sizeof(Mt6835TxFrame) == 6U && offsetof(Mt6835TxFrame, dummy) == 2U) ? 1 : -1];
typedef char Mt6835RxFrameLayoutCheck[(sizeof(Mt6835RxFrame) == 6U &&
                                       offsetof(Mt6835RxFrame, ignored) == 0U &&
                                       offsetof(Mt6835RxFrame, angle_high) == 2U)
                                          ? 1
                                          : -1];

volatile Mt6835TxFrame s_encoder_tx;
volatile Mt6835RxFrame s_encoder_rx;

void McuEncoder_Init(void)
{
    crm_periph_clock_enable(BSP_ENCODER_GPIO_CLOCK, TRUE);
    crm_periph_clock_enable(MCU_ENCODER_SPI_CLOCK, TRUE);
    gpio_bits_set(BSP_ENCODER_GPIO_PORT, (1U << BSP_ENCODER_CS_PIN));
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_UP;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = (1U << BSP_ENCODER_CS_PIN);
    gpio_init(BSP_ENCODER_GPIO_PORT, &pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_pins =
        ((1U << BSP_ENCODER_SCK_PIN) | (1U << BSP_ENCODER_MISO_PIN) | (1U << BSP_ENCODER_MOSI_PIN));
    gpio_init(BSP_ENCODER_GPIO_PORT, &pins);
    gpio_pin_mux_config(BSP_ENCODER_GPIO_PORT,
                        (gpio_pins_source_type)BSP_ENCODER_SCK_PIN,
                        (gpio_mux_sel_type)BSP_ENCODER_GPIO_AF);
    gpio_pin_mux_config(BSP_ENCODER_GPIO_PORT,
                        (gpio_pins_source_type)BSP_ENCODER_MISO_PIN,
                        (gpio_mux_sel_type)BSP_ENCODER_GPIO_AF);
    gpio_pin_mux_config(BSP_ENCODER_GPIO_PORT,
                        (gpio_pins_source_type)BSP_ENCODER_MOSI_PIN,
                        (gpio_mux_sel_type)BSP_ENCODER_GPIO_AF);
    spi_i2s_reset(MCU_ENCODER_SPI);
    spi_init_type spi;
    spi_default_para_init(&spi);
    spi.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi.master_slave_mode = SPI_MODE_MASTER;
    spi.mclk_freq_division = SPI_MCLK_DIV_32;
    spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi.frame_bit_num = SPI_FRAME_16BIT;
    spi.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(MCU_ENCODER_SPI, &spi);
    spi_software_cs_internal_level_set(MCU_ENCODER_SPI, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi_enable(MCU_ENCODER_SPI, TRUE);
    crm_periph_clock_enable(MCU_ENCODER_DMA_CLOCK, TRUE);
    dma_reset(MCU_ENCODER_RX_DMA_CHANNEL);
    dma_reset(MCU_ENCODER_TX_DMA_CHANNEL);
    dma_init_type config;
    dma_default_para_init(&config);
    config.peripheral_base_addr = (uint32_t)(uintptr_t)&MCU_ENCODER_SPI->dt;
    config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    config.memory_base_addr = (uint32_t)(uintptr_t)&s_encoder_rx;
    config.buffer_size = MT6835_FRAME_WORD_COUNT;
    config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    config.memory_inc_enable = TRUE;
    config.priority = DMA_PRIORITY_HIGH;
    dma_init(MCU_ENCODER_RX_DMA_CHANNEL, &config);
    config.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    config.memory_base_addr = (uint32_t)(uintptr_t)&s_encoder_tx;
    config.priority = DMA_PRIORITY_MEDIUM;
    dma_init(MCU_ENCODER_TX_DMA_CHANNEL, &config);
    dmamux_enable(MCU_ENCODER_DMA, TRUE);
    dmamux_init(MCU_ENCODER_RX_DMA_MUX, MCU_ENCODER_RX_DMA_REQUEST);
    dmamux_init(MCU_ENCODER_TX_DMA_MUX, MCU_ENCODER_TX_DMA_REQUEST);
}

/* 保持MT6835片选建立/保持时间，不使用计时器或超时状态。 */
static void chip_select_delay(void)
{
    for (unsigned cycle = 0U; cycle < 36U; ++cycle)
    {
        __NOP();
    }
}

void BspEncoder_Begin(void)
{
    s_encoder_tx = (Mt6835TxFrame){.command = 0xAU, .address = 0x003U, .dummy = {0U, 0U}};
    MCU_ENCODER_DMA->clr = MCU_ENCODER_DMA_CLEAR_MASK;
    MCU_ENCODER_RX_DMA_CHANNEL->dtcnt = MT6835_FRAME_WORD_COUNT;
    MCU_ENCODER_TX_DMA_CHANNEL->dtcnt = MT6835_FRAME_WORD_COUNT;
    BSP_ENCODER_GPIO_PORT->clr = 1U << BSP_ENCODER_CS_PIN;
    chip_select_delay();
    __DMB();
    MCU_ENCODER_RX_DMA_CHANNEL->ctrl |= 1U;
    MCU_ENCODER_SPI->ctrl2 |= 1U;
    MCU_ENCODER_TX_DMA_CHANNEL->ctrl |= 1U;
    MCU_ENCODER_SPI->ctrl2 |= 2U;
}

uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution)
{
    while (MCU_ENCODER_RX_DMA_CHANNEL->dtcnt != 0U)
    {
    }
    while ((MCU_ENCODER_SPI->sts & SPI_I2S_BF_FLAG) != 0U)
    {
    }
    MCU_ENCODER_SPI->ctrl2 &= ~3U;
    MCU_ENCODER_TX_DMA_CHANNEL->ctrl &= ~1U;
    MCU_ENCODER_RX_DMA_CHANNEL->ctrl &= ~1U;
    chip_select_delay();
    BSP_ENCODER_GPIO_PORT->scr = 1U << BSP_ENCODER_CS_PIN;
    if (resolution == BSP_ENCODER_21BIT)
    {
        return ((uint32_t)s_encoder_rx.angle_high << 5U) | s_encoder_rx.angle_low;
    }
    /* 控制环继续使用高16位；两种格式均未检查状态或CRC。 */
    return s_encoder_rx.angle_high;
}
