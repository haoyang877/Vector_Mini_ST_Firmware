/**
 * @file encoder_spi.c
 * @brief AT32编码器初始化与SPI/DMA直接采样。
 */
#include "../../bsp_internal.h"
#include "bsp_encoder.h"
#include "at32f45x.h"
#include "board_config.h"

static volatile uint16_t s_encoder_tx[3];
static volatile uint16_t s_encoder_rx[3];

void McuEncoder_Init(void)
{
    crm_periph_clock_enable(BSP_ENCODER_GPIO_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
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
    spi_i2s_reset(SPI2);
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
    spi_init(SPI2, &spi);
    spi_software_cs_internal_level_set(SPI2, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi_enable(SPI2, TRUE);
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    dma_reset(DMA2_CHANNEL1);
    dma_reset(DMA2_CHANNEL2);
    dma_init_type config;
    dma_default_para_init(&config);
    config.peripheral_base_addr = (uint32_t)(uintptr_t)&SPI2->dt;
    config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    config.memory_base_addr = (uint32_t)(uintptr_t)s_encoder_rx;
    config.buffer_size = 3U;
    config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    config.memory_inc_enable = TRUE;
    config.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA2_CHANNEL1, &config);
    config.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    config.memory_base_addr = (uint32_t)(uintptr_t)s_encoder_tx;
    config.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA2_CHANNEL2, &config);
    dmamux_enable(DMA2, TRUE);
    dmamux_init(DMA2MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI2_RX);
    dmamux_init(DMA2MUX_CHANNEL2, DMAMUX_DMAREQ_ID_SPI2_TX);
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
    s_encoder_tx[0] = 0xA003U;
    s_encoder_tx[1] = 0U;
    s_encoder_tx[2] = 0U;
    DMA2->clr = 0xFFU;
    DMA2_CHANNEL1->dtcnt = 3U;
    DMA2_CHANNEL2->dtcnt = 3U;
    BSP_ENCODER_GPIO_PORT->clr = 1U << BSP_ENCODER_CS_PIN;
    chip_select_delay();
    __DMB();
    DMA2_CHANNEL1->ctrl |= 1U;
    SPI2->ctrl2 |= 1U;
    DMA2_CHANNEL2->ctrl |= 1U;
    SPI2->ctrl2 |= 2U;
}

uint16_t BspEncoder_ReadAngle(void)
{
    while (DMA2_CHANNEL1->dtcnt != 0U)
    {
    }
    while ((SPI2->sts & SPI_I2S_BF_FLAG) != 0U)
    {
    }
    SPI2->ctrl2 &= ~3U;
    DMA2_CHANNEL2->ctrl &= ~1U;
    DMA2_CHANNEL1->ctrl &= ~1U;
    chip_select_delay();
    BSP_ENCODER_GPIO_PORT->scr = 1U << BSP_ENCODER_CS_PIN;
    return s_encoder_rx[1];
}
