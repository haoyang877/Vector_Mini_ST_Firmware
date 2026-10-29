#include "encoder_spi.h"

#include "at32f45x.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_config.h"
#include "at32f45x_dma.h"

/* 本板独占SPI2的四线传输：管理CS、字长、DMA、所有权及全帧期限。
 * 命令与地址来自器件驱动；本层不解释角度、状态位、CRC或配置寄存器。 */
#define ENCODER_SPI_ERRORS (SPI_MMERR_FLAG | SPI_I2S_ROERR_FLAG | SPI_CCERR_FLAG)
#define ENCODER_SPI_TIMEOUT_CYCLES (R40_CORE_HZ / 1000000U * BSP_ENCODER_TIMEOUT_US)
#define ENCODER_SPI_POLL_LIMIT 4096U
#define ENCODER_SPI_STAGE_DATA 2U
#define ENCODER_SPI_STAGE_FAILED 255U

/* 完整APP显式选择11.25MHz；基础目标保持5.625MHz，均低于MT6835的16MHz上限。 */
#ifndef BSP_ENCODER_FAST_SPI
#define BSP_ENCODER_FAST_SPI 0
#endif
#ifndef BSP_ENCODER_WORD_TRANSFER
#define BSP_ENCODER_WORD_TRANSFER 0
#endif
#ifndef BSP_ENCODER_DMA_TRANSFER
#define BSP_ENCODER_DMA_TRANSFER 0
#endif
#if BSP_ENCODER_DMA_TRANSFER && !BSP_ENCODER_WORD_TRANSFER
#error "DMA samples require 16-bit SPI frames"
#endif
#if BSP_ENCODER_WORD_TRANSFER != 0 && BSP_ENCODER_WORD_TRANSFER != 1
#error "R40 MT6835 word transfer selection must be 0 or 1"
#endif
#if BSP_ENCODER_FAST_SPI != 0 && BSP_ENCODER_FAST_SPI != 1
#error "R40 MT6835 SPI selection must be 0 or 1"
#endif
#if BSP_ENCODER_FAST_SPI && R40_CORE_HZ / 16U > BSP_ENCODER_SPI_MAX_HZ
#error "MT6835 SPI exceeds the 16MHz device limit"
#endif

typedef enum
{
    ENCODER_SPI_OWNER_NONE,
    ENCODER_SPI_OWNER_SAMPLE,
    ENCODER_SPI_OWNER_DIAGNOSTIC
} EncoderSpiOwner;

static volatile EncoderSpiOwner s_owner;
static bool s_initialized;
static uint8_t s_stage;
static uint32_t s_started_cycles;
static uint32_t s_polls;
static bool s_word_frame;
static uint8_t s_command_low;
#if BSP_ENCODER_DMA_TRANSFER
/* DMA2与ADC的DMA1完全分离；三半字保持A0 03及后续32位读回的线序。 */
static uint16_t s_dma_tx[3];
static volatile uint16_t s_dma_rx[3];
static bool s_dma_frame;

static void sample_dma_stop(void)
{
    /* CTRL2低两位为RX/TX DMA请求；合并关闭后仍先停TX通道，再停RX通道。 */
    SPI2->ctrl2 &= ~3U;
    dma_channel_enable(DMA2_CHANNEL2, FALSE);
    dma_channel_enable(DMA2_CHANNEL1, FALSE);
}

static void sample_dma_init(void)
{
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    dma_reset(DMA2_CHANNEL1);
    dma_reset(DMA2_CHANNEL2);
    dma_init_type config;
    dma_default_para_init(&config);
    config.peripheral_base_addr = (uint32_t)(uintptr_t)&SPI2->dt;
    config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    config.memory_base_addr = (uint32_t)(uintptr_t)s_dma_rx;
    config.buffer_size = 3U;
    config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    config.memory_inc_enable = TRUE;
    config.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA2_CHANNEL1, &config);
    config.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    config.memory_base_addr = (uint32_t)(uintptr_t)s_dma_tx;
    config.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA2_CHANNEL2, &config);
    dmamux_enable(DMA2, TRUE);
    dmamux_init(DMA2MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI2_RX);
    dmamux_init(DMA2MUX_CHANNEL2, DMAMUX_DMAREQ_ID_SPI2_TX);
}

static void sample_dma_start(void)
{
    dma_flag_clear(DMA2_GL1_FLAG | DMA2_GL2_FLAG);
    DMA2_CHANNEL1->maddr = (uint32_t)(uintptr_t)s_dma_rx;
    DMA2_CHANNEL2->maddr = (uint32_t)(uintptr_t)s_dma_tx;
    dma_data_number_set(DMA2_CHANNEL1, 3U);
    dma_data_number_set(DMA2_CHANNEL2, 3U);
    dma_channel_enable(DMA2_CHANNEL1, TRUE);
    spi_i2s_dma_receiver_enable(SPI2, TRUE);
    dma_channel_enable(DMA2_CHANNEL2, TRUE);
    spi_i2s_dma_transmitter_enable(SPI2, TRUE);
}

static bool sample_dma_complete(void)
{
    /* SDK标志含DMA2选择位，不能将它作为原始STS中的错误或完成位。 */
    const uint32_t error_mask = (DMA2_DTERR1_FLAG | DMA2_DTERR2_FLAG) & 0x0FFFFFFFU;
    const uint32_t complete_mask = (DMA2_FDT1_FLAG | DMA2_FDT2_FLAG) & 0x0FFFFFFFU;
    while (++s_polls <= ENCODER_SPI_POLL_LIMIT &&
           (uint32_t)(DWT->CYCCNT - s_started_cycles) < ENCODER_SPI_TIMEOUT_CYCLES)
    {
        if (spi_i2s_flag_get(SPI2, ENCODER_SPI_ERRORS) != RESET)
        {
            return false;
        }
        uint32_t status = DMA2->sts;
        if ((status & error_mask) != 0U)
        {
            return false;
        }
        if ((status & complete_mask) == complete_mask)
        {
            __DMB();
            return true;
        }
    }
    return false;
}
#endif

static void cs_guard(void)
{
    /* 至少36个内核周期：180MHz时>=200ns，覆盖CS建立100ns和保持半个SCK。
     * 按DWT计时，避免36次循环的分支开销扩大等待；计数器停走仍有固定迭代上界。 */
    uint32_t started = DWT->CYCCNT;
    for (uint32_t i = 0U; i < 36U && (uint32_t)(DWT->CYCCNT - started) < 36U; ++i)
    {
        __NOP();
    }
}

static void spi_configure(void)
{
    spi_i2s_reset(SPI2);
    spi_init_type config;
    spi_default_para_init(&config);
    config.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    config.master_slave_mode = SPI_MODE_MASTER;
    config.mclk_freq_division = BSP_ENCODER_FAST_SPI ? SPI_MCLK_DIV_16 : SPI_MCLK_DIV_32;
    config.first_bit_transmission = SPI_FIRST_BIT_MSB;
    config.frame_bit_num = SPI_FRAME_8BIT;
    config.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    config.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    config.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI2, &config);
    spi_software_cs_internal_level_set(SPI2, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi_enable(SPI2, TRUE);
}

static bool bus_ready(void)
{
    return SPI2->ctrl1_bit.spien &&
           spi_i2s_flag_get(SPI2, ENCODER_SPI_ERRORS | SPI_I2S_BF_FLAG | SPI_I2S_RDBF_FLAG) ==
               RESET &&
           spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) != RESET;
}

static bool start_frame(EncoderSpiOwner owner, const EncoderSpiRequest *request)
{
    uint32_t state = BspSystem_EnterCritical();
    if (!s_initialized || s_owner != ENCODER_SPI_OWNER_NONE)
    {
        BspSystem_ExitCritical(state);
        return false;
    }
    if (!bus_ready())
    {
        /* 异常残留只清理本端口独占的SPI2；本拍失败，下拍再尝试。 */
        spi_configure();
        BspSystem_ExitCritical(state);
        return false;
    }
    s_word_frame = BSP_ENCODER_WORD_TRANSFER && owner == ENCODER_SPI_OWNER_SAMPLE;
    spi_frame_bit_num_type frame_bits = s_word_frame ? SPI_FRAME_16BIT : SPI_FRAME_8BIT;
    if (SPI2->ctrl1_bit.fbn != (uint32_t)frame_bits)
    {
        /* 只在总线空闲且无所有者时改字长；诊断寄存器读取保留原8位事务。 */
        spi_enable(SPI2, FALSE);
        spi_frame_bit_num_set(SPI2, frame_bits);
        spi_enable(SPI2, TRUE);
    }
    s_owner = owner;
    s_stage = 1U;
    s_polls = 0U;
    s_started_cycles = DWT->CYCCNT;
    s_command_low = (uint8_t)request->command;
    gpio_bits_reset(BSP_ENCODER_GPIO_PORT, (1U << BSP_ENCODER_CS_PIN));
    cs_guard();
#if BSP_ENCODER_DMA_TRANSFER
    s_dma_frame = owner == ENCODER_SPI_OWNER_SAMPLE;
    if (s_dma_frame)
    {
        s_dma_tx[0] = request->command;
        sample_dma_start();
    }
    else
#endif
    {
        /* 两字节请求由器件驱动提供；8位路径分段发送，16位路径一次发送。 */
        spi_i2s_data_transmit(SPI2, s_word_frame ? request->command : request->command >> 8);
    }
    BspSystem_ExitCritical(state);
    return true;
}

static bool wait_flag(uint32_t flag, bool set)
{
    /* 全帧共用目标配置的周期预算及轮询次数上限；DWT停走时也不会无限等待。
     * 预算包含begin/readback/complete之间的计算，超预算样本直接丢弃。 */
    while (++s_polls <= ENCODER_SPI_POLL_LIMIT &&
           (uint32_t)(DWT->CYCCNT - s_started_cycles) < ENCODER_SPI_TIMEOUT_CYCLES)
    {
        if (spi_i2s_flag_get(SPI2, ENCODER_SPI_ERRORS) != RESET)
        {
            return false;
        }
        if ((spi_i2s_flag_get(SPI2, flag) != RESET) == set)
        {
            return true;
        }
    }
    return false;
}

static bool receive_byte(uint8_t *value)
{
    if (!wait_flag(SPI_I2S_RDBF_FLAG, true))
    {
        return false;
    }
    *value = (uint8_t)spi_i2s_data_receive(SPI2);
    return true;
}

static bool receive_word(uint16_t *value)
{
    if (!wait_flag(SPI_I2S_RDBF_FLAG, true))
    {
        return false;
    }
    *value = spi_i2s_data_receive(SPI2);
    return true;
}

static bool send_byte(uint8_t value)
{
    if (!wait_flag(SPI_I2S_TDBE_FLAG, true))
    {
        return false;
    }
    spi_i2s_data_transmit(SPI2, value);
    return true;
}

static bool finish_frame(bool success)
{
#if BSP_ENCODER_DMA_TRANSFER
    if (s_dma_frame)
    {
        sample_dma_stop();
        s_dma_frame = false;
    }
#endif
    if (success)
    {
        success = wait_flag(SPI_I2S_BF_FLAG, false);
    }
    if (!success)
    {
        spi_enable(SPI2, FALSE);
    }
    cs_guard();
    gpio_bits_set(BSP_ENCODER_GPIO_PORT, (1U << BSP_ENCODER_CS_PIN));
    if (!success)
    {
        /* 终止残帧并恢复配置，避免迟到RX字节污染下拍。 */
        spi_configure();
    }
    s_stage = 0U;
    s_owner = ENCODER_SPI_OWNER_NONE;
    return success;
}

bool EncoderSpi_Init(void)
{
    if (s_initialized)
    {
        return false;
    }
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    if (clocks.apb1_freq != R40_CORE_HZ || clocks.ahb_freq != R40_CORE_HZ)
    {
        BspPwm_Disable();
        for (;;)
        {
            __WFI();
        }
    }
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
    spi_configure();
#if BSP_ENCODER_DMA_TRANSFER
    sample_dma_init();
#endif
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    cs_guard();
    if (DWT->CYCCNT == before)
    {
        BspPwm_Disable();
        for (;;)
        {
            __WFI();
        }
    }
    s_initialized = true;
    return true;
}
bool EncoderSpi_Begin(const EncoderSpiRequest *request)
{
    return start_frame(ENCODER_SPI_OWNER_SAMPLE, request);
}

uint8_t EncoderSpi_BeginReadback(bool started)
{
    if (!started)
    {
        return 0U;
    }
    if (s_owner != ENCODER_SPI_OWNER_SAMPLE || s_stage != 1U)
    {
        return ENCODER_SPI_STAGE_FAILED;
    }
#if BSP_ENCODER_DMA_TRANSFER
    if (s_dma_frame)
    {
        s_stage = ENCODER_SPI_STAGE_DATA;
        return s_stage;
    }
#endif
    uint8_t ignored;
    uint16_t ignored_word;
    bool ready = s_word_frame ? (receive_word(&ignored_word) && send_byte(0U))
                              : (receive_byte(&ignored) && send_byte(s_command_low) &&
                                 receive_byte(&ignored) && send_byte(0U));
    if (!ready)
    {
        s_stage = ENCODER_SPI_STAGE_FAILED;
        return s_stage;
    }
    s_stage = ENCODER_SPI_STAGE_DATA;
    return s_stage;
}
bool EncoderSpi_Complete(uint8_t sample_stage, const EncoderSpiRequest *request, uint8_t *data)
{
    bool ready = false;
    if (sample_stage == 0U && s_owner == ENCODER_SPI_OWNER_NONE)
    {
        sample_stage = EncoderSpi_Begin(request) ? 1U : ENCODER_SPI_STAGE_FAILED;
    }
    if (s_owner == ENCODER_SPI_OWNER_SAMPLE)
    {
        if (sample_stage == 1U)
        {
            sample_stage = EncoderSpi_BeginReadback(true);
        }
        ready = sample_stage == ENCODER_SPI_STAGE_DATA && s_stage == ENCODER_SPI_STAGE_DATA;
#if BSP_ENCODER_DMA_TRANSFER
        if (ready && s_dma_frame)
        {
            ready = sample_dma_complete();
            if (ready)
            {
                uint16_t high = s_dma_rx[1], low = s_dma_rx[2];
                data[0] = (uint8_t)(high >> 8);
                data[1] = (uint8_t)high;
                data[2] = (uint8_t)(low >> 8);
                data[3] = (uint8_t)low;
            }
        }
        else
#endif
            if (ready && s_word_frame)
        {
            uint16_t high, low;
            ready = receive_word(&high) && send_byte(0U) && receive_word(&low);
            if (ready)
            {
                data[0] = (uint8_t)(high >> 8);
                data[1] = (uint8_t)high;
                data[2] = (uint8_t)(low >> 8);
                data[3] = (uint8_t)low;
            }
        }
        else
        {
            for (uint8_t i = 0U; ready && i < request->response_bytes; ++i)
            {
                ready =
                    receive_byte(&data[i]) && (i + 1U == request->response_bytes || send_byte(0U));
            }
        }
        ready = finish_frame(ready);
    }
    return ready;
}

EncoderSpiReadStatus EncoderSpi_Read(const EncoderSpiRequest *request, uint8_t *data)
{
    if (!start_frame(ENCODER_SPI_OWNER_DIAGNOSTIC, request))
    {
        return ENCODER_SPI_READ_BUSY;
    }
    uint8_t ignored, value;
    bool ok = receive_byte(&ignored) && send_byte((uint8_t)request->command) &&
              receive_byte(&ignored) && send_byte(0U) && receive_byte(&value);
    if (!finish_frame(ok))
    {
        return ENCODER_SPI_READ_ERROR;
    }
    *data = value;
    return ENCODER_SPI_READ_OK;
}
