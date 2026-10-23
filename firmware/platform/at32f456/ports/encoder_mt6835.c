#include "encoder_sensor.h"

#include <stddef.h>
#include "at32f45x.h"
#include "critical_hw.h"
#include "power_stage_hw.h"
#include "r40_clock.h"

/* R40专用四线SPI2，不复用旧板的16位半双工encoder_spi契约。
 * 48时钟突发：A0 03 + 角度/状态/CRC四字节；CS下降沿锁存本拍。
 * 21位角度截去低5位，保持既有每圈65536计数。CRC仍覆盖全部21位和3位状态。
 * 原始诊断字按接收顺序拼成两个16位字，不混入旧传感器的位域含义。 */
#define MT6835_SPI_ERRORS (SPI_MMERR_FLAG | SPI_I2S_ROERR_FLAG | SPI_CCERR_FLAG)
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
/* 完整APP在两段回读之间计算热模型/观测器；整帧仍须小于50us控制周期。 */
#define MT6835_TIMEOUT_CYCLES (R40_CORE_HZ / 25000U)
volatile uint32_t g_r40_mt6835_frames, g_r40_mt6835_transfer_errors, g_r40_mt6835_max_cycles;
#else
#define MT6835_TIMEOUT_CYCLES (R40_CORE_HZ / 50000U)
#endif
#define MT6835_POLL_LIMIT 4096U
#define MT6835_STAGE_DATA 2U
#define MT6835_STAGE_FAILED 255U

/* 仅限时APP显式选择11.25MHz；其余目标保持5.625MHz，均低于MT6835的16MHz上限。 */
#ifndef R40_MT6835_FAST_SPI
#define R40_MT6835_FAST_SPI 0
#endif
#if R40_MT6835_FAST_SPI != 0 && R40_MT6835_FAST_SPI != 1
#error "R40 MT6835 SPI selection must be 0 or 1"
#endif
#if R40_MT6835_FAST_SPI && R40_CORE_HZ / 16U > 16000000U
#error "MT6835 SPI exceeds the 16MHz device limit"
#endif

typedef enum
{
    MT6835_OWNER_NONE,
    MT6835_OWNER_SAMPLE,
    MT6835_OWNER_DIAGNOSTIC
} Mt6835Owner;

static volatile Mt6835Owner s_owner;
static bool s_initialized;
static uint8_t s_stage;
static uint32_t s_started_cycles;
static uint32_t s_polls;
static volatile uint16_t s_config_words[ENCODER_SENSOR_CONFIG_WORDS];

static void cs_guard(void)
{
    /* 至少36个内核周期：180MHz时>=200ns，覆盖CS建立100ns和保持半个SCK。 */
    for (uint32_t i = 0U; i < 36U; ++i)
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
    config.mclk_freq_division = R40_MT6835_FAST_SPI ? SPI_MCLK_DIV_16 : SPI_MCLK_DIV_32;
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
           spi_i2s_flag_get(SPI2, MT6835_SPI_ERRORS | SPI_I2S_BF_FLAG | SPI_I2S_RDBF_FLAG) ==
               RESET &&
           spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) != RESET;
}

static bool start_frame(Mt6835Owner owner, uint8_t command)
{
    uint32_t state = critical_hw_enter();
    if (!s_initialized || s_owner != MT6835_OWNER_NONE)
    {
        critical_hw_exit(state);
        return false;
    }
    if (!bus_ready())
    {
        /* 异常残留只清理本端口独占的SPI2；本拍失败，下拍再尝试。 */
        spi_configure();
        critical_hw_exit(state);
        return false;
    }
    s_owner = owner;
    s_stage = 1U;
    s_polls = 0U;
    s_started_cycles = DWT->CYCCNT;
    gpio_bits_reset(GPIOB, GPIO_PINS_12);
    cs_guard();
    spi_i2s_data_transmit(SPI2, command);
    critical_hw_exit(state);
    return true;
}

static bool wait_flag(uint32_t flag, bool set)
{
    /* 全帧共用目标配置的周期预算及轮询次数上限；DWT停走时也不会无限等待。
     * 预算包含begin/readback/complete之间的计算，超预算样本直接丢弃。 */
    while (++s_polls <= MT6835_POLL_LIMIT &&
           (uint32_t)(DWT->CYCCNT - s_started_cycles) < MT6835_TIMEOUT_CYCLES)
    {
        if (spi_i2s_flag_get(SPI2, MT6835_SPI_ERRORS) != RESET)
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
    if (success)
    {
        success = wait_flag(SPI_I2S_BF_FLAG, false);
    }
    if (!success)
    {
        spi_enable(SPI2, FALSE);
    }
    cs_guard();
    gpio_bits_set(GPIOB, GPIO_PINS_12);
    if (!success)
    {
        /* 终止残帧并恢复配置，避免迟到RX字节污染下拍。 */
        spi_configure();
    }
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
    uint32_t elapsed = DWT->CYCCNT - s_started_cycles;
    ++g_r40_mt6835_frames;
    g_r40_mt6835_transfer_errors += success ? 0U : 1U;
    if (elapsed > g_r40_mt6835_max_cycles)
    {
        g_r40_mt6835_max_cycles = elapsed;
    }
#endif
    s_stage = 0U;
    s_owner = MT6835_OWNER_NONE;
    return success;
}

static uint8_t frame_crc(const uint8_t data[3])
{
    /* CRC-8/0x07，初值0，无反射、无末尾异或。半字节余式由多项式推导。 */
    static const uint8_t s_remainder[16] = {0x00,
                                            0x07,
                                            0x0E,
                                            0x09,
                                            0x1C,
                                            0x1B,
                                            0x12,
                                            0x15,
                                            0x38,
                                            0x3F,
                                            0x36,
                                            0x31,
                                            0x24,
                                            0x23,
                                            0x2A,
                                            0x2D};
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        crc ^= data[i];
        crc = (uint8_t)((crc << 4) ^ s_remainder[crc >> 4]);
        crc = (uint8_t)((crc << 4) ^ s_remainder[crc >> 4]);
    }
    return crc;
}

void encoder_sensor_init(void)
{
    if (s_initialized)
    {
        return;
    }
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    if (clocks.apb1_freq != R40_CORE_HZ || clocks.ahb_freq != R40_CORE_HZ)
    {
        power_stage_hw_stop();
        for (;;)
        {
            __WFI();
        }
    }
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    gpio_bits_set(GPIOB, GPIO_PINS_12);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_UP;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = GPIO_PINS_12;
    gpio_init(GPIOB, &pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOB, &pins);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE13, GPIO_MUX_5);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE14, GPIO_MUX_5);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE15, GPIO_MUX_5);
    spi_configure();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    cs_guard();
    if (DWT->CYCCNT == before)
    {
        power_stage_hw_stop();
        for (;;)
        {
            __WFI();
        }
    }
    s_initialized = true;
    encoder_sensor_capture_config();
}

bool encoder_sensor_begin(void)
{
    return start_frame(MT6835_OWNER_SAMPLE, 0xA0U);
}

uint8_t encoder_sensor_begin_readback(bool started)
{
    if (!started)
    {
        return 0U;
    }
    if (s_owner != MT6835_OWNER_SAMPLE || s_stage != 1U)
    {
        return MT6835_STAGE_FAILED;
    }
    uint8_t ignored;
    if (!receive_byte(&ignored) || !send_byte(0x03U) || !receive_byte(&ignored) || !send_byte(0U))
    {
        s_stage = MT6835_STAGE_FAILED;
        return s_stage;
    }
    s_stage = MT6835_STAGE_DATA;
    return s_stage;
}

EncoderSensorStatus encoder_sensor_complete(uint8_t sample_stage, EncoderSensorSample *out)
{
    EncoderSensorStatus status = ENCODER_SENSOR_BUS_TIMEOUT;
    uint8_t data[4];
    bool ready = false;
    if (sample_stage == 0U && s_owner == MT6835_OWNER_NONE)
    {
        sample_stage = encoder_sensor_begin() ? 1U : MT6835_STAGE_FAILED;
    }
    if (s_owner == MT6835_OWNER_SAMPLE)
    {
        if (sample_stage == 1U)
        {
            sample_stage = encoder_sensor_begin_readback(true);
        }
        ready = sample_stage == MT6835_STAGE_DATA && s_stage == MT6835_STAGE_DATA;
        for (uint8_t i = 0U; ready && i < 4U; ++i)
        {
            ready = receive_byte(&data[i]) && (i == 3U || send_byte(0U));
        }
        ready = finish_frame(ready);
    }
    if (out == NULL)
    {
        return ENCODER_SENSOR_FRAME_ERROR;
    }
    out->crc_ok = false;
    if (ready)
    {
        out->crc_ok = frame_crc(data) == data[3];
        status = !out->crc_ok           ? ENCODER_SENSOR_FRAME_ERROR
                 : (data[2] & 7U) != 0U ? ENCODER_SENSOR_SENSOR_FAULT
                                        : ENCODER_SENSOR_OK;
        if (status == ENCODER_SENSOR_OK)
        {
            out->angle_q15 = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            out->frame_word = out->angle_q15;
            out->safety_word = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
        }
    }
    out->status = status;
    return status;
}

EncoderSensorType encoder_sensor_type(void)
{
    return ENCODER_SENSOR_TYPE_MT6835;
}

void encoder_sensor_capture_config(void)
{
    /* 依次为ABZ分辨率高/低、零位高/低、方向/迟滞、带宽；只发送0x3读取。 */
    static const uint8_t s_address[ENCODER_SENSOR_CONFIG_WORDS] = {7U, 8U, 9U, 10U, 13U, 17U};
    for (uint8_t i = 0U; i < ENCODER_SENSOR_CONFIG_WORDS; ++i)
    {
        if (!start_frame(MT6835_OWNER_DIAGNOSTIC, 0x30U))
        {
            return;
        }
        uint8_t ignored, value;
        bool ok = receive_byte(&ignored) && send_byte(s_address[i]) && receive_byte(&ignored) &&
                  send_byte(0U) && receive_byte(&value);
        if (finish_frame(ok))
        {
            s_config_words[i] = value;
        }
    }
}

uint16_t encoder_sensor_config_word(uint8_t index)
{
    return index < ENCODER_SENSOR_CONFIG_WORDS ? s_config_words[index] : 0U;
}
