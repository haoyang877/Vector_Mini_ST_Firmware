#include "at32f45x.h"
#include <assert.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../firmware/platform/at32f456/ports/encoder_mt6835.c"
#include "r40_angle_contract.inc"

spi_type sim_spi;
SimDwt sim_dwt;
SimCoreDebug sim_core_debug;
gpio_type fake_gpiob;
static spi_init_type sim_config;
static unsigned selected, pending, count, selects, releases, resets, irq, mux_count;
static uint32_t ready_cycle, last_receive_cycle, idle_flags, clock_hz = R40_CORE_HZ;
static uint8_t tx[6], reply[4], registers[256];
static unsigned fault, fault_byte, drop_config_address = UINT_MAX;
static bool clock_stopped, power_off;
static jmp_buf fatal;

void __NOP(void)
{
    if (!clock_stopped)
    {
        ++sim_dwt.CYCCNT;
    }
}
void __WFI(void)
{
    assert(power_off);
    longjmp(fatal, 1);
}
uint32_t critical_hw_enter(void)
{
    unsigned old = irq;
    irq = 1U;
    return old;
}
void critical_hw_exit(uint32_t old)
{
    irq = old;
}
void power_stage_hw_stop(void)
{
    power_off = true;
}
void crm_clocks_freq_get(crm_clocks_freq_type *clocks)
{
    clocks->ahb_freq = clock_hz;
    clocks->apb1_freq = clock_hz;
}
void crm_periph_clock_enable(unsigned id, unsigned enabled)
{
    assert((id == CRM_GPIOB_PERIPH_CLOCK || id == CRM_SPI2_PERIPH_CLOCK) && enabled);
}
void gpio_default_para_init(gpio_init_type *pins)
{
    memset(pins, 0, sizeof(*pins));
}
void gpio_init(gpio_type *gpio, gpio_init_type *pins)
{
    assert(gpio == GPIOB && pins->gpio_pull == GPIO_PULL_UP);
    assert(gpio->output & GPIO_PINS_12);
    assert(pins->gpio_out_type == GPIO_OUTPUT_PUSH_PULL);
    if (pins->gpio_mode == GPIO_MODE_OUTPUT)
    {
        assert(pins->gpio_pins == GPIO_PINS_12);
    }
    else
    {
        assert(pins->gpio_mode == GPIO_MODE_MUX);
        assert(pins->gpio_pins == (GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15));
    }
}
void gpio_pin_mux_config(gpio_type *gpio, unsigned pin, unsigned mux)
{
    assert(gpio == GPIOB && pin == 13U + mux_count && mux == GPIO_MUX_5);
    ++mux_count;
}
void gpio_bits_reset(gpio_type *gpio, unsigned pins)
{
    assert(gpio == GPIOB && pins == GPIO_PINS_12 && !selected && irq);
    gpio->output &= ~pins;
    selected = 1U;
    pending = count = 0U;
    ++selects;
    last_receive_cycle = sim_dwt.CYCCNT;
}
void gpio_bits_set(gpio_type *gpio, unsigned pins)
{
    assert(gpio == GPIOB && pins == GPIO_PINS_12);
    if (selected)
    {
        assert(!pending || !sim_spi.ctrl1_bit.spien);
        assert((uint32_t)(sim_dwt.CYCCNT - last_receive_cycle) >= 36U || clock_stopped);
        selected = 0U;
        ++releases;
    }
    gpio->output |= pins;
}
void spi_i2s_reset(spi_type *spi)
{
    assert(spi == SPI2);
    memset(spi, 0, sizeof(*spi));
    idle_flags = pending = 0U;
    ++resets;
}
void spi_default_para_init(spi_init_type *config)
{
    memset(config, 0, sizeof(*config));
}
void spi_init(spi_type *spi, spi_init_type *config)
{
    assert(spi == SPI2);
    sim_config = *config;
}
void spi_software_cs_internal_level_set(spi_type *spi, spi_software_cs_level_type level)
{
    assert(spi == SPI2 && level == SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi->ctrl1_bit.swcsil = 1U;
}
void spi_enable(spi_type *spi, confirm_state enabled)
{
    assert(spi == SPI2);
    spi->ctrl1_bit.spien = enabled;
}
void spi_i2s_data_transmit(spi_type *spi, uint16_t value)
{
    assert(spi == SPI2 && selected && !pending && count < 6U && value <= UINT8_MAX);
    if (count == 0U)
    {
        assert((uint32_t)(sim_dwt.CYCCNT - last_receive_cycle) >= 36U);
        assert(value == 0xA0U || value == 0x30U);
    }
    else if (count == 1U && tx[0] == 0xA0U)
    {
        assert(value == 0x03U);
    }
    else if (count > 1U)
    {
        assert(value == 0U);
    }
    tx[count++] = (uint8_t)value;
    pending = 1U;
    ready_cycle = sim_dwt.CYCCNT + 256U;
}
uint16_t spi_i2s_data_receive(spi_type *spi)
{
    assert(spi == SPI2 && selected && pending);
    assert((int32_t)(sim_dwt.CYCCNT - ready_cycle) >= 0);
    pending = 0U;
    last_receive_cycle = sim_dwt.CYCCNT;
    if (count <= 2U)
    {
        return 0xFFU;
    }
    return tx[0] == 0xA0U ? reply[count - 3U] : registers[tx[1]];
}
flag_status spi_i2s_flag_get(spi_type *spi, uint32_t mask)
{
    assert(spi == SPI2);
    if (!clock_stopped)
    {
        sim_dwt.CYCCNT += 8U;
    }
    uint32_t flags = idle_flags | SPI_I2S_TDBE_FLAG;
    if (selected)
    {
        bool inject = count - 1U == fault_byte;
        if (pending)
        {
            flags |= SPI_I2S_BF_FLAG;
            if ((int32_t)(sim_dwt.CYCCNT - ready_cycle) >= 0 && !(fault == 1U && inject) &&
                !(tx[0] == 0x30U && count == 3U && tx[1] == drop_config_address))
            {
                flags |= SPI_I2S_RDBF_FLAG;
            }
        }
        if (fault == 2U && inject)
        {
            flags &= ~SPI_I2S_TDBE_FLAG;
        }
        if (fault == 3U)
        {
            flags |= SPI_I2S_BF_FLAG;
        }
        if (fault >= 4U && inject)
        {
            flags |= fault == 4U   ? SPI_I2S_ROERR_FLAG
                     : fault == 5U ? SPI_MMERR_FLAG
                                   : SPI_CCERR_FLAG;
        }
    }
    return (flags & mask) ? SET : RESET;
}

static void make_reply(uint32_t angle, uint8_t status)
{
    /* 独立按多项式长除法生成参考余数，不调用被测CRC实现。 */
    uint32_t payload = (angle << 3) | status;
    reply[0] = (uint8_t)(payload >> 16);
    reply[1] = (uint8_t)(payload >> 8);
    reply[2] = (uint8_t)payload;
    uint32_t dividend = payload << 8;
    for (int bit = 31; bit >= 8; --bit)
    {
        if (dividend & (UINT32_C(1) << bit))
        {
            dividend ^= UINT32_C(0x107) << (bit - 8);
        }
    }
    reply[3] = (uint8_t)dividend;
}

static void check_sample(unsigned pipeline, EncoderSensorStatus expected, uint16_t angle)
{
    EncoderSensorSample sample = {ENCODER_SENSOR_OK, 0x1357U, 0x2468U, 0x369AU, false};
    uint8_t stage = 0U;
    if (pipeline)
    {
        assert(encoder_sensor_begin());
        assert(selected && count == 1U && pending && !irq);
        stage = pipeline == 1U ? 1U : encoder_sensor_begin_readback(true);
    }
    assert(encoder_sensor_complete(stage, &sample) == expected);
    assert(sample.status == expected && !selected && !pending && !irq);
    assert(selects == releases);
    if (expected == ENCODER_SENSOR_OK)
    {
        assert(sample.angle_q15 == angle && sample.crc_ok && count == 6U);
        assert(sample.frame_word == angle);
        assert(sample.safety_word == (uint16_t)(((uint16_t)reply[2] << 8) | reply[3]));
    }
    else
    {
        assert(sample.angle_q15 == 0x1357U && sample.frame_word == 0x2468U &&
               sample.safety_word == 0x369AU);
        assert(sample.crc_ok == (expected == ENCODER_SENSOR_SENSOR_FAULT));
    }
}

static void test_samples(void)
{
    for (unsigned pipeline = 0U; pipeline < 3U; ++pipeline)
    {
        /* 穷举21位角的全部65536个输出桶及低5位边界，验证回绕。 */
        for (uint32_t angle = 0U; angle < (1U << 21); angle += 32U)
        {
            make_reply(angle, 0U);
            check_sample(pipeline, ENCODER_SENSOR_OK, (uint16_t)(angle >> 5));
            make_reply(angle + 31U, 0U);
            check_sample(pipeline, ENCODER_SENSOR_OK, (uint16_t)(angle >> 5));
        }
        for (uint8_t status = 1U; status < 8U; ++status)
        {
            make_reply(0x123456U, status);
            check_sample(pipeline, ENCODER_SENSOR_SENSOR_FAULT, 0U);
        }
        for (unsigned bit = 0U; bit < 32U; ++bit)
        {
            make_reply(0x123456U, 0U);
            reply[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
            check_sample(pipeline, ENCODER_SENSOR_FRAME_ERROR, 0U);
        }
    }
    make_reply(0U, 0U);
    assert(reply[3] == 0U);
    make_reply(0x1FFFFFU, 7U);
    assert(reply[3] == 0x0FU);
}

static void test_faults(void)
{
    for (unsigned pipeline = 0U; pipeline < 3U; ++pipeline)
    {
        for (fault = 1U; fault <= 6U; ++fault)
        {
            for (fault_byte = 0U; fault_byte < 6U; ++fault_byte)
            {
                /* 最后一个字节收完后不再发送，TDBE低不影响已完成帧。 */
                if (fault == 2U && fault_byte == 5U)
                {
                    continue;
                }
                make_reply(0x123456U, 0U);
                unsigned before = resets;
                check_sample(pipeline, ENCODER_SENSOR_BUS_TIMEOUT, 0U);
                assert(resets == before + 1U);
                unsigned saved = fault;
                fault = 0U;
                check_sample(2U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));
                fault = saved;
            }
        }
    }
    fault = 0U;
    /* 拒绝残留RX、错误、busy或未使能；重置后下一帧恢复。 */
    const uint32_t masks[] = {
        SPI_I2S_BF_FLAG, SPI_I2S_RDBF_FLAG, SPI_MMERR_FLAG, SPI_CCERR_FLAG, SPI_I2S_ROERR_FLAG};
    for (unsigned i = 0U; i < sizeof(masks) / sizeof(masks[0]); ++i)
    {
        idle_flags = masks[i];
        assert(!encoder_sensor_begin() && !selected);
        check_sample(0U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));
    }
    sim_spi.ctrl1_bit.spien = 0U;
    assert(!encoder_sensor_begin());
    check_sample(0U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));

    assert(encoder_sensor_begin());
    unsigned before = selects;
    encoder_sensor_capture_config();
    assert(selected && count == 1U && selects == before);
    assert(!encoder_sensor_begin());
    EncoderSensorSample sample;
    assert(encoder_sensor_complete(0U, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    assert(!selected && selects == releases);
    check_sample(2U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));

    assert(encoder_sensor_begin());
    sim_dwt.CYCCNT += MT6835_TIMEOUT_CYCLES;
    assert(encoder_sensor_complete(1U, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    /* 模拟APP两段SPI之间25us计算；独立台架20us拒绝，完整APP40us允许。 */
    make_reply(0x123456U, 0U);
    assert(encoder_sensor_begin());
    uint8_t overlap_stage = encoder_sensor_begin_readback(true);
    sim_dwt.CYCCNT += R40_CORE_HZ / 40000U;
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
    assert(encoder_sensor_complete(overlap_stage, &sample) == ENCODER_SENSOR_OK);
#else
    assert(encoder_sensor_complete(overlap_stage, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
#endif
    sim_dwt.CYCCNT = UINT32_MAX - 100U;
    check_sample(2U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));
    assert(encoder_sensor_begin());
    clock_stopped = true;
    assert(encoder_sensor_complete(1U, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    assert(s_polls == MT6835_POLL_LIMIT + 1U);
    clock_stopped = false;
    check_sample(2U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));

    assert(encoder_sensor_begin());
    assert(encoder_sensor_complete(99U, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    assert(!selected);
    assert(encoder_sensor_begin());
    uint8_t stage = encoder_sensor_begin_readback(true);
    assert(stage == MT6835_STAGE_DATA);
    assert(encoder_sensor_begin_readback(true) == MT6835_STAGE_FAILED);
    assert(encoder_sensor_complete(MT6835_STAGE_FAILED, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    assert(encoder_sensor_complete(0U, NULL) == ENCODER_SENSOR_FRAME_ERROR);
    assert(!selected && selects == releases);

    /* 诊断拥有总线时，抢占它的快环失败不能结束或改写诊断事务。 */
    assert(start_frame(MT6835_OWNER_DIAGNOSTIC, 0x30U));
    before = selects;
    assert(!encoder_sensor_begin());
    assert(encoder_sensor_complete(0U, &sample) == ENCODER_SENSOR_BUS_TIMEOUT);
    assert(selected && count == 1U && selects == before && s_owner == MT6835_OWNER_DIAGNOSTIC);
    (void)finish_frame(false);
    check_sample(2U, ENCODER_SENSOR_OK, (uint16_t)(0x123456U >> 5));
}

static void test_angle_contract(void)
{
    /* 编译未修改的中间层函数，与真实MT端口组合验证错误映射和角度单位。 */
    EncoderState encoder = {0};
    uint16_t raw = 0x1357U;
    make_reply(0x1FFFFFU, 0U);
    bool started = Encoder_BeginSample();
    assert(encoder_read_frame(&encoder, &raw, Encoder_BeginReadback(started)));
    assert(raw == 65535U && encoder.read_status == ENCODER_READ_OK);
    assert(encoder.frame_word == 65535U);
    for (unsigned error = 0U; error < 3U; ++error)
    {
        make_reply(0x1FFFFFU, error == 1U ? 2U : 0U);
        if (error == 0U)
        {
            reply[3] ^= 1U;
        }
        fault = error == 2U ? 1U : 0U;
        fault_byte = 0U;
        started = Encoder_BeginSample();
        assert(!encoder_read_frame(&encoder, &raw, Encoder_BeginReadback(started)));
        EncoderReadStatus expected = error == 0U   ? ENCODER_READ_CRC_MISMATCH
                                     : error == 1U ? ENCODER_READ_TLE_SYSTEM_ERROR
                                                   : ENCODER_READ_SPI_TIMEOUT;
        assert(encoder.read_status == expected && encoder.read_status_latched == expected);
        assert(encoder.read_error_count == error + 1U && encoder.bad_frame_streak == error + 1U);
        assert(raw == 65535U && encoder.frame_word == 65535U);
    }
    fault = 0U;
    make_reply(0U, 0U);
    assert(encoder_read_frame(&encoder, &raw, 0U));
    assert(raw == 0U && encoder.bad_frame_streak == 0U && encoder.read_error_count == 3U);
    EncoderCalibration calibration = {0};
    assert(encoder_apply_direction_q15(&calibration, 0x1234U) == 0x1234U);
    calibration.reverse = 1U;
    assert(encoder_apply_direction_q15(&calibration, 0x1234U) == (uint16_t)(0U - 0x1234U));
    encoder.previous_linearized_q15 = 65530U;
    encoder.shadow_q15 = 65530;
    encoder.linearized_q15 = 65535U;
    encoder_update_shadow(&encoder);
    encoder.linearized_q15 = 0U;
    encoder_update_shadow(&encoder);
    assert(encoder.shadow_q15 == 65536);
    puts("PASS unchanged angle layer + MT6835: units, error mapping, stale rejection, direction "
         "and wrap");
}

int main(int argc, char **argv)
{
    if (argc > 1 && atoi(argv[1]) != 0)
    {
        if (atoi(argv[1]) == 1)
        {
            clock_hz = 170000000U;
        }
        else
        {
            clock_stopped = true;
        }
        if (setjmp(fatal) == 0)
        {
            encoder_sensor_init();
            assert(!"initialization should halt");
        }
        assert(!s_initialized && power_off);
        puts("PASS MT6835: initialization fails closed");
        return 0;
    }
    memset(registers, 0x5A, sizeof(registers));
    encoder_sensor_init();
    assert(encoder_sensor_type() == ENCODER_SENSOR_TYPE_MT6835 && mux_count == 3U);
    assert(sim_config.transmission_mode == SPI_TRANSMIT_FULL_DUPLEX);
    assert(sim_config.master_slave_mode == SPI_MODE_MASTER);
    assert(sim_config.mclk_freq_division ==
           (R40_MT6835_FAST_SPI ? SPI_MCLK_DIV_16 : SPI_MCLK_DIV_32));
    assert(sim_config.first_bit_transmission == SPI_FIRST_BIT_MSB);
    assert(sim_config.frame_bit_num == SPI_FRAME_8BIT);
    assert(sim_config.clock_polarity == SPI_CLOCK_POLARITY_HIGH);
    assert(sim_config.clock_phase == SPI_CLOCK_PHASE_2EDGE);
    assert(sim_config.cs_mode_selection == SPI_CS_SOFTWARE_MODE);
    assert(sim_spi.ctrl1_bit.swcsil && sim_spi.ctrl1_bit.spien);
    unsigned before = selects;
    encoder_sensor_init();
    assert(selects == before);
    for (unsigned i = 0U; i < ENCODER_SENSOR_CONFIG_WORDS; ++i)
    {
        assert(encoder_sensor_config_word((uint8_t)i) == 0x5AU);
    }
    assert(encoder_sensor_config_word(6U) == 0U);
    drop_config_address = 9U;
    memset(registers, 0xA5, sizeof(registers));
    encoder_sensor_capture_config();
    for (unsigned i = 0U; i < ENCODER_SENSOR_CONFIG_WORDS; ++i)
    {
        assert(encoder_sensor_config_word((uint8_t)i) == (i == 2U ? 0x5AU : 0xA5U));
    }
    drop_config_address = UINT_MAX;
    test_samples();
    test_faults();
    test_angle_contract();
    puts("PASS MT6835: 393216 angle/CRC buckets, all status and single-bit errors, "
         "lifecycle/timeouts/CS/recovery/config");
    return 0;
}
