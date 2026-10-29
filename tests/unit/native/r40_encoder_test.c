#include "at32f45x.h"
#include <assert.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_encoder.h"
#include "bsp_internal.h"
#include "encoder_protocol.h"

#include "../../../firmware/platform/bsp/mcu/at32f456/encoder_spi.c"
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
static bool clock_stopped, power_off = true;
static unsigned nop_step = 1U, nop_calls;
static jmp_buf fatal;
#if BSP_ENCODER_DMA_TRANSFER
dma_type sim_dma;
dma_channel_type sim_dma_rx, sim_dma_tx;
dmamux_channel_type sim_rx_mux, sim_tx_mux;
static bool dma_rx_enabled, dma_tx_enabled;
void dma_reset(dma_channel_type *channel)
{
    memset(channel, 0, sizeof(*channel));
}
void dma_default_para_init(dma_init_type *config)
{
    memset(config, 0, sizeof(*config));
}
void dma_init(dma_channel_type *channel, dma_init_type *config)
{
    assert(config->buffer_size == 3U && config->memory_inc_enable && !config->loop_mode_enable);
    assert(config->peripheral_data_width == DMA_PERIPHERAL_DATA_WIDTH_HALFWORD);
    assert(config->memory_data_width == DMA_MEMORY_DATA_WIDTH_HALFWORD);
    assert(config->direction == (channel == DMA2_CHANNEL1 ? DMA_DIR_PERIPHERAL_TO_MEMORY
                                                          : DMA_DIR_MEMORY_TO_PERIPHERAL));
}
void dmamux_enable(dma_type *dma, confirm_state enabled)
{
    assert(dma == DMA2 && enabled);
}
void dmamux_init(dmamux_channel_type *mux, dmamux_requst_id_sel_type request)
{
    assert(request ==
           (mux == DMA2MUX_CHANNEL1 ? DMAMUX_DMAREQ_ID_SPI2_RX : DMAMUX_DMAREQ_ID_SPI2_TX));
}
void dma_channel_enable(dma_channel_type *channel, confirm_state enabled)
{
    if (channel == DMA2_CHANNEL1)
    {
        if (!enabled)
        {
            assert(!dma_tx_enabled && !sim_spi.ctrl2_bit.dmaten && !sim_spi.ctrl2_bit.dmaren);
        }
        dma_rx_enabled = enabled;
    }
    else
    {
        assert(channel == DMA2_CHANNEL2);
        if (!enabled)
        {
            assert(!sim_spi.ctrl2_bit.dmaten && !sim_spi.ctrl2_bit.dmaren);
        }
        dma_tx_enabled = enabled;
    }
}
void dma_data_number_set(dma_channel_type *channel, uint16_t number)
{
    assert(number == 3U);
    channel->dtcnt = number;
}
void spi_i2s_dma_receiver_enable(spi_type *spi, confirm_state enabled)
{
    assert(spi == SPI2);
    sim_spi.ctrl2_bit.dmaren = enabled;
}
void spi_i2s_dma_transmitter_enable(spi_type *spi, confirm_state enabled)
{
    assert(spi == SPI2);
    sim_spi.ctrl2_bit.dmaten = enabled;
    if (enabled)
    {
        assert(dma_rx_enabled && sim_spi.ctrl2_bit.dmaren && dma_tx_enabled);
        spi_i2s_data_transmit(spi, s_dma_tx[0]);
        --sim_dma_tx.dtcnt;
    }
}
void dma_flag_clear(uint32_t flags)
{
    if (flags & 1U)
        sim_dma.sts &= ~15U;
    if (flags & 16U)
        sim_dma.sts &= ~240U;
}
flag_status dma_flag_get(uint32_t flags)
{
    return (sim_dma.sts & flags & 0xFFU) ? SET : RESET;
}
#endif

void __NOP(void)
{
    ++nop_calls;
    if (!clock_stopped)
    {
        sim_dwt.CYCCNT += nop_step;
    }
}
void __WFI(void)
{
    assert(power_off);
    longjmp(fatal, 1);
}
uint32_t BspSystem_EnterCritical(void)
{
    unsigned old = irq;
    irq = 1U;
    return old;
}
void BspSystem_ExitCritical(uint32_t old)
{
    irq = old;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return power_off;
}
void BspPwm_Disable(void)
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
    assert((id == CRM_GPIOB_PERIPH_CLOCK || id == CRM_SPI2_PERIPH_CLOCK ||
            (BSP_ENCODER_DMA_TRANSFER && id == CRM_DMA2_PERIPH_CLOCK)) &&
           enabled);
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
    spi->ctrl1_bit.fbn = config->frame_bit_num;
}
void spi_frame_bit_num_set(spi_type *spi, spi_frame_bit_num_type bits)
{
    assert(spi == SPI2 && !selected && !spi->ctrl1_bit.spien);
    spi->ctrl1_bit.fbn = bits;
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
    unsigned width = spi->ctrl1_bit.fbn == SPI_FRAME_16BIT ? 2U : 1U;
    assert(spi == SPI2 && selected && !pending && count + width <= 6U);
    assert(width == 2U || value <= UINT8_MAX);
    for (unsigned i = 0U; i < width; ++i)
    {
        uint8_t byte = width == 2U && i == 0U ? (uint8_t)(value >> 8) : (uint8_t)value;
        if (count == 0U)
        {
            assert((uint32_t)(sim_dwt.CYCCNT - last_receive_cycle) >= 36U);
            assert(byte == 0xA0U || byte == 0x30U);
        }
        else if (count == 1U && tx[0] == 0xA0U)
        {
            assert(byte == 0x03U);
        }
        else if (count > 1U)
        {
            assert(byte == 0U);
        }
        tx[count++] = byte;
    }
    pending = width;
    ready_cycle = sim_dwt.CYCCNT + 256U * width;
}
uint16_t spi_i2s_data_receive(spi_type *spi)
{
    assert(spi == SPI2 && selected && pending);
    assert((int32_t)(sim_dwt.CYCCNT - ready_cycle) >= 0);
    unsigned width = pending;
    unsigned start = count - width;
    pending = 0U;
    last_receive_cycle = sim_dwt.CYCCNT;
    uint16_t value = 0U;
    for (unsigned i = 0U; i < width; ++i)
    {
        unsigned index = start + i;
        uint8_t byte = index < 2U ? 0xFFU : tx[0] == 0xA0U ? reply[index - 2U] : registers[tx[1]];
        value = (uint16_t)((value << 8) | byte);
    }
    return value;
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
        unsigned width = spi->ctrl1_bit.fbn == SPI_FRAME_16BIT ? 2U : 1U;
        bool inject = fault_byte >= count - width && fault_byte < count;
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
#if BSP_ENCODER_DMA_TRANSFER
    /* SPI周期推进DMA硬件，不能依赖被测代码是否调用SDK查标志函数。 */
    if (dma_rx_enabled && dma_tx_enabled && sim_spi.ctrl2_bit.dmaren && sim_spi.ctrl2_bit.dmaten &&
        pending)
    {
        bool inject = fault_byte >= count - 2U && fault_byte < count;
        bool ready = (flags & SPI_I2S_RDBF_FLAG) != RESET;
        if (ready && !(fault == 2U && inject))
        {
            unsigned index = 3U - sim_dma_rx.dtcnt;
            s_dma_rx[index] = spi_i2s_data_receive(SPI2);
            --sim_dma_rx.dtcnt;
            if (sim_dma_tx.dtcnt)
            {
                spi_i2s_data_transmit(SPI2, s_dma_tx[3U - sim_dma_tx.dtcnt]);
                --sim_dma_tx.dtcnt;
            }
            if (!sim_dma_rx.dtcnt)
                sim_dma.sts |= DMA2_FDT1_FLAG & 0xFFU;
            if (!sim_dma_tx.dtcnt)
                sim_dma.sts |= DMA2_FDT2_FLAG & 0xFFU;
        }
    }
#endif
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

static void check_sample(unsigned pipeline, BspResult expected, uint16_t angle)
{
    BspEncoderSample sample = {0x1357U, 0x2468U}, before = sample;
    if (!pipeline)
    {
        idle_flags = SPI_I2S_BF_FLAG;
    }
    assert(BspEncoder_Begin() == BSP_OK);
    idle_flags = 0U;
    if (pipeline)
    {
        assert(selected && count == (BSP_ENCODER_WORD_TRANSFER ? 2U : 1U) && pending && !irq);
        if (pipeline == 2U)
        {
            (void)BspEncoder_BeginReadback();
        }
    }
    assert(BspEncoder_Complete(&sample) == expected);
    assert(!selected && !pending && !irq && selects == releases);
    BspEncoderDiagnostics diagnostic;
    assert(BspEncoder_ReadDiagnostics(&diagnostic) == BSP_OK &&
           diagnostic.sample_result == expected);
    if (expected == BSP_OK)
    {
        assert(sample.angle_counts == angle && count == 6U);
        assert(diagnostic.sample_sequence == sample.sequence && diagnostic.frame_word == angle);
        assert(diagnostic.crc_checked && diagnostic.valid_mask == 3U);
        assert(diagnostic.safety_word == (uint16_t)(((uint16_t)reply[2] << 8) | reply[3]));
    }
    else
    {
        assert(memcmp(&sample, &before, sizeof(sample)) == 0);
        assert(diagnostic.sample_sequence == 0U);
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
            check_sample(pipeline, BSP_OK, (uint16_t)(angle >> 5));
            make_reply(angle + 31U, 0U);
            check_sample(pipeline, BSP_OK, (uint16_t)(angle >> 5));
        }
        for (uint8_t status = 1U; status < 8U; ++status)
        {
            make_reply(0x123456U, status);
            check_sample(pipeline, BSP_SENSOR_FAULT, 0U);
        }
        for (unsigned bit = 0U; bit < 32U; ++bit)
        {
            make_reply(0x123456U, 0U);
            reply[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
            check_sample(pipeline, BSP_CRC_ERROR, 0U);
        }
    }
    make_reply(0U, 0U);
    assert(reply[3] == 0U);
    make_reply(0x1FFFFFU, 7U);
    assert(reply[3] == 0x0FU);
}

#if BSP_ENCODER_DMA_TRANSFER
static void test_dma_status_and_stop(void)
{
    /* 包含单边完成、完成与错误并存，以及其它通道/SDK选择位；错误始终优先。 */
    clock_stopped = true;
    const uint32_t spi_errors[] = {0U, SPI_I2S_ROERR_FLAG, SPI_MMERR_FLAG, SPI_CCERR_FLAG};
    for (unsigned error = 0U; error < sizeof(spi_errors) / sizeof(spi_errors[0]); ++error)
    {
        idle_flags = spi_errors[error];
        for (uint32_t flags = 0U; flags < 256U; ++flags)
        {
            sim_dma.sts = flags | 0x10000800U;
            s_started_cycles = sim_dwt.CYCCNT;
            s_polls = ENCODER_SPI_POLL_LIMIT - 1U;
            bool expected = error == 0U && !(flags & 0x88U) && (flags & 0x22U) == 0x22U;
            assert(sample_dma_complete() == expected);
        }
    }
    idle_flags = 0U;
    sim_dma.sts = 0x22U;
    s_started_cycles = sim_dwt.CYCCNT - ENCODER_SPI_TIMEOUT_CYCLES;
    s_polls = 0U;
    assert(!sample_dma_complete() && s_polls == 1U);
    clock_stopped = false;
    sim_dma.sts = 0U;
    uint32_t saved = sim_spi.ctrl2;
    for (uint32_t bits = 0U; bits < 256U; ++bits)
    {
        uint32_t control = 0xA55A0000U | bits;
        sim_spi.ctrl2 = control;
        dma_rx_enabled = dma_tx_enabled = true;
        sample_dma_stop();
        assert(sim_spi.ctrl2 == (control & ~3U) && !dma_rx_enabled && !dma_tx_enabled);
    }
    sim_spi.ctrl2 = saved;
    puts("PASS DMA: 1024 status/error cases, expired deadline, 256 stop/control-bit cases");
}
#endif

static void test_faults(void)
{
#if BSP_ENCODER_DMA_TRANSFER
    for (unsigned dma_error = 8U; dma_error <= 128U; dma_error *= 16U)
    {
        make_reply(0x123456U, 0U);
        assert(BspEncoder_Begin() == BSP_OK);
        sim_dma.sts |= dma_error;
        BspEncoderSample failed;
        assert(BspEncoder_Complete(&failed) == BSP_TIMEOUT);
        assert(!dma_rx_enabled && !dma_tx_enabled && !sim_spi.ctrl2_bit.dmaren &&
               !sim_spi.ctrl2_bit.dmaten);
        check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));
    }
#endif
    for (unsigned pipeline = 0U; pipeline < 3U; ++pipeline)
    {
        for (fault = 1U; fault <= 6U; ++fault)
        {
            for (fault_byte = 0U; fault_byte < 6U; ++fault_byte)
            {
                /* 最后一个字节收完后不再发送，TDBE低不影响已完成帧。 */
                if (fault == 2U && fault_byte >= (BSP_ENCODER_WORD_TRANSFER ? 4U : 5U))
                {
                    continue;
                }
                make_reply(0x123456U, 0U);
                unsigned before = resets;
                check_sample(pipeline, BSP_TIMEOUT, 0U);
                /* 同步回退先尝试一次Begin，清残留可能比纯同步旧夹具多一次复位。 */
                assert(resets >= before + 1U && resets <= before + 2U);
                unsigned saved = fault;
                fault = 0U;
                check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));
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
        assert(BspEncoder_Begin() == BSP_OK && !selected);
        BspEncoderSample denied;
        assert(BspEncoder_Complete(&denied) == BSP_OK);
        check_sample(0U, BSP_OK, (uint16_t)(0x123456U >> 5));
    }
    sim_spi.ctrl1_bit.spien = 0U;
    assert(BspEncoder_Begin() == BSP_OK);
    BspEncoderSample not_enabled;
    assert(BspEncoder_Complete(&not_enabled) == BSP_OK);
    check_sample(0U, BSP_OK, (uint16_t)(0x123456U >> 5));

    assert(BspEncoder_Begin() == BSP_OK);
    unsigned before = selects;
    BspEncoder_CaptureConfig();
    assert(selected && count == (BSP_ENCODER_WORD_TRANSFER ? 2U : 1U) && selects == before);
    assert(BspEncoder_Begin() == BSP_BUSY);
    BspEncoderSample sample;
    assert(BspEncoder_Complete(&sample) == BSP_OK);
    assert(!selected && selects == releases);
    check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));

    assert(BspEncoder_Begin() == BSP_OK);
    sim_dwt.CYCCNT += ENCODER_SPI_TIMEOUT_CYCLES;
    assert(BspEncoder_Complete(&sample) == BSP_TIMEOUT);
    /* 模拟APP两段SPI之间25us计算；独立台架20us拒绝，完整APP40us允许。 */
    make_reply(0x123456U, 0U);
    assert(BspEncoder_Begin() == BSP_OK);
    (void)BspEncoder_BeginReadback();
    sim_dwt.CYCCNT += R40_CORE_HZ / 40000U;
#if defined(BSP_APP_LIFECYCLE) && BSP_APP_LIFECYCLE
    assert(BspEncoder_Complete(&sample) == BSP_OK);
#else
    assert(BspEncoder_Complete(&sample) == BSP_TIMEOUT);
#endif
    sim_dwt.CYCCNT = UINT32_MAX - 100U;
    check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));
    assert(BspEncoder_Begin() == BSP_OK);
    clock_stopped = true;
    assert(BspEncoder_Complete(&sample) == BSP_TIMEOUT);
    assert(s_polls == ENCODER_SPI_POLL_LIMIT + 1U);
    clock_stopped = false;
    check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));

    assert(BspEncoder_Begin() == BSP_OK);
    assert(BspEncoder_BeginReadback() == BSP_OK);
    assert(BspEncoder_BeginReadback() == BSP_FRAME_ERROR);
    assert(BspEncoder_Complete(&sample) == BSP_FRAME_ERROR);
    assert(BspEncoder_Complete(NULL) == BSP_NOT_READY);
    assert(!selected && selects == releases);

    /* 诊断拥有总线时，抢占它的快环失败不能结束或改写诊断事务。 */
    assert(start_frame(ENCODER_SPI_OWNER_DIAGNOSTIC, &(EncoderSpiRequest){0x3007U, 1U}));
    before = selects;
    assert(BspEncoder_Begin() == BSP_OK);
    assert(BspEncoder_Complete(&sample) == BSP_TIMEOUT);
    assert(selected && count == 1U && selects == before && s_owner == ENCODER_SPI_OWNER_DIAGNOSTIC);
    (void)finish_frame(false);
    check_sample(2U, BSP_OK, (uint16_t)(0x123456U >> 5));
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
    assert(Encoder_BeginSample());
    assert(encoder_read_frame(&encoder, &raw, 1U));
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

/* 片选保护覆盖计数回绕与停走；加速时间推进不能缩短最低周期数。 */
static void test_cs_guard(void)
{
    for (unsigned step = 1U; step <= 17U; step += 4U)
    {
        nop_step = step;
        sim_dwt.CYCCNT = UINT32_MAX - 10U;
        uint32_t before = sim_dwt.CYCCNT;
        nop_calls = 0U;
        cs_guard();
        assert((uint32_t)(sim_dwt.CYCCNT - before) >= 36U);
        assert(nop_calls <= 36U);
    }
    clock_stopped = true;
    nop_calls = 0U;
    cs_guard();
    assert(nop_calls == 36U);
    clock_stopped = false;
    nop_step = 1U;
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
            Encoder_Init();
            assert(!"initialization should halt");
        }
        assert(!s_initialized && power_off);
        puts("PASS MT6835: initialization fails closed");
        return 0;
    }
    test_cs_guard();
    memset(registers, 0x5A, sizeof(registers));
    Encoder_Init();
    assert(BspEncoder_GetType() == BSP_ENCODER_MT6835 && mux_count == 3U);
    assert(sim_config.transmission_mode == SPI_TRANSMIT_FULL_DUPLEX);
    assert(sim_config.master_slave_mode == SPI_MODE_MASTER);
    assert(sim_config.mclk_freq_division ==
           (BSP_ENCODER_FAST_SPI ? SPI_MCLK_DIV_16 : SPI_MCLK_DIV_32));
    assert(sim_config.first_bit_transmission == SPI_FIRST_BIT_MSB);
    assert(sim_config.frame_bit_num == SPI_FRAME_8BIT);
    assert(sim_config.clock_polarity == SPI_CLOCK_POLARITY_HIGH);
    assert(sim_config.clock_phase == SPI_CLOCK_PHASE_2EDGE);
    assert(sim_config.cs_mode_selection == SPI_CS_SOFTWARE_MODE);
    assert(sim_spi.ctrl1_bit.swcsil && sim_spi.ctrl1_bit.spien);
    unsigned before = selects;
    Encoder_Init();
    assert(selects == before);
    for (unsigned i = 0U; i < BSP_ENCODER_CONFIG_WORDS; ++i)
    {
        uint16_t word = 0U;
        assert(BspEncoder_ReadConfigWord((uint8_t)i, &word) == BSP_OK && word == 0x5AU);
    }
    uint16_t invalid = 0x1357U;
    assert(BspEncoder_ReadConfigWord(6U, &invalid) == BSP_INVALID_ARGUMENT && invalid == 0x1357U);
    drop_config_address = 9U;
    memset(registers, 0xA5, sizeof(registers));
    BspEncoder_CaptureConfig();
    for (unsigned i = 0U; i < BSP_ENCODER_CONFIG_WORDS; ++i)
    {
        uint16_t word = 0x1357U;
        BspResult read = BspEncoder_ReadConfigWord((uint8_t)i, &word);
        assert(i == 2U ? read == BSP_NOT_READY && word == 0x1357U
                       : read == BSP_OK && word == 0xA5U);
    }
    drop_config_address = UINT_MAX;
#if BSP_ENCODER_DMA_TRANSFER
    test_dma_status_and_stop();
#endif
    test_samples();
    test_faults();
    make_reply(0x123456U, 0U);
    assert(BspEncoder_Begin() == BSP_OK);
    (void)BspEncoder_BeginReadback();
    assert(BspEncoder_Complete(NULL) == BSP_INVALID_ARGUMENT);
    assert(!selected && !pending && !irq && selects == releases);
    test_angle_contract();
    puts("PASS MT6835: 393216 angle/CRC buckets, all status and single-bit errors, "
         "lifecycle/timeouts/CS/recovery/config");
    return 0;
}
