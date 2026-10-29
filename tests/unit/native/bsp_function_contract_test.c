/* 执行真实共用BSP；只在MCU边界注入硬件事件，覆盖新功能合同和失败收尾。 */
#include <assert.h>
#include <math.h>
#include <fenv.h>
#include <float.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_system.h"
#include "bsp_config.h"
#include "pwm_output_config.h"
#include "sampling_scale.h"
#include "bsp_internal.h"
static uint32_t mask, now_us = 100U, writes, initializations, period_reads;
static uint16_t duty_counts[3], adc_counts[4] = {8193U, 8192U, 8191U, 12000U};
static bool any_output, all_outputs, master, can_write = true, healthy = true, complete = true;
static bool fail_enable, fail_close;
static unsigned inject_frames, inject_offsets, inject_timestamp;
static bool require_unmasked_read;
static unsigned pwm_reads, critical_entries;
static AdcTemperatureSample s_temperature_input;
static bool s_temperature_ready, s_inject_temperature, s_inject_aux;
static AdcTemperatureValue s_temperature_value;
static unsigned s_temperature_conversions;
static jmp_buf fatal;
uint32_t BspSystem_EnterCritical(void)
{
    ++critical_entries;
    uint32_t old = mask;
    mask = 1U;
    return old;
}
void BspSystem_ExitCritical(uint32_t old)
{
    mask = old;
}
uint32_t BspSystem_TimeUs(void)
{
    if (inject_timestamp != 0U)
    {
        --inject_timestamp;
        complete = true;
        assert(Adc_CompleteFast());
    }
    return now_us;
}
uint32_t BspSystem_TimeMs(void)
{
    return now_us / 1000U;
}
void McuPwm_Init(void)
{
    ++initializations;
}
void McuPwm_EnableOutputs(void)
{
    any_output = master = true;
    all_outputs = !fail_enable;
}
void McuPwm_DisableOutputs(void)
{
    any_output = fail_close;
    all_outputs = master = false;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return !any_output;
}
float McuPwm_PeriodCounts(void)
{
    assert(initializations == 1U);
    ++period_reads;
    return 4500.0f;
}
bool McuPwm_CommitThree(uint16_t a, uint16_t b, uint16_t c)
{
    assert(period_reads == 1U);
    if (!can_write && (any_output || master))
    {
        return false;
    }
    duty_counts[0] = a;
    duty_counts[1] = b;
    duty_counts[2] = c;
    ++writes;
    return true;
}
bool McuPwm_CommitPhase(BspPwmPhase p, uint16_t value)
{
    assert(period_reads == 1U);
    if (!can_write && (any_output || master))
    {
        return false;
    }
    duty_counts[p] = value;
    ++writes;
    return true;
}
void McuPwm_ReadOutputs(bool *any, bool *all, bool *moe)
{
    ++pwm_reads;
    *any = any_output;
    *all = all_outputs;
    *moe = master;
}

static void pwm_snapshot_contract(void)
{
    /* 串行所有者下每次只读一次；不依赖BSP内部屏蔽中断，也不改变调用方已有屏蔽状态。 */
    for (unsigned masked = 0U; masked <= 1U; ++masked)
    {
        mask = masked;
        unsigned before = critical_entries;
        for (unsigned bits = 0U; bits < 8U; ++bits)
        {
            any_output = (bits & 1U) != 0U;
            all_outputs = (bits & 2U) != 0U;
            master = (bits & 4U) != 0U;
            BspPwmState expected = !any_output && !master  ? BSP_PWM_OFF
                                   : all_outputs && master ? BSP_PWM_ON
                                                           : BSP_PWM_INCOMPLETE;
            pwm_reads = 0U;
            BspPwmStatus status = BspPwm_GetStatus();
            assert(status.state == expected && pwm_reads == 1U);
            uint32_t sequence = status.disable_sequence;
            BspPwm_Disable();
            assert(BspPwm_GetStatus().disable_sequence == sequence + 1U);
        }
        assert(BspPwm_SetDuty(.5f, .5f, .5f) == BSP_OK);
        assert(BspPwm_SetPhaseDuty(BSP_PWM_PHASE_A, .5f) == BSP_OK);
        assert(BspPwm_Enable() == BSP_OK);
        BspPwm_Disable();
        assert(critical_entries == before && mask == masked);
    }
    mask = 0U;
}
void McuPwm_Fatal(void)
{
    assert(any_output && !master);
    longjmp(fatal, 1);
}
bool McuAdc_Init(void)
{
    return true;
}
bool McuAdc_Start(void)
{
    return true;
}
bool McuAdc_CompleteFast(void)
{
    bool ready = complete;
    complete = false;
    return ready;
}
bool Adc_IsHealthy(void)
{
    if (require_unmasked_read)
    {
        assert(mask == 0U);
    }
    if (inject_frames != 0U)
    {
        --inject_frames;
        complete = true;
        ++adc_counts[0];
        assert(Adc_CompleteFast());
    }
    if (inject_offsets != 0U)
    {
        --inject_offsets;
        const float offsets[] = {2047.0f, 2047.0f, 2047.0f};
        assert(BspAdc_SetPhaseOffsets(offsets) == BSP_OK);
    }
    return healthy;
}
uint16_t McuAdc_ReadRank(unsigned rank)
{
    assert(rank < 4U);
    return adc_counts[rank];
}
uint16_t McuAdc_ReadBusCurrent(void)
{
    return 2050U;
}
bool McuAdc_PollTemperature(AdcTemperatureSample *out)
{
    if (!s_temperature_ready)
    {
        return false;
    }
    *out = s_temperature_input;
    s_temperature_ready = false;
    return true;
}
void McuAdc_ContinueTemperature(void)
{
}
void McuAdc_ConvertTemperature(const AdcTemperatureSample *sample, AdcTemperatureValue *out)
{
    assert(sample->mcu_temperature_counts == 1200U || sample->mcu_temperature_counts == 1201U);
    ++s_temperature_conversions;
    *out = s_temperature_value;
    if (s_inject_temperature)
    {
        s_inject_temperature = false;
        AdcTemperatureSample newer = {1250U, 1600U, 0U, 0U};
        AdcFrames_PublishTemperature(&newer, 10U);
        now_us = 10000U;
    }
}
BspResult McuAdc_MotorTemperature(uint16_t counts, float *celsius)
{
    (void)counts;
    (void)celsius;
    if (s_inject_aux)
    {
        s_inject_aux = false;
        AdcAuxSample newer = {1900U, 0U, 0U};
        AdcFrames_PublishAux(&newer, 20U);
        now_us = 20000U;
    }
    return BSP_SENSOR_FAULT;
}
static void adc_contract(void)
{
    BspAdcFastSample value, before;
    memset(&value, 0xA5, sizeof(value));
    before = value;
    assert(BspAdc_ReadFast(&value) == BSP_NOT_READY && memcmp(&value, &before, sizeof(value)) == 0);
    assert(Adc_Init() && !Adc_Init() && Adc_Start() && !Adc_Start());
    assert(Adc_CompleteFast() && !Adc_CompleteFast());
    assert(BspAdc_ReadFast(&value) == BSP_OK);
    assert(value.valid_mask == BSP_ADC_FAST_BUS_VOLTAGE_VALID && isnan(value.phase_current_a[0]) &&
           isnan(value.bus_current_a));
    BspAdcValue uncalibrated = {123.0f, 456U, 789U};
    assert(BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_A, &uncalibrated) == BSP_NOT_CALIBRATED);
    assert(uncalibrated.value == 123.0f && uncalibrated.sequence == 456U &&
           uncalibrated.sampled_ms == 789U);
    uint32_t sequence = value.sequence;
    BspAdcRawCurrent raw;
    assert(BspAdc_ReadRawCurrent(&raw) == BSP_OK && raw.phase_counts[0] == 2048.25f &&
           raw.sequence == sequence);
    assert((raw.valid_mask & BSP_ADC_FAST_PHASE_CURRENT_VALID) != 0U);
    assert(((raw.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) != 0U) == !!BSP_ENABLED_BUS_CURRENT);
    const float offsets[] = {2048.0f, 2048.0f, 2048.0f};
    any_output = true;
    assert(BspAdc_SetPhaseOffsets(offsets) == BSP_REJECTED);
    BspPwm_Disable();
    assert(BspAdc_SetPhaseOffsets(offsets) == BSP_OK);
    assert(BspAdc_ReadFast(&value) == BSP_OK && value.sequence == sequence);
    assert((value.valid_mask & BSP_ADC_FAST_PHASE_CURRENT_VALID) != 0U);
    assert(fabsf(value.phase_current_a[0] + .25f * MOTOR_SENSING_CURRENT_A_PER_COUNT) < 1e-6f);
    assert(value.phase_current_a[1] == 0.0f &&
           fabsf(value.phase_current_a[2] - .25f * MOTOR_SENSING_CURRENT_A_PER_COUNT) < 1e-6f);
    assert(BspAdc_SetBusCurrentOffset(2048.0f) ==
           (BSP_ENABLED_BUS_CURRENT ? BSP_OK : BSP_NOT_SUPPORTED));
    assert(BspAdc_ReadFast(&value) == BSP_OK && value.sequence == sequence);
    assert(((value.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) != 0U) ==
           !!BSP_ENABLED_BUS_CURRENT);
    BspAdcValue channel;
    assert(BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_A, &channel) == BSP_OK &&
           channel.sequence == sequence && channel.value == value.phase_current_a[0]);
    BspAdcRawValue raw_channel;
    memset(&raw_channel, 0x5A, sizeof(raw_channel));
    BspAdcRawValue raw_before = raw_channel;
    assert(BspAdc_ReadRawChannel(BSP_ADC_CHANNEL_COUNT, &raw_channel) == BSP_INVALID_ARGUMENT &&
           memcmp(&raw_channel, &raw_before, sizeof(raw_channel)) == 0);
    assert(BspAdc_ReadRawChannel(BSP_ADC_HARDWARE_VERSION, &raw_channel) == BSP_NOT_SUPPORTED);
    float invalid[] = {2048.0f, NAN, 2048.0f};
    assert(BspAdc_SetPhaseOffsets(invalid) == BSP_INVALID_ARGUMENT);
    now_us += 101U;
    before = value;
    assert(BspAdc_ReadFast(&value) == BSP_STALE && memcmp(&value, &before, sizeof(value)) == 0);
    now_us -= 101U;
    healthy = false;
    assert(BspAdc_ReadFast(&value) == BSP_IO_ERROR && memcmp(&value, &before, sizeof(value)) == 0);
    healthy = true;
    require_unmasked_read = true;
    inject_frames = 1U;
    assert(BspAdc_ReadFast(&value) == BSP_OK && value.sequence == sequence + 1U);
    require_unmasked_read = false;
    inject_frames = 2U;
    assert(BspAdc_ReadFast(&value) == BSP_OK && value.sequence == sequence + 3U);
    assert(inject_frames == 0U);
    require_unmasked_read = true;
    inject_offsets = 1U;
    assert(BspAdc_ReadRawCurrent(&raw) == BSP_OK && raw.sequence == sequence + 3U);
    inject_offsets = 2U;
    require_unmasked_read = false;
    assert(BspAdc_ReadFast(&value) == BSP_OK && inject_offsets == 0U);
    sequence = value.sequence;
    inject_timestamp = 1U;
    assert(BspAdc_ReadFast(&value) == BSP_OK && value.sequence == sequence + 1U);
    require_unmasked_read = false;
    /* 单通道物理值/原码必须与同一快帧一致，读取不新增采集或改动序号和时间。 */
    uint32_t random = 0x12345678U;
    for (unsigned n = 0U; n < 4096U; ++n)
    {
        for (unsigned rank = 0U; rank < 4U; ++rank)
        {
            random = random * 1664525U + 1013904223U;
            adc_counts[rank] = (uint16_t)(random % 16381U);
        }
        now_us += 50U;
        complete = true;
        unsigned before_publish = critical_entries;
        assert(Adc_CompleteFast());
#ifndef ADC_REFERENCE_WITH_CRITICAL
        assert(critical_entries == before_publish);
#else
        (void)before_publish;
#endif
        assert(BspAdc_ReadFast(&value) == BSP_OK);
        assert(BspAdc_ReadRawCurrent(&raw) == BSP_OK && raw.sequence == value.sequence);
        for (unsigned id = 0U; id <= BSP_ADC_BUS_CURRENT; ++id)
        {
            BspAdcChannel signal = (BspAdcChannel)id;
            BspAdcValue physical;
            BspAdcRawValue counts;
            if (id == BSP_ADC_BUS_CURRENT && !BSP_ENABLED_BUS_CURRENT)
            {
                assert(BspAdc_ReadChannel(signal, &physical) == BSP_NOT_SUPPORTED);
                assert(BspAdc_ReadRawChannel(signal, &counts) == BSP_NOT_SUPPORTED);
                continue;
            }
            float expected = id <= BSP_ADC_PHASE_CURRENT_C ? value.phase_current_a[id]
                             : id == BSP_ADC_BUS_VOLTAGE   ? value.bus_voltage_v
                                                           : value.bus_current_a;
            float expected_counts =
                id <= BSP_ADC_PHASE_CURRENT_C ? raw.phase_counts[id]
                : id == BSP_ADC_BUS_VOLTAGE
                    ? adc_counts[BSP_SAMPLE_VBUS_RANK] * MOTOR_SENSING_ADC_SUM_TO_COUNTS
                    : raw.bus_current_counts;
            assert(BspAdc_ReadChannel(signal, &physical) == BSP_OK);
            assert(memcmp(&physical.value, &expected, sizeof(expected)) == 0);
            assert(physical.sequence == value.sequence && physical.sampled_ms == now_us / 1000U);
            assert(BspAdc_ReadRawChannel(signal, &counts) == BSP_OK);
            assert(counts.counts == expected_counts && counts.sequence == value.sequence);
            assert(counts.sampled_ms == physical.sampled_ms && !counts.reference_valid);
        }
    }
    now_us = UINT32_MAX - 50U;
    complete = true;
    assert(Adc_CompleteFast());
    /* 原码读取不依赖物理量运算；大零偏会让物理换算产生舍入，原码路径不应产生。 */
    const float large_offsets[] = {FLT_MAX, FLT_MAX, FLT_MAX};
    const float restored_offsets[] = {2047.0f, 2047.0f, 2047.0f};
    assert(BspAdc_SetPhaseOffsets(large_offsets) == BSP_OK);
    assert(feclearexcept(FE_ALL_EXCEPT) == 0);
    assert(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_A, &raw_channel) == BSP_OK);
    assert(fetestexcept(FE_INEXACT) == 0);
    assert(BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_A, &channel) == BSP_OK);
    assert(isfinite(channel.value) && fetestexcept(FE_INEXACT) != 0);
    assert(BspAdc_SetPhaseOffsets(restored_offsets) == BSP_OK);
#if BSP_ENABLED_BUS_CURRENT
    assert(BspAdc_SetBusCurrentOffset(FLT_MAX) == BSP_OK);
    assert(feclearexcept(FE_ALL_EXCEPT) == 0);
    assert(BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT, &raw_channel) == BSP_OK);
    assert(fetestexcept(FE_INEXACT) == 0);
    assert(BspAdc_ReadChannel(BSP_ADC_BUS_CURRENT, &channel) == BSP_OK);
    assert(isfinite(channel.value) && fetestexcept(FE_INEXACT) != 0);
    assert(BspAdc_SetBusCurrentOffset(2048.0f) == BSP_OK);
#endif
    assert(feclearexcept(FE_ALL_EXCEPT) == 0);
    now_us += 100U;
    assert(BspAdc_ReadFast(&value) == BSP_OK);
    before = value;
    ++now_us;
    assert(BspAdc_ReadFast(&value) == BSP_STALE && memcmp(&value, &before, sizeof(value)) == 0);
    AdcFrames_Invalidate();
    assert(BspAdc_ReadFast(&value) == BSP_NOT_READY);
}

static void publication_sequence_contract(void)
{
    /* 输入序号不授予发布权；两组独立计数，失效不重置，读取不推进。 */
    AdcAuxSample aux = {321U, 0x12345678U, 0xABCDEF12U};
    AdcTemperatureSample temperature = {1200U, 1500U, 0x87654321U, 0xABCDEF12U};
    uint32_t aux_sequence = AdcFrames_PublishAux(&aux, now_us / 1000U);
    uint32_t temperature_sequence = AdcFrames_PublishTemperature(&temperature, now_us / 1000U);
    assert(aux.sequence == 0x12345678U && temperature.sequence == 0x87654321U);
    assert(aux_sequence == (BSP_ENABLED_AUX_SAMPLING ? 1U : 0U));
    assert(temperature_sequence == (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING ? 1U : 0U));
#if BSP_ENABLED_AUX_SAMPLING
    for (unsigned i = 0U; i < 3U; ++i)
    {
        AdcAuxSample received;
        assert(Adc_ReadAux(&received));
        assert(received.sequence == 1U && received.motor_temperature_counts == 321U);
        assert(received.sampled_ms == now_us / 1000U);
    }
#endif
    AdcFrames_Invalidate();
    assert(AdcFrames_PublishAux(&aux, now_us / 1000U) == (BSP_ENABLED_AUX_SAMPLING ? 2U : 0U));
    assert(AdcFrames_PublishTemperature(&temperature, now_us / 1000U) ==
           (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING ? 2U : 0U));
}

#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
static void temperature_input(uint16_t ts, uint16_t ref, uint32_t ms)
{
    s_temperature_input = (AdcTemperatureSample){ts, ref, 0U, 0U};
#if BSP_TEMPERATURE_SOFTWARE_TRIGGER
    (void)ms;
    s_temperature_ready = true;
#else
    AdcFrames_PublishTemperature(&s_temperature_input, ms);
#endif
}
#endif
static void auxiliary_channel_contract(void)
{
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    now_us = 8000U;
    temperature_input(1200U, 1500U, 7U);
    s_temperature_value = (AdcTemperatureValue){NAN, NAN, BSP_NOT_CALIBRATED, BSP_NOT_CALIBRATED};
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    BspAdcRawValue raw;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK);
    assert(raw.counts == 1200.0f && raw.reference_counts == 1500.0f && raw.reference_valid);
    const uint32_t first_sequence = raw.sequence;
    assert(raw.sampled_ms == (BSP_TEMPERATURE_SOFTWARE_TRIGGER ? 8U : 7U));
    BspAdcValue value = {123.0f, 456U, 789U}, previous = value;
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_NOT_CALIBRATED);
    assert(memcmp(&value, &previous, sizeof(value)) == 0);
    assert(BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &value) == BSP_NOT_CALIBRATED);
    assert(memcmp(&value, &previous, sizeof(value)) == 0);
    assert(BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &raw) == BSP_OK);
    assert(raw.counts == 1500.0f && !raw.reference_valid);
    unsigned conversions = s_temperature_conversions;
    assert(BspAdc_UpdateAuxiliary() == BSP_NOT_READY && s_temperature_conversions == conversions);

    now_us = 9000U;
    temperature_input(1201U, 1501U, 8U);
    s_temperature_value = (AdcTemperatureValue){42.5f, 3.125f, BSP_OK, BSP_OK};
    s_inject_temperature = true;
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    conversions = s_temperature_conversions;
    const uint32_t sampled_ms = BSP_TEMPERATURE_SOFTWARE_TRIGGER ? 9U : 8U;
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_OK);
    assert(value.value == 42.5f && value.sequence == first_sequence + 1U &&
           value.sampled_ms == sampled_ms);
    assert(BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &value) == BSP_OK);
    assert(value.value == 3.125f && value.sequence == first_sequence + 1U &&
           value.sampled_ms == sampled_ms);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK);
    assert(raw.counts == 1201.0f && raw.reference_counts == 1501.0f);
    assert(raw.sequence == first_sequence + 1U && raw.sampled_ms == sampled_ms);
    assert(s_temperature_conversions == conversions);

    previous = value;
    BspAdcRawValue raw_previous = raw;
    healthy = false;
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_IO_ERROR);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_IO_ERROR);
    assert(memcmp(&value, &previous, sizeof(value)) == 0);
    assert(memcmp(&raw, &raw_previous, sizeof(raw)) == 0);
    healthy = true;
    now_us += (BSP_TEMPERATURE_MAX_AGE_MS + 1U) * 1000U;
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_STALE);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_STALE);
    assert(memcmp(&value, &previous, sizeof(value)) == 0);
    assert(memcmp(&raw, &raw_previous, sizeof(raw)) == 0);
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE
    now_us = 19000U;
    AdcAuxSample aux = {1800U, 0U, 0U};
    uint32_t sequence = AdcFrames_PublishAux(&aux, 18U);
    s_inject_aux = true;
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    BspAdcRawValue ntc_raw;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE, &ntc_raw) == BSP_OK);
    assert(ntc_raw.counts == 1800.0f && ntc_raw.sequence == sequence && ntc_raw.sampled_ms == 18U);
    BspAdcValue ntc_value;
    assert(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE, &ntc_value) == BSP_SENSOR_FAULT);
#endif
}
static void uart_contract(void)
{
    uint8_t data = 0xA5U;
    size_t received = 99U;
    BspUartTxResult result = BSP_UART_TX_OK;
    assert(BspUart_TryWrite(&data, 1U) == BSP_NOT_SUPPORTED);
    assert(BspUart_TakeTxResult(&result) == BSP_NOT_SUPPORTED && result == BSP_UART_TX_OK);
    assert(BspUart_Read(&data, 1U, &received) == BSP_NOT_SUPPORTED && received == 0U &&
           data == 0xA5U);
    assert(BspUart_GetState() == BSP_UART_DISABLED);
}
int main(int argc, char **argv)
{
    unsigned scenario = argc > 1 ? (unsigned)atoi(argv[1]) : 0U;
    /* 返回路径精简仍须先拒绝非法参数，再报告未初始化，且不触及MCU提交。 */
    assert(BspPwm_SetDuty(NAN, .5f, .5f) == BSP_INVALID_ARGUMENT && writes == 0U);
    assert(BspPwm_SetPhaseDuty((BspPwmPhase)-1, .5f) == BSP_INVALID_ARGUMENT && writes == 0U);
    assert(BspPwm_SetPhaseDuty(BSP_PWM_PHASE_A, NAN) == BSP_INVALID_ARGUMENT && writes == 0U);
    assert(BspPwm_SetPhaseDuty(BSP_PWM_PHASE_A, .5f) == BSP_NOT_READY && writes == 0U);
    assert(BspPwm_SetDuty(.5f, .5f, .5f) == BSP_NOT_READY && writes == 0U);
    Pwm_Init();
    Pwm_Init();
    assert(initializations == 1U && period_reads == 1U);
    pwm_snapshot_contract();
    /* 空闲载波即使已接近换相沿，完全关相的预装载也不会产生功率输出。 */
    can_write = false;
    assert(BspPwm_SetDuty(.5f, .5f, .5f) == BSP_OK);
    assert(BspPwm_SetPhaseDuty(BSP_PWM_PHASE_B, .25f) == BSP_OK);
    assert(BspPwm_GetStatus().state == BSP_PWM_OFF);
    if (scenario >= 3U)
    {
        /* 主输出未关或仍有单通道开启，都不能当作安全关相绕过提交期限。 */
        any_output = scenario == 3U;
        master = scenario == 4U;
        uint32_t before = writes;
        assert((scenario == 3U ? BspPwm_SetDuty(.2f, .3f, .4f)
                               : BspPwm_SetPhaseDuty(BSP_PWM_PHASE_A, .2f)) == BSP_TIMEOUT);
        assert(writes == before && BspPwm_GetStatus().state == BSP_PWM_OFF);
        assert(BspPwm_Enable() == BSP_REJECTED);
        assert(mask == 0U);
        return 0;
    }
    can_write = true;
    assert(BspPwm_SetDuty(.5f, .25f, .75f) == BSP_OK && duty_counts[0] == 2250U &&
           duty_counts[1] == 1125U && duty_counts[2] == 3375U);
    /* 边界位型覆盖负零、次正规数、1相邻值和两种符号NaN。 */
    const uint32_t patterns[] = {0U,
                                 0x80000000U,
                                 1U,
                                 0x3F800000U,
                                 0x3F800001U,
                                 0x80000001U,
                                 0x7F800000U,
                                 0xFF800000U,
                                 0x7FC00000U,
                                 0x7F800001U,
                                 0xFFC00000U};
    for (unsigned i = 0U; i < sizeof(patterns) / sizeof(patterns[0]); ++i)
    {
        float duty;
        memcpy(&duty, &patterns[i], sizeof(duty));
        bool expected = isfinite(duty) && duty >= 0.0f && duty <= 1.0f;
        uint32_t before = writes;
        BspResult result = BspPwm_SetDuty(duty, .5f, .5f);
        assert(result == (expected ? BSP_OK : BSP_INVALID_ARGUMENT));
        assert(writes == before + (expected ? 1U : 0U));
    }
    uint32_t old_writes = writes;
    assert(BspPwm_SetDuty(.1f, NAN, .3f) == BSP_INVALID_ARGUMENT && writes == old_writes);
    assert(BspPwm_SetPhaseDuty((BspPwmPhase)99U, .5f) == BSP_INVALID_ARGUMENT &&
           writes == old_writes);
    assert(BspPwm_SetDuty(-.1f, .2f, .3f) == BSP_INVALID_ARGUMENT && writes == old_writes);
    adc_contract();
    publication_sequence_contract();
    auxiliary_channel_contract();
    uart_contract();
    uint32_t sequence = BspPwm_GetStatus().disable_sequence;
    if (scenario == 0U)
    {
        assert(BspPwm_Enable() == BSP_OK && BspPwm_GetStatus().state == BSP_PWM_ON);
        can_write = false;
        assert(BspPwm_SetDuty(.2f, .3f, .4f) == BSP_TIMEOUT && writes == old_writes);
        assert(BspPwm_GetStatus().state == BSP_PWM_OFF &&
               BspPwm_GetStatus().disable_sequence == sequence + 1U);
        can_write = true;
        assert(BspPwm_Enable() == BSP_REJECTED);
    }
    else
    {
        fail_enable = true;
        fail_close = scenario == 2U;
        if (scenario == 2U)
        {
            if (setjmp(fatal) == 0)
            {
                (void)BspPwm_Enable();
                assert(false);
            }
            assert(BspPwm_GetStatus().state == BSP_PWM_INCOMPLETE);
            return 0;
        }
        assert(BspPwm_Enable() == BSP_IO_ERROR && BspPwm_GetStatus().state == BSP_PWM_OFF);
        assert(BspPwm_GetStatus().disable_sequence == sequence + 1U);
        fail_enable = false;
        assert(BspPwm_Enable() == BSP_REJECTED);
    }
    assert(mask == 0U);
    puts("PASS actual common BSP: ADC masks/offset/freshness, serialized PWM "
         "validation/late/partial/fatal close, disabled UART");
    return 0;
}
