/* 真实APP状态装配及协议编码测试；仅用内存替身提供ADC、时钟和关相事实。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bsp_config.h"
#include "bus_current_feedback.h"
#include "motor_axis.h"
#include "yg_protocol_link.h"
#include "yg_protocol_status_adapter.h"

static const yg_protocol_motor_status_source_t *s_bound;
static MotorStatus s_motor = {.bus_current = 0.75f, .bus_voltage = 29.0f};

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    s_bound = source;
    return true;
}

bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    return source != NULL;
}

MotorState MotorAxis_ReadStatus(MotorStatus *sample,
                                uint8_t *valid_bits,
                                MotorMaintenanceKind *kind,
                                MotorControlProgress *progress)
{
    *sample = s_motor;
    *valid_bits = 0x54U;
    *kind = MOTOR_MAINTENANCE_NONE;
    *progress = (MotorControlProgress){0};
    return MOTOR_STATE_IDLE;
}

static yg_protocol_motor_feedback_t feedback(void)
{
    YgProtocolStatusAdapter_Refresh();
    yg_protocol_motor_feedback_t value;
    assert(yg_protocol_motor_feedback_from_source(s_bound, 1U, 0U, 0U, &value) == YG_PROTOCOL_OK);
    /* 除母线电流外，装配不能污染原电机状态或有效位。 */
    MotorStatus copy = s_bound->sample;
    copy.bus_current = s_motor.bus_current;
    assert(memcmp(&copy, &s_motor, sizeof(copy)) == 0);
    assert((s_bound->measurement_valid_bits & (uint8_t)~0x40U) == 0x14U);
    assert(s_motor.bus_current == 0.75f);
    return value;
}

#if BSP_ENABLED_BUS_CURRENT
#include "../../../firmware/app/runtime/bus_current_feedback.c"
#define BUS_CURRENT_A_PER_COUNT                                                                    \
    (BUS_CURRENT_ADC_REFERENCE_V / (float)BUS_CURRENT_ADC_MAX_COUNTS / BUS_CURRENT_GAIN /          \
     BUS_CURRENT_SHUNT_OHM)

static uint32_t s_now, s_seq, s_critical_depth;
static bool s_available = true, s_disabled = true;
static uint16_t s_raw;
static unsigned s_read_race, s_raw_reads;

uint32_t BspSystem_TimeMs(void)
{
    return s_now;
}

uint32_t BspSystem_EnterCritical(void)
{
    return s_critical_depth++;
}

void BspSystem_ExitCritical(uint32_t previous)
{
    assert(s_critical_depth == previous + 1U);
    s_critical_depth = previous;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return s_disabled;
}

static float s_offset;
BspResult BspAdc_ReadRawCurrent(BspAdcRawCurrent *sample)
{
    assert(s_critical_depth == 0U);
    ++s_raw_reads;
    if (s_read_race != 0U)
    {
        if (s_read_race == 1U)
        {
            g_r40_bus_current_diagnostic.sample_valid = false;
            s_read_race = 0U;
        }
        else
        {
            ++g_r40_bus_current_diagnostic.sequence;
            g_r40_bus_current_diagnostic.filtered_a = 0.25f;
            if (s_read_race == 2U)
            {
                s_read_race = 0U;
            }
        }
    }
    if (!s_available)
    {
        return BSP_STALE;
    }
    *sample = (BspAdcRawCurrent){.bus_current_counts = s_raw,
                                 .valid_mask = BSP_ADC_FAST_BUS_CURRENT_VALID,
                                 .sequence = s_seq};
    return BSP_OK;
}
BspResult BspAdc_SetBusCurrentOffset(float offset)
{
    if (!s_disabled)
    {
        return BSP_REJECTED;
    }
    s_offset = offset;
    return BSP_OK;
}
BspResult BspAdc_ReadFast(BspAdcFastSample *sample)
{
    if (!s_available)
    {
        return BSP_STALE;
    }
    *sample =
        (BspAdcFastSample){.bus_current_a = ((float)s_raw - s_offset) * BUS_CURRENT_A_PER_COUNT,
                           .valid_mask = BSP_ADC_FAST_BUS_CURRENT_VALID,
                           .sequence = s_seq};
    return BSP_OK;
}

static void sample(uint16_t raw)
{
    s_raw = raw;
    s_seq += 10U;
    s_now += (s_seq / 10U) & 1U;
    BusCurrentFeedback_Sample();
}

static void calibrate(void)
{
    for (unsigned i = 0U; i < 128U; ++i)
    {
        sample(2060U + (i & 1U));
    }
    assert(g_r40_bus_current_diagnostic.offset_valid);
    assert(g_r40_bus_current_diagnostic.offset_samples == 128U);
    assert(g_r40_bus_current_diagnostic.offset_counts == 2060.5f);
}

static void expect_invalid(void)
{
    yg_protocol_motor_feedback_t value = feedback();
    assert(value.bus_current_mA == INT16_MIN && !(value.valid_bits & (1U << 5U)));
    assert(isnan(s_bound->sample.bus_current));
    assert(s_critical_depth == 0U);
}

static void hardware_feedback(void)
{
    BusCurrentFeedback_Init();
    expect_invalid();
    sample(2060U);
    for (unsigned i = 0U; i < 150U; ++i)
    {
        BusCurrentFeedback_Sample();
    }
    assert(g_r40_bus_current_diagnostic.offset_samples == 1U);
    expect_invalid();

    BusCurrentFeedback_Init();
    s_disabled = false;
    for (unsigned i = 0U; i < 150U; ++i)
    {
        sample(2060U);
    }
    assert(!g_r40_bus_current_diagnostic.offset_valid);
    expect_invalid();
    s_disabled = true;
    calibrate();

    float initial = g_r40_bus_current_diagnostic.filtered_a;
    sample(2160U);
    float expected = (2160.0f - 2060.5f) * BUS_CURRENT_A_PER_COUNT;
    assert(fabsf(g_r40_bus_current_diagnostic.current_a - expected) < 0.000001f);
    float filtered = initial + 0.05f * (expected - initial);
    assert(fabsf(g_r40_bus_current_diagnostic.filtered_a - filtered) < 0.000001f);
    for (unsigned i = 0U; i < 300U; ++i)
    {
        sample(2160U);
    }
    assert(g_r40_bus_current_diagnostic.offset_counts == 2060.5f);
    yg_protocol_motor_feedback_t value = feedback();
    assert(value.bus_current_mA == (int16_t)lroundf(expected * 1000.0f));
    assert(value.valid_bits & (1U << 5U));
    uint8_t wire[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
    assert(yg_protocol_motor_feedback_encode(&value, wire, sizeof(wire)) == YG_PROTOCOL_OK);
    assert(wire[28] == (uint8_t)value.bus_current_mA);
    assert(wire[29] == (uint8_t)((uint16_t)value.bus_current_mA >> 8U));

    /* 再生负电流保留符号，且运行中始终使用相同偏置。 */
    s_disabled = false;
    for (unsigned i = 0U; i < 300U; ++i)
    {
        sample(1961U);
    }
    value = feedback();
    assert(value.bus_current_mA == (int16_t)-lroundf(expected * 1000.0f));
    assert(g_r40_bus_current_diagnostic.offset_counts == 2060.5f);

    for (unsigned i = 0U; i < 10U; ++i)
    {
        BusCurrentFeedback_Sample();
    }
    float unchanged = g_r40_bus_current_diagnostic.filtered_a;
    BusCurrentFeedback_Sample();
    assert(unchanged == g_r40_bus_current_diagnostic.filtered_a);
    s_now += 2U;
    assert(feedback().valid_bits & (1U << 5U));
    ++s_now;
    expect_invalid();
    BusCurrentFeedback_Sample();
    expect_invalid();
    sample(2060U);
    assert(feedback().valid_bits & (1U << 5U));

    const uint16_t invalid[] = {0U, 2U, 4093U, 4095U, 65535U};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        sample(invalid[i]);
        expect_invalid();
        sample(2060U);
        assert(feedback().valid_bits & (1U << 5U));
    }
    s_available = false;
    expect_invalid();
    BusCurrentFeedback_Sample();
    expect_invalid();
    s_available = true;
    sample(2060U);
    assert(feedback().valid_bits & (1U << 5U));

    /* 序号缺口不继续累计校准；计数与毫秒自然回绕仍能采集。 */
    BusCurrentFeedback_Init();
    s_disabled = true;
    sample(2060U);
    sample(2060U);
    s_seq += 20U;
    sample(2060U);
    assert(g_r40_bus_current_diagnostic.offset_samples == 1U);
    BusCurrentFeedback_Init();
    s_seq = UINT32_MAX - 64U;
    s_now = UINT32_MAX - 16U;
    calibrate();
    assert(feedback().valid_bits & (1U << 5U));

    /* 噪声窗口和偏离中点的直流电平不能获准成为零点。 */
    BusCurrentFeedback_Init();
    for (unsigned i = 0U; i < 200U; ++i)
    {
        sample(i & 1U ? 2040U : 2060U);
    }
    assert(!g_r40_bus_current_diagnostic.offset_valid);
    expect_invalid();
    for (unsigned i = 0U; i < 200U; ++i)
    {
        sample(2400U);
    }
    assert(!g_r40_bus_current_diagnostic.offset_valid);
    expect_invalid();
    calibrate();
    assert(!BusCurrentFeedback_Read(NULL));
    float current_a;
    s_read_race = 1U;
    s_raw_reads = 0U;
    assert(!BusCurrentFeedback_Read(&current_a) && isnan(current_a));
    assert(s_raw_reads == 2U);
    sample(2060U);
    s_read_race = 2U;
    s_raw_reads = 0U;
    assert(BusCurrentFeedback_Read(&current_a) && current_a == 0.25f);
    assert(s_raw_reads == 2U);
    s_read_race = 3U;
    s_raw_reads = 0U;
    assert(!BusCurrentFeedback_Read(&current_a) && isnan(current_a));
    assert(s_raw_reads == 2U);
    s_read_race = 0U;
    assert(s_critical_depth == 0U);
    puts("PASS PA4 offset, signed conversion, filter, stale/fault invalidity and APP wire path");
}
#endif

int main(void)
{
    assert(YgProtocolStatusAdapter_Init());
#if BSP_ENABLED_BUS_CURRENT
    hardware_feedback();
#else
    yg_protocol_motor_feedback_t value = feedback();
    assert(value.bus_current_mA == 750 && (value.valid_bits & (1U << 5U)));
    puts("PASS legacy hardware-free estimate and validity remain unchanged");
#endif
    return 0;
}
