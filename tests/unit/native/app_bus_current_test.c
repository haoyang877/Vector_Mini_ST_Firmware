/* 真实APP状态装配及协议编码测试；仅用内存替身提供ADC、时钟和关相事实。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_bus_current.h"
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

#if BOARD_HAS_BUS_CURRENT_SENSOR
#include "../../../firmware/platform/at32f456/application/bus_current_feedback.c"

static uint32_t s_now, s_seq, s_critical_depth;
static bool s_available = true, s_disabled = true;
static uint16_t s_raw;

uint32_t time_hw_now_ms(void)
{
    return s_now;
}

uint32_t critical_hw_enter(void)
{
    return s_critical_depth++;
}

void critical_hw_exit(uint32_t previous)
{
    assert(s_critical_depth == previous + 1U);
    s_critical_depth = previous;
}

bool power_stage_hw_phases_disabled(void)
{
    return s_disabled;
}

bool R40_AdcReadAux(R40AuxSamples *sample)
{
    if (!s_available)
    {
        return false;
    }
    *sample = (R40AuxSamples){.bus_current_counts = s_raw, .sequence = s_seq};
    return true;
}

static void sample(uint16_t raw)
{
    s_raw = raw;
    ++s_seq;
    s_now += s_seq & 1U;
    AppBusCurrent_Sample();
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
    AppBusCurrent_Init();
    expect_invalid();
    sample(2060U);
    for (unsigned i = 0U; i < 150U; ++i)
    {
        AppBusCurrent_Sample();
    }
    assert(g_r40_bus_current_diagnostic.offset_samples == 1U);
    expect_invalid();

    AppBusCurrent_Init();
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
        AppBusCurrent_Sample();
    }
    float unchanged = g_r40_bus_current_diagnostic.filtered_a;
    AppBusCurrent_Sample();
    assert(unchanged == g_r40_bus_current_diagnostic.filtered_a);
    s_now += 2U;
    assert(feedback().valid_bits & (1U << 5U));
    ++s_now;
    expect_invalid();
    AppBusCurrent_Sample();
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
    AppBusCurrent_Sample();
    expect_invalid();
    s_available = true;
    sample(2060U);
    assert(feedback().valid_bits & (1U << 5U));

    /* 序号缺口不继续累计校准；计数与毫秒自然回绕仍能采集。 */
    AppBusCurrent_Init();
    s_disabled = true;
    sample(2060U);
    sample(2060U);
    s_seq += 10U;
    sample(2060U);
    assert(g_r40_bus_current_diagnostic.offset_samples == 1U);
    AppBusCurrent_Init();
    s_seq = UINT32_MAX - 64U;
    s_now = UINT32_MAX - 16U;
    calibrate();
    assert(feedback().valid_bits & (1U << 5U));

    /* 噪声窗口和偏离中点的直流电平不能获准成为零点。 */
    AppBusCurrent_Init();
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
    AppBusCurrent_Apply(NULL, NULL);
    assert(s_critical_depth == 0U);
    puts("PASS PA4 offset, signed conversion, filter, stale/fault invalidity and APP wire path");
}
#endif

int main(void)
{
    assert(YgProtocolStatusAdapter_Init());
#if BOARD_HAS_BUS_CURRENT_SENSOR
    hardware_feedback();
#else
    yg_protocol_motor_feedback_t value = feedback();
    assert(value.bus_current_mA == 750 && (value.valid_bits & (1U << 5U)));
    float unchanged = 0.25f;
    uint8_t bits = 0x95U;
    AppBusCurrent_Apply(&unchanged, &bits);
    assert(unchanged == 0.25f && bits == 0x95U);
    puts("PASS legacy hardware-free estimate and validity remain unchanged");
#endif
    return 0;
}
