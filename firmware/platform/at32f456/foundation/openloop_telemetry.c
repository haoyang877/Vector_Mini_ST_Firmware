#include "openloop_telemetry.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "SEGGER_RTT.h"

/* J-Scope标准帧：微秒时间戳在前，后续11列顺序必须与通道名一致。 */
typedef struct
{
    uint32_t timestamp_us;
    float ia_a, ib_a, ic_a, angle_deg;
    uint32_t sin_raw, cos_raw;
    float speed_rpm;
    uint32_t state, fault;
    float bus_v;
    uint32_t sequence;
} R40RttFrame;

_Static_assert(sizeof(R40RttFrame) == 48U, "RTT frame must match J-Scope format");

volatile R40OpenLoopDebug g_openloop_debug;
static char s_wave_buffer[8192];
static uint32_t s_previous_fast, s_velocity_fast, s_text_fast, s_sent, s_dropped;
static uint16_t s_previous_angle;
static int32_t s_position, s_velocity_position;
static float s_velocity_rpm;
static bool s_initialized;

void R40OpenLoopTelemetry_Init(void)
{
    memset((void *)&g_openloop_debug, 0, sizeof(g_openloop_debug));
    s_previous_fast = s_velocity_fast = s_text_fast = s_sent = s_dropped = 0U;
    s_previous_angle = 0U;
    s_position = s_velocity_position = 0;
    s_velocity_rpm = 0.0f;
    s_initialized = false;
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1U,
                              "JScope_t4f4f4f4f4u4u4f4u4u4f4u4",
                              s_wave_buffer,
                              sizeof(s_wave_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_WriteString(
        0U,
        "R40 openloop: Watch g_openloop_enable=1 starts once; 0 stops.\n"
        "RTT1: us,ia,ib,ic,angle_deg,sin_raw,cos_raw,rpm,state,fault,vbus,seq\n"
        "RTT0: ms,state,fault,ia_mA,ib_mA,ic_mA,angle_mdeg,sin,cos,rpm_milli,drop\n");
}

void R40OpenLoopTelemetry_Publish(const R40OpenLoopTelemetrySample *sample)
{
    uint32_t fast = sample->fast_samples;
    if (s_initialized && fast / R40_RTT_FAST_TICKS == s_previous_fast / R40_RTT_FAST_TICKS)
    {
        return;
    }
    s_previous_fast = fast;
    uint16_t angle = (uint16_t)sample->angle_counts;
    if (!s_initialized)
    {
        s_position = s_velocity_position = angle;
        s_velocity_fast = fast;
        s_initialized = true;
    }
    else
    {
        s_position += (int16_t)(uint16_t)(angle - s_previous_angle);
    }
    s_previous_angle = angle;
    uint32_t velocity_ticks = fast - s_velocity_fast;
    if (velocity_ticks >= 200U)
    {
        s_velocity_rpm = (float)(s_position - s_velocity_position) * (1200000.0f / 65536.0f) /
                         (float)velocity_ticks;
        s_velocity_position = s_position;
        s_velocity_fast = fast;
    }
    uint32_t version = (g_openloop_debug.begin + 2U) & ~1U;
    g_openloop_debug.begin = version | 1U;
    g_openloop_debug.state = sample->state;
    g_openloop_debug.fault = sample->fault;
    g_openloop_debug.elapsed_ms = sample->running_samples / 20U;
    g_openloop_debug.encoder_errors = sample->encoder_errors;
    g_openloop_debug.angle_counts = angle;
    g_openloop_debug.sin_raw = sample->sin_raw;
    g_openloop_debug.cos_raw = sample->cos_raw;
    g_openloop_debug.sincos_sequence = sample->sincos_sequence;
    g_openloop_debug.sincos_age_ms = sample->sincos_age_ms;
    g_openloop_debug.sincos_valid = sample->sincos_valid;
    g_openloop_debug.ia_a = sample->ia_a;
    g_openloop_debug.ib_a = sample->ib_a;
    g_openloop_debug.ic_a = sample->ic_a;
    g_openloop_debug.angle_deg = (float)angle * (360.0f / 65536.0f);
    g_openloop_debug.speed_rpm = s_velocity_rpm;
    g_openloop_debug.bus_v = sample->bus_v;
    R40RttFrame frame = {fast * 50U,
                         sample->ia_a,
                         sample->ib_a,
                         sample->ic_a,
                         g_openloop_debug.angle_deg,
                         sample->sin_raw,
                         sample->cos_raw,
                         s_velocity_rpm,
                         sample->state,
                         sample->fault,
                         sample->bus_v,
                         fast / R40_RTT_FAST_TICKS};
    if (SEGGER_RTT_WriteNoLock(1U, &frame, sizeof(frame)) == sizeof(frame))
    {
        ++s_sent;
    }
    else
    {
        ++s_dropped;
    }
    g_openloop_debug.rtt_sent = s_sent;
    g_openloop_debug.rtt_dropped = s_dropped;
    g_openloop_debug.end = version;
    g_openloop_debug.begin = version;
    if ((uint32_t)(fast - s_text_fast) >= 1000U)
    {
        s_text_fast = fast;
        char line[192];
        int length = snprintf(line,
                              sizeof(line),
                              "%lu,%lu,%lu,%d,%d,%d,%lu,%lu,%lu,%d,%lu\n",
                              (unsigned long)(fast / 20U),
                              (unsigned long)sample->state,
                              (unsigned long)sample->fault,
                              (int)(sample->ia_a * 1000.0f),
                              (int)(sample->ib_a * 1000.0f),
                              (int)(sample->ic_a * 1000.0f),
                              (unsigned long)(g_openloop_debug.angle_deg * 1000.0f),
                              (unsigned long)sample->sin_raw,
                              (unsigned long)sample->cos_raw,
                              (int)(s_velocity_rpm * 1000.0f),
                              (unsigned long)s_dropped);
        if (length > 0 && (unsigned)length < sizeof(line))
        {
            SEGGER_RTT_WriteNoLock(0U, line, (unsigned)length);
        }
    }
}
