/* 编译官方RTT实现，验证整帧丢弃、主机停读、回绕及实际编码。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "openloop_telemetry.h"
#include "SEGGER_RTT.h"

int main(void)
{
    R40OpenLoopTelemetry_Init();
    assert(strcmp(_SEGGER_RTT.aUp[1].sName, "JScope_t4f4f4f4f4u4u4f4u4u4f4u4") == 0);
    assert(_SEGGER_RTT.aUp[1].Flags == SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    R40OpenLoopTelemetrySample sample = {.fast_samples = 10U,
                                         .state = 1U,
                                         .angle_counts = 32768U,
                                         .sin_raw = 2048U,
                                         .cos_raw = 3000U,
                                         .sincos_valid = 1U,
                                         .ia_a = 1.25f,
                                         .ib_a = -0.5f,
                                         .ic_a = -0.75f,
                                         .bus_v = 29.0f};
    R40OpenLoopTelemetry_Publish(&sample);
    uint32_t words[12];
    memcpy(words, _SEGGER_RTT.aUp[1].pBuffer, sizeof(words));
    assert(words[0] == 500U && words[5] == 2048U && words[6] == 3000U && words[11] == 1U);
    float current, angle;
    memcpy(&current, &words[1], sizeof(current));
    memcpy(&angle, &words[4], sizeof(angle));
    assert(current == 1.25f && angle == 180.0f);
    assert(g_openloop_debug.begin == g_openloop_debug.end && !(g_openloop_debug.begin & 1U));
    assert(g_openloop_debug.sincos_valid && g_openloop_debug.cos_raw == 3000U);
    for (uint32_t i = 2U; i <= 2000U; ++i)
    {
        sample.fast_samples = i * 10U;
        sample.angle_counts = (uint16_t)(32768U + i * 100U);
        R40OpenLoopTelemetry_Publish(&sample);
    }
    assert(g_openloop_debug.rtt_sent + g_openloop_debug.rtt_dropped == 2000U);
    assert(g_openloop_debug.rtt_dropped > 0U);
    assert(_SEGGER_RTT.aUp[1].WrOff == 8160U);
    unsigned write_before = _SEGGER_RTT.aUp[1].WrOff;
    sample.fast_samples += 10U;
    R40OpenLoopTelemetry_Publish(&sample);
    assert(_SEGGER_RTT.aUp[1].WrOff == write_before);
    _SEGGER_RTT.aUp[1].RdOff = write_before;
    sample.fast_samples += 10U;
    R40OpenLoopTelemetry_Publish(&sample);
    assert(_SEGGER_RTT.aUp[1].WrOff == 16U);
    unsigned char packet[48];
    for (unsigned i = 0U; i < sizeof(packet); ++i)
    {
        packet[i] = (unsigned char)_SEGGER_RTT.aUp[1].pBuffer[(write_before + i) % 8192U];
    }
    memcpy(words, packet, sizeof(words));
    assert(words[0] == sample.fast_samples * 50U && words[11] == sample.fast_samples / 10U);
    assert(isfinite(g_openloop_debug.speed_rpm));
    uint32_t sent_before = g_openloop_debug.rtt_sent;
    R40OpenLoopTelemetry_Publish(&sample);
    assert(g_openloop_debug.rtt_sent == sent_before);
    /* 同时隙不重复；前台有几拍抖动时也不额外漏掉下一时隙。 */
    sample.fast_samples += 9U;
    R40OpenLoopTelemetry_Publish(&sample);
    assert(g_openloop_debug.rtt_sent == sent_before);
    sample.fast_samples += 1U;
    R40OpenLoopTelemetry_Publish(&sample);
    assert(g_openloop_debug.rtt_sent == sent_before + 1U);
    puts("PASS RTT: actual 48-byte frame, full-buffer skip, host recovery, wrap and 2kHz slots");
    return 0;
}
