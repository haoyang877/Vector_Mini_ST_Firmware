/* 真实RTT实现核对完整APP的2kHz帧、慢主机丢帧和只读输入。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_telemetry.h"
#include "SEGGER_RTT.h"

static uint32_t s_depth;
uint32_t BspSystem_EnterCritical(void)
{
    return s_depth++;
}
void BspSystem_ExitCritical(uint32_t key)
{
    s_depth = key;
}

int main(void)
{
    FocState foc = {.phase_a_current_a = 1.0f,
                    .phase_b_current_a = -0.25f,
                    .phase_c_current_a = -0.75f,
                    .id_a = 0.5f,
                    .iq_a = -0.2f,
                    .bus_voltage_v = 29.0f};
    MotorControlState motor = {.id_target_a = 0.6f, .iq_target_a = -0.3f};
    EncoderState encoder = {.raw_q15 = 65530U, .linearized_q15 = 65500U};
    FocState original_foc = foc;
    MotorControlState original_motor = motor;
    EncoderState original_encoder = encoder;
    R40AppTelemetry_Init();
    for (uint32_t tick = 1U; tick <= 20U; ++tick)
    {
        R40AppTelemetry_Capture(tick, &foc, &motor, &encoder, 2U, 3U, 8U);
        R40AppTelemetry_Service();
    }
    assert(g_r40_app_telemetry.queued == 2U && g_r40_app_telemetry.sent == 2U);
    R40AppTelemetryFrame frames[2];
    memcpy(frames, _SEGGER_RTT.aUp[1].pBuffer, sizeof(frames));
    assert(frames[0].timestamp_us == 500U && frames[1].timestamp_us == 1000U);
    assert(frames[0].sequence == 1U && frames[1].sequence == 2U);
    assert(frames[0].id_reference_a == 0.6f && frames[0].id_a == 0.5f);
    assert(frames[0].raw_q15 == 65530U && frames[0].calibration_flags == 3U);
    assert(frames[0].ia_a + frames[0].ib_a + frames[0].ic_a == 0.0f);
    assert(memcmp(&foc, &original_foc, sizeof(foc)) == 0);
    assert(memcmp(&motor, &original_motor, sizeof(motor)) == 0);
    assert(memcmp(&encoder, &original_encoder, sizeof(encoder)) == 0);
    /* 前台停调度只填32帧，之后丢整帧；不会等主机或覆盖未消费记录。 */
    for (uint32_t tick = 30U; tick <= 420U; tick += 10U)
    {
        R40AppTelemetry_Capture(tick, &foc, &motor, &encoder, 2U, 3U, 8U);
    }
    assert(g_r40_app_telemetry.queued == 34U);
    assert(g_r40_app_telemetry.queue_dropped == 8U);
    R40AppTelemetry_Service();
    assert(g_r40_app_telemetry.sent == 6U);
    assert(g_r40_app_telemetry.begin == g_r40_app_telemetry.end);
    for (unsigned n = 0U; n < 500U; ++n)
    {
        R40AppTelemetry_Capture(500U + n * 10U, &foc, &motor, &encoder, 2U, 3U, 8U);
        R40AppTelemetry_Service();
    }
    assert(g_r40_app_telemetry.rtt_dropped > 0U);
    assert(_SEGGER_RTT.aUp[1].WrOff % sizeof(R40AppTelemetryFrame) == 0U);
    _SEGGER_RTT.aUp[1].RdOff = _SEGGER_RTT.aUp[1].WrOff;
    uint32_t sent = g_r40_app_telemetry.sent;
    R40AppTelemetry_Capture(6000U, &foc, &motor, &encoder, 2U, 3U, 8U);
    R40AppTelemetry_Service();
    assert(g_r40_app_telemetry.sent == sent + 1U && s_depth == 0U);
    puts("PASS APP RTT: 2kHz real frames, read-only inputs, bounded queue, host stall/recovery");
    return 0;
}
