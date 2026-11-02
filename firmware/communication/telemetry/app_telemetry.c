#include "app_telemetry.h"

#include <string.h>
#include "SEGGER_RTT.h"
#include "bsp_system.h"

#define AT32_APP_TELEMETRY_QUEUE_SIZE 32U

/* C99编译期检查：与旧板工具链一致，帧布局不变。 */
typedef char AT32AppTelemetryFrameSizeCheck[(sizeof(AT32AppTelemetryFrame) == 88U) ? 1 : -1];

volatile AT32AppTelemetryDiagnostic g_at32_app_telemetry;
static AT32AppTelemetryFrame s_queue[AT32_APP_TELEMETRY_QUEUE_SIZE];
static char s_rtt_buffer[8192];
static volatile uint32_t s_produced, s_consumed;

void AT32AppTelemetry_Init(void)
{
    memset((void *)&g_at32_app_telemetry, 0, sizeof(g_at32_app_telemetry));
    s_produced = s_consumed = 0U;
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1U,
                              "JScope_t4u4f4f4f4f4f4f4f4f4f4f4f4f4u4u4u4u4u4u4u4f4",
                              s_rtt_buffer,
                              sizeof(s_rtt_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_WriteString(
        0U,
        "APP RTT1 2kHz: us,seq,ia,ib,ic,id_ref,id,iq_ref,iq,vd,vq,theta_e,"
        "speed_ref,speed,raw,linear,state,fault,calib,reserved,maintenance,vbus\n");
}

void AT32AppTelemetry_Capture(uint32_t fast_samples,
                              const FocState *foc,
                              const MotorControlState *motor,
                              const EncoderState *encoder,
                              uint32_t state,
                              uint32_t calibration_flags,
                              uint32_t maintenance_kind)
{
    if (fast_samples % 10U != 0U)
    {
        return;
    }
    AT32AppTelemetryFrame frame = {
        fast_samples * 50U,
        fast_samples / 10U,
        foc->phase_current_a.a,
        foc->phase_current_a.b,
        foc->phase_current_a.c,
        motor->id_target_a,
        foc->current_dq_a.d,
        motor->iq_target_a,
        foc->current_dq_a.q,
        foc->voltage_dq_v.d,
        foc->voltage_dq_v.q,
        encoder->electrical_angle_rad,
        motor->speed_ramped_rad_s,
        encoder->mechanical_velocity_continuous_rad_s,
        encoder->raw_q15,
        encoder->linearized_q15,
        state,
        motor->fault_code,
        calibration_flags,
        0U,
        maintenance_kind,
        foc->bus_voltage_v,
    };
    uint32_t version = g_at32_app_telemetry.begin + 2U;
    g_at32_app_telemetry.begin = version | 1U;
    g_at32_app_telemetry.sample = frame;
    /* 单生产者先复制再发布索引；前台取出时用短临界区，不能读到半帧。 */
    if (s_produced - s_consumed < AT32_APP_TELEMETRY_QUEUE_SIZE)
    {
        s_queue[s_produced % AT32_APP_TELEMETRY_QUEUE_SIZE] = frame;
        BspSystem_MemoryBarrier();
        ++s_produced;
        ++g_at32_app_telemetry.queued;
    }
    else
    {
        ++g_at32_app_telemetry.queue_dropped;
    }
    g_at32_app_telemetry.end = version;
    g_at32_app_telemetry.begin = version;
}

void AT32AppTelemetry_Service(void)
{
    for (unsigned n = 0U; n < 4U; ++n)
    {
        AT32AppTelemetryFrame frame;
        if (s_consumed == s_produced)
        {
            return;
        }
        BspSystem_MemoryBarrier();
        frame = s_queue[s_consumed % AT32_APP_TELEMETRY_QUEUE_SIZE];
        BspSystem_MemoryBarrier();
        ++s_consumed;
        if (SEGGER_RTT_WriteNoLock(1U, &frame, sizeof(frame)) == sizeof(frame))
        {
            ++g_at32_app_telemetry.sent;
        }
        else
        {
            ++g_at32_app_telemetry.rtt_dropped;
        }
    }
}
