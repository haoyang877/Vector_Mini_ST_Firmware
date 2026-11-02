/* 验证真实APP状态适配器与固定协议布局；无测量源或硬件访问。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include "motor_lifecycle.h"
#include "motor_axis.h"
#include "yg_protocol_link.h"
#include "app_protocol.h"

static const yg_protocol_motor_status_source_t *s_bound;
static MotorStatus s_status;

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    s_bound = source;
    return source != NULL;
}

bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    return source != NULL;
}

MotorStatus MotorLifecycle_ReadStatus(void)
{
    return s_status;
}

bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service)
{
    return service != NULL;
}
bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service)
{
    return service != NULL;
}
bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service)
{
    return service != NULL;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return true;
}
bool MotorLifecycle_RequestRemoteStop(void)
{
    return true;
}
MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode)
{
    (void)mode;
    return MOTOR_CONTROL_DENIED;
}
MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType type)
{
    (void)type;
    return MOTOR_CONTROL_DENIED;
}
MotorControlResult MotorLifecycle_RequestClear(void)
{
    return MOTOR_CONTROL_DENIED;
}

int main(void)
{
    assert(AppProtocol_Init());
    const uint8_t states[] = {5U, 3U, 4U, 6U, 8U};
    for (unsigned i = 0; i <= MOTOR_STATE_FAULT; ++i)
    {
        s_status = (MotorStatus){
            .state = (MotorState)i, .mode = MOTOR_CONTROL_SPEED, .fault = MOTOR_FAULT_NONE};
        AppProtocol_Refresh();
        assert(s_bound->state == states[i] && s_bound->mode == 2U && s_bound->sample_available);
        yg_protocol_motor_feedback_t feedback;
        assert(yg_protocol_motor_feedback_from_source(s_bound, 1U, 0U, 0U, &feedback) ==
               YG_PROTOCOL_OK);
        assert(feedback.valid_bits == 0U && feedback.bus_current_mA == INT16_MIN &&
               feedback.reference_position_mrad == INT32_MIN);
    }
    s_status = (MotorStatus){.state = MOTOR_STATE_RUNNING,
                             .mode = MOTOR_CONTROL_CURRENT,
                             .measurements_valid = true,
                             .position_reference_rad = NAN,
                             .position_rad = -1.25f,
                             .speed_reference_rad_s = NAN,
                             .speed_rad_s = 2.5f,
                             .bus_voltage_v = 30.5f,
                             .bus_current_a = 0.125f,
                             .iq_reference_a = -0.25f,
                             .iq_a = -0.249f,
                             .temperature_c = 25.5f,
                             .vq_v = -1.5f,
                             .vd_v = 0.0625f};
    AppProtocol_Refresh();
    yg_protocol_motor_feedback_t live;
    assert(yg_protocol_motor_feedback_from_source(s_bound, 1U, 0U, 123U, &live) == YG_PROTOCOL_OK);
    assert(live.valid_bits == 0x0DFAU && live.actual_position_mrad == -1250 &&
           live.actual_speed_mrad_s == 2500 && live.bus_voltage_cV == 3050 &&
           live.bus_current_mA == 125 && live.reference_iq_mA == -250 &&
           live.actual_iq_mA == -249 && live.mcu_temperature_centi_c == 2550 &&
           live.motor_temperature_centi_c == INT16_MIN && live.v_q_mV == -1500 &&
           live.v_d_mV == 63 && live.correlated_seq == 123U);
    uint8_t payload[45];
    assert(yg_protocol_motor_feedback_encode(&live, payload, sizeof(payload)) == YG_PROTOCOL_OK);
    printf("VECTOR live_current_feedback ");
    for (unsigned i = 0; i < sizeof(payload); ++i)
    {
        printf("%02x", payload[i]);
    }
    puts("");
    s_status.bus_current_a = INFINITY;
    s_status.position_rad = 3000000.0f;
    s_status.temperature_c = -327.68f;
    AppProtocol_Refresh();
    assert(yg_protocol_motor_feedback_from_source(s_bound, 1U, 0U, 0U, &live) == YG_PROTOCOL_OK);
    assert((live.valid_bits & ((1U << 1) | (1U << 5) | (1U << 8))) == 0U &&
           live.actual_position_mrad == INT32_MIN && live.bus_current_mA == INT16_MIN &&
           live.mcu_temperature_centi_c == INT16_MIN);
    s_status = (MotorStatus){.state = MOTOR_STATE_RUNNING,
                             .mode = MOTOR_CONTROL_CURRENT,
                             .fault = MOTOR_FAULT_OVERCURRENT};
    AppProtocol_Refresh();
    assert(s_bound->state == 8U && s_bound->faults == 1U && s_bound->mode == 1U);
    puts("PASS APP state/fault mapping and invalid measurement feedback");
    return 0;
}

MotorControlResult MotorLifecycle_SetTarget(const MotorTarget *target)
{
    (void)target;
    return MOTOR_CONTROL_DENIED;
}
uint32_t MotorLifecycle_WorkEpoch(void)
{
    return 1U;
}
