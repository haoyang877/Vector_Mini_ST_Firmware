#include "app_protocol.h"
#include "motor_lifecycle.h"
#include "yg_protocol.h"
#include <math.h>
#include <limits.h>

/* 只装配协议和运行所有者；协议值、端口及停机确认均由前台串行访问。 */
/* 此处只做协议到领域值的映射；配置与事务由 motor_state 所有者决定。 */

static ProtocolStatus control_status(MotorControlResult result)
{
    static const ProtocolStatus statuses[] = {
        [MOTOR_CONTROL_ACCEPTED] = PROTOCOL_ACCEPTED,
        [MOTOR_CONTROL_INVALID] = PROTOCOL_INVALID_ARGUMENT,
        [MOTOR_CONTROL_DENIED] = PROTOCOL_DENIED,
        [MOTOR_CONTROL_BUSY] = PROTOCOL_BUSY,
        [MOTOR_CONTROL_OUT_OF_RANGE] = PROTOCOL_OUT_OF_RANGE,
        [MOTOR_CONTROL_FAULT] = PROTOCOL_FAULT,
    };
    return (unsigned)result < sizeof(statuses) / sizeof(statuses[0]) ? statuses[result]
                                                                     : PROTOCOL_FAILED;
}

static bool decode_mode(uint8_t wire_mode, MotorControlMode *mode)
{
    if (mode == NULL)
    {
        return false;
    }
    switch (wire_mode)
    {
    case 1U:
        *mode = MOTOR_CONTROL_CURRENT;
        return true;
    case 2U:
        *mode = MOTOR_CONTROL_SPEED;
        return true;
    case 3U:
        *mode = MOTOR_CONTROL_POSITION;
        return true;
    case 6U:
        *mode = MOTOR_CONTROL_VQ;
        return true;
    case 9U:
        *mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        return true;
    case 10U:
        *mode = MOTOR_CONTROL_OPEN_CURRENT;
        return true;
    default:
        return false;
    }
}

static void start_motor(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    MotorControlMode mode;
    if (!decode_mode(request->mode, &mode))
    {
        reply->status = PROTOCOL_UNSUPPORTED;
        return;
    }
    reply->status = control_status(MotorLifecycle_RequestEnable(adapter->lifecycle, mode));
    if (reply->status == PROTOCOL_DENIED)
    {
        reply->detail = 0x0105U;
    }
    else if (reply->status == PROTOCOL_OUT_OF_RANGE || reply->status == PROTOCOL_FAILED)
    {
        reply->status = PROTOCOL_INVALID_ARGUMENT;
    }
}

static bool same_target(const ProtocolCommand *previous, const ProtocolCommand *request)
{
    return request->mode == previous->mode && request->position_mrad == previous->position_mrad &&
           request->speed_mrad_s == previous->speed_mrad_s && request->iq_mA == previous->iq_mA &&
           request->v_q_mV == previous->v_q_mV &&
           request->electrical_frequency_mHz == previous->electrical_frequency_mHz &&
           request->id_mA == previous->id_mA && request->vd_mV == previous->vd_mV;
}

static void set_motor_target(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    MotorControlMode mode;
    if (!decode_mode(request->mode, &mode))
    {
        reply->status = PROTOCOL_UNSUPPORTED;
        return;
    }
    if (adapter->target_valid &&
        (int16_t)(uint16_t)(request->sequence - adapter->last_target.sequence) <= 0)
    {
        bool same = request->sequence == adapter->last_target.sequence &&
                    same_target(&adapter->last_target, request);
        reply->status =
            same && adapter->target_epoch == MotorLifecycle_WorkEpoch(adapter->lifecycle)
                ? PROTOCOL_OK
            : request->sequence == adapter->last_target.sequence && !same
                ? PROTOCOL_INVALID_ARGUMENT
                : PROTOCOL_DENIED;
        return;
    }
    MotorTarget target = {.mode = (uint16_t)mode,
                          .position_rad = request->position_mrad * 0.001f,
                          .speed_rad_s = request->speed_mrad_s * 0.001f,
                          .iq_a = request->iq_mA * 0.001f,
                          .vq_v = request->v_q_mV * 0.001f,
                          .frequency_hz = request->electrical_frequency_mHz * 0.001f,
                          .id_a = request->id_mA * 0.001f,
                          .vd_v = request->vd_mV * 0.001f};
    MotorControlResult result = MotorLifecycle_SetTarget(adapter->lifecycle, &target);
    reply->status = result == MOTOR_CONTROL_ACCEPTED ? PROTOCOL_OK : control_status(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        adapter->last_target = *request;
        adapter->target_epoch = MotorLifecycle_WorkEpoch(adapter->lifecycle);
        adapter->target_valid = true;
    }
    AppProtocol_Refresh(adapter);
}

static void stop_motor(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    (void)request;
    if (!MotorLifecycle_RequestRemoteStop(adapter->lifecycle))
    {
        reply->status = PROTOCOL_DENIED;
        return;
    }
    bool disabled = MotorLifecycle_ArePhasesDisabled(adapter->lifecycle);
    if (adapter->stop_token == 0U || (adapter->stop_confirmed && !disabled))
    {
        if (++adapter->stop_token == 0U)
        {
            ++adapter->stop_token;
        }
    }
    adapter->stop_confirmed = disabled;
    reply->status = disabled ? PROTOCOL_OK : PROTOCOL_ACCEPTED;
    reply->detail = disabled ? 0U : 1U;
    reply->token = adapter->stop_token;
}

static void clear_motor_fault(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    (void)request;
    MotorControlResult result = MotorLifecycle_RequestClear(adapter->lifecycle);
    /* 清错仅回空闲，启动必须再次明确请求。 */
    reply->status = result == MOTOR_CONTROL_ACCEPTED ? PROTOCOL_OK
                    : result == MOTOR_CONTROL_BUSY   ? PROTOCOL_BUSY
                                                     : PROTOCOL_DENIED;
}

static void handle_calibration_request(void *context,
                                       const ProtocolJobRequest *request,
                                       ProtocolJobReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolJobReply){.status = PROTOCOL_UNSUPPORTED};
    if (request->operation != PROTOCOL_JOB_START)
    {
        return;
    }

    MotorCalibrationType calibration_type;
    switch (request->kind)
    {
    case PROTOCOL_JOB_KIND_ENCODER_LUT:
        calibration_type = MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
        break;
    case PROTOCOL_JOB_KIND_ELECTRICAL_ZERO:
        calibration_type = MOTOR_CALIBRATION_ELECTRICAL_ZERO;
        break;
    case PROTOCOL_JOB_KIND_MECHANICAL_ZERO:
        calibration_type = MOTOR_CALIBRATION_MECHANICAL_ZERO;
        break;
    default:
        reply->detail = PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND;
        return;
    }

    if (adapter->calibration_request_number == UINT32_MAX)
    {
        reply->status = PROTOCOL_FAILED;
        reply->detail = PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
        return;
    }

    MotorControlResult result =
        MotorLifecycle_RequestCalibration(adapter->lifecycle, calibration_type);
    reply->status = control_status(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        reply->token = ++adapter->calibration_request_number;
        reply->kind = request->kind;
        reply->phase = PROTOCOL_JOB_PHASE_IN_PROGRESS;
        reply->effects = PROTOCOL_JOB_EFFECT_UNCOMMITTED;
    }
    else if (result == MOTOR_CONTROL_DENIED)
    {
        reply->detail = PROTOCOL_JOB_DETAIL_WRONG_STATE;
    }
    else if (result == MOTOR_CONTROL_INVALID)
    {
        reply->detail = PROTOCOL_JOB_DETAIL_BAD_FIELD;
    }
    else if (reply->status == PROTOCOL_FAILED)
    {
        reply->detail = PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
    }
}

static uint32_t fault_bits(MotorFaultCode error)
{
    switch (error)
    {
    case MOTOR_FAULT_OVERCURRENT:
        return 1U << 0U;
    case MOTOR_FAULT_OVERVOLTAGE:
        return 1U << 1U;
    case MOTOR_FAULT_UNDERVOLTAGE:
        return 1U << 2U;
    case MOTOR_FAULT_OVERTEMPERATURE:
        /* 现有温度来自 MCU，而非电机绕组。 */
        return 1U << 4U;
    case MOTOR_FAULT_TEMPERATURE_SENSOR:
        return 1U << 12U;
    case MOTOR_FAULT_ENCODER:
        return 1U << 5U;
    case MOTOR_FAULT_CAN_DISCONNECTED:
        return 1U << 6U;
    case MOTOR_FAULT_INVALID_PARAMETERS:
    case MOTOR_FAULT_POLE_PAIRS:
    case MOTOR_FAULT_ENCODER_NOT_CALIBRATED:
        return 1U << 9U;
    case MOTOR_FAULT_CURRENT_OFFSET:
    case MOTOR_FAULT_COGGING_CALIBRATION:
    case MOTOR_FAULT_LINEAR_HALL_CALIBRATION:
        return 1U << 14U;
    case MOTOR_FAULT_PHASE_RESISTANCE:
    case MOTOR_FAULT_PHASE_INDUCTANCE:
    case MOTOR_FAULT_FRICTION_IDENTIFICATION:
        return 1U << 15U;
    case MOTOR_FAULT_NONE:
        return 0U;
    default:
        return 1U << 12U;
    }
}

static uint8_t protocol_mode(MotorControlMode mode)
{
    switch (mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return 1U;
    case MOTOR_CONTROL_SPEED:
        return 2U;
    case MOTOR_CONTROL_POSITION:
        return 3U;
    case MOTOR_CONTROL_VQ:
        return 6U;
    case MOTOR_CONTROL_OPEN_VOLTAGE:
        return 9U;
    case MOTOR_CONTROL_OPEN_CURRENT:
        return 10U;
    default:
        return 0U;
    }
}

static uint8_t protocol_state(MotorState state)
{
    switch (state)
    {
    case MOTOR_STATE_SELF_TEST:
        return 3U;
    case MOTOR_STATE_IDLE:
        return 5U;
    case MOTOR_STATE_RUNNING:
        return 6U;
    case MOTOR_STATE_CALIBRATION:
        return 4U;
    case MOTOR_STATE_FAULT:
        return 8U;
    default:
        return 8U;
    }
}

void AppProtocol_Init(AppProtocol *adapter, ProtocolEndpoint *endpoint, MotorLifecycle *life)
{
    adapter->lifecycle = life;
    endpoint->motor = (ProtocolMotor){.context = adapter,
                                      .start = start_motor,
                                      .target = set_motor_target,
                                      .stop = stop_motor,
                                      .clear_fault = clear_motor_fault,
                                      .status = &adapter->status};
    endpoint->jobs = (ProtocolJobs){adapter, handle_calibration_request};
    endpoint->device_status.role = PROTOCOL_DEVICE_ROLE_APP;
}

/* 仅前台缩放；保留整数最小值作无效哨兵，半值向远离零方向舍入。 */
static int32_t scale_measurement(
    float value, unsigned scale, int32_t minimum, int32_t maximum, uint16_t bit, uint16_t *valid)
{
    double rounded = round((double)value * scale);
    if (!isfinite(rounded) || rounded <= minimum || rounded > maximum)
    {
        return minimum;
    }
    *valid |= bit;
    return (int32_t)rounded;
}

void AppProtocol_Refresh(AppProtocol *adapter)
{
    MotorStatus status = MotorLifecycle_ReadStatus(adapter->lifecycle);
    adapter->status.faults = fault_bits((MotorFaultCode)status.fault);
    adapter->status.mode = protocol_mode((MotorControlMode)status.mode);
    adapter->status.state = status.fault != MOTOR_FAULT_NONE ? 8U : protocol_state(status.state);
    adapter->status.sample_available = true;
    ProtocolFeedback *values = &adapter->status.measurements;
    (void)Protocol_BuildFeedback(NULL, 1U, 0U, 0U, values);
    adapter->status.measurements_available = true;
    if (!status.measurements_valid)
    {
        return;
    }
    uint16_t *valid = &values->valid_bits;
    values->reference_position_mrad = scale_measurement(
        status.position_reference_rad, 1000, INT32_MIN, INT32_MAX, 1U << 0, valid);
    values->actual_position_mrad =
        scale_measurement(status.position_rad, 1000, INT32_MIN, INT32_MAX, 1U << 1, valid);
    values->reference_speed_mrad_s =
        scale_measurement(status.speed_reference_rad_s, 1000, INT32_MIN, INT32_MAX, 1U << 2, valid);
    values->actual_speed_mrad_s =
        scale_measurement(status.speed_rad_s, 1000, INT32_MIN, INT32_MAX, 1U << 3, valid);
    values->bus_voltage_cV =
        (int16_t)scale_measurement(status.bus_voltage_v, 100, INT16_MIN, INT16_MAX, 1U << 4, valid);
    values->bus_current_mA = (int16_t)scale_measurement(
        status.bus_current_a, 1000, INT16_MIN, INT16_MAX, 1U << 5, valid);
    values->reference_iq_mA = (int16_t)scale_measurement(
        status.iq_reference_a, 1000, INT16_MIN, INT16_MAX, 1U << 6, valid);
    values->actual_iq_mA =
        (int16_t)scale_measurement(status.iq_a, 1000, INT16_MIN, INT16_MAX, 1U << 7, valid);
    values->mcu_temperature_centi_c =
        (int16_t)scale_measurement(status.temperature_c, 100, INT16_MIN, INT16_MAX, 1U << 8, valid);
    values->v_q_mV =
        (int16_t)scale_measurement(status.vq_v, 1000, INT16_MIN, INT16_MAX, 1U << 10, valid);
    values->v_d_mV =
        (int16_t)scale_measurement(status.vd_v, 1000, INT16_MIN, INT16_MAX, 1U << 11, valid);
}
