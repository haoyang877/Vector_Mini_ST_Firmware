#include "app_protocol.h"
#include "motor_state_machine.h"
#include "yg_protocol.h"
#include "scaled_integer.h"
#include "bsp_system.h"
#include <limits.h>
#include <math.h>

/* 线上编号与内部枚举独立。0 表示没有可启动的模式。 */
static const uint8_t s_wire_modes[] = {
    [MOTOR_CONTROL_CURRENT] = 1U,
    [MOTOR_CONTROL_SPEED] = 2U,
    [MOTOR_CONTROL_POSITION] = 3U,
    [MOTOR_CONTROL_VQ] = 6U,
    [MOTOR_CONTROL_OPEN_VOLTAGE] = 9U,
    [MOTOR_CONTROL_OPEN_CURRENT] = 10U,
};

static ProtocolStatus encode_control_result(MotorControlResult result)
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

static MotorControlMode decode_control_mode(uint8_t wire_mode)
{
    if (wire_mode != 0U)
    {
        for (unsigned mode = 0; mode < sizeof(s_wire_modes); ++mode)
        {
            if (s_wire_modes[mode] == wire_mode)
            {
                return (MotorControlMode)mode;
            }
        }
    }
    return MOTOR_CONTROL_NONE;
}

static void request_motor_start(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    MotorControlMode mode = decode_control_mode(request->mode);
    if (mode == MOTOR_CONTROL_NONE)
    {
        return;
    }
    MotorControlResult result = request->start_flags == PROTOCOL_START_HOST_POSITION_UNLIMITED
                                    ? MotorStateMachine_RequestHostPosition(adapter->state_machine)
                                    : MotorStateMachine_RequestStart(adapter->state_machine, mode);
    reply->status = encode_control_result(result);
    if (reply->status == PROTOCOL_ACCEPTED)
    {
        adapter->state_machine->motion_renewed_ms = BspSystem_TimeMs();
        BspSystem_MemoryBarrier();
        adapter->state_machine->motion_lease_active = true;
    }
    if (reply->status == PROTOCOL_DENIED)
    {
        reply->detail = 0x0105U;
    }
    else if (reply->status == PROTOCOL_OUT_OF_RANGE || reply->status == PROTOCOL_FAILED)
    {
        reply->status = PROTOCOL_INVALID_ARGUMENT;
    }
}

static bool target_values_match(const ProtocolCommand *previous, const ProtocolCommand *request)
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
    MotorControlMode mode = decode_control_mode(request->mode);
    if (mode == MOTOR_CONTROL_NONE)
    {
        return;
    }
    /* 线路模式 0 无法受理，零初始化的记录表示尚未收到有效目标。 */
    if (adapter->last_target.mode != 0U)
    {
        uint16_t sequence_delta = (uint16_t)(request->sequence - adapter->last_target.sequence);
        if (sequence_delta == 0U)
        {
            reply->status = PROTOCOL_INVALID_ARGUMENT;
            if (target_values_match(&adapter->last_target, request))
            {
                reply->status = adapter->last_target_epoch == adapter->state_machine->epoch
                                    ? PROTOCOL_OK
                                    : PROTOCOL_DENIED;
            }
            return;
        }
        if (sequence_delta > INT16_MAX)
        {
            reply->status = PROTOCOL_DENIED;
            return;
        }
    }
    if (adapter->state_machine->motion_lease_active &&
        (uint32_t)(BspSystem_TimeMs() - adapter->state_machine->motion_renewed_ms) >=
            PROTOCOL_MOTION_LEASE_MS)
    {
        reply->status = PROTOCOL_DENIED;
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
    /* 只有前台能启动新会话；STOP 可在目标提交后抢占，重发仍须属于原会话。 */
    uint32_t epoch = adapter->state_machine->epoch;
    MotorControlResult result = MotorStateMachine_SetTarget(adapter->state_machine, &target);
    reply->status = result == MOTOR_CONTROL_ACCEPTED ? PROTOCOL_OK : encode_control_result(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        adapter->state_machine->motion_renewed_ms = BspSystem_TimeMs();
        adapter->last_target = *request;
        adapter->last_target_epoch = epoch;
    }
}

static void
renew_motion_permission(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    MotorStateMachine *machine = adapter->state_machine;
    *reply = (ProtocolReply){.status = PROTOCOL_DENIED};
    uint32_t delta = request->renewal_counter - adapter->renewal_counter;
    if (request->renewal_counter == 0U || delta == 0U || delta > INT32_MAX ||
        !machine->motion_lease_active || machine->stop_pending ||
        machine->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        (uint32_t)(BspSystem_TimeMs() - machine->motion_renewed_ms) >= PROTOCOL_MOTION_LEASE_MS)
    {
        return;
    }
    machine->motion_renewed_ms = BspSystem_TimeMs();
    adapter->renewal_counter = request->renewal_counter;
    reply->status = PROTOCOL_OK;
}

static void request_motor_stop(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    (void)request;
    MotorStateMachine_RequestStop(adapter->state_machine);
    bool disabled = MotorStateMachine_ArePhasesDisabled(adapter->state_machine);
    *reply = (ProtocolReply){.status = disabled ? PROTOCOL_OK : PROTOCOL_ACCEPTED,
                             .detail = disabled ? 0U : 1U};
}

static void
request_motor_fault_clear(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
    (void)request;
    MotorControlResult result = MotorStateMachine_RequestFaultClear(adapter->state_machine);
    /* 清错仅回空闲，启动必须再次明确请求。 */
    reply->status = result == MOTOR_CONTROL_ACCEPTED ? PROTOCOL_OK
                    : result == MOTOR_CONTROL_BUSY   ? PROTOCOL_BUSY
                                                     : PROTOCOL_DENIED;
}

static void read_calibration_job_result(AppProtocol *adapter, ProtocolJobReply *reply)
{
    *reply = adapter->job;
    reply->status = PROTOCOL_OK;
    MotorStateMachine *machine = adapter->state_machine;
    if (machine->calibration_result_epoch != adapter->job_epoch)
    {
        reply->status = PROTOCOL_DENIED;
        reply->detail = PROTOCOL_JOB_DETAIL_STALE_TOKEN;
        return;
    }
    reply->effects = machine->calibration_saved_epoch == adapter->job_epoch
                         ? PROTOCOL_JOB_EFFECT_COMMITTED
                         : PROTOCOL_JOB_EFFECT_UNCOMMITTED;
    switch (machine->calibration_result)
    {
    case MOTOR_CALIBRATION_RESULT_COMPLETED:
        reply->phase = PROTOCOL_JOB_PHASE_COMPLETED;
        break;
    case MOTOR_CALIBRATION_RESULT_CANCELLED:
        reply->phase = PROTOCOL_JOB_PHASE_CANCELLED;
        break;
    case MOTOR_CALIBRATION_RESULT_FAILED:
        reply->phase = PROTOCOL_JOB_PHASE_FAILED;
        reply->detail = machine->calibration_failure == MOTOR_FAULT_CALIBRATION_STORAGE
                            ? 8U
                            : PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
        break;
    default:
        reply->phase = machine->stop_pending ? PROTOCOL_JOB_PHASE_CANCEL_REQUESTED
                                             : PROTOCOL_JOB_PHASE_IN_PROGRESS;
        if (!machine->stop_pending)
        {
            if (machine->calibration_handoff == MOTOR_CALIBRATION_AWAITING_PUBLICATION)
            {
                reply->detail = 0x1010U;
            }
            else if (machine->calibration_handoff == MOTOR_CALIBRATION_AWAITING_COMPLETION)
            {
                reply->detail = 0x1011U;
            }
            else
            {
                switch (machine->calibration_type)
                {
                case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
                    reply->detail = 0x1001U;
                    break;
                case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
                    reply->detail = 0x1002U;
                    break;
                case MOTOR_CALIBRATION_MECHANICAL_ZERO:
                    reply->detail = 0x1003U;
                    break;
                case MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO:
                    reply->detail = 0x1004U;
                    break;
                default:
                    reply->detail = 0x1000U;
                    break;
                }
            }
        }
        break;
    }
}

static void request_encoder_calibration(void *context,
                                        const ProtocolJobRequest *request,
                                        ProtocolJobReply *reply)
{
    AppProtocol *adapter = context;
    *reply = (ProtocolJobReply){.status = PROTOCOL_UNSUPPORTED};
    if (request->operation != PROTOCOL_JOB_START)
    {
        if (adapter->job.token == 0U || request->job_id != adapter->job.token)
        {
            reply->status = PROTOCOL_DENIED;
            reply->detail = PROTOCOL_JOB_DETAIL_STALE_TOKEN;
            return;
        }
        read_calibration_job_result(adapter, reply);
        if (request->operation == PROTOCOL_JOB_CANCEL && reply->status == PROTOCOL_OK &&
            reply->phase == PROTOCOL_JOB_PHASE_IN_PROGRESS)
        {
            MotorStateMachine_RequestStop(adapter->state_machine);
            read_calibration_job_result(adapter, reply);
        }
        return;
    }
    if (adapter->job.token != 0U)
    {
        uint16_t delta = (uint16_t)(request->sequence - adapter->job_start_sequence);
        if (delta == 0U)
        {
            if (request->kind != adapter->job.kind)
            {
                reply->status = PROTOCOL_INVALID_ARGUMENT;
                reply->detail = PROTOCOL_JOB_DETAIL_CONFLICTING_RETRY;
                return;
            }
            read_calibration_job_result(adapter, reply);
            return;
        }
        if (delta > INT16_MAX)
        {
            reply->status = PROTOCOL_DENIED;
            reply->detail = PROTOCOL_JOB_DETAIL_STALE_SEQUENCE;
            return;
        }
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
        MotorStateMachine_RequestCalibration(adapter->state_machine, calibration_type);
    reply->status = encode_control_result(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        reply->token = ++adapter->calibration_request_number;
        reply->kind = request->kind;
        reply->phase = PROTOCOL_JOB_PHASE_IN_PROGRESS;
        reply->effects = PROTOCOL_JOB_EFFECT_UNCOMMITTED;
        adapter->job = *reply;
        adapter->job_epoch = adapter->state_machine->calibration_result_epoch;
        adapter->job_start_sequence = request->sequence;
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

static uint32_t encode_fault_bits(MotorFaultCode error)
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

static uint8_t encode_control_mode(MotorControlMode mode)
{
    return (unsigned)mode < sizeof(s_wire_modes) ? s_wire_modes[mode] : 0U;
}

static uint8_t encode_motor_state(MotorState state)
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

/* 仅前台缩放；保留整数最小值作无效哨兵，半值向远离零方向舍入。 */
static int32_t encode_feedback_measurement(
    float value, uint32_t scale, int32_t maximum, uint16_t bit, uint16_t *valid)
{
    ScaledInteger rounded;
    if (!ScaledInteger_Round(value, scale, &rounded) || rounded.magnitude > (uint32_t)maximum)
    {
        return -maximum - 1;
    }
    *valid |= bit;
    return rounded.negative ? -(int32_t)rounded.magnitude : (int32_t)rounded.magnitude;
}

static bool read_motor_feedback(void *context, ProtocolFeedback *values)
{
    AppProtocol *adapter = context;
    MotorStatus status;
    if (!MotorStateMachine_ReadStatus(adapter->state_machine, &status))
    {
        return false;
    }
    values->fault_code = encode_fault_bits((MotorFaultCode)status.fault);
    values->control_mode = encode_control_mode((MotorControlMode)status.mode);
    values->motor_state = status.fault != MOTOR_FAULT_NONE ? 8U : encode_motor_state(status.state);
    if (!status.measurements_valid)
    {
        return true;
    }
    uint16_t *valid = &values->valid_bits;
    values->reference_position_mrad =
        encode_feedback_measurement(status.position_reference_rad, 1000, INT32_MAX, 1U << 0, valid);
    values->actual_position_mrad =
        encode_feedback_measurement(status.position_rad, 1000, INT32_MAX, 1U << 1, valid);
    values->reference_speed_mrad_s =
        encode_feedback_measurement(status.speed_reference_rad_s, 1000, INT32_MAX, 1U << 2, valid);
    values->actual_speed_mrad_s =
        encode_feedback_measurement(status.speed_rad_s, 1000, INT32_MAX, 1U << 3, valid);
    values->bus_voltage_cV =
        (int16_t)encode_feedback_measurement(status.bus_voltage_v, 100, INT16_MAX, 1U << 4, valid);
    values->bus_current_mA =
        (int16_t)encode_feedback_measurement(status.bus_current_a, 1000, INT16_MAX, 1U << 5, valid);
    values->reference_iq_mA = (int16_t)encode_feedback_measurement(
        status.iq_reference_a, 1000, INT16_MAX, 1U << 6, valid);
    values->actual_iq_mA =
        (int16_t)encode_feedback_measurement(status.iq_a, 1000, INT16_MAX, 1U << 7, valid);
    values->mcu_temperature_centi_c =
        (int16_t)encode_feedback_measurement(status.temperature_c, 100, INT16_MAX, 1U << 8, valid);
    values->motor_temperature_centi_c = (int16_t)encode_feedback_measurement(
        status.motor_temperature_valid ? status.motor_temperature_c : NAN,
        100,
        INT16_MAX,
        1U << 9,
        valid);
    values->v_q_mV =
        (int16_t)encode_feedback_measurement(status.vq_v, 1000, INT16_MAX, 1U << 10, valid);
    values->v_d_mV =
        (int16_t)encode_feedback_measurement(status.vd_v, 1000, INT16_MAX, 1U << 11, valid);
    return true;
}

static bool read_output_feedback(void *context, MotorOutputFeedback *feedback)
{
    AppProtocol *adapter = context;
    MotorStatus status;
    if (!MotorStateMachine_ReadStatus(adapter->state_machine, &status))
    {
        return false;
    }
    uint16_t valid = 0U;
    *feedback = (MotorOutputFeedback){.flags = status.output.flags & (uint16_t)~3U,
                                      .age_ms = status.output_age_ms,
                                      .sample_counter = status.output.sample_counter,
                                      .hall_mrad = INT32_MIN,
                                      .tracked_mrad = INT32_MIN,
                                      .zero_mrad = INT32_MIN,
                                      .gear_ratio_milli = INT32_MIN};
    if ((status.output.flags & 1U) != 0U)
    {
        feedback->hall_mrad = encode_feedback_measurement(
            status.output.hall_position_rad, 1000, INT32_MAX, 1U, &valid);
    }
    if ((status.output.flags & 2U) != 0U)
    {
        feedback->tracked_mrad = encode_feedback_measurement(
            status.output.tracked_position_rad, 1000, INT32_MAX, 2U, &valid);
    }
    feedback->flags |= valid;
    feedback->zero_mrad =
        encode_feedback_measurement(status.output.zero_rad, 1000, INT32_MAX, 0U, &valid);
    feedback->gear_ratio_milli =
        encode_feedback_measurement(status.output.gear_ratio, 1000, INT32_MAX, 0U, &valid);
    return true;
}

static void keep_host_position_alive(void *context)
{
    AppProtocol *adapter = context;
    MotorStateMachine_KeepHostPositionAlive(adapter->state_machine);
}

void AppProtocol_Init(AppProtocol *adapter, ProtocolEndpoint *endpoint, MotorStateMachine *machine)
{
    adapter->state_machine = machine;
    endpoint->motor = (ProtocolMotor){.context = adapter,
                                      .renew = renew_motion_permission,
                                      .start = request_motor_start,
                                      .target = set_motor_target,
                                      .stop = request_motor_stop,
                                      .clear_fault = request_motor_fault_clear,
                                      .host_position_keepalive = keep_host_position_alive,
                                      .read_feedback = read_motor_feedback,
                                      .read_output = read_output_feedback};
    endpoint->jobs = (ProtocolJobs){adapter, request_encoder_calibration};
    endpoint->device_status.role = PROTOCOL_DEVICE_ROLE_APP;
}
