#include "app_protocol.h"
#include "motor_lifecycle.h"
#include "motor_stop_request.h"
#include "bsp_pwm.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor_adapter.h"
#include <math.h>
#include <limits.h>

/* 只装配协议和运行所有者；协议值、端口及停机确认均由前台串行访问。 */
static motor_stop_service_t s_stop_service;
static yg_protocol_motor_adapter_t s_motor_adapter;
static yg_protocol_motor_status_source_t s_source;
static const yg_protocol_readonly_device_status_t s_boot_source = {.role =
                                                                       YG_PROTOCOL_DEVICE_ROLE_APP};
static bool s_status_ready;
static bool s_target_valid;
static yg_protocol_motor_request_t s_last_target;
static uint32_t s_target_epoch;

/* 前台独占受理编号；仅用于回复关联，不保存任务历史或授权后续操作。 */
static uint32_t s_calibration_request_number;

/* 此处只做协议到领域值的映射；配置与事务由 motor_state 所有者决定。 */

static yg_protocol_service_status_t control_status(MotorControlResult result)
{
    static const yg_protocol_service_status_t statuses[] = {
        [MOTOR_CONTROL_ACCEPTED] = YG_PROTOCOL_SERVICE_ACCEPTED,
        [MOTOR_CONTROL_INVALID] = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT,
        [MOTOR_CONTROL_DENIED] = YG_PROTOCOL_SERVICE_DENIED,
        [MOTOR_CONTROL_BUSY] = YG_PROTOCOL_SERVICE_BUSY,
        [MOTOR_CONTROL_OUT_OF_RANGE] = YG_PROTOCOL_SERVICE_OUT_OF_RANGE,
        [MOTOR_CONTROL_FAULT] = YG_PROTOCOL_SERVICE_FAULT,
    };
    return (unsigned)result < sizeof(statuses) / sizeof(statuses[0]) ? statuses[result]
                                                                     : YG_PROTOCOL_SERVICE_FAILED;
}

static void handle_start(const yg_protocol_motor_request_t *request,
                         yg_protocol_service_reply_t *reply)
{
    MotorControlMode mode;
    if (!yg_protocol_motor_decode_mode(request->mode, &mode))
    {
        reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
        return;
    }
    reply->status = control_status(MotorLifecycle_RequestEnable(mode));
    if (reply->status == YG_PROTOCOL_SERVICE_DENIED)
    {
        reply->detail = 0x0105U;
    }
    else if (reply->status == YG_PROTOCOL_SERVICE_OUT_OF_RANGE ||
             reply->status == YG_PROTOCOL_SERVICE_FAILED)
    {
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
}

static bool same_target(const yg_protocol_motor_request_t *request)
{
    return request->mode == s_last_target.mode &&
           request->position_mrad == s_last_target.position_mrad &&
           request->speed_mrad_s == s_last_target.speed_mrad_s &&
           request->iq_mA == s_last_target.iq_mA && request->v_q_mV == s_last_target.v_q_mV &&
           request->electrical_frequency_mHz == s_last_target.electrical_frequency_mHz &&
           request->id_mA == s_last_target.id_mA && request->vd_mV == s_last_target.vd_mV;
}

static void handle_target(const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    MotorControlMode mode;
    if (!yg_protocol_motor_decode_mode(request->mode, &mode))
    {
        return;
    }
    if (s_target_valid && (int16_t)(uint16_t)(request->sequence - s_last_target.sequence) <= 0)
    {
        bool same = request->sequence == s_last_target.sequence && same_target(request);
        reply->status = same && s_target_epoch == MotorLifecycle_WorkEpoch()
                            ? YG_PROTOCOL_SERVICE_OK
                        : request->sequence == s_last_target.sequence && !same
                            ? YG_PROTOCOL_SERVICE_INVALID_ARGUMENT
                            : YG_PROTOCOL_SERVICE_DENIED;
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
    MotorControlResult result = MotorLifecycle_SetTarget(&target);
    reply->status =
        result == MOTOR_CONTROL_ACCEPTED ? YG_PROTOCOL_SERVICE_OK : control_status(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        s_last_target = *request;
        s_target_epoch = MotorLifecycle_WorkEpoch();
        s_target_valid = true;
    }
    AppProtocol_Refresh();
}

static void handle_lifecycle(void *context,
                             const yg_protocol_motor_request_t *request,
                             yg_protocol_service_reply_t *reply)
{
    (void)context;
    if (reply == NULL)
    {
        return;
    }
    *reply = (yg_protocol_service_reply_t){.status = YG_PROTOCOL_SERVICE_UNSUPPORTED};
    if (request == NULL)
    {
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        return;
    }
    if (request->operation == YG_PROTOCOL_MOTOR_CLEAR_FAULT)
    {
        MotorControlResult result = MotorLifecycle_RequestClear();
        /* 清错仅回空闲，启动必须再次明确请求。 */
        reply->status = result == MOTOR_CONTROL_ACCEPTED ? YG_PROTOCOL_SERVICE_OK
                        : result == MOTOR_CONTROL_BUSY   ? YG_PROTOCOL_SERVICE_BUSY
                                                         : YG_PROTOCOL_SERVICE_DENIED;
    }
    else if (request->operation == YG_PROTOCOL_MOTOR_START_CONTROL)
    {
        handle_start(request, reply);
    }
    else if (request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL)
    {
        handle_target(request, reply);
    }
    /* 101 的 enable=1 是低侧制动候选，不能偷换成电流模式启动。 */
}

static void handle_calibration_request(void *context,
                                       const yg_protocol_job_request_t *request,
                                       yg_protocol_job_reply_t *reply)
{
    (void)context;
    *reply = (yg_protocol_job_reply_t){.status = YG_PROTOCOL_SERVICE_UNSUPPORTED};
    if (request->operation != YG_PROTOCOL_JOB_START)
    {
        return;
    }

    MotorCalibrationType calibration_type;
    switch (request->kind)
    {
    case YG_PROTOCOL_JOB_KIND_ENCODER_LUT:
        calibration_type = MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
        break;
    case YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO:
        calibration_type = MOTOR_CALIBRATION_ELECTRICAL_ZERO;
        break;
    case YG_PROTOCOL_JOB_KIND_MECHANICAL_ZERO:
        calibration_type = MOTOR_CALIBRATION_MECHANICAL_ZERO;
        break;
    default:
        reply->detail = YG_PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND;
        return;
    }

    if (s_calibration_request_number == UINT32_MAX)
    {
        reply->status = YG_PROTOCOL_SERVICE_FAILED;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
        return;
    }

    MotorControlResult result = MotorLifecycle_RequestCalibration(calibration_type);
    reply->status = control_status(result);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        reply->token = ++s_calibration_request_number;
        reply->kind = request->kind;
        reply->phase = YG_PROTOCOL_JOB_PHASE_IN_PROGRESS;
        reply->effects = YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED;
    }
    else if (result == MOTOR_CONTROL_DENIED)
    {
        reply->detail = YG_PROTOCOL_JOB_DETAIL_WRONG_STATE;
    }
    else if (result == MOTOR_CONTROL_INVALID)
    {
        reply->detail = YG_PROTOCOL_JOB_DETAIL_BAD_FIELD;
    }
    else if (reply->status == YG_PROTOCOL_SERVICE_FAILED)
    {
        reply->detail = YG_PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
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

static bool request_stop(void *context, uint8_t source_node, uint16_t sequence)
{
    (void)context;
    (void)source_node;
    (void)sequence;
    return MotorLifecycle_RequestRemoteStop();
}

static bool phases_disabled(void *context)
{
    (void)context;
    return BspPwm_ArePhasesDisabled();
}

bool AppProtocol_Init(void)
{
    static const motor_stop_owner_port_t owner = {request_stop, phases_disabled};
    const yg_protocol_motor_service_t motor = {&s_motor_adapter, yg_protocol_motor_adapter_handle};
    const yg_protocol_job_service_t calibration = {NULL, handle_calibration_request};
    s_source = (yg_protocol_motor_status_source_t){0};
    s_target_valid = false;
    s_status_ready = YgProtocolLink_BindMotorStatusSource(&s_source) &&
                     YgProtocolLink_BindDeviceStatusSource(&s_boot_source);
    s_motor_adapter.stop_service = &s_stop_service;
    s_motor_adapter.lifecycle_handler = handle_lifecycle;
    s_motor_adapter.lifecycle_context = NULL;
    return s_status_ready && MotorStopService_Init(&s_stop_service, &owner, NULL) &&
           YgProtocolLink_BindMotorService(&motor) && YgProtocolLink_BindJobService(&calibration);
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

void AppProtocol_Refresh(void)
{
    if (!s_status_ready)
    {
        return;
    }
    MotorStatus status = MotorLifecycle_ReadStatus();
    s_source.faults = fault_bits((MotorFaultCode)status.fault);
    s_source.mode = protocol_mode((MotorControlMode)status.mode);
    s_source.state = status.fault != MOTOR_FAULT_NONE ? 8U : protocol_state(status.state);
    s_source.sample_available = true;
    yg_protocol_motor_feedback_t *values = &s_source.measurements;
    (void)yg_protocol_motor_feedback_from_source(NULL, 1U, 0U, 0U, values);
    s_source.measurements_available = true;
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
