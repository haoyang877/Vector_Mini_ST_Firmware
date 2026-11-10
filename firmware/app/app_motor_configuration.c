#include "app_motor_configuration.h"
#include "motor_axis_profile.h"
#include "crc32.h"
#include "bsp_system.h"
#include "scaled_integer.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static uint8_t model_id(const MotorAxisPreset *preset)
{
    if (preset == NULL)
    {
        return 0U;
    }
    if (strcmp(preset->model_name, HT8115_MODEL_NAME) == 0)
    {
        return 1U;
    }
    if (strcmp(preset->model_name, YQ5708_MODEL_NAME) == 0)
    {
        return 2U;
    }
    return strcmp(preset->model_name, YQ5740_MODEL_NAME) == 0 ? 3U : 0U;
}

static bool encode_value(float value, uint32_t scale, int32_t *output)
{
    ScaledInteger rounded;
    if (!ScaledInteger_Round(value, scale, &rounded) || rounded.negative ||
        rounded.magnitude > INT32_MAX)
    {
        return false;
    }
    *output = (int32_t)rounded.magnitude;
    return true;
}

static bool describe(const MotorConfiguration *config,
                     uint32_t revision,
                     bool saved,
                     ProtocolMotorConfigurationReply *reply)
{
    reply->version = 1U;
    reply->model = model_id(config->preset);
    reply->node = config->device.node_id;
    uint8_t calibration = config->device.encoder.calib_flag;
    reply->flags = (config->configuration_confirmed ? 1U : 0U) |
                   ((calibration & ENC_CALIB_LINEARIZED) != 0U ? 2U : 0U) |
                   ((calibration & ENC_CALIB_ELECTRICAL_ZERO) != 0U ? 4U : 0U) | (saved ? 8U : 0U);
    reply->revision = revision;
    if (!encode_value(config->application.speed_limit_rad_s, 1000U, &reply->speed_mrad_s))
    {
        return false;
    }
    const MotorWindingThermalConfig *heat = &config->model.thermal;
    const float values[] = {heat->continuous_vector_a,
                            heat->peak_a,
                            heat->heating_tau_s,
                            heat->derating_enter_heat,
                            heat->derating_exit_heat,
                            heat->trip_heat,
                            heat->clear_heat};
    for (unsigned i = 0; i < 7U; ++i)
    {
        if (!encode_value(values[i], i < 3U ? 1000U : 1000000U, &reply->thermal[i]))
        {
            return false;
        }
    }
    return true;
}

static bool same_request(const ProtocolMotorConfigurationRequest *left,
                         const ProtocolMotorConfigurationRequest *right)
{
    return left->revision == right->revision && left->model == right->model &&
           left->node == right->node && left->speed_mrad_s == right->speed_mrad_s &&
           memcmp(left->thermal, right->thermal, sizeof(left->thermal)) == 0;
}

static bool stopped(MotorAxis *axis)
{
    MotorStatus status;
    return axis->state_machine.state == MOTOR_STATE_IDLE && !axis->state_machine.stop_pending &&
           !axis->state_machine.start_pending &&
           axis->state_machine.calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF &&
           axis->motor.fault_code == MOTOR_FAULT_NONE &&
           MotorStateMachine_ArePhasesDisabled(&axis->state_machine) &&
           MotorStateMachine_ReadStatus(&axis->state_machine, &status) &&
           status.measurements_valid && isfinite(status.speed_rad_s) &&
           fabsf(status.speed_rad_s) < 0.523598776f;
}

static void configure(void *context,
                      const ProtocolMotorConfigurationRequest *request,
                      ProtocolMotorConfigurationReply *reply)
{
    AppMotorConfiguration *service = context;
    MotorAxis *axis = service->axis;
    reply->status = PROTOCOL_DENIED;
    if (request->operation == 0U)
    {
        if (request->page == 1U && !service->saved)
        {
            reply->status = PROTOCOL_UNSUPPORTED;
            return;
        }
        const MotorConfiguration *config =
            request->page == 1U ? &service->pending : &axis->configuration;
        uint32_t revision = request->page == 1U ? service->saved_revision
                            : service->saved    ? service->previous_revision
                                                : axis->parameters.stored_record_crc;
        reply->status =
            describe(config, revision, service->saved, reply) ? PROTOCOL_OK : PROTOCOL_FAILED;
        if (service->storage_failed)
        {
            reply->flags |= 16U;
        }
        return;
    }
    if (request->operation >= 2U)
    {
        uint8_t actual_uid[BSP_SYSTEM_UID_BYTES];
        if (BspSystem_ReadUid(actual_uid) != BSP_OK ||
            memcmp(actual_uid, request->uid, sizeof(actual_uid)) != 0)
        {
            return;
        }
    }
    if (request->operation == 3U)
    {
        if (service->saved && !service->storage_failed &&
            request->revision == service->saved_revision && axis->state_machine.starts_locked &&
            axis->motor.fault_code == MOTOR_FAULT_NONE &&
            MotorStateMachine_ArePhasesDisabled(&axis->state_machine))
        {
            if (!service->restart_requested)
            {
                service->restart_at_ms = BspSystem_TimeMs();
            }
            service->restart_requested = true;
            reply->status = PROTOCOL_ACCEPTED;
        }
        return;
    }
    if (request->operation == 2U)
    {
        if (!service->prepared || request->revision != service->prepare_number ||
            service->storage_failed)
        {
            return;
        }
        if (service->saved)
        {
            (void)describe(&service->pending, service->saved_revision, true, reply);
            reply->status = PROTOCOL_OK;
            return;
        }
        if (!stopped(axis) || service->saved_request.revision != axis->parameters.stored_record_crc)
        {
            return;
        }
        if (!MotorStateMachine_LockStartsAndRequestStop(&axis->state_machine))
        {
            return;
        }
        service->previous_revision = service->saved_request.revision;
        Param_Upload(&axis->record.motor, &service->pending);
        memset(&axis->record.output, 0, sizeof(axis->record.output));
        uint32_t expected = Crc32_Compute(&axis->record, sizeof(axis->record), 0U);
        uint32_t mask = BspSystem_EnterCritical();
        bool off = axis->state_machine.state == MOTOR_STATE_IDLE &&
                   axis->motor.fault_code == MOTOR_FAULT_NONE &&
                   axis->hardware->power.phases_disabled();
        bool stored = off &&
                      axis->hardware->write_record(&axis->record, sizeof(axis->record)) == BSP_OK &&
                      axis->hardware->read_record(&axis->record, sizeof(axis->record)) == BSP_OK &&
                      expected == Crc32_Compute(&axis->record, sizeof(axis->record), 0U);
        BspSystem_ExitCritical(mask);
        if (!stored)
        {
            service->storage_failed = true;
            reply->status = PROTOCOL_FAILED;
            return;
        }
        service->saved = true;
        service->saved_revision = axis->record.motor.record_crc32;
        axis->parameters.stored_record_crc = service->saved_revision;
        (void)describe(&service->pending, service->saved_revision, true, reply);
        reply->status = PROTOCOL_OK;
        return;
    }
    if (service->saved)
    {
        reply->status =
            same_request(request, &service->saved_request) ? PROTOCOL_OK : PROTOCOL_BUSY;
        if (reply->status == PROTOCOL_OK &&
            !describe(&service->pending, service->prepare_number, true, reply))
        {
            reply->status = PROTOCOL_FAILED;
        }
        return;
    }
    if (service->storage_failed || !stopped(axis))
    {
        return;
    }
    if (request->revision != axis->parameters.stored_record_crc)
    {
        reply->detail = 1U;
        return;
    }
    const MotorAxisPreset *preset = MotorAxisProfile_ForNode(request->node);
    if (request->version != 1U || request->flags != 1U || request->model != model_id(preset) ||
        preset == NULL)
    {
        reply->status = PROTOCOL_INVALID_ARGUMENT;
        return;
    }
    bool default_heat = true;
    float heat_values[7];
    for (unsigned i = 0; i < 7U; ++i)
    {
        default_heat = default_heat && request->thermal[i] == 0;
        heat_values[i] = request->thermal[i] * (i < 3U ? 0.001f : 0.000001f);
    }
    MotorWindingThermalConfig heat = {heat_values[0],
                                      heat_values[1],
                                      heat_values[2],
                                      heat_values[3],
                                      heat_values[4],
                                      heat_values[5],
                                      heat_values[6]};
    if (service->prepared && same_request(request, &service->saved_request))
    {
        (void)describe(&service->pending, service->prepare_number, false, reply);
        reply->flags |= 32U;
        reply->status = PROTOCOL_OK;
        return;
    }
    /* 新候选可能验证失败，先撤销旧票据，不能用旧提交请求保存半更新的候选。 */
    service->prepared = false;
    float travel = preset->output_travel_limit_rad * preset->gear_ratio;
    if (!Param_CreateAxis(request->node,
                          -travel,
                          travel,
                          request->speed_mrad_s * 0.001f,
                          default_heat ? NULL : &heat,
                          &service->pending))
    {
        reply->status = PROTOCOL_OUT_OF_RANGE;
        return;
    }
    service->pending.configuration_confirmed = true;
    if (!describe(&service->pending, 0U, false, reply))
    {
        reply->status = PROTOCOL_OUT_OF_RANGE;
        return;
    }
    if (!service->prepared)
    {
        if (service->prepare_number == UINT32_MAX)
        {
            reply->status = PROTOCOL_BUSY;
            return;
        }
        ++service->prepare_number;
    }
    service->prepared = true;
    service->saved_request = *request;
    (void)describe(&service->pending, service->prepare_number, false, reply);
    reply->flags |= 32U;
    reply->status = PROTOCOL_OK;
}

void AppMotorConfiguration_Init(AppMotorConfiguration *service,
                                MotorAxis *axis,
                                ProtocolEndpoint *endpoint)
{
    service->axis = axis;
    service->endpoint = endpoint;
    endpoint->configuration = (ProtocolMotorConfiguration){service, configure};
}

void AppMotorConfiguration_Process(AppMotorConfiguration *service, uint32_t now_ms)
{
    if (service->restart_requested && (uint32_t)(now_ms - service->restart_at_ms) >= 100U &&
        queue_count(&service->endpoint->tx_queue) == 0U && !BspCan_IsTransmitBusy() &&
        BspCan_IsTransmitComplete() && service->axis->hardware->power.phases_disabled() &&
        service->axis->state_machine.starts_locked)
    {
        service->restart_requested = false;
        BspSystem_Reset();
    }
}
