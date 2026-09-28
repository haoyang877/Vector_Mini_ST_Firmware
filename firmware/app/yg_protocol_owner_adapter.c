#include "yg_protocol_owner_adapter.h"

#include <string.h>

#include "foc_run_state.h"
#include "critical_hw.h"
#include "motor_state.h"
#include "yg_protocol_link.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_job.h"

/* 此处只做协议到领域值的映射；配置与事务由 motor_state 所有者决定。 */
static void map_parameter_result(ParameterStatus status, yg_protocol_service_reply_t *reply)
{
    reply->detail = 0U;
    switch (status)
    {
    case PARAMETER_OK:
        reply->status = YG_PROTOCOL_SERVICE_OK;
        break;
    case PARAMETER_ACCEPTED:
        reply->status = YG_PROTOCOL_SERVICE_ACCEPTED;
        break;
    case PARAMETER_BUSY:
        reply->status = YG_PROTOCOL_SERVICE_BUSY;
        break;
    case PARAMETER_UNSUPPORTED:
        reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_UNSUPPORTED_ID;
        break;
    case PARAMETER_READ_ONLY:
        reply->status = YG_PROTOCOL_SERVICE_DENIED;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_READ_ONLY;
        break;
    case PARAMETER_REVISION_CONFLICT:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_REVISION_CONFLICT;
        break;
    case PARAMETER_PERSIST_FAILED:
        reply->status = YG_PROTOCOL_SERVICE_FAILED;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_PERSIST_FAILED;
        break;
    case PARAMETER_STALE_SEQUENCE:
    case PARAMETER_STALE_TICKET:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_STALE_SEQUENCE;
        break;
    case PARAMETER_CONFLICTING_RETRY:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_CONFLICTING_RETRY;
        break;
    case PARAMETER_WRONG_STATE:
    case PARAMETER_CANCELLED:
        reply->status = YG_PROTOCOL_SERVICE_DENIED;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_WRONG_STATE;
        break;
    case PARAMETER_OUT_OF_RANGE:
        reply->status = YG_PROTOCOL_SERVICE_OUT_OF_RANGE;
        break;
    case PARAMETER_FAULT:
        reply->status = YG_PROTOCOL_SERVICE_FAULT;
        break;
    default:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_PARAMETER_DETAIL_BAD_FIELD;
        break;
    }
}

static void handle_parameter(void *context,
                             const yg_protocol_parameter_request_t *request,
                             yg_protocol_service_reply_t *reply)
{
    (void)context;
    if (reply == NULL)
    {
        return;
    }
    *reply = (yg_protocol_service_reply_t){.status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT};
    if (request == NULL || request->parameter_id > UINT8_MAX)
    {
        return;
    }
    ParameterValue value = {0};
    ParameterStatus result;
    uint32_t raw_value;
    memcpy(&raw_value, &request->value, sizeof(raw_value));
    switch (request->operation)
    {
    case YG_PROTOCOL_PARAMETER_READ:
    {
        ParameterReadRequest command = {
            (uint8_t)request->parameter_id, request->source_node, request->sequence};
        result = MotorControl_ReadParameter(&command, &value);
        break;
    }
    case YG_PROTOCOL_PARAMETER_WRITE:
    {
        ParameterWriteRequest command = {(uint8_t)request->parameter_id,
                                         request->value_type,
                                         request->source_node,
                                         0U,
                                         request->sequence,
                                         0U,
                                         request->expected_revision,
                                         raw_value};
        result = MotorControl_WriteParameter(&command, &value);
        break;
    }
    case YG_PROTOCOL_PARAMETER_SAVE:
    {
        ParameterSaveRequest command = {
            request->source_node, 0U, request->sequence, request->expected_revision};
        uint32_t ticket = 0U, persisted_revision = 0U;
        result = MotorControl_RequestSave(&command, &ticket);
        if (result == PARAMETER_ACCEPTED || result == PARAMETER_OK)
        {
            result = MotorControl_PollSave(ticket, &persisted_revision);
            reply->token = ticket;
            reply->revision = result == PARAMETER_OK ? persisted_revision : 0U;
        }
        map_parameter_result(result, reply);
        return;
    }
    default:
        result = PARAMETER_UNSUPPORTED;
        break;
    }
    map_parameter_result(result, reply);
    if (result == PARAMETER_OK)
    {
        reply->revision = value.revision;
        memcpy(&reply->value, &value.raw_value, sizeof(reply->value));
        reply->value_type = value.value_type;
        reply->access = value.access;
    }
}

void YgProtocolOwner_HandleLifecycle(void *context,
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
        MotorControlResult result = FocRunState_RequestClear();
        /* owner 已同步清锁存；SELF_TEST/READY 仍由后续监督拍与状态反馈确认。 */
        reply->status = result == MOTOR_CONTROL_ACCEPTED ? YG_PROTOCOL_SERVICE_OK
                        : result == MOTOR_CONTROL_BUSY   ? YG_PROTOCOL_SERVICE_BUSY
                                                         : YG_PROTOCOL_SERVICE_DENIED;
    }
    /* 101 的 enable=1 是低侧制动候选，不能偷换成电流模式启动。 */
}

static uint16_t job_wire_kind(MotorMaintenanceKind kind)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
        return YG_PROTOCOL_JOB_KIND_ENCODER_LUT;
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO;
    case MOTOR_MAINTENANCE_COGGING:
        return YG_PROTOCOL_JOB_KIND_COGGING;
    default:
        return 0U;
    }
}

static void job_progress_reply(const MotorMaintenanceProgress *progress,
                               yg_protocol_job_reply_t *reply)
{
    reply->token = progress->ticket;
    reply->kind = job_wire_kind(progress->kind);
    switch (progress->result)
    {
    case APP_OPERATION_NO_RESULT:
        reply->phase = YG_PROTOCOL_JOB_PHASE_IN_PROGRESS;
        break;
    case APP_OPERATION_COMPLETED:
        reply->phase = YG_PROTOCOL_JOB_PHASE_COMPLETED;
        break;
    case APP_OPERATION_CANCEL_REQUESTED:
        reply->phase = YG_PROTOCOL_JOB_PHASE_CANCEL_REQUESTED;
        break;
    case APP_OPERATION_CANCELLED:
        reply->phase = YG_PROTOCOL_JOB_PHASE_CANCELLED;
        break;
    case APP_OPERATION_FAILED:
        reply->phase = YG_PROTOCOL_JOB_PHASE_FAILED;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_OPERATION_FAILED;
        break;
    default:
        reply->phase = YG_PROTOCOL_JOB_PHASE_NONE;
        break;
    }
    switch (progress->effects)
    {
    case APP_EFFECT_UNCOMMITTED:
        reply->effects = YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED;
        break;
    case APP_EFFECT_COMMITTED:
        reply->effects = YG_PROTOCOL_JOB_EFFECT_COMMITTED;
        break;
    default:
        reply->effects = YG_PROTOCOL_JOB_EFFECT_UNKNOWN;
        break;
    }
}

static void job_error_reply(MotorControlResult result, yg_protocol_job_reply_t *reply)
{
    switch (result)
    {
    case MOTOR_CONTROL_BUSY:
        reply->status = YG_PROTOCOL_SERVICE_BUSY;
        break;
    case MOTOR_CONTROL_FAULT:
        reply->status = YG_PROTOCOL_SERVICE_FAULT;
        break;
    case MOTOR_CONTROL_DENIED:
        reply->status = YG_PROTOCOL_SERVICE_DENIED;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_WRONG_STATE;
        break;
    case MOTOR_CONTROL_STALE_SEQUENCE:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_STALE_SEQUENCE;
        break;
    case MOTOR_CONTROL_CONFLICTING_RETRY:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_CONFLICTING_RETRY;
        break;
    default:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        reply->detail = YG_PROTOCOL_JOB_DETAIL_BAD_FIELD;
        break;
    }
}

static void
handle_job(void *context, const yg_protocol_job_request_t *request, yg_protocol_job_reply_t *reply)
{
    (void)context;
    if (reply == NULL)
    {
        return;
    }
    *reply = (yg_protocol_job_reply_t){.status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT,
                                       .detail = YG_PROTOCOL_JOB_DETAIL_BAD_FIELD};
    if (request == NULL)
    {
        return;
    }
    uint32_t ticket = request->job_id;
    MotorControlResult result = MOTOR_CONTROL_ACCEPTED;
    if (request->operation == YG_PROTOCOL_JOB_START)
    {
        MotorMaintenanceRequest command = {.sequence = request->sequence,
                                           .source_node = request->source_node};
        switch (request->kind)
        {
        case YG_PROTOCOL_JOB_KIND_ENCODER_LUT:
            command.kind = MOTOR_MAINTENANCE_ENCODER_LUT;
            break;
        case YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO:
            command.kind = MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
            break;
        case YG_PROTOCOL_JOB_KIND_COGGING:
            command.kind = MOTOR_MAINTENANCE_COGGING;
            break;
        default:
            reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
            reply->detail = YG_PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND;
            return;
        }
        result = FocRunState_RequestIdentifiedMaintenance(&command, &ticket);
    }
    else if (request->operation == YG_PROTOCOL_JOB_CANCEL)
    {
        /* 162只开放三个标定种类；类别核对与按ticket取消共同串行化，不能取消参数SAVE。 */
        uint32_t mask = critical_hw_enter();
        MotorMaintenanceProgress candidate;
        bool calibration =
            FocRunState_ReadMaintenance(ticket, &candidate) && job_wire_kind(candidate.kind) != 0U;
        result = calibration ? FocRunState_CancelMaintenance(ticket) : MOTOR_CONTROL_INVALID;
        critical_hw_exit(mask);
        if (result == MOTOR_CONTROL_INVALID)
        {
            reply->detail = YG_PROTOCOL_JOB_DETAIL_STALE_TOKEN;
            return;
        }
    }
    else if (request->operation != YG_PROTOCOL_JOB_QUERY)
    {
        return;
    }
    if (result != MOTOR_CONTROL_ACCEPTED)
    {
        job_error_reply(result, reply);
        return;
    }
    MotorMaintenanceProgress progress;
    if (!FocRunState_ReadMaintenance(ticket, &progress) || job_wire_kind(progress.kind) == 0U)
    {
        reply->detail = YG_PROTOCOL_JOB_DETAIL_STALE_TOKEN;
        return;
    }
    reply->detail = 0U;
    job_progress_reply(&progress, reply);
    /* QUERY的OK仅表示读到进展；phase/effects仍描述原任务实际结果。 */
    reply->status =
        request->operation == YG_PROTOCOL_JOB_QUERY ? YG_PROTOCOL_SERVICE_OK
        : progress.result == APP_OPERATION_FAILED   ? YG_PROTOCOL_SERVICE_FAILED
        : progress.result == APP_OPERATION_COMPLETED || progress.result == APP_OPERATION_CANCELLED
            ? YG_PROTOCOL_SERVICE_OK
            : YG_PROTOCOL_SERVICE_ACCEPTED;
}

bool YgProtocolOwner_BindServices(void)
{
    const yg_protocol_parameter_service_t parameters = {NULL, handle_parameter};
    const yg_protocol_job_service_t jobs = {NULL, handle_job};
    return YgProtocolLink_BindParameterService(&parameters) && YgProtocolLink_BindJobService(&jobs);
}
