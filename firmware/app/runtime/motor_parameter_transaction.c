#include "motor_parameter_transaction.h"

#include <stddef.h>

#include "critical_hw.h"
#include "foc_param.h"
#include "param_store.h"
#include "maintenance_workspace.h"
#include "motor_execution.h"

/* 参数记录与其它维护任务互斥借用既有固定工作区，不常驻完整参数副本。 */
typedef char ParamRecord_FitsWorkspace
    [(sizeof(InterfaceParam) <= MAINTENANCE_WORKSPACE_CAPACITY_BYTES) ? 1 : -1];

/* 活动修订与最近变更身份；不常驻 InterfaceParam 副本，不复用 schema 版本。 */
volatile uint32_t s_config_revision;
uint32_t s_revision_work_epoch;
struct
{
    ParameterReadRequest request;
    ParameterValue value;
    bool valid;
} s_last_read;
struct
{
    ParameterWriteRequest request;
    ParameterValue applied;
    bool valid;
} s_last_write;
struct
{
    ParameterSaveRequest request;
    uint32_t ticket, persisted_revision;
    ParameterStatus result;
    bool valid;
} s_save_job;

void MotorParameterTransaction_AdvanceRevision(void)
{
    if (s_config_revision != UINT32_MAX)
    {
        ++s_config_revision;
    }
}

/* 启动时只借用一份既有大小的固定工作区记录，不额外常驻配置副本。 */
static void load_parameters(MotorControlState *motor, EncoderState *encoder)
{
    InterfaceParam *record = MaintenanceWorkspace_BorrowParameter();
    if (record == NULL || !param_store_read(record, sizeof(*record)))
    {
        Param_Return_Default(motor, encoder);
    }
    else
    {
        Param_Download(record, motor, encoder);
    }
    if (record != NULL)
    {
        MaintenanceWorkspace_ReleaseParameter(record);
    }
}

void MotorParameterTransaction_Init(MotorControlState *motor, EncoderState *encoder)
{
    s_config_revision = 1U;
    s_revision_work_epoch = UINT32_MAX;
    s_last_read.valid = s_last_write.valid = s_save_job.valid = false;
    load_parameters(motor, encoder);
}

void MotorParameterTransaction_AdvanceMaintenanceRevision(uint32_t work_epoch)
{
    if (work_epoch != s_revision_work_epoch)
    {
        s_revision_work_epoch = work_epoch;
        MotorParameterTransaction_AdvanceRevision();
    }
}

bool MotorParameterTransaction_SaveRecord(const MotorControlState *motor,
                                          const EncoderState *encoder,
                                          const FocBackgroundClaim *claim)
{
    bool success = false;
    InterfaceParam *record = MaintenanceWorkspace_BorrowParameter();
    if (record != NULL)
    {
        Param_Upload(record, motor, encoder);
        uint32_t mask = critical_hw_enter();
        if (MotorStateMachine_BackgroundValid(claim))
        {
            success = param_store_write(record, sizeof(*record));
        }
        critical_hw_exit(mask);
        MaintenanceWorkspace_ReleaseParameter(record);
    }
    return success;
}

void MotorParameterTransaction_FinishSave(uint32_t ticket_epoch, bool current, bool success)
{
    if (s_save_job.valid && s_save_job.ticket == ticket_epoch)
    {
        s_save_job.result = !current  ? PARAMETER_CANCELLED
                            : success ? PARAMETER_OK
                                      : PARAMETER_PERSIST_FAILED;
        if (current && success)
        {
            s_save_job.persisted_revision = s_save_job.request.expected_revision;
        }
    }
}

/* 比较实际字段而非含 padding 的结构内存；幂等身份包含原始整数位模式。 */
static bool same_write(const ParameterWriteRequest *left, const ParameterWriteRequest *right)
{
    return left->parameter_id == right->parameter_id && left->value_type == right->value_type &&
           left->expected_revision == right->expected_revision &&
           left->raw_value == right->raw_value;
}
static bool stale_sequence(uint8_t source, uint16_t sequence)
{
    /* 单一受权主站的半圈窗口；相同身份重放在调用方先匹配缓存，避免再执行。 */
    if (s_last_write.valid && source == s_last_write.request.source_node &&
        (uint16_t)(sequence - s_last_write.request.sequence) >= 0x8000U)
    {
        return true;
    }
    if (s_save_job.valid && source == s_save_job.request.source_node &&
        (uint16_t)(sequence - s_save_job.request.sequence) >= 0x8000U)
    {
        return true;
    }
    return false;
}

/* 调用方持有临界区；身份冲突：来源与序号命中任一已缓存票据。各入口先各自重放主缓存结果。 */
static bool matches_cached_request(uint8_t source_node, uint16_t sequence)
{
    return (s_last_read.valid && source_node == s_last_read.request.source_node &&
            sequence == s_last_read.request.sequence) ||
           (s_last_write.valid && source_node == s_last_write.request.source_node &&
            sequence == s_last_write.request.sequence) ||
           (s_save_job.valid && source_node == s_save_job.request.source_node &&
            sequence == s_save_job.request.sequence);
}

/* 调用方持有临界区；新请求也收敛旧SAVE，不能依赖主机继续轮询已取消任务。 */
static void refresh_save_result(void)
{
    if (!s_save_job.valid || s_save_job.result != PARAMETER_ACCEPTED)
    {
        return;
    }
    MotorMaintenanceProgress progress;
    if (!MotorStateMachine_ReadMaintenance(s_save_job.ticket, &progress) ||
        progress.result == MOTOR_MAINTENANCE_CANCELLED)
    {
        s_save_job.result = PARAMETER_CANCELLED;
    }
    else if (progress.result == MOTOR_MAINTENANCE_FAILED)
    {
        s_save_job.result = PARAMETER_PERSIST_FAILED;
    }
}

ParameterStatus MotorParameterTransaction_Read(const MotorControlState *motor,
                                               const EncoderState *encoder,
                                               const ParameterReadRequest *request,
                                               ParameterValue *value)
{
    if (value == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *value = (ParameterValue){0};
    if (request == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    ParameterStatus result;
    if (s_last_read.valid && request->source_node == s_last_read.request.source_node &&
        request->sequence == s_last_read.request.sequence)
    {
        result = request->parameter_id == s_last_read.request.parameter_id
                     ? PARAMETER_OK
                     : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *value = s_last_read.value;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else
    {
        MotorState state = MotorStateMachine_GetState();
        result = state == MOTOR_STATE_MAINTENANCE || state == MOTOR_STATE_STARTING ||
                         state == MOTOR_STATE_STOPPING
                     ? PARAMETER_BUSY
                     : ParameterValue_Read(
                           motor, encoder, s_config_revision, request->parameter_id, value);
        if (request->parameter_id >= 0x4FU && request->parameter_id <= 0x51U)
        {
            MotorReadiness readiness;
            MotorStateMachine_ReadReadiness(&readiness);
            uint32_t raw = request->parameter_id == 0x4FU   ? readiness.start_blockers
                           : request->parameter_id == 0x50U ? readiness.self_test_checks
                                                            : readiness.startable_modes;
            *value = (ParameterValue){request->parameter_id,
                                      PARAMETER_VALUE_U32,
                                      PARAMETER_ACCESS_READ,
                                      0U,
                                      s_config_revision,
                                      raw};
            result = PARAMETER_OK;
        }
        if (result == PARAMETER_OK)
        {
            if (!MotorStateMachine_ConfigurationAllowed())
            {
                value->access &= (uint8_t)~PARAMETER_ACCESS_WRITE_IDLE;
            }
            s_last_read.request = *request;
            s_last_read.value = *value;
            s_last_read.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorParameterTransaction_Write(MotorControlState *motor,
                                                FocState *foc,
                                                PiController *speed_pi,
                                                const EncoderState *encoder,
                                                const ParameterWriteRequest *request,
                                                ParameterValue *applied)
{
    if (applied == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *applied = (ParameterValue){0};
    if (request == NULL || request->reserved != 0U || request->reserved_sequence != 0U)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    ParameterStatus result;
    if (s_last_write.valid && request->source_node == s_last_write.request.source_node &&
        request->sequence == s_last_write.request.sequence)
    {
        result =
            same_write(request, &s_last_write.request) ? PARAMETER_OK : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *applied = s_last_write.applied;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = PARAMETER_FAULT;
    }
    else if (!MotorStateMachine_ConfigurationAllowed() ||
             (s_save_job.valid && s_save_job.result == PARAMETER_ACCEPTED))
    {
        result = PARAMETER_WRONG_STATE;
    }
    else if (request->expected_revision != s_config_revision || s_config_revision == UINT32_MAX)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        result = ParameterValue_Apply(motor, request);
        if (result == PARAMETER_OK)
        {
            MotorParameterTransaction_AdvanceRevision();
            /* 原有控制器复位边界，不在这里重写控制数学或增益公式。 */
            MotorOuterLoop_RequestReset();
            FOC_CurrentController_Reset(foc);
            PiController_Reset(speed_pi);
            result = ParameterValue_Read(
                motor, encoder, s_config_revision, request->parameter_id, applied);
            if (result == PARAMETER_OK)
            {
                s_last_write.request = *request;
                s_last_write.applied = *applied;
                s_last_write.valid = true;
            }
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorParameterTransaction_RequestSave(const ParameterSaveRequest *request,
                                                      uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *ticket = 0U;
    if (request == NULL || request->reserved != 0U)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    ParameterStatus result;
    if (s_save_job.valid && request->source_node == s_save_job.request.source_node &&
        request->sequence == s_save_job.request.sequence)
    {
        result = request->expected_revision == s_save_job.request.expected_revision
                     ? s_save_job.result
                     : PARAMETER_CONFLICTING_RETRY;
        if (result != PARAMETER_CONFLICTING_RETRY)
        {
            *ticket = s_save_job.ticket;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (s_save_job.valid && s_save_job.result == PARAMETER_ACCEPTED)
    {
        result = PARAMETER_BUSY;
    }
    else if (request->expected_revision != s_config_revision)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        MotorControlResult accepted =
            MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, ticket);
        result = accepted == MOTOR_CONTROL_ACCEPTED ? PARAMETER_ACCEPTED
                 : accepted == MOTOR_CONTROL_BUSY   ? PARAMETER_BUSY
                 : accepted == MOTOR_CONTROL_FAULT  ? PARAMETER_FAULT
                                                    : PARAMETER_WRONG_STATE;
        if (result == PARAMETER_ACCEPTED)
        {
            s_save_job.request = *request;
            s_save_job.ticket = *ticket;
            s_save_job.persisted_revision = 0U;
            s_save_job.result = result;
            s_save_job.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorParameterTransaction_PollSave(uint32_t ticket, uint32_t *persisted_revision)
{
    if (persisted_revision == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *persisted_revision = 0U;
    uint32_t mask = critical_hw_enter();
    ParameterStatus result = PARAMETER_STALE_TICKET;
    if (s_save_job.valid && ticket != 0U && ticket == s_save_job.ticket)
    {
        refresh_save_result();
        result = s_save_job.result;
        if (result == PARAMETER_OK)
        {
            *persisted_revision = s_save_job.persisted_revision;
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorParameterTransaction_SaveIdle(void)
{
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    bool idle = !s_save_job.valid || s_save_job.result != PARAMETER_ACCEPTED;
    critical_hw_exit(mask);
    return idle;
}

bool MotorParameterTransaction_RevisionAvailable(void)
{
    return s_config_revision != 0U && s_config_revision != UINT32_MAX;
}
