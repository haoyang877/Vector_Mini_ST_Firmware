/* 编译完整生产参数事务与 Axis 转发/后台体；桩只提供算法、存储和生命周期边界。 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "motor_state_machine.h"

void MotorStateMachine_ReadReadiness(MotorReadiness *readiness)
{
    *readiness = (MotorReadiness){.start_blockers = 3U, .self_test_checks = MOTOR_SELF_TEST_ALL};
}
#include "critical_hw.h"
#include "motor_state_machine.h"
#include "parameter_value_service.h"
#include "motor_execution.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "maintenance_workspace.h"
#include "param_store.h"
#include "motor_thermal_control.h"

bool MotorThermalControl_Configure(const MotorWindingThermalConfig *config, bool outputs_disabled)
{
    (void)config;
    return outputs_disabled;
}

/* ---- 复刻 motor_axis.c 的私有状态与对象实例；函数体由 runner 从真实源码切片追加 ---- */
static MotorControlState s_motor_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FocState s_foc_state;
#if defined(MOTOR_PARAMETER_TRANSACTION_PRESENT)
/* 真实所有者的状态和全部函数一并编译；不在夹具复制第二份新事务状态。 */
#include "motor_parameter_transaction.c"
#else
static volatile uint32_t s_config_revision;
static struct
{
    ParameterReadRequest request;
    ParameterValue value;
    bool valid;
} s_last_read;
static struct
{
    ParameterWriteRequest request;
    ParameterValue applied;
    bool valid;
} s_last_write;
static struct
{
    ParameterSaveRequest request;
    uint32_t ticket, persisted_revision;
    ParameterStatus result;
    bool valid;
} s_save_job;
#endif

/* ---- 桩：临界区、通信节点与控制器复位 ---- */
static uint32_t critical_depth;
static uint32_t call_trace = 2166136261U;
static uint32_t boundary_errors;
static uint32_t reset_calls;
static void trace_call(uint32_t event)
{
    call_trace = (call_trace ^ event) * 16777619U;
}
uint32_t critical_hw_enter(void)
{
    trace_call(1U);
    return ++critical_depth;
}
void critical_hw_exit(uint32_t state)
{
    (void)state;
    trace_call(2U);
    if (state != critical_depth || critical_depth == 0U)
    {
        ++boundary_errors;
    }
    if (critical_depth > 0U)
    {
        --critical_depth;
    }
}

void MotorOuterLoop_RequestReset(void)
{
    if (critical_depth != 1U)
    {
        ++boundary_errors;
    }
    ++reset_calls;
    trace_call(3U);
}
void FOC_CurrentController_Reset(FocState *foc)
{
    if (foc != &s_foc_state || critical_depth != 1U)
    {
        ++boundary_errors;
    }
    ++reset_calls;
    trace_call(4U);
}
void PiController_Reset(PiController *controller)
{
    if (controller != &s_speed_pi || critical_depth != 1U)
    {
        ++boundary_errors;
    }
    ++reset_calls;
    trace_call(5U);
}

/* ---- 桩：运行所有者；保存作业默认视为已完成，便于核对结果重放 ---- */
static MotorState stub_state = MOTOR_STATE_IDLE;
static bool stub_config_allowed = true;
static bool stub_maintenance_found = true;
static MotorMaintenanceResult stub_maintenance_result = MOTOR_MAINTENANCE_COMPLETED;
static MotorControlResult stub_request_maintenance = MOTOR_CONTROL_ACCEPTED;
static uint32_t stub_ticket = 4242U;
static bool stub_background_available, stub_background_current;
static FocBackgroundKind stub_background_kind = FOC_BACKGROUND_SAVE;
static InterfaceParam stub_record;
static bool stub_borrow_available = true, stub_store_read_ok, stub_store_write_ok = true;
static bool stub_cancel_during_upload, stub_cancel_during_release;
static uint32_t store_writes, record_releases, default_calls, download_calls;

InterfaceParam *MaintenanceWorkspace_BorrowParameter(void)
{
    trace_call(6U);
    if (critical_depth != 0U)
    {
        ++boundary_errors;
    }
    return stub_borrow_available ? &stub_record : NULL;
}
void MaintenanceWorkspace_ReleaseParameter(InterfaceParam *record)
{
    trace_call(7U);
    ++record_releases;
    if (record != &stub_record || critical_depth != 0U)
    {
        ++boundary_errors;
    }
    if (stub_cancel_during_release)
    {
        stub_background_current = false;
    }
}
bool param_store_read(void *record, uint32_t size_bytes)
{
    trace_call(8U);
    if (record != &stub_record || size_bytes != sizeof(stub_record) || critical_depth != 0U)
    {
        ++boundary_errors;
    }
    return stub_store_read_ok;
}
bool param_store_write(const void *record, uint32_t size_bytes)
{
    trace_call(9U);
    ++store_writes;
    if (record != &stub_record || size_bytes != sizeof(stub_record) || critical_depth != 1U)
    {
        ++boundary_errors;
    }
    return stub_store_write_ok;
}
void Param_Return_Default(MotorControlState *motor, EncoderState *encoder)
{
    trace_call(10U);
    ++default_calls;
    if (motor != &s_motor_state || encoder != &s_rotor_encoder)
    {
        ++boundary_errors;
    }
}
void Param_Download(const void *record,
                    uint32_t size_bytes,
                    MotorControlState *motor,
                    EncoderState *encoder)
{
    trace_call(11U);
    ++download_calls;
    if (record != &stub_record || size_bytes != sizeof(stub_record) || motor != &s_motor_state ||
        encoder != &s_rotor_encoder)
    {
        ++boundary_errors;
    }
}
void Param_Upload(InterfaceParam *record,
                  const MotorControlState *motor,
                  const EncoderState *encoder)
{
    trace_call(12U);
    if (record != &stub_record || motor != &s_motor_state || encoder != &s_rotor_encoder ||
        critical_depth != 0U)
    {
        ++boundary_errors;
    }
    if (stub_cancel_during_upload)
    {
        stub_background_current = false;
    }
}

bool MotorStateMachine_BeginBackground(FocBackgroundClaim *claim)
{
    trace_call(14U);
    *claim = (FocBackgroundClaim){stub_ticket, stub_background_kind};
    return stub_background_available;
}
bool MotorStateMachine_BackgroundValid(const FocBackgroundClaim *claim)
{
    trace_call(15U);
    if (claim->epoch != stub_ticket || critical_depth != 1U)
    {
        ++boundary_errors;
    }
    return stub_background_current;
}
void MotorStateMachine_FinishBackground(const FocBackgroundClaim *claim, bool success)
{
    trace_call(16U + (uint32_t)success);
    if (claim->epoch != stub_ticket || critical_depth != 1U)
    {
        ++boundary_errors;
    }
}

MotorState MotorStateMachine_GetState(void)
{
    return stub_state;
}
bool MotorStateMachine_ConfigurationAllowed(void)
{
    return stub_config_allowed;
}
bool MotorStateMachine_ReadMaintenance(uint32_t ticket, MotorMaintenanceProgress *progress)
{
    (void)ticket;
    if (!stub_maintenance_found)
    {
        return false;
    }
    *progress = (MotorMaintenanceProgress){0};
    progress->result = stub_maintenance_result;
    return true;
}
MotorControlResult MotorStateMachine_RequestMaintenance(MotorMaintenanceKind kind, uint32_t *ticket)
{
    (void)kind;
    *ticket = stub_ticket;
    return stub_request_maintenance;
}

/* ---- 桩：参数值服务；记录调用以核对冲突重试不重复应用 ---- */
static ParameterStatus stub_read_status = PARAMETER_OK;
static ParameterStatus stub_apply_status = PARAMETER_OK;
static uint32_t stub_apply_calls;
static uint32_t stub_read_calls;

ParameterStatus ParameterValue_Read(const MotorControlState *motor,
                                    const EncoderState *encoder,
                                    uint32_t revision,
                                    uint8_t parameter_id,
                                    ParameterValue *value)
{
    (void)motor;
    (void)encoder;
    ++stub_read_calls;
    trace_call(18U);
    *value = (ParameterValue){0};
    value->parameter_id = parameter_id;
    value->revision = revision;
    value->access = PARAMETER_ACCESS_READ | PARAMETER_ACCESS_WRITE_IDLE;
    return stub_read_status;
}
ParameterStatus ParameterValue_Apply(MotorControlState *motor, const ParameterWriteRequest *request)
{
    (void)request;
    if (motor != &s_motor_state || critical_depth != 1U)
    {
        ++boundary_errors;
    }
    ++stub_apply_calls;
    trace_call(19U);
    if (stub_apply_status == PARAMETER_OK)
    {
        motor->application.current_limit_a = (float)request->raw_value;
    }
    return stub_apply_status;
}

static uint32_t failures;
#define CHECK(cond)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL line %d: %s\n", __LINE__, #cond);                                         \
            ++failures;                                                                            \
        }                                                                                          \
    } while (0)

static void reset_case(void)
{
    memset(&s_motor_state, 0, sizeof(s_motor_state));
    s_config_revision = 5U;
    s_last_read.valid = s_last_write.valid = s_save_job.valid = false;
    stub_state = MOTOR_STATE_IDLE;
    stub_config_allowed = true;
    stub_maintenance_found = true;
    stub_maintenance_result = MOTOR_MAINTENANCE_COMPLETED;
    stub_request_maintenance = MOTOR_CONTROL_ACCEPTED;
    stub_read_status = stub_apply_status = PARAMETER_OK;
    stub_background_available = stub_background_current = true;
    stub_background_kind = FOC_BACKGROUND_SAVE;
    stub_borrow_available = stub_store_write_ok = true;
    stub_cancel_during_upload = stub_cancel_during_release = false;
    store_writes = record_releases = 0U;
}

static void test_write_contract(void)
{
    reset_case();
    ParameterWriteRequest write = {7U, PARAMETER_VALUE_I32, 1U, 0U, 65535U, 0U, 5U, 1234U};
    ParameterValue applied;
    uint32_t resets_before = reset_calls;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_OK);
    CHECK(applied.revision == 6U && s_config_revision == 6U);
    CHECK(reset_calls == resets_before + 3U);
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_OK);
    CHECK(s_config_revision == 6U && reset_calls == resets_before + 3U);
    for (unsigned field = 0U; field < 4U; ++field)
    {
        ParameterWriteRequest conflict = write;
        if (field == 0U)
        {
            ++conflict.parameter_id;
        }
        else if (field == 1U)
        {
            ++conflict.value_type;
        }
        else if (field == 2U)
        {
            ++conflict.expected_revision;
        }
        else
        {
            conflict.raw_value ^= 0x80000000U;
        }
        CHECK(MotorAxis_WriteParameter(&conflict, &applied) == PARAMETER_CONFLICTING_RETRY);
        CHECK(applied.parameter_id == 0U && applied.revision == 0U);
    }
    /* 半圈窗口与 65535 -> 0 回绕；拒绝时不复位或推进配置。 */
    write.sequence = 32767U;
    write.expected_revision = 6U;
    uint32_t applies_before = stub_apply_calls;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_STALE_SEQUENCE);
    write.sequence = 0U;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_OK);
    CHECK(s_config_revision == 7U && stub_apply_calls == applies_before + 1U);
    write.sequence = 1U;
    write.expected_revision = 7U;
    applies_before = stub_apply_calls;
    resets_before = reset_calls;
    write.reserved = 1U;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_INVALID_ARGUMENT);
    write.reserved = 0U;
    write.reserved_sequence = 1U;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_INVALID_ARGUMENT);
    write.reserved_sequence = 0U;
    s_motor_state.fault_code = MOTOR_FAULT_ENCODER;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_FAULT);
    s_motor_state.fault_code = MOTOR_FAULT_NONE;
    stub_config_allowed = false;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_WRONG_STATE);
    stub_config_allowed = true;
    write.expected_revision = 6U;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_REVISION_CONFLICT);
    write.expected_revision = 7U;
    stub_apply_status = PARAMETER_OUT_OF_RANGE;
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_OUT_OF_RANGE);
    CHECK(stub_apply_calls == applies_before + 1U && reset_calls == resets_before);
    CHECK(s_config_revision == 7U && s_motor_state.application.current_limit_a == 1234.0f);
    s_config_revision = UINT32_MAX;
    write.expected_revision = UINT32_MAX;
    CHECK(!MotorAxis_ConfigurationRevisionAvailable());
    CHECK(MotorAxis_WriteParameter(&write, &applied) == PARAMETER_REVISION_CONFLICT);
}

static void test_save_contract(void)
{
    for (unsigned scenario = 0U; scenario < 7U; ++scenario)
    {
        reset_case();
        ParameterSaveRequest save = {1U, 0U, 30U, 5U};
        uint32_t ticket, persisted_revision = UINT32_MAX;
        CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_ACCEPTED);
        /* 生命周期报告 COMPLETED 仍不能代替真实前台存储完成。 */
        CHECK(MotorAxis_PollSave(ticket, &persisted_revision) == PARAMETER_ACCEPTED);
        CHECK(persisted_revision == 0U && !MotorAxis_SaveIdle());
        stub_borrow_available = scenario != 1U;
        stub_store_write_ok = scenario != 2U;
        stub_cancel_during_upload = scenario == 3U;
        stub_cancel_during_release = scenario == 4U;
        stub_background_available = scenario != 5U;
        stub_background_kind = scenario == 6U ? FOC_BACKGROUND_NONE : FOC_BACKGROUND_SAVE;
        MotorAxis_ServiceBackground();
        ParameterStatus expected = scenario == 0U   ? PARAMETER_OK
                                   : scenario <= 2U ? PARAMETER_PERSIST_FAILED
                                   : scenario <= 4U ? PARAMETER_CANCELLED
                                                    : PARAMETER_ACCEPTED;
        CHECK(MotorAxis_PollSave(ticket, &persisted_revision) == expected);
        CHECK(persisted_revision == (scenario == 0U ? 5U : 0U));
        CHECK(store_writes == ((scenario == 0U || scenario == 2U || scenario == 4U) ? 1U : 0U));
        CHECK(record_releases ==
              ((scenario == 0U || (scenario >= 2U && scenario <= 4U)) ? 1U : 0U));
        CHECK(MotorAxis_RequestSave(&save, &ticket) == expected);
        CHECK(critical_depth == 0U);
    }
    reset_case();
    ParameterSaveRequest save = {1U, 0U, 30U, 5U};
    uint32_t ticket, persisted_revision;
    CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_ACCEPTED);
    uint32_t original_ticket = ticket;
    save.expected_revision = 4U;
    CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_CONFLICTING_RETRY && ticket == 0U);
    stub_maintenance_found = false;
    CHECK(MotorAxis_SaveIdle());
    CHECK(MotorAxis_PollSave(original_ticket, &persisted_revision) == PARAMETER_CANCELLED);
    save.sequence = 31U;
    save.expected_revision = 5U;
    ++stub_ticket;
    stub_maintenance_found = true;
    CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_ACCEPTED);
    CHECK(ticket != original_ticket);
    persisted_revision = UINT32_MAX;
    CHECK(MotorAxis_PollSave(original_ticket, &persisted_revision) == PARAMETER_STALE_TICKET);
    CHECK(persisted_revision == 0U);
    stub_maintenance_result = MOTOR_MAINTENANCE_FAILED;
    CHECK(MotorAxis_PollSave(ticket, &persisted_revision) == PARAMETER_PERSIST_FAILED);
}

static void test_read_and_invalid_contract(void)
{
    reset_case();
    ParameterValue value = {.revision = UINT32_MAX};
    ParameterReadRequest read = {7U, 1U, 10U};
    CHECK(MotorAxis_ReadParameter(NULL, &value) == PARAMETER_INVALID_ARGUMENT);
    CHECK(value.parameter_id == 0U && value.revision == 0U);
    CHECK(MotorAxis_ReadParameter(&read, NULL) == PARAMETER_INVALID_ARGUMENT);
    CHECK(MotorAxis_WriteParameter(NULL, &value) == PARAMETER_INVALID_ARGUMENT);
    CHECK(MotorAxis_WriteParameter(NULL, NULL) == PARAMETER_INVALID_ARGUMENT);
    uint32_t ticket = UINT32_MAX;
    CHECK(MotorAxis_RequestSave(NULL, &ticket) == PARAMETER_INVALID_ARGUMENT && ticket == 0U);
    CHECK(MotorAxis_RequestSave(NULL, NULL) == PARAMETER_INVALID_ARGUMENT);
    CHECK(MotorAxis_PollSave(0U, NULL) == PARAMETER_INVALID_ARGUMENT);
    CHECK(MotorAxis_PollSave(0U, &ticket) == PARAMETER_STALE_TICKET && ticket == 0U);
    stub_state = MOTOR_STATE_STARTING;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_BUSY);
    read.parameter_id = 0x4FU;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.raw_value == 3U && value.revision == 5U);
    stub_state = MOTOR_STATE_IDLE;
    stub_config_allowed = false;
    read.parameter_id = 7U;
    ++read.sequence;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.access == PARAMETER_ACCESS_READ);
}

int main(void)
{
    ParameterValue value;
    uint32_t ticket;
    const uint8_t node = 1U;

    /* 1. 读身份：同来源+序号同参数重放；同身份换参数命中 s_last_read 冲突。 */
    ParameterReadRequest read = {.parameter_id = 7U, .source_node = node, .sequence = 10U};
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    read.parameter_id = 8U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_CONFLICTING_RETRY);

    /* 2. 写身份：成功应用并缓存 s_last_write；读复用写身份必须冲突，且不重复应用。 */
    ParameterWriteRequest write = {.parameter_id = 7U,
                                   .value_type = PARAMETER_VALUE_I32,
                                   .source_node = node,
                                   .sequence = 20U,
                                   .expected_revision = 0U,
                                   .raw_value = 1234U};
    CHECK(MotorAxis_WriteParameter(&write, &value) == PARAMETER_OK);
    read.source_node = node;
    read.sequence = 20U;
    read.parameter_id = 7U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_CONFLICTING_RETRY);
    CHECK(MotorAxis_WriteParameter(&write, &value) == PARAMETER_OK); /* 同字段重放原结果 */
    uint32_t apply_before = stub_apply_calls;
    ParameterWriteRequest write_other = write;
    write_other.raw_value = 9999U;
    CHECK(MotorAxis_WriteParameter(&write_other, &value) == PARAMETER_CONFLICTING_RETRY);
    CHECK(stub_apply_calls == apply_before); /* 冲突重试不重复应用 */

    /* 3. 保存身份：受理后读/写命中 s_save_job 冲突；同身份保存重放原结果与票据。 */
    ParameterSaveRequest save = {.source_node = node, .sequence = 30U, .expected_revision = 1U};
    ticket = 0U;
    CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_ACCEPTED);
    CHECK(ticket == stub_ticket);
    read.sequence = 30U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_CONFLICTING_RETRY);
    write.sequence = 30U;
    CHECK(MotorAxis_WriteParameter(&write, &value) == PARAMETER_CONFLICTING_RETRY);
    ticket = 0U;
    CHECK(MotorAxis_RequestSave(&save, &ticket) == PARAMETER_ACCEPTED);
    CHECK(ticket == stub_ticket);

    /* 4. 不同身份的保存请求在旧 SAVE 未完成时 BUSY，且不返回票据。 */
    ParameterSaveRequest save_other = save;
    save_other.sequence = 40U;
    ticket = 0U;
    CHECK(MotorAxis_RequestSave(&save_other, &ticket) == PARAMETER_BUSY);
    CHECK(ticket == 0U);

    /* 准备资格可在 SELF_TEST 查询；原身份重试仍保持参数身份冲突契约。 */
    stub_state = MOTOR_STATE_INIT;
    read = (ParameterReadRequest){.parameter_id = 0x4FU, .source_node = node, .sequence = 50U};
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.raw_value == 3U && value.access == PARAMETER_ACCESS_READ);
    CHECK(value.value_type == PARAMETER_VALUE_U32 && value.revision == 1U);
    read.parameter_id = 0x50U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_CONFLICTING_RETRY);
    read.sequence = 51U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.raw_value == MOTOR_SELF_TEST_ALL);

    test_write_contract();
    test_save_contract();
    test_read_and_invalid_contract();
    CHECK(boundary_errors == 0U && critical_depth == 0U);
    printf("EQUIVALENCE trace=%u read=%u apply=%u reset=%u revision=%u\n",
           call_trace,
           stub_read_calls,
           stub_apply_calls,
           reset_calls,
           s_config_revision);
#if defined(MOTOR_PARAMETER_TRANSACTION_PRESENT)
    /* 初始化只借用原工作区；维护修订由同一所有者去重并保持饱和语义。 */
    reset_case();
    stub_store_read_ok = true;
    MotorParameterTransaction_Init(&s_motor_state, &s_rotor_encoder);
    CHECK(download_calls == 1U && record_releases == 1U && s_config_revision == 1U);
    CHECK(!s_last_read.valid && !s_last_write.valid && !s_save_job.valid);
    MotorParameterTransaction_AdvanceMaintenanceRevision(4U);
    CHECK(s_config_revision == 2U && s_revision_work_epoch == 4U);
    MotorParameterTransaction_AdvanceMaintenanceRevision(4U);
    CHECK(s_config_revision == 2U);
    MotorParameterTransaction_AdvanceRevision();
    CHECK(s_config_revision == 3U);
    s_config_revision = UINT32_MAX;
    MotorParameterTransaction_AdvanceMaintenanceRevision(5U);
    CHECK(s_config_revision == UINT32_MAX && s_revision_work_epoch == 5U);
    stub_store_read_ok = false;
    MotorParameterTransaction_Init(&s_motor_state, &s_rotor_encoder);
    CHECK(default_calls == 1U && record_releases == 2U && s_config_revision == 1U);
    stub_borrow_available = false;
    MotorParameterTransaction_Init(&s_motor_state, &s_rotor_encoder);
    CHECK(default_calls == 2U && record_releases == 2U);
    CHECK(boundary_errors == 0U && critical_depth == 0U);
#endif
    if (failures == 0U)
    {
        printf("PASS parameter request identity contract (%u read calls)\n", stub_read_calls);
        return 0;
    }
    printf("FAILURES: %u\n", failures);
    return 1;
}
