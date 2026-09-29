/* 使用真实 motor_axis.c 验证参数读/写/保存请求的来源+序号身份冲突合同；不访问硬件。 */

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
#include "param_comm_bridge.h"

/* ---- 复刻 motor_axis.c 的私有状态与对象实例；函数体由 runner 从真实源码切片追加 ---- */
static MotorControlState s_motor_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FocState s_foc_state;
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

/* ---- 桩：临界区、通信节点与控制器复位 ---- */
static uint32_t critical_depth;
uint32_t critical_hw_enter(void)
{
    return ++critical_depth;
}
void critical_hw_exit(uint32_t state)
{
    (void)state;
    if (critical_depth > 0U)
    {
        --critical_depth;
    }
}

uint8_t Protocol_NodeId_Get(void)
{
    return 1U;
}
void MotorOuterLoop_RequestReset(void)
{
}
void FOC_CurrentController_Reset(FocState *foc)
{
    (void)foc;
}
void PiController_Reset(PiController *controller)
{
    (void)controller;
}

/* ---- 桩：运行所有者；保存作业默认视为已完成，便于核对结果重放 ---- */
static AppLifecycleState stub_state = APP_READY;
static bool stub_config_allowed = true;
static bool stub_maintenance_found = true;
static AppOperationResult stub_maintenance_result = APP_OPERATION_COMPLETED;
static MotorControlResult stub_request_maintenance = MOTOR_CONTROL_ACCEPTED;
static uint32_t stub_ticket = 4242U;

AppLifecycleState MotorStateMachine_GetState(void)
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
                                    uint8_t node_id,
                                    uint8_t parameter_id,
                                    ParameterValue *value)
{
    (void)motor;
    (void)encoder;
    (void)node_id;
    ++stub_read_calls;
    *value = (ParameterValue){0};
    value->parameter_id = parameter_id;
    value->revision = revision;
    return stub_read_status;
}
ParameterStatus ParameterValue_Apply(MotorControlState *motor,
                                     uint8_t node_id,
                                     const ParameterWriteRequest *request)
{
    (void)motor;
    (void)node_id;
    (void)request;
    ++stub_apply_calls;
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
    stub_state = APP_SELF_TEST;
    read = (ParameterReadRequest){.parameter_id = 0x4FU, .source_node = node, .sequence = 50U};
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.raw_value == 3U && value.access == PARAMETER_ACCESS_READ);
    CHECK(value.value_type == PARAMETER_VALUE_U32 && value.revision == 1U);
    read.parameter_id = 0x50U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_CONFLICTING_RETRY);
    read.sequence = 51U;
    CHECK(MotorAxis_ReadParameter(&read, &value) == PARAMETER_OK);
    CHECK(value.raw_value == MOTOR_SELF_TEST_ALL);

    if (failures == 0U)
    {
        printf("PASS parameter request identity contract (%u read calls)\n", stub_read_calls);
        return 0;
    }
    printf("FAILURES: %u\n", failures);
    return 1;
}
