/* v1.1 八状态黄金摘要：覆盖优先级、身份、取消、提交效果和代际耗尽。
 * 无故障事实由 active_faults 推导，删除旧夹具互相矛盾的故障布尔组合。 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "motor_state_internal.h"

static uint32_t seed = UINT32_C(0x91ab32cd);
static uint64_t digest = UINT64_C(14695981039346656037);
static uint32_t random_value(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}
static void record(uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        digest = (digest ^ (uint8_t)(value >> (8U * i))) * UINT64_C(1099511628211);
    }
}

/* 失败专用冻结矩阵：原代/旧代、类别、效果、活动性、守卫及高优先事件交叉。
 * 完整记录规则字段，避免成功摘要掩盖失败被 STOP 吞掉或终态效果变化。 */

/* 冻结本批状态职责：待命 STOP 无收尾、停止仅回待命、清错直达待命、FATAL 无出口。 */
static int test_state_exit_contract(void)
{
    MotorStateFacts facts = {0U, false, true, true, true, true, true, true, true, true, true, true};
    const MotorState expected[] = {MOTOR_STATE_INIT,
                                   MOTOR_STATE_IDLE,
                                   MOTOR_STATE_STOPPING,
                                   MOTOR_STATE_STOPPING,
                                   MOTOR_STATE_STOPPING,
                                   MOTOR_STATE_STOPPING,
                                   MOTOR_STATE_FAULT,
                                   MOTOR_STATE_FATAL};
    for (unsigned i = 0U; i < sizeof(expected) / sizeof(expected[0]); ++i)
    {
        MotorStateRulesContext context;
        MotorStateRules_Init(&context);
        context.state = (MotorState)(MOTOR_STATE_INIT + i);
        context.epoch = 7U;
        if (context.state == MOTOR_STATE_MAINTENANCE)
        {
            context.maintenance_kind = MOTOR_MAINTENANCE_ENCODER_LUT;
            context.maintenance_active = true;
        }
        MotorStateEvent stop = {.events = MOTOR_STATE_EVENT_STOP};
        MotorStateDecision decision = MotorStateRules_Step(&context, &stop, &facts);
        if (context.state != expected[i] ||
            (i < 2U &&
             (context.epoch != 7U || decision.actions != MOTOR_STATE_ACTION_DISABLE_POWER)))
        {
            return 1;
        }
    }
    MotorStateRulesContext context;
    MotorStateRules_Init(&context);
    context.state = MOTOR_STATE_STOPPING;
    MotorStateEvent poll = {0};
    facts.phases_disabled = true;
    (void)MotorStateRules_Step(&context, &poll, &facts);
    if (context.state != MOTOR_STATE_IDLE)
    {
        return 1;
    }
    context.state = MOTOR_STATE_FAULT;
    context.first_fault = context.latched_faults = MOTOR_STATE_FAULT_GUARD;
    MotorStateEvent clear = {.events = MOTOR_STATE_EVENT_CLEAR, .request_epoch = context.epoch};
    facts.self_test_ready = false;
    (void)MotorStateRules_Step(&context, &clear, &facts);
    if (context.state != MOTOR_STATE_FAULT || !context.latched_faults)
    {
        return 1;
    }
    facts.self_test_ready = true;
    (void)MotorStateRules_Step(&context, &clear, &facts);
    if (context.state != MOTOR_STATE_IDLE || context.first_fault || context.latched_faults)
    {
        return 1;
    }
    /* 代际耗尽只能锁入 FATAL，不能报告 CLEAR 成功或清除原故障。 */
    context.state = MOTOR_STATE_FAULT;
    context.epoch = UINT32_MAX;
    context.first_fault = context.latched_faults = MOTOR_STATE_FAULT_GUARD;
    clear.request_epoch = context.epoch;
    MotorStateDecision exhausted = MotorStateRules_Step(&context, &clear, &facts);
    if (context.state != MOTOR_STATE_FATAL || exhausted.rejection != MOTOR_STATE_REJECT_FATAL ||
        context.first_fault != MOTOR_STATE_FAULT_GUARD ||
        !(context.latched_faults & MOTOR_STATE_FAULT_GUARD) ||
        (exhausted.actions & MOTOR_STATE_ACTION_FAULT_CLEARED))
    {
        fprintf(stderr, "FAIL exhausted CLEAR must reject and retain faults in FATAL\n");
        return 1;
    }
    context.state = MOTOR_STATE_FATAL;
    clear.request_epoch = context.epoch;
    (void)MotorStateRules_Step(&context, &clear, &facts);
    MotorStateEvent linked = {.events = MOTOR_STATE_EVENT_LINK_RECOVERED};
    (void)MotorStateRules_Step(&context, &linked, &facts);
    if (context.state != MOTOR_STATE_FATAL)
    {
        return 1;
    }
    puts(
        "PASS state exit contract: idle STOP, stopping IDLE, healthy CLEAR IDLE and latched FATAL");
    return 0;
}

/* 连续事件轨迹补足随机单拍摘要；只比较显式字段，不散列结构padding。 */
static void trace_step(const char *name,
                       MotorStateRulesContext *context,
                       MotorStateEvent event,
                       const MotorStateFacts *facts)
{
    MotorStateDecision result = MotorStateRules_Step(context, &event, facts);
    printf("TRACE RULE %s events=%u request=%u completion=%u kind=%u effects=%u "
           "state=%u epoch=%u active=%u kind=%u result=%u effects=%u first=%u faults=%u "
           "actions=%u rejection=%u\n",
           name,
           event.events,
           event.request_epoch,
           event.completion_epoch,
           event.maintenance_kind,
           event.effects,
           context->state,
           context->epoch,
           context->maintenance_active,
           context->maintenance_kind,
           context->operation_result,
           context->operation_effects,
           context->first_fault,
           context->latched_faults,
           result.actions,
           result.rejection);
}

static void test_ordered_trace(void)
{
    MotorStateRulesContext context;
    MotorStateRules_Init(&context);
    MotorStateFacts facts = {0U, false, true, true, true, true, true, true, true, true, true, true};
    facts.phases_disabled = true;
    MotorStateEvent event = {.events = MOTOR_STATE_EVENT_SELF_TEST_DONE,
                             .completion_epoch = context.epoch};
    trace_step("init-complete", &context, event, &facts);
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_ENABLE,
                              .request_epoch = context.epoch,
                              .control_mode = MOTOR_CONTROL_SPEED};
    trace_step("start-request", &context, event, &facts);
    facts.phases_disabled = false;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_START_DONE,
                              .completion_epoch = context.epoch};
    trace_step("start-complete", &context, event, &facts);
    event =
        (MotorStateEvent){.events = MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_FAULT, .faults = 8U};
    trace_step("fault-and-stop", &context, event, &facts);
    facts.phases_disabled = true;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_CLEAR, .request_epoch = context.epoch};
    trace_step("clear-idle", &context, event, &facts);
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_BEGIN_OPERATION,
                              .request_epoch = context.epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE};
    trace_step("save-request", &context, event, &facts);
    facts.operation_idle = facts.maintenance_released = false;
    uint32_t active_epoch = context.epoch;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_OPERATION_DONE,
                              .completion_epoch = active_epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE,
                              .effects = MOTOR_MAINTENANCE_EFFECT_COMMITTED};
    trace_step("stop-and-save-complete", &context, event, &facts);
    facts.operation_idle = facts.maintenance_released = true;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_CANCEL_DONE,
                              .completion_epoch = active_epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE,
                              .effects = MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED};
    trace_step("stale-cancel", &context, event, &facts);
    event.completion_epoch = context.epoch;
    event.events |= MOTOR_STATE_EVENT_ENABLE;
    event.request_epoch = context.epoch;
    event.control_mode = MOTOR_CONTROL_SPEED;
    trace_step("cancel-before-new-enable", &context, event, &facts);
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_BEGIN_OPERATION,
                              .request_epoch = context.epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE};
    trace_step("new-save", &context, event, &facts);
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_OPERATION_DONE,
                              .completion_epoch = context.epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE,
                              .effects = MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED};
    trace_step("uncommitted-not-success", &context, event, &facts);
    event.effects = MOTOR_MAINTENANCE_EFFECT_COMMITTED;
    trace_step("committed-save", &context, event, &facts);

    /* 完成事件先于同拍新请求；故障释放后仍须显式 CLEAR。 */
    MotorStateRules_Init(&context);
    context.state = MOTOR_STATE_FAULT;
    context.maintenance_active = true;
    context.maintenance_kind = MOTOR_MAINTENANCE_SAVE;
    context.operation_result = MOTOR_MAINTENANCE_CANCEL_REQUESTED;
    context.first_fault = context.latched_faults = MOTOR_STATE_FAULT_GUARD;
    event = (MotorStateEvent){.events =
                                  MOTOR_STATE_EVENT_CANCEL_DONE | MOTOR_STATE_EVENT_BEGIN_OPERATION,
                              .completion_epoch = context.epoch,
                              .request_epoch = context.epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE};
    trace_step("fault-cancel-before-new-operation", &context, event, &facts);

    /* 记录代际耗尽的原始公开决定，不能把 ACCEPTED 自动改成 REJECT_FATAL。 */
    MotorStateRules_Init(&context);
    context.state = MOTOR_STATE_IDLE;
    context.epoch = UINT32_MAX;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_ENABLE,
                              .request_epoch = context.epoch,
                              .control_mode = MOTOR_CONTROL_SPEED};
    trace_step("epoch-exhausted-enable", &context, event, &facts);
    MotorStateRules_Init(&context);
    context.state = MOTOR_STATE_IDLE;
    context.epoch = UINT32_MAX;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_BEGIN_OPERATION,
                              .request_epoch = context.epoch,
                              .maintenance_kind = MOTOR_MAINTENANCE_SAVE};
    trace_step("epoch-exhausted-operation", &context, event, &facts);

    /* 同时不满足守卫和类型时，记录先发生的拒绝。 */
    MotorStateRules_Init(&context);
    context.state = MOTOR_STATE_IDLE;
    facts.inputs_valid = false;
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_ENABLE,
                              .request_epoch = context.epoch,
                              .control_mode = (MotorControlMode)99};
    trace_step("guard-before-invalid-mode", &context, event, &facts);
    event = (MotorStateEvent){.events = MOTOR_STATE_EVENT_BEGIN_OPERATION,
                              .request_epoch = context.epoch,
                              .maintenance_kind = (MotorMaintenanceKind)99};
    trace_step("guard-before-invalid-kind", &context, event, &facts);
}

/* 标定空轮询只消费故障、输入及维护许可；其余事实所有组合均不得影响结果。 */
static int test_calibration_poll_facts(void)
{
    for (unsigned kind = 0U; kind < 2U; ++kind)
    {
        for (unsigned bits = 0U; bits < 4096U; ++bits)
        {
            MotorStateFacts full = {.active_faults =
                                        (bits & 2048U) ? MOTOR_STATE_FAULT_OPERATION : 0U,
                                    .phases_disabled = (bits & 1U) != 0U,
                                    .control_idle = (bits & 2U) != 0U,
                                    .operation_idle = (bits & 4U) != 0U,
                                    .maintenance_released = (bits & 8U) != 0U,
                                    .inputs_valid = (bits & 16U) != 0U,
                                    .configuration_valid = (bits & 32U) != 0U,
                                    .self_test_ready = (bits & 64U) != 0U,
                                    .start_permitted = (bits & 128U) != 0U,
                                    .operation_permitted = (bits & 256U) != 0U,
                                    .startup_confirmed = (bits & 512U) != 0U,
                                    .request_fresh = (bits & 1024U) != 0U};
            MotorStateFacts minimal = {.active_faults = full.active_faults,
                                       .inputs_valid = full.inputs_valid,
                                       .operation_permitted = full.operation_permitted};
            MotorStateRulesContext a = {0};
            MotorStateRules_Init(&a);
            a.state = MOTOR_STATE_MAINTENANCE;
            a.maintenance_active = true;
            a.maintenance_kind =
                kind == 0U ? MOTOR_MAINTENANCE_ENCODER_LUT : MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
            MotorStateRulesContext b = a;
            MotorStateEvent poll = {0};
            MotorStateDecision first = MotorStateRules_Step(&a, &poll, &full);
            MotorStateDecision second = MotorStateRules_Step(&b, &poll, &minimal);
            if (first.actions != second.actions || first.rejection != second.rejection ||
                memcmp(&a, &b, sizeof(a)) != 0)
            {
                return 1;
            }
        }
    }
    puts("PASS 8192 calibration poll fact combinations preserve all decisions");
    return 0;
}

int main(void)
{
    if (test_calibration_poll_facts())
    {
        return 1;
    }
    test_ordered_trace();
    if (test_state_exit_contract())
    {
        return 1;
    }
    /* 空输入的关断语义，以及全部实际维护种类的安全分类，不靠摘要隐含覆盖。 */
    MotorStateRulesContext initial;
    MotorStateRules_Init(NULL);
    MotorStateRules_Init(&initial);
    MotorStateDecision misuse = MotorStateRules_Step(NULL, NULL, NULL);
    if (misuse.rejection != MOTOR_STATE_REJECT_ARGUMENT ||
        misuse.actions != MOTOR_STATE_ACTION_DISABLE_POWER || initial.state != MOTOR_STATE_INIT ||
        initial.epoch != 2U || initial.maintenance_kind != MOTOR_MAINTENANCE_NONE)
    {
        return 1;
    }
    misuse = MotorStateRules_Step(&initial, NULL, NULL);
    if (initial.state != MOTOR_STATE_FATAL || !(misuse.actions & MOTOR_STATE_ACTION_DISABLE_POWER))
    {
        return 1;
    }
    const MotorMaintenanceClass classes[] = {MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_SAVE,
                                             MOTOR_MAINTENANCE_CLASS_INVALID,
                                             MOTOR_MAINTENANCE_CLASS_INVALID,
                                             MOTOR_MAINTENANCE_CLASS_INVALID,
                                             MOTOR_MAINTENANCE_CLASS_ZERO,
                                             MOTOR_MAINTENANCE_CLASS_DEFAULTS,
                                             MOTOR_MAINTENANCE_CLASS_NONE,
                                             MOTOR_MAINTENANCE_CLASS_INVALID};
    for (unsigned kind = 0U; kind < sizeof(classes) / sizeof(classes[0]); ++kind)
    {
        if (MotorStateRules_Classify((MotorMaintenanceKind)kind) != classes[kind])
        {
            return 1;
        }
    }
    const MotorMaintenanceKind kinds[] = {MOTOR_MAINTENANCE_NONE,
                                          MOTOR_MAINTENANCE_ENCODER_LUT,
                                          MOTOR_MAINTENANCE_SAVE,
                                          MOTOR_MAINTENANCE_DEFAULTS,
                                          MOTOR_MAINTENANCE_MECHANICAL_ZERO,
                                          (MotorMaintenanceKind)9};
    const MotorControlMode modes[] = {MOTOR_CONTROL_NONE,
                                      MOTOR_CONTROL_CURRENT,
                                      MOTOR_CONTROL_SPEED,
                                      MOTOR_CONTROL_POSITION,
                                      MOTOR_CONTROL_VQ,
                                      (MotorControlMode)5};
    const uint32_t event_sets[] = {0U,
                                   UINT32_C(1), /* 已退役 BOOT 完成位按非法事件拒绝。 */
                                   MOTOR_STATE_EVENT_SELF_TEST_DONE,
                                   MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_START_DONE,
                                   MOTOR_STATE_EVENT_STOP,
                                   MOTOR_STATE_EVENT_FAULT,
                                   MOTOR_STATE_EVENT_FATAL,
                                   MOTOR_STATE_EVENT_CLEAR,
                                   MOTOR_STATE_EVENT_BEGIN_OPERATION,
                                   MOTOR_STATE_EVENT_OPERATION_DONE,
                                   MOTOR_STATE_EVENT_OPERATION_FAILED,
                                   MOTOR_STATE_EVENT_LINK_RECOVERED,
                                   MOTOR_STATE_EVENT_CANCEL,
                                   MOTOR_STATE_EVENT_CANCEL_DONE,
                                   MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_OPERATION_FAILED,
                                   MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_CANCEL | MOTOR_STATE_EVENT_OPERATION_DONE,
                                   MOTOR_STATE_EVENT_CLEAR | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_OPERATION_DONE | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_CANCEL_DONE | MOTOR_STATE_EVENT_FAULT,
                                   UINT32_C(0x8000),
                                   UINT32_C(0x3fff)};
    for (unsigned i = 0; i < 262144U; ++i)
    {
        MotorStateRulesContext a;
        MotorStateRules_Init(&a);
        a.state = (MotorState)(MOTOR_STATE_INIT + i % 8U);
        /* 保持旧输入序列；已删除的完成标志不再参与规则上下文。 */
        (void)random_value();
        a.epoch = i % 31U == 0U ? UINT32_MAX : 1U + random_value() % 5U;
        a.maintenance_kind = kinds[random_value() % 5U];
        a.maintenance_active = a.maintenance_kind != MOTOR_MAINTENANCE_NONE;
        a.operation_result = (MotorMaintenanceResult)(random_value() % 5U);
        a.operation_effects = (MotorMaintenanceEffect)(random_value() % 3U);
        a.first_fault = i % 19U == 0U ? 1U : 0U;
        a.latched_faults = i % 19U == 0U ? 3U : 0U;
        uint32_t bits = random_value();
        MotorStateFacts g = {.active_faults = i % 29U == 0U ? 4U : 0U,
                             .phases_disabled = (bits & 1U) != 0U,
                             .control_idle = (bits & 2U) != 0U,
                             .operation_idle = (bits & 4U) != 0U,
                             .maintenance_released = (bits & 8U) != 0U,
                             .inputs_valid = (bits & 16U) != 0U,
                             .configuration_valid = true,
                             .self_test_ready = (bits & 64U) != 0U,
                             .start_permitted = (bits & 128U) != 0U,
                             .operation_permitted = (bits & 256U) != 0U,
                             .startup_confirmed = (bits & 512U) != 0U,
                             .request_fresh = (bits & 1024U) != 0U};
        if (i % 4U == 0U)
        {
            g = (MotorStateFacts){
                0U, true, true, true, true, true, true, true, true, true, true, true};
        }
        MotorStateEvent e = {
            .events = event_sets[(i / 8U) % (sizeof(event_sets) / sizeof(event_sets[0]))],
            .faults = i % 17U == 0U ? 8U : 0U,
            .completion_epoch = i % 5U == 0U ? 0U : a.epoch,
            .request_epoch = i % 7U == 0U ? 0U : a.epoch,
            .control_mode = modes[random_value() % 6U],
            .maintenance_kind = kinds[random_value() % 6U],
            .effects = (MotorMaintenanceEffect)(random_value() % 4U)};
        MotorStateDecision r = MotorStateRules_Step(&a, &e, &g);
        record(a.state);
        record(a.epoch);
        record(a.maintenance_active ? MotorStateRules_Classify(a.maintenance_kind)
                                    : MOTOR_MAINTENANCE_CLASS_NONE);
        record(a.operation_result);
        record(a.operation_effects);
        record(a.first_fault);
        record(a.latched_faults);
        record(r.actions);
        record(r.rejection);
    }
    printf("262144 cases, digest %016llx\n", (unsigned long long)digest);
    if (digest != UINT64_C(0x9dd9307dcc7bb53f))
    {
        fprintf(stderr, "FAIL frozen motor-state behavior digest\n");
        return 1;
    }
    return 0;
}
