#include "motor_state_internal.h"

#include <stddef.h>

/* 优先级与资源校验先于各状态转换；规则只修改调用方上下文，不访问硬件。 */
#define SHUTDOWN_ACTIONS                                                                           \
    (MOTOR_STATE_ACTION_DISABLE_POWER | MOTOR_STATE_ACTION_REVOKE_REQUESTS |                       \
     MOTOR_STATE_ACTION_CANCEL_CONTROL | MOTOR_STATE_ACTION_CANCEL_OPERATION)
#define COMPLETION_EVENTS                                                                          \
    (MOTOR_STATE_EVENT_SELF_TEST_DONE | MOTOR_STATE_EVENT_START_DONE |                             \
     MOTOR_STATE_EVENT_OPERATION_DONE | MOTOR_STATE_EVENT_OPERATION_FAILED |                       \
     MOTOR_STATE_EVENT_CANCEL_DONE)
#define REQUEST_EVENTS                                                                             \
    (MOTOR_STATE_EVENT_ENABLE | MOTOR_STATE_EVENT_CLEAR | MOTOR_STATE_EVENT_BEGIN_OPERATION)
#define ALL_EVENTS                                                                                 \
    (COMPLETION_EVENTS | REQUEST_EVENTS | MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_FAULT |       \
     MOTOR_STATE_EVENT_FATAL)

/* 分类按具体任务推导，不形成可变镜像。 */
MotorMaintenanceClass MotorStateRules_Classify(MotorMaintenanceKind kind)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_SAVE:
        return MOTOR_MAINTENANCE_CLASS_SAVE;
    case MOTOR_MAINTENANCE_DEFAULTS:
        return MOTOR_MAINTENANCE_CLASS_DEFAULTS;
    case MOTOR_MAINTENANCE_MECHANICAL_ZERO:
        return MOTOR_MAINTENANCE_CLASS_ZERO;
    case MOTOR_MAINTENANCE_NONE:
        return MOTOR_MAINTENANCE_CLASS_NONE;
    case MOTOR_MAINTENANCE_ENCODER_LUT:
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return MOTOR_MAINTENANCE_CLASS_CALIBRATION;
    default:
        return MOTOR_MAINTENANCE_CLASS_INVALID;
    }
}

/* 最近任务身份保留给定向清理；仅活动会话参与准入与归还匹配。 */
static MotorMaintenanceClass active_maintenance_class(const MotorStateRulesContext *context)
{
    return context->maintenance_active ? MotorStateRules_Classify(context->maintenance_kind)
                                       : MOTOR_MAINTENANCE_CLASS_NONE;
}

/* 相已关、控制/操作空闲且维护资源已释放，缺一不可确认停止。 */
static bool resources_quiescent(const MotorStateFacts *facts)
{
    return facts->phases_disabled && facts->control_idle && facts->operation_idle &&
           facts->maintenance_released;
}

static bool running_permitted(const MotorStateFacts *facts)
{
    return facts->inputs_valid && facts->start_permitted && facts->operation_idle &&
           facts->maintenance_released;
}

static void enter_fatal(MotorStateRulesContext *context, uint32_t *actions)
{
    context->latched_faults |= MOTOR_STATE_FAULT_INTERNAL;
    context->state = MOTOR_STATE_FATAL;
    *actions |= SHUTDOWN_ACTIONS;
}

/* 代际不可回绕；耗尽后保留锁存并进入 FATAL。 */
static bool advance_epoch(MotorStateRulesContext *context, uint32_t *actions)
{
    if (context->epoch == UINT32_MAX)
    {
        enter_fatal(context, actions);
        return false;
    }
    ++context->epoch;
    *actions |= MOTOR_STATE_ACTION_REVOKE_REQUESTS;
    return true;
}

/* 发起关断并撤销本代请求；真实停止由资源归还事实确认。 */
static void begin_shutdown(MotorStateRulesContext *context, MotorState state, uint32_t *actions)
{
    /* 取消不回滚外部效果；等实际归还前只能记录 UNKNOWN。 */
    if (context->maintenance_active && context->operation_result == MOTOR_MAINTENANCE_NO_RESULT)
    {
        context->operation_result = MOTOR_MAINTENANCE_CANCEL_REQUESTED;
        context->operation_effects = MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
    }
    *actions |= SHUTDOWN_ACTIONS;
    if (advance_epoch(context, actions))
    {
        context->state = state;
    }
}

/* 已处 FAULT 时只累积故障，避免重复推进取消代际。 */
static void enter_fault(MotorStateRulesContext *context, uint32_t faults, uint32_t *actions)
{
    context->latched_faults |= faults;
    *actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
    if (context->state != MOTOR_STATE_FAULT)
    {
        begin_shutdown(context, MOTOR_STATE_FAULT, actions);
    }
}

/* FAULT 中的停止只补充取消，不清锁存或改成 STOPPING。 */
static void enter_stopping(MotorStateRulesContext *context, uint32_t *actions)
{
    switch (context->state)
    {
    case MOTOR_STATE_STARTING:
    case MOTOR_STATE_RUNNING:
    case MOTOR_STATE_MAINTENANCE:
        begin_shutdown(context, MOTOR_STATE_STOPPING, actions);
        break;
    case MOTOR_STATE_STOPPING:
        *actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
        break;
    case MOTOR_STATE_FAULT:
        *actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
        *actions |= MOTOR_STATE_ACTION_CANCEL_CONTROL | MOTOR_STATE_ACTION_CANCEL_OPERATION;
        break;
    default:
        /* INIT／IDLE 没有活动控制；输出关闭由公共约束保持。 */
        break;
    }
}

/* STOPPING 已执行关相和取消，确认全部结束后仅返回 IDLE 或锁存故障。 */
static void confirm_stopped(MotorStateRulesContext *context, const MotorStateFacts *facts)
{
    if (context->state != MOTOR_STATE_STOPPING || !resources_quiescent(facts) ||
        context->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
    {
        return;
    }
    context->maintenance_active = false;
    if (context->latched_faults)
    {
        context->state = MOTOR_STATE_FAULT;
    }
    else
    {
        context->state = MOTOR_STATE_IDLE;
    }
}

/* 已收到的维护失败按原代际记录为故障及实际提交效果。 */
static uint32_t record_detected_faults(MotorStateRulesContext *context,
                                       const MotorStateEvent *event,
                                       const MotorStateFacts *facts)
{
    uint32_t faults = facts->active_faults;
    if (event->events == MOTOR_STATE_EVENT_FAULT)
    {
        faults |= event->faults ? event->faults : MOTOR_STATE_FAULT_INTERNAL;
    }
    if (event->events == MOTOR_STATE_EVENT_OPERATION_FAILED &&
        context->state == MOTOR_STATE_MAINTENANCE && event->completion_epoch == context->epoch &&
        MotorStateRules_Classify(event->maintenance_kind) == active_maintenance_class(context))
    {
        context->operation_result = MOTOR_MAINTENANCE_FAILED;
        context->operation_effects = (unsigned)event->effects <= MOTOR_MAINTENANCE_EFFECT_COMMITTED
                                         ? event->effects
                                         : MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
        faults |= event->faults ? event->faults : MOTOR_STATE_FAULT_OPERATION;
    }
    return faults;
}

/* 运行中的许可是持续约束；活动故障已由公共优先级入口处理，不在此重复判断。 */
static bool execution_guard_failed(const MotorStateRulesContext *context,
                                   const MotorStateEvent *event,
                                   const MotorStateFacts *facts)
{
    switch (context->state)
    {
    case MOTOR_STATE_STARTING:
        return !running_permitted(facts);
    case MOTOR_STATE_RUNNING:
        return !running_permitted(facts) ||
               (facts->phases_disabled && event->events != MOTOR_STATE_EVENT_STOP);
    case MOTOR_STATE_MAINTENANCE:
        return !facts->operation_permitted ||
               (active_maintenance_class(context) == MOTOR_MAINTENANCE_CLASS_CALIBRATION
                    ? !facts->inputs_valid
                    : !facts->phases_disabled);
    default:
        return false;
    }
}

/* 返回 true 表示本次事件已收口；优先级结束后才检查普通事件格式和身份。 */
static bool handle_priority_events(MotorStateRulesContext *context,
                                   const MotorStateEvent *event,
                                   const MotorStateFacts *facts,
                                   MotorStateDecision *decision)
{
    if (context->state == MOTOR_STATE_FATAL)
    {
        decision->actions = SHUTDOWN_ACTIONS;
        decision->accepted = false;
        return true;
    }
    if (event->events == MOTOR_STATE_EVENT_FATAL)
    {
        context->latched_faults |= event->faults;
        enter_fatal(context, &decision->actions);
        return true;
    }
    uint32_t faults = record_detected_faults(context, event, facts);
    if (faults)
    {
        /* 锁存故障仍允许取消归还确认。 */
        bool releasing_fault =
            context->state == MOTOR_STATE_FAULT && event->events == MOTOR_STATE_EVENT_CANCEL_DONE;
        enter_fault(context, faults, &decision->actions);
        if (!releasing_fault)
        {
            decision->accepted = false;
            return true;
        }
    }
    if (execution_guard_failed(context, event, facts))
    {
        enter_fault(context, MOTOR_STATE_FAULT_GUARD, &decision->actions);
        decision->accepted = false;
        return true;
    }
    if (event->events == MOTOR_STATE_EVENT_STOP)
    {
        enter_stopping(context, &decision->actions);
        confirm_stopped(context, facts);
        return true;
    }
    return false;
}

/* STOPPING 与 FAULT 共用的归还契约：身份、效果和真实静默必须同时满足。 */
static bool confirm_cancelled_operation(MotorStateRulesContext *context,
                                        const MotorStateEvent *event,
                                        const MotorStateFacts *facts)
{
    if (MotorStateRules_Classify(event->maintenance_kind) != active_maintenance_class(context) ||
        (unsigned)event->effects > MOTOR_MAINTENANCE_EFFECT_COMMITTED ||
        !resources_quiescent(facts))
    {
        return false;
    }
    if (context->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
    {
        context->operation_result = MOTOR_MAINTENANCE_CANCELLED;
        context->operation_effects = event->effects;
    }
    context->maintenance_active = false;
    confirm_stopped(context, facts);
    return true;
}

/* 普通事件按状态分派；IDLE 的控制与维护准入分别处理，公共优先级已处理。 */
static bool handle_state_event(MotorStateRulesContext *context,
                               uint32_t event_type,
                               const MotorStateEvent *event,
                               const MotorStateFacts *facts,
                               uint32_t *actions)
{
    /* 待命 CLEAR 不执行重置；停止和故障共用同一归还确认。 */
    if (event_type == MOTOR_STATE_EVENT_CLEAR &&
        (context->state == MOTOR_STATE_INIT || context->state == MOTOR_STATE_IDLE))
    {
        return true;
    }
    if (event_type == MOTOR_STATE_EVENT_CANCEL_DONE &&
        (context->state == MOTOR_STATE_STOPPING || context->state == MOTOR_STATE_FAULT))
    {
        return confirm_cancelled_operation(context, event, facts);
    }
    switch (context->state)
    {
    case MOTOR_STATE_INIT:
        if (event_type != MOTOR_STATE_EVENT_SELF_TEST_DONE)
        {
            break;
        }
        if (!facts->self_test_ready || !facts->inputs_valid || !resources_quiescent(facts))
        {
            enter_fault(context, MOTOR_STATE_FAULT_GUARD, actions);
            return false;
        }
        context->state = MOTOR_STATE_IDLE;
        return true;

    case MOTOR_STATE_IDLE:
        if (event_type != MOTOR_STATE_EVENT_ENABLE &&
            event_type != MOTOR_STATE_EVENT_BEGIN_OPERATION)
        {
            break;
        }
        if (!resources_quiescent(facts) || !facts->inputs_valid || context->latched_faults)
        {
            return false;
        }
        if (event_type == MOTOR_STATE_EVENT_ENABLE)
        {
            if (!MotorControlMode_IsValid(event->control_mode) || !facts->start_permitted)
            {
                return false;
            }
        }
        else
        {
            if ((unsigned)event->maintenance_kind >= MOTOR_MAINTENANCE_NONE ||
                MotorStateRules_Classify(event->maintenance_kind) ==
                    MOTOR_MAINTENANCE_CLASS_INVALID ||
                !facts->operation_permitted)
            {
                return false;
            }
        }
        /* 同一代只受理一种工作；控制准备和维护子流程各自继续执行。 */
        if (advance_epoch(context, actions))
        {
            if (event_type == MOTOR_STATE_EVENT_ENABLE)
            {
                context->state = MOTOR_STATE_STARTING;
            }
            else
            {
                context->maintenance_kind = event->maintenance_kind;
                context->maintenance_active = true;
                context->operation_result = MOTOR_MAINTENANCE_NO_RESULT;
                context->operation_effects = MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
                context->state = MOTOR_STATE_MAINTENANCE;
            }
        }
        return true;

    case MOTOR_STATE_STARTING:
        if (event_type != MOTOR_STATE_EVENT_START_DONE)
        {
            break;
        }
        if (!facts->startup_confirmed || facts->phases_disabled)
        {
            enter_fault(context, MOTOR_STATE_FAULT_GUARD, actions);
            return false;
        }
        context->state = MOTOR_STATE_RUNNING;
        return true;

    case MOTOR_STATE_MAINTENANCE:
        if (event_type != MOTOR_STATE_EVENT_OPERATION_DONE)
        {
            break;
        }
        if (MotorStateRules_Classify(event->maintenance_kind) !=
                active_maintenance_class(context) ||
            (unsigned)event->effects > MOTOR_MAINTENANCE_EFFECT_COMMITTED ||
            (event->maintenance_kind == MOTOR_MAINTENANCE_SAVE &&
             event->effects != MOTOR_MAINTENANCE_EFFECT_COMMITTED))
        {
            return false;
        }
        context->operation_effects = event->effects;
        context->operation_result = MOTOR_MAINTENANCE_COMPLETED;
        enter_stopping(context, actions);
        confirm_stopped(context, facts);
        return true;

    case MOTOR_STATE_STOPPING:
        if (event_type == 0U)
        {
            confirm_stopped(context, facts);
        }
        break;

    case MOTOR_STATE_FAULT:
        if (event_type != MOTOR_STATE_EVENT_CLEAR)
        {
            break;
        }
        if (!resources_quiescent(facts) || !facts->inputs_valid || !facts->self_test_ready ||
            context->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
        {
            return false;
        }
        /* 先推进代际，耗尽转 FATAL 时保留全部锁存。 */
        if (!advance_epoch(context, actions))
        {
            return false;
        }
        context->state = MOTOR_STATE_IDLE;
        context->maintenance_active = false;
        context->latched_faults = 0U;
        *actions |= MOTOR_STATE_ACTION_DISABLE_POWER | MOTOR_STATE_ACTION_FAULT_CLEARED;
        return true;

    default:
        break;
    }

    /* 空事件不启动或清错；错误状态拒绝新请求。 */
    if (event_type == 0U)
    {
        return true;
    }
    return false;
}

/* 所有者每次只提交一个事件，异步完成保留创建时的代际。 */
static bool handle_normal_event(MotorStateRulesContext *context,
                                const MotorStateEvent *event,
                                const MotorStateFacts *facts,
                                uint32_t *actions)
{
    uint32_t event_type = event->events;
    if ((event_type & ~ALL_EVENTS) || (event_type && (event_type & (event_type - 1U))))
    {
        return false;
    }
    if (((event_type & COMPLETION_EVENTS) && event->completion_epoch != context->epoch) ||
        ((event_type & REQUEST_EVENTS) && !facts->request_fresh))
    {
        return false;
    }
    return handle_state_event(context, event_type, event, facts, actions);
}

/* 保持所有停机状态的关相要求；转换拒绝和提前返回也必须经过此约束。 */
static bool output_must_be_disabled(const MotorStateRulesContext *context)
{
    switch (context->state)
    {
    case MOTOR_STATE_INIT:
    case MOTOR_STATE_IDLE:
    case MOTOR_STATE_STOPPING:
    case MOTOR_STATE_FAULT:
    case MOTOR_STATE_FATAL:
        return true;
    case MOTOR_STATE_MAINTENANCE:
        return active_maintenance_class(context) != MOTOR_MAINTENANCE_CLASS_CALIBRATION;
    default:
        return false;
    }
}

void MotorStateRules_Init(MotorStateRulesContext *context)
{
    if (context)
    {
        *context = (MotorStateRulesContext){
            .state = MOTOR_STATE_INIT, .maintenance_kind = MOTOR_MAINTENANCE_NONE, .epoch = 2U};
    }
}

MotorStateDecision MotorStateRules_Step(MotorStateRulesContext *context,
                                        const MotorStateEvent *event,
                                        const MotorStateFacts *facts)
{
    MotorStateDecision decision = {.actions = MOTOR_STATE_ACTION_DISABLE_POWER, .accepted = false};
    if (!context)
    {
        return decision;
    }
    if (!event || !facts)
    {
        enter_fatal(context, &decision.actions);
        return decision;
    }
    decision.actions = 0U;
    decision.accepted = true;
    if (!handle_priority_events(context, event, facts, &decision))
    {
        decision.accepted = handle_normal_event(context, event, facts, &decision.actions);
    }
    if (output_must_be_disabled(context))
    {
        decision.actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
    }
    return decision;
}
