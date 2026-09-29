#include "motor_state_internal.h"

#include <stddef.h>

/* 停止/故障时统一撤销的请求与作业。 */
#define CANCEL_ACTIONS                                                                             \
    (MOTOR_STATE_ACTION_DISABLE_POWER | MOTOR_STATE_ACTION_REVOKE_REQUESTS |                       \
     MOTOR_STATE_ACTION_CANCEL_CONTROL | MOTOR_STATE_ACTION_CANCEL_OPERATION)
#define ALL_EVENTS UINT32_C(0x3fff)
/* 异步完成通知：必须匹配阶段 epoch。 */
#define COMPLETIONS                                                                                \
    (MOTOR_STATE_EVENT_BOOT_DONE | MOTOR_STATE_EVENT_SELF_TEST_DONE |                              \
     MOTOR_STATE_EVENT_START_DONE | MOTOR_STATE_EVENT_OPERATION_DONE |                             \
     MOTOR_STATE_EVENT_OPERATION_FAILED | MOTOR_STATE_EVENT_CANCEL_DONE)
/* 新请求：必须匹配 request_epoch 且 request_fresh。 */
#define REQUESTS                                                                                   \
    (MOTOR_STATE_EVENT_ENABLE | MOTOR_STATE_EVENT_CLEAR | MOTOR_STATE_EVENT_BEGIN_OPERATION)

/* 仅内部规则和事实收集共享；分类按具体任务推导，不形成可变镜像。 */
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
    default:
        return (unsigned)kind < MOTOR_MAINTENANCE_NONE ? MOTOR_MAINTENANCE_CLASS_CALIBRATION
                                                       : MOTOR_MAINTENANCE_CLASS_INVALID;
    }
}

/* 会话释放后仍保留最近身份用于既有定向清理，不作为当前操作分类。 */
static MotorMaintenanceClass active_maintenance_class(const MotorStateRulesContext *context)
{
    return context->maintenance_active ? MotorStateRules_Classify(context->maintenance_kind)
                                       : MOTOR_MAINTENANCE_CLASS_NONE;
}
/* 资源静默：相已关、控制空闲、操作空闲、维护资源已释放。 */
static bool quiet(const MotorStateFacts *g)
{
    return g->phases_disabled && g->control_idle && g->operation_idle && g->maintenance_released;
}
/* 健康：样本新鲜、故障源已恢复且无活动故障。 */
static bool healthy(const MotorStateFacts *g)
{
    return g->samples_fresh && g->fault_sources_clear && !g->active_faults;
}
/* 启动许可：健康 + 角色许可 + 无操作占用。 */
static bool start_allowed(const MotorStateFacts *g)
{
    return healthy(g) && g->start_permitted && g->operation_idle && g->maintenance_released;
}
/* 事件集必须恰好一位（完成/请求不允许同拍叠加）。 */
static bool one_bit(uint32_t value)
{
    return value && !(value & (value - 1U));
}
/* 锁存故障：首故障取最低位，活动故障按位累计。 */
static void latch(MotorStateRulesContext *a, uint32_t faults)
{
    if (!a->first_fault)
    {
        a->first_fault = faults & (0U - faults);
    }
    a->latched_faults |= faults;
}
/* API 误用/不可恢复：锁内部故障并终止，仅复位可退出。 */
static void fatal(MotorStateRulesContext *a, uint32_t *actions)
{
    latch(a, MOTOR_STATE_FAULT_INTERNAL);
    a->state = MOTOR_STATE_FATAL;

    *actions |= CANCEL_ACTIONS;
}
/* 推进代际：epoch 用尽锁 FATAL，不回绕。 */
static bool advance(MotorStateRulesContext *a, uint32_t *actions)
{
    if (a->epoch == UINT32_MAX)
    {
        fatal(a, actions);
        return false;
    }
    ++a->epoch;
    *actions |= MOTOR_STATE_ACTION_REVOKE_REQUESTS;
    return true;
}
/* 请求取消当前操作：效果标记未知——取消无法撤销外部已提交写入。 */
static void cancel_operation(MotorStateRulesContext *a)
{
    if (a->maintenance_active && a->operation_result == MOTOR_MAINTENANCE_NO_RESULT)
    {

        a->operation_result = MOTOR_MAINTENANCE_CANCEL_REQUESTED;
        a->operation_effects = MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
    }
}
/* 进入停止/故障态：取消操作、撤销作业、推进代际；代际用尽转 FATAL 时不再改状态。 */
static void enter_stopped_state(MotorStateRulesContext *a, MotorState state, uint32_t *actions)
{
    cancel_operation(a);
    *actions |= CANCEL_ACTIONS;
    if (!advance(a, actions))
    {
        return;
    }
    a->state = state;
}
/* 故障入口：立即关相、撤销作业；已处 FAULT 时只累计锁存。 */
static void fault(MotorStateRulesContext *a, uint32_t faults, uint32_t *actions)
{
    latch(a, faults);
    *actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
    if (a->state != MOTOR_STATE_FAULT)
    {
        enter_stopped_state(a, MOTOR_STATE_FAULT, actions);
    }
}
/* 进入自检：推进代际后请求关相与自检。 */
static void self_test(MotorStateRulesContext *a, uint32_t *actions)
{
    if (!advance(a, actions))
    {
        return;
    }
    a->tested = false;
    a->state = MOTOR_STATE_SELF_TEST;

    a->maintenance_active = false;
    *actions |= MOTOR_STATE_ACTION_DISABLE_POWER | MOTOR_STATE_ACTION_SELF_TEST;
}
/* 进入停止：立即关相；FAULT 态只补充取消作业。 */
static void stopping(MotorStateRulesContext *a, uint32_t *actions)
{
    *actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
    if (a->state == MOTOR_STATE_FAULT)
    {
        *actions |= MOTOR_STATE_ACTION_CANCEL_CONTROL | MOTOR_STATE_ACTION_CANCEL_OPERATION;
        return;
    }
    if (a->state != MOTOR_STATE_STOPPING)
    {
        enter_stopped_state(a, MOTOR_STATE_STOPPING, actions);
    }
}
/* 停止确认：资源静默且无待收取消时，按锁存/自检状态收口。 */
static void confirm_stopped(MotorStateRulesContext *a, const MotorStateFacts *g, uint32_t *actions)
{
    if (a->state != MOTOR_STATE_STOPPING || !quiet(g) ||
        a->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
    {
        return;
    }
    a->maintenance_active = false;
    if (a->latched_faults)
    {
        a->state = MOTOR_STATE_FAULT;
    }
    else if (a->tested)
    {
        a->state = MOTOR_STATE_READY;
    }
    else
    {
        /* 启动期间 STOP 不能绕过自检。 */
        self_test(a, actions);
    }
}
void MotorStateRules_Init(MotorStateRulesContext *app)
{
    if (app)
    {
        const MotorStateRulesContext initial = {
            .state = MOTOR_STATE_BOOT, .maintenance_kind = MOTOR_MAINTENANCE_NONE, .epoch = 1U};
        *app = initial;
    }
}
MotorStateDecision
MotorStateRules_Step(MotorStateRulesContext *a, const MotorStateEvent *e, const MotorStateFacts *g)
{
    MotorStateDecision r = {.actions = MOTOR_STATE_ACTION_DISABLE_POWER,
                            .rejection = MOTOR_STATE_REJECT_ARGUMENT};
    uint32_t events, detected, complete, request;

    if (!a)
    {
        return r;
    }
    if (!e || !g)
    {
        fatal(a, &r.actions);
        return r;
    }
    r.actions = 0;
    r.rejection = MOTOR_STATE_ACCEPTED;
    events = e->events;
    if (a->state == MOTOR_STATE_FATAL)
    {
        r.actions = CANCEL_ACTIONS;
        r.rejection = MOTOR_STATE_REJECT_FATAL;
        goto done;
    }
    if (events & MOTOR_STATE_EVENT_FATAL)
    {
        latch(a, e->faults);
        fatal(a, &r.actions);
        goto done;
    }
    detected = g->active_faults | ((events & MOTOR_STATE_EVENT_FAULT)
                                       ? (e->faults ? e->faults : MOTOR_STATE_FAULT_INTERNAL)
                                       : 0U);
    /* 有效失败是故障源而非可丢弃的完成；陈旧 worker 不能污染新操作；
     * 保留真实提交不确定性。 */
    if ((events & MOTOR_STATE_EVENT_OPERATION_FAILED) && a->state == MOTOR_STATE_MAINTENANCE &&
        e->completion_epoch == a->epoch &&
        MotorStateRules_Classify(e->maintenance_kind) == active_maintenance_class(a))
    {

        a->operation_result = MOTOR_MAINTENANCE_FAILED;
        a->operation_effects = (unsigned)e->effects <= MOTOR_MAINTENANCE_EFFECT_COMMITTED
                                   ? e->effects
                                   : MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
        detected |= e->faults ? e->faults : MOTOR_STATE_FAULT_OPERATION;
    }
    if (detected)
    {
        /* 已锁存FAULT仍允许单独的归还确认；不清故障，也不受理同拍新请求。 */
        bool releasing_fault =
            a->state == MOTOR_STATE_FAULT && events == MOTOR_STATE_EVENT_CANCEL_DONE;
        fault(a, detected, &r.actions);
        if (!releasing_fault)
        {
            r.rejection = MOTOR_STATE_REJECT_FAULT;
            goto done;
        }
    }
    if (((a->state == MOTOR_STATE_STARTING || a->state == MOTOR_STATE_RUNNING) &&
         !start_allowed(g)) ||
        (a->state == MOTOR_STATE_RUNNING && g->phases_disabled &&
         !(events & (MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_CANCEL))))
    {
        fault(a, MOTOR_STATE_FAULT_GUARD, &r.actions);
        r.rejection = MOTOR_STATE_REJECT_GUARD;
        goto done;
    }
    if (a->state == MOTOR_STATE_MAINTENANCE &&
        (!g->operation_permitted || !g->fault_sources_clear ||
         (active_maintenance_class(a) == MOTOR_MAINTENANCE_CLASS_CALIBRATION
              ? !g->samples_fresh
              : !g->phases_disabled)))
    {
        fault(a, MOTOR_STATE_FAULT_GUARD, &r.actions);
        r.rejection = MOTOR_STATE_REJECT_GUARD;
        goto done;
    }
    if (events & MOTOR_STATE_EVENT_STOP)
    {
        stopping(a, &r.actions);
        confirm_stopped(a, g, &r.actions);
        if (events & (COMPLETIONS | REQUESTS))
        {
            r.rejection = MOTOR_STATE_REJECT_INTERRUPTED;
        }
        goto done;
    }
    if (events & MOTOR_STATE_EVENT_CANCEL)
    {
        stopping(a, &r.actions);
        confirm_stopped(a, g, &r.actions);
        if (events & (COMPLETIONS | REQUESTS))
        {
            r.rejection = MOTOR_STATE_REJECT_INTERRUPTED;
        }
        goto done;
    }
    if (events & ~ALL_EVENTS)
    {
        r.rejection = MOTOR_STATE_REJECT_EVENT;
        goto done;
    }
    complete = events & COMPLETIONS;
    request = events & REQUESTS;
    if (complete && !one_bit(complete))
    {
        r.rejection = MOTOR_STATE_REJECT_EVENT;
        goto done;
    }
    if (complete)
    {
        if (e->completion_epoch != a->epoch)
        {
            r.rejection = MOTOR_STATE_REJECT_STALE_EPOCH;
            goto done;
        }
        switch (complete)
        {
        case MOTOR_STATE_EVENT_BOOT_DONE:
            if (a->state != MOTOR_STATE_BOOT)
            {
                r.rejection = MOTOR_STATE_REJECT_STATE;
            }
            else if (!quiet(g))
            {
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else
            {
                self_test(a, &r.actions);
            }
            break;
        case MOTOR_STATE_EVENT_SELF_TEST_DONE:
            if (a->state != MOTOR_STATE_SELF_TEST)
            {
                r.rejection = MOTOR_STATE_REJECT_STATE;
            }
            else if (!g->self_test_passed || !healthy(g) || !quiet(g))
            {
                fault(a, MOTOR_STATE_FAULT_GUARD, &r.actions);
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else
            {
                a->tested = true;
                a->state = MOTOR_STATE_READY;
            }
            break;
        case MOTOR_STATE_EVENT_START_DONE:
            if (a->state != MOTOR_STATE_STARTING)
            {
                r.rejection = MOTOR_STATE_REJECT_STATE;
            }
            else if (!g->startup_confirmed || g->phases_disabled)
            {
                fault(a, MOTOR_STATE_FAULT_GUARD, &r.actions);
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else
            {
                a->state = MOTOR_STATE_RUNNING;
            }
            break;
        case MOTOR_STATE_EVENT_OPERATION_DONE:
        case MOTOR_STATE_EVENT_OPERATION_FAILED:
            if (a->state != MOTOR_STATE_MAINTENANCE)
            {
                r.rejection = MOTOR_STATE_REJECT_STATE;
            }
            else if (MotorStateRules_Classify(e->maintenance_kind) != active_maintenance_class(a) ||
                     (unsigned)e->effects > MOTOR_MAINTENANCE_EFFECT_COMMITTED)
            {
                r.rejection = MOTOR_STATE_REJECT_OPERATION;
            }
            else if (complete == MOTOR_STATE_EVENT_OPERATION_DONE &&
                     e->maintenance_kind == MOTOR_MAINTENANCE_SAVE &&
                     e->effects != MOTOR_MAINTENANCE_EFFECT_COMMITTED)
            {
                r.rejection = MOTOR_STATE_REJECT_OPERATION;
            }
            else
            {

                a->operation_effects = e->effects;
                a->operation_result = complete == MOTOR_STATE_EVENT_OPERATION_DONE
                                          ? MOTOR_MAINTENANCE_COMPLETED
                                          : MOTOR_MAINTENANCE_FAILED;
                if (complete == MOTOR_STATE_EVENT_OPERATION_FAILED)
                {
                    fault(a, e->faults ? e->faults : MOTOR_STATE_FAULT_OPERATION, &r.actions);
                }
                else
                {
                    stopping(a, &r.actions);
                    confirm_stopped(a, g, &r.actions);
                }
            }
            break;
        case MOTOR_STATE_EVENT_CANCEL_DONE:
            if (a->state != MOTOR_STATE_STOPPING && a->state != MOTOR_STATE_FAULT)
            {
                r.rejection = MOTOR_STATE_REJECT_STATE;
            }
            else if (MotorStateRules_Classify(e->maintenance_kind) != active_maintenance_class(a) ||
                     (unsigned)e->effects > MOTOR_MAINTENANCE_EFFECT_COMMITTED)
            {
                r.rejection = MOTOR_STATE_REJECT_OPERATION;
            }
            else if (!quiet(g))
            {
                r.rejection = MOTOR_STATE_REJECT_BUSY;
            }
            else
            {
                if (a->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
                {
                    a->operation_result = MOTOR_MAINTENANCE_CANCELLED;
                    a->operation_effects = e->effects;
                }
                a->maintenance_active = false;
                confirm_stopped(a, g, &r.actions);
            }
            break;
        default:
            break;
        }
        /* 完成通知绝不把同拍新请求串成一次新启动。 */
        if (request && r.rejection == MOTOR_STATE_ACCEPTED)
        {
            r.rejection = MOTOR_STATE_REJECT_INTERRUPTED;
        }
        goto done;
    }
    if (request && !one_bit(request))
    {
        r.rejection = MOTOR_STATE_REJECT_EVENT;
        goto done;
    }
    if (request && e->request_epoch != a->epoch)
    {
        r.rejection = MOTOR_STATE_REJECT_STALE_EPOCH;
        goto done;
    }
    if (request && !g->request_fresh)
    {
        r.rejection = MOTOR_STATE_REJECT_GUARD;
        goto done;
    }
    if (request == MOTOR_STATE_EVENT_CLEAR)
    {
        if (a->state == MOTOR_STATE_FAULT)
        {
            if (!quiet(g) || !healthy(g) ||
                a->operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
            {
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else
            {
                /* 先推进代际：generation 用尽时必须保留全部故障。 */
                self_test(a, &r.actions);
                if (a->state != MOTOR_STATE_FATAL)
                {
                    a->first_fault = a->latched_faults = 0U;
                    r.actions |= MOTOR_STATE_ACTION_FAULT_CLEARED;
                }
            }
        }
        else if (a->state != MOTOR_STATE_READY && a->state != MOTOR_STATE_SELF_TEST)
        {
            r.rejection = MOTOR_STATE_REJECT_STATE;
        }
    }
    else if (request)
    {
        if (a->state != MOTOR_STATE_READY)
        {
            r.rejection = a->latched_faults ? MOTOR_STATE_REJECT_FAULT : MOTOR_STATE_REJECT_STATE;
        }
        else if (!quiet(g) || !healthy(g) || a->latched_faults)
        {
            r.rejection = MOTOR_STATE_REJECT_GUARD;
        }
        else if (request == MOTOR_STATE_EVENT_ENABLE)
        {
            if ((unsigned)e->control_mode > MOTOR_CONTROL_VQ)
            {
                r.rejection = MOTOR_STATE_REJECT_MODE;
            }
            else if (!g->start_permitted)
            {
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else if (advance(a, &r.actions))
            {

                a->state = MOTOR_STATE_STARTING;
                r.actions |= MOTOR_STATE_ACTION_START_CONTROL;
            }
        }
        else
        {
            if ((unsigned)e->maintenance_kind >= MOTOR_MAINTENANCE_NONE)
            {
                r.rejection = MOTOR_STATE_REJECT_OPERATION;
            }
            else if (!g->operation_permitted)
            {
                r.rejection = MOTOR_STATE_REJECT_GUARD;
            }
            else if (advance(a, &r.actions))
            {
                a->maintenance_kind = e->maintenance_kind;
                a->maintenance_active = true;

                a->operation_result = MOTOR_MAINTENANCE_NO_RESULT;
                a->operation_effects = MOTOR_MAINTENANCE_EFFECT_UNKNOWN;
                a->state = MOTOR_STATE_MAINTENANCE;
                r.actions |= MOTOR_STATE_ACTION_BEGIN_OPERATION;
            }
        }
    }
    else
    {
        confirm_stopped(a, g, &r.actions);
    }
done:
    if (a->state == MOTOR_STATE_BOOT || a->state == MOTOR_STATE_SELF_TEST ||
        a->state == MOTOR_STATE_READY || a->state == MOTOR_STATE_STOPPING ||
        a->state == MOTOR_STATE_FAULT || a->state == MOTOR_STATE_FATAL ||
        (a->state == MOTOR_STATE_MAINTENANCE &&
         active_maintenance_class(a) != MOTOR_MAINTENANCE_CLASS_CALIBRATION))
    {
        r.actions |= MOTOR_STATE_ACTION_DISABLE_POWER;
    }
    return r;
}
