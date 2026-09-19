#include "foc_run_state.h"

#include "app_lifecycle.h"
#include "common_inc.h"
#include "motor_state.h"

/* 运行状态机适配器：
 * - READY/STARTING/RUNNING/STOPPING/FAULT 由 AppLifecycle 纯核心拥有；
 * - 适配器负责：worker 结果与外部模式写入 → 事件合成、真实 guards 供给、
 *   动作执行（START_CONTROL/DISABLE_POWER）与兼容投影；
 * - 维护会话（阶段 C）：Save/Default/Zero 进入 MAINTENANCE，标定会话随
 *   foc_calibration 拆解后接入；
 * - 兼容期：未迁移的 worker 仍直接写 ModeNow，本层按旧差分语义转为事件。 */

static AppLifecycle lifecycle;
static bool power_on; /* 已执行 START_CONTROL 且未执行 DISABLE_POWER */
/* 已完成会话的模式值：同值持续期间不重复开启维护会话。 */
static ModeNow_TypeDef operation_mode;
/* 前台保存结果：0 无、1 成功、2 失败；由快速环消费一次。 */
enum
{
    SAVE_FINISH_NONE = 0,
    SAVE_FINISH_COMMITTED,
    SAVE_FINISH_FAILED
};
static volatile uint8_t save_finish;

/* 需要编码器反馈的模式在坏帧超限后立即置编码器故障。 */
static bool Encoder_FeedbackRequired(const MotorControl_TypeDef *MotorControl)
{
    if (MotorControl->ModeNow == Current_Mode)
    {
        return !MotorControl->isUseSensorless;
    }

    return MotorControl->ModeNow == Speed_Mode || MotorControl->ModeNow == Position_Mode ||
           MotorControl->ModeNow == Position_Impedance_Mode || MotorControl->ModeNow == Vq_Mode ||
           MotorControl->ModeNow == Calib_EncoderOffset ||
           MotorControl->ModeNow == Calib_EncoderObserver ||
           MotorControl->ModeNow == Calib_EleAngelOffset ||
           MotorControl->ModeNow == Calib_Friction || MotorControl->ModeNow == Calib_Anticogging ||
           MotorControl->ModeNow == Set_ZeroPosition;
}

/* 命令类模式不进入自动使能列表：由外部流程或后续周期处理。 */
static bool RunState_ModeIsAutoStartable(ModeNow_TypeDef mode)
{
    return mode != Save_Param && mode != Default_Param && mode != Clear_Error &&
           mode != Set_ZeroPosition && mode != Calib_Anticogging;
}

/* 位置/速度模式首个周期先在功率输出关闭时校验控制器，下一快周期再使能。 */
static bool RunState_ModeNeedsPreparation(ModeNow_TypeDef mode)
{
    return mode == Position_Mode || mode == Speed_Mode;
}

/* 控制模式投影：CAN 模式编号不变，仅用于核心内部语义。 */
static AppControlMode RunState_ProjectControlMode(ModeNow_TypeDef mode)
{
    switch (mode)
    {
    case Current_Mode:
        return APP_CONTROL_CURRENT;
    case Speed_Mode:
        return APP_CONTROL_SPEED;
    case Position_Mode:
        return APP_CONTROL_POSITION;
    case Position_Impedance_Mode:
        return APP_CONTROL_IMPEDANCE;
    case Sensorless_Speed_Mode:
        return APP_CONTROL_SENSORLESS;
    case Voltage_OpenLoop:
        return APP_CONTROL_OPEN_VOLTAGE;
    case Vq_Mode:
        return APP_CONTROL_VQ;
    default:
        return APP_CONTROL_NONE;
    }
}

/* guards 由现有信号构造：故障即时锁存，启动许可沿用轴配置校验。 */
static AppLifecycleGuards RunState_BuildGuards(void)
{
    AppLifecycleGuards guards;

    guards.active_faults = (uint32_t)MotorControl.ErrorNow;
    guards.phases_disabled = !power_on;
    guards.control_idle = true;
    guards.operation_idle = true;
    guards.maintenance_released = true;
    guards.samples_fresh = true;
    guards.fault_sources_clear = MotorControl.ErrorNow == No_Error;
    guards.self_test_passed = true;
    guards.start_permitted = MotorControl.axis_profile_valid;
    guards.operation_permitted = true;
    /* 位置/速度模式需要外环控制器就绪后才确认启动，其余模式可同拍确认。 */
    guards.startup_confirmed =
        !RunState_ModeNeedsPreparation(MotorControl.ModeNow) || MotorOuterLoop_IsReady();
    guards.request_fresh = true;
    return guards;
}

/* 执行核心动作位；apply_power_actions 由调用点按旧停机条件授权：
 * 旧实现只在“曾运行（ModeLast != Disable）”的停止路径执行清理与关相。 */
static void RunState_ApplyActions(uint32_t actions, bool apply_power_actions)
{
    if ((actions & APP_ACTION_DISABLE_POWER) && apply_power_actions)
    {
        Clear_RunningData();
        Stop_PWM_Generate();
        power_on = false;
    }
    if (actions & APP_ACTION_START_CONTROL)
    {
        Start_PWM_Generate();
        power_on = true;
    }
}

/* 发送一次核心事件并执行动作；defer_start_action 用于挂起位置/速度的启相。 */
static bool RunState_Send(uint32_t events,
                          uint32_t faults,
                          AppControlMode control_mode,
                          bool defer_start_action,
                          bool apply_power_actions)
{
    AppLifecycleEvent event;
    AppLifecycleGuards guards;
    AppLifecycleResult result;

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.faults = faults;
    event.control_mode = control_mode;
    event.completion_epoch = lifecycle.snapshot.epoch;
    event.request_epoch = lifecycle.snapshot.epoch;
    guards = RunState_BuildGuards();
    result = AppLifecycle_Step(&lifecycle, &event, &guards);
    if (defer_start_action)
    {
        result.actions &= ~(uint32_t)APP_ACTION_START_CONTROL;
    }
    RunState_ApplyActions(result.actions, apply_power_actions);
    return result.rejection == APP_ACCEPTED;
}

/* 启动链：BOOT → SELF_TEST → READY（无对外动作）。 */
static void RunState_RunBootChain(void)
{
    if (lifecycle.snapshot.state == APP_BOOT)
    {
        (void)RunState_Send(APP_EVENT_BOOT_DONE, 0U, APP_CONTROL_NONE, false, false);
    }
    if (lifecycle.snapshot.state == APP_SELF_TEST)
    {
        (void)RunState_Send(APP_EVENT_SELF_TEST_DONE, 0U, APP_CONTROL_NONE, false, false);
    }
}

/* 停止请求：关相并尝试确认；确认可能延后一拍（quiet 需要功率级真实关闭）。 */
static void RunState_RequestStop(bool apply_power_actions)
{
    (void)RunState_Send(APP_EVENT_STOP, 0U, APP_CONTROL_NONE, false, apply_power_actions);
    (void)RunState_Send(0U, 0U, APP_CONTROL_NONE, false, false);
}

/* 位置/速度的就绪确认：先执行挂起启相，再向核心确认（确认要求相已开）。 */
static void RunState_ConfirmStart(void)
{
    if (!power_on)
    {
        Start_PWM_Generate();
        power_on = true;
    }
    (void)RunState_Send(APP_EVENT_START_DONE, 0U, APP_CONTROL_NONE, false, false);
}

/* 维护目标模式 → 核心操作；标定会话随 foc_calibration 拆解后接入。 */
static AppOperation RunState_OperationFor(ModeNow_TypeDef mode)
{
    switch (mode)
    {
    case Save_Param:
        return APP_OPERATION_SAVE;
    case Default_Param:
        return APP_OPERATION_DEFAULTS;
    case Set_ZeroPosition:
        return APP_OPERATION_ZERO;
    default:
        return APP_OPERATION_NONE;
    }
}

/* 发送维护会话事件（BEGIN/DONE/FAILED/CANCEL_DONE）；动作只作请求，不写硬件。 */
static bool RunState_SendOperation(uint32_t events,
                                   AppOperation operation,
                                   AppOperationEffect effects,
                                   uint32_t faults)
{
    AppLifecycleEvent event;
    AppLifecycleGuards guards;
    AppLifecycleResult result;

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.faults = faults;
    event.operation = operation;
    event.effects = effects;
    event.control_mode = APP_CONTROL_NONE;
    event.completion_epoch = lifecycle.snapshot.epoch;
    event.request_epoch = lifecycle.snapshot.epoch;
    guards = RunState_BuildGuards();
    result = AppLifecycle_Step(&lifecycle, &event, &guards);
    RunState_ApplyActions(result.actions, false);
    return result.rejection == APP_ACCEPTED;
}

/* 会话服务：SAVE 结果上报、ZERO/DEFAULTS 派生完成、被打断会话的取消确认。
 * 陈旧完成不污染新会话；被故障/停止打断的会话必须确认取消，否则 CLEAR 被永久阻塞。
 * 返回 true 表示本拍已上报会话事件（完成/失败/取消确认），调用方推迟一拍再开新会话。 */
static bool RunState_ServiceOperations(ModeNow_TypeDef target)
{
    AppLifecycleState state = lifecycle.snapshot.state;
    bool sent = false;

    if (state != APP_MAINTENANCE || lifecycle.snapshot.operation != APP_OPERATION_SAVE)
    {
        save_finish = (uint8_t)SAVE_FINISH_NONE;
    }

    if (state == APP_MAINTENANCE)
    {
        if (lifecycle.snapshot.operation == APP_OPERATION_SAVE)
        {
            if (save_finish == (uint8_t)SAVE_FINISH_COMMITTED)
            {
                save_finish = (uint8_t)SAVE_FINISH_NONE;
                sent = true;
                (void)RunState_SendOperation(
                    APP_EVENT_OPERATION_DONE, APP_OPERATION_SAVE, APP_EFFECT_COMMITTED, 0U);
            }
            else if (save_finish == (uint8_t)SAVE_FINISH_FAILED)
            {
                save_finish = (uint8_t)SAVE_FINISH_NONE;
                sent = true;
                (void)RunState_SendOperation(APP_EVENT_OPERATION_FAILED,
                                             APP_OPERATION_SAVE,
                                             APP_EFFECT_UNKNOWN,
                                             (uint32_t)MotorParam_Error);
            }
        }
        else if (lifecycle.snapshot.operation == APP_OPERATION_ZERO && target == Save_Param)
        {
            /* 零位写入成功由 worker 切往 Save_Param 表达：先完成 ZERO 会话。 */
            sent = true;
            (void)RunState_SendOperation(
                APP_EVENT_OPERATION_DONE, APP_OPERATION_ZERO, APP_EFFECT_UNCOMMITTED, 0U);
        }
        else if (lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS && target == Default_Param)
        {
            /* Default_Param 由调度层每拍执行：进入会话后的下一拍即可确认完成。 */
            sent = true;
            (void)RunState_SendOperation(
                APP_EVENT_OPERATION_DONE, APP_OPERATION_DEFAULTS, APP_EFFECT_UNCOMMITTED, 0U);
        }
    }
    else if ((state == APP_STOPPING || state == APP_FAULT) &&
             lifecycle.snapshot.operation_result == APP_OPERATION_CANCEL_REQUESTED)
    {
        sent = true;
        (void)RunState_SendOperation(
            APP_EVENT_CANCEL_DONE, lifecycle.snapshot.operation, APP_EFFECT_UNKNOWN, 0U);
    }
    return sent;
}

/* 维护目标在 READY 下开启会话；同值持续不重复开启，非操作目标允许下次重新开启。 */
static void RunState_BeginOperationIfRequested(ModeNow_TypeDef target)
{
    AppOperation operation = RunState_OperationFor(target);

    if (lifecycle.snapshot.operation != APP_OPERATION_NONE)
    {
        return;
    }
    if (operation == APP_OPERATION_NONE)
    {
        operation_mode = Motor_Disable;
        return;
    }
    if (target == operation_mode)
    {
        return;
    }
    if (lifecycle.snapshot.state == APP_READY &&
        RunState_SendOperation(APP_EVENT_BEGIN_OPERATION, operation, APP_EFFECT_UNCOMMITTED, 0U))
    {
        operation_mode = target;
    }
}

void FocRunState_Init(void)
{
    AppLifecycle_Init(&lifecycle);
    power_on = false;
    operation_mode = Motor_Disable;
    save_finish = (uint8_t)SAVE_FINISH_NONE;
}

void FocRunState_SaveFinished(bool committed)
{
    save_finish = committed ? (uint8_t)SAVE_FINISH_COMMITTED : (uint8_t)SAVE_FINISH_FAILED;
}

void FocRunState_CheckFastFaults(void)
{
    if (Encoder_FeedbackRequired(&MotorControl) &&
        OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
    {
        Set_ErrorNow(Encoder_Error);
    }

    /* 同时覆盖内部模式赋值，不只通信请求。 */
    if (!MotorControl.axis_profile_valid && MotorControl.ModeNow != Motor_Disable &&
        MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Clear_Error)
    {
        Set_ErrorNow(MotorParam_Error);
        Set_ModeNow(Motor_Disable);
    }
}

void FocRunState_Tick(MotorWorkOutcome_TypeDef outcome)
{
    ModeNow_TypeDef target;
    bool needs_preparation;
    bool defer_start;
    bool operation_event;

    /* 1. worker 结果 → 目标模式与前置动作（与旧实现同序）。 */
    target = MotorControl.ModeNow;
    switch (outcome.result)
    {
    case MOTOR_WORK_SWITCH_MODE:
        if (outcome.power_off)
        {
            Stop_PWM_Generate();
            power_on = false;
        }
        MotorControl.ModeNow = outcome.next_mode;
        target = outcome.next_mode;
        break;
    case MOTOR_WORK_STOP:
        MotorControl.ModeNow = Motor_Disable;
        target = Motor_Disable;
        break;
    case MOTOR_WORK_FAULT:
        Set_ErrorNow(outcome.error);
        break;
    default:
        break;
    }

    /* 2. 故障策略与指示：Save/Default 期间容错保留当前模式。 */
    if (MotorControl.ErrorNow != No_Error)
    {
        if (target != Save_Param && target != Default_Param)
        {
            MotorControl.ModeNow = Motor_Disable;
            target = Motor_Disable;
        }
        LED_SetState(1, (uint8_t)MotorControl.ErrorNow);
    }
    else
    {
        LED_SetState(0, (uint8_t)MotorControl.ModeNow);
    }

    /* 3. 启动链、维护会话与故障/清除处理。 */
    RunState_RunBootChain();
    operation_event = RunState_ServiceOperations(target);
    if (MotorControl.ErrorNow != No_Error)
    {
        /* 故障本身不动功率级；停机清理统一由下方停止路径按旧条件执行一次。 */
        (void)RunState_Send(
            APP_EVENT_FAULT, (uint32_t)MotorControl.ErrorNow, APP_CONTROL_NONE, false, false);
    }
    else if (lifecycle.snapshot.state == APP_FAULT)
    {
        if (RunState_Send(APP_EVENT_CLEAR, 0U, APP_CONTROL_NONE, false, false))
        {
            (void)RunState_Send(APP_EVENT_SELF_TEST_DONE, 0U, APP_CONTROL_NONE, false, false);
        }
    }
    else if (lifecycle.snapshot.state == APP_STOPPING)
    {
        (void)RunState_Send(0U, 0U, APP_CONTROL_NONE, false, false);
    }

    /* 4. 目标迁移：停止、使能或退出运行态。 */
    needs_preparation = RunState_ModeNeedsPreparation(target);
    if (target == Motor_Disable)
    {
        if (ModeLast != Motor_Disable || lifecycle.snapshot.state == APP_STARTING)
        {
            RunState_RequestStop(ModeLast != Motor_Disable);
        }
    }
    else if (RunState_ModeIsAutoStartable(target))
    {
        if (ModeLast == Motor_Disable && MotorControl.axis_profile_valid)
        {
            if (lifecycle.snapshot.state == APP_STARTING)
            {
                if (RunState_BuildGuards().startup_confirmed)
                {
                    RunState_ConfirmStart();
                }
            }
            else if (lifecycle.snapshot.state == APP_READY)
            {
                if (RunState_Send(APP_EVENT_ENABLE,
                                  0U,
                                  RunState_ProjectControlMode(target),
                                  needs_preparation,
                                  false))
                {
                    if (!needs_preparation)
                    {
                        RunState_ConfirmStart();
                    }
                }
            }
        }
        else if (lifecycle.snapshot.state == APP_STARTING)
        {
            RunState_RequestStop(false);
        }
    }
    else if (lifecycle.snapshot.state == APP_STARTING || lifecycle.snapshot.state == APP_RUNNING)
    {
        /* 命令/标定类目标：退出运行态；操作本身仍走旧投影（阶段 C 迁移）。 */
        RunState_RequestStop(false);
    }
    if (!operation_event)
    {
        RunState_BeginOperationIfRequested(target);
    }

    /* 5. 兼容提交：启动等待（STARTING）等同旧的 defer 语义。 */
    defer_start = lifecycle.snapshot.state == APP_STARTING;
    Detect_Mode_Error_Change();

    if (!defer_start)
    {
        ModeLast = MotorControl.ModeNow;
    }
    ErrorLast = MotorControl.ErrorNow;

    MotorControl.ModeNow_f = MotorControl.ModeNow;
    MotorControl.ErrorNow_f = MotorControl.ErrorNow;
}
