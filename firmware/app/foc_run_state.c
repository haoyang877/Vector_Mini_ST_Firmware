#include "foc_run_state.h"

#include <math.h>
#include <string.h>

#include "angle_feedback.h"
#include "app_lifecycle.h"
#include "bus_voltage_profile.h"
#include "critical_hw.h"
#include "foc_bench_speed.h"
#include "foc_errhandle.h"
#include "foc_algorithm.h"
#include "foc_run.h"
#include "foc_sensing.h"
#include "indicator_hw.h"
#include "mcu_temperature.h"
#include "motor_sensing.h"
#include "motor_state.h"
#include "power_stage_hw.h"
#include "utils.h"

/* 阶段 D 恢复矩阵参数：过流回落驻留 100 ms（20 kHz 快环 2000 拍），高温恢复滞回 80 °C。 */
#define RUN_STATE_OC_RECOVER_TICKS 2000U
#define RUN_STATE_TEMP_RECOVER_C 80.0f
/* BENCH_SPEED_CONTEXT_BEGIN */
#define BENCH_SPEED_MAX_DURATION_MS 5000U
#define BENCH_SPEED_MAX_PHASE_A 6.0f
#define BENCH_SPEED_MAX_RAD_S 31.4159265f
#define BENCH_SPEED_FAST_TICKS_PER_MS (FOC_FREQ / 1000U)

/* 显式请求区可由 SWD 写入；其余状态仍由运行所有者私有持有。 */
volatile FocBenchSpeedMailbox FocBenchSpeed;
static struct
{
    volatile bool active, stop_requested;
    volatile uint32_t remaining_fast_ticks, stop_reason;
    uint32_t session_seq, last_request_seq, applied_seq;
    volatile uint32_t blocked_seq;
    float phase_limit_a, speed_limit_rad_s;
    float saved_current_limit_a, saved_speed_limit_rad_s;
} bench_speed;
/* 与目标邮箱同所有者；0 表示 CAN 提交，非零表示 Watch 请求关联序号。 */
static uint32_t pending_control_watch_seq;
/* BENCH_SPEED_CONTEXT_END */

/* 运行状态机适配器：
 * - READY/STARTING/RUNNING/STOPPING/FAULT 由 AppLifecycle 纯核心拥有；
 * - 适配器负责：worker 结果与外部模式写入 → 事件合成、真实 guards 供给、
 *   动作执行（START_CONTROL/DISABLE_POWER）与兼容投影；
 * - 维护会话（阶段 C）：Save/Default/Zero 与齿槽/相电阻标定进入 MAINTENANCE；
 * - 兼容期：未迁移的 worker 仍直接写 ModeNow，本层按旧差分语义转为事件。 */

static AppLifecycle lifecycle;
/* 已完成会话的模式值：同值持续期间不重复开启维护会话。 */
static ModeNow_TypeDef operation_mode;
/* 前台保存结果：0 无、1 成功、2 失败；由 2 kHz 监督消费一次。 */
enum
{
    SAVE_FINISH_NONE = 0,
    SAVE_FINISH_COMMITTED,
    SAVE_FINISH_FAILED
};
static volatile uint8_t save_finish;
/* Over_Current 回落驻留计数；饱和于 RUN_STATE_OC_RECOVER_TICKS。 */
static uint16_t overcurrent_recover_ticks;
/* 快环结果邮箱：20 kHz 写入、2 kHz 消费；只保留最新的非 RUNNING 结果。 */
static MotorWorkOutcome_TypeDef pending_outcome;
static volatile bool outcome_latched;
/* 前台写入、监督周期消费的单槽目标邮箱；STOP 与故障优先于目标。 */
static MotorControlCommand pending_control;
static volatile bool control_pending;
/* 唯一生产调用方位于 ADC2 JEOS 快环末尾，计数表示本次感测处理已经完成。 */
static volatile uint32_t fast_sample_epoch;
static uint32_t observed_fast_sample_epoch;
static bool fast_samples_fresh;
static bool RunState_VbusInEnableWindow(void);

/* STOP/FAULT 撤销已提交或正在写入的台架请求；不因迟到提交重新启动。 */
static void RunState_BenchCancel(uint32_t reason)
{
    uint32_t seq = FocBenchSpeed.request_seq;
    uint32_t committed_seq = (seq & 1U) != 0U && seq != UINT32_MAX ? seq + 1U : seq;
    if (committed_seq > bench_speed.blocked_seq)
    {
        bench_speed.blocked_seq = committed_seq;
    }
    if (bench_speed.active)
    {
        bench_speed.stop_requested = true;
        if (bench_speed.stop_reason == FOC_BENCH_SPEED_NO_STOP ||
            (reason == FOC_BENCH_SPEED_FAULT &&
             bench_speed.stop_reason != FOC_BENCH_SPEED_PHASE_CURRENT &&
             bench_speed.stop_reason != FOC_BENCH_SPEED_OVERSPEED))
        {
            bench_speed.stop_reason = reason;
        }
    }
}

static MotorControlResult validate_control(const MotorControlCommand *command)
{
    ModeNow_TypeDef mode;
    if (command == NULL || !isfinite(command->position_rad) || !isfinite(command->speed_rad_s) ||
        !isfinite(command->iq_a) || !isfinite(command->vq_v))
    {
        return MOTOR_CONTROL_INVALID;
    }
    switch (command->mode)
    {
    case MOTOR_CONTROL_CURRENT:
        mode = Current_Mode;
        break;
    case MOTOR_CONTROL_SPEED:
        mode = Speed_Mode;
        break;
    case MOTOR_CONTROL_POSITION:
        mode = Position_Mode;
        break;
    case MOTOR_CONTROL_VQ:
        mode = Vq_Mode;
        break;
    default:
        return MOTOR_CONTROL_INVALID;
    }
    if ((command->mode != MOTOR_CONTROL_POSITION && command->position_rad != 0.0f) ||
        (command->mode != MOTOR_CONTROL_SPEED && command->speed_rad_s != 0.0f) ||
        (command->mode != MOTOR_CONTROL_CURRENT && command->iq_a != 0.0f) ||
        (command->mode != MOTOR_CONTROL_VQ && command->vq_v != 0.0f))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (MotorControl.ErrorNow != No_Error)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (power_stage_hw_phases_disabled() || lifecycle.snapshot.state != APP_RUNNING ||
        MotorControl.ModeNow != mode || !MotorControl.axis_profile_valid ||
        MotorControl.isUseSensorless || !Encoder_IsOnline(&OnBoard_Encoder) ||
        (Encoder_GetCalibFlag(&OnBoard_Encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        !RunState_VbusInEnableWindow() || !isfinite(FOC.Vbus) ||
        FOC.Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        return MOTOR_CONTROL_DENIED;
    }
    switch (command->mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return isfinite(MotorControl.current_limit) && MotorControl.current_limit > 0.0f &&
                       fabsf(command->iq_a) <= MotorControl.current_limit
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_SPEED:
        return isfinite(MotorControl.speed_limit) && MotorControl.speed_limit > 0.0f &&
                       fabsf(command->speed_rad_s) <= MotorControl.speed_limit &&
                       (MotorControl.axis_profile.magic == 0U ||
                        (isfinite(MotorControl.axis_profile.maximum_speed_rad_s) &&
                         MotorControl.axis_profile.maximum_speed_rad_s > 0.0f &&
                         fabsf(command->speed_rad_s) <=
                             MotorControl.axis_profile.maximum_speed_rad_s))
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_POSITION:
        return MotorAxisProfile_AllowsPosition(&MotorControl.axis_profile,
                                               MotorControl.axis_profile_valid,
                                               Encoder_GetMecPos(&OnBoard_Encoder),
                                               command->position_rad)
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_VQ:
        return fabsf(command->vq_v) <= FOC_MaximumVoltage(FOC.Vbus_filt)
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    default:
        return MOTOR_CONTROL_INVALID;
    }
}

static void apply_pending_control(void)
{
    /* 防止快环在状态复核与目标写入之间锁存故障；临界区只做有界值检查和一次赋值。 */
    uint32_t state = critical_hw_enter();
    if (!control_pending)
    {
        critical_hw_exit(state);
        return;
    }
    control_pending = false;
    if (validate_control(&pending_control) != MOTOR_CONTROL_ACCEPTED)
    {
        critical_hw_exit(state);
        return;
    }
    switch (pending_control.mode)
    {
    case MOTOR_CONTROL_CURRENT:
        MotorControl.iqRef = pending_control.iq_a;
        break;
    case MOTOR_CONTROL_SPEED:
        MotorControl.speedRef = pending_control.speed_rad_s;
        break;
    case MOTOR_CONTROL_POSITION:
        MotorControl.posRef = pending_control.position_rad;
        break;
    case MOTOR_CONTROL_VQ:
        MotorControl.vqRef = pending_control.vq_v;
        break;
    default:
        break;
    }
    if (pending_control_watch_seq != 0U)
    {
        bench_speed.applied_seq = pending_control_watch_seq;
    }
    critical_hw_exit(state);
}

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
           mode != Set_ZeroPosition && mode != Calib_Anticogging && mode != Calib_Friction &&
           mode != Calib_EncoderObserver && mode != Calib_EleAngelOffset;
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

/* 同一监督拍共用采样进展，不能让多次核心 Step 把同一帧误判为陈旧。 */
static void RunState_UpdateSampleProgress(void)
{
    uint32_t state = critical_hw_enter();
    uint32_t current_epoch = fast_sample_epoch;
    critical_hw_exit(state);
    fast_samples_fresh = current_epoch != observed_fast_sample_epoch;
    observed_fast_sample_epoch = current_epoch;
}

/* 基础健康不要求编码器标定；各模式的标定准入仍由既有模式校验拥有。 */
static AppLifecycleGuards RunState_BuildGuards(void)
{
    AppLifecycleGuards guards;
    uint32_t state = critical_hw_enter();
    ModeNow_TypeDef mode = MotorControl.ModeNow;

    guards.active_faults = (uint32_t)MotorControl.ErrorNow;
    guards.phases_disabled = power_stage_hw_phases_disabled();
    guards.control_idle = !control_pending && !outcome_latched;
    /* 维护释放尚无统一查询：本批保留兼容值，不能将其当作已完成释放的证据。 */
    guards.operation_idle = true;
    guards.maintenance_released = true;
    guards.samples_fresh =
        fast_samples_fresh && isfinite(FOC.Vbus) && isfinite(FOC.Vbus_filt) && isfinite(FOC.Ia) &&
        isfinite(FOC.Ib) && isfinite(FOC.Ic) && McuTemperature.valid != 0U &&
        McuTemperature.missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS && isfinite(FOC.temp);
    guards.fault_sources_clear = MotorControl.ErrorNow == No_Error;
    guards.self_test_passed =
        MotorControl.axis_profile_valid && guards.samples_fresh && guards.fault_sources_clear;
    guards.start_permitted = MotorControl.axis_profile_valid;
    if (lifecycle.snapshot.state != APP_RUNNING)
    {
        guards.start_permitted = guards.start_permitted && RunState_VbusInEnableWindow() &&
                                 isfinite(FOC.Vbus) && FOC.Vbus < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
    }
    if (Encoder_FeedbackRequired(&MotorControl))
    {
        guards.start_permitted = guards.start_permitted && Encoder_IsOnline(&OnBoard_Encoder);
    }
    if (mode == Speed_Mode || mode == Position_Mode || mode == Position_Impedance_Mode ||
        mode == Vq_Mode || mode == Calib_Anticogging || mode == Calib_Friction ||
        (mode == Current_Mode && !MotorControl.isUseSensorless))
    {
        guards.start_permitted = guards.start_permitted && (Encoder_GetCalibFlag(&OnBoard_Encoder) &
                                                            ENC_CALIB_ALL) == ENC_CALIB_ALL;
    }
    guards.operation_permitted = true;
    /* 位置/速度模式需要外环控制器就绪后才确认启动，其余模式可同拍确认。 */
    guards.startup_confirmed =
        !RunState_ModeNeedsPreparation(MotorControl.ModeNow) || MotorOuterLoop_IsReady();
    guards.request_fresh = true;
    critical_hw_exit(state);
    return guards;
}

/* 功率事实来自平台回读；取消控制数据与数字关相请求分别处理。 */
static void RunState_ApplyActions(uint32_t actions, bool clear_control)
{
    if (actions & APP_ACTION_REVOKE_REQUESTS)
    {
        control_pending = false;
    }
    if (actions & APP_ACTION_DISABLE_POWER)
    {
        if (clear_control)
        {
            Clear_RunningData();
        }
        if (!power_stage_hw_phases_disabled())
        {
            power_stage_hw_stop();
        }
    }
    if (actions & APP_ACTION_START_CONTROL)
    {
        power_stage_hw_start();
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
    uint32_t state = critical_hw_enter();

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
    critical_hw_exit(state);
    return result.rejection == APP_ACCEPTED;
}

/* 启动链：BOOT → SELF_TEST → READY（无对外动作）。 */
static void RunState_RunBootChain(void)
{
    if (lifecycle.snapshot.state == APP_BOOT)
    {
        (void)RunState_Send(APP_EVENT_BOOT_DONE, 0U, APP_CONTROL_NONE, false, false);
    }
    if (lifecycle.snapshot.state == APP_SELF_TEST && RunState_BuildGuards().self_test_passed)
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
    uint32_t state = critical_hw_enter();
    AppLifecycleGuards guards = RunState_BuildGuards();
    if (lifecycle.snapshot.state == APP_STARTING && guards.start_permitted &&
        guards.samples_fresh && guards.fault_sources_clear && guards.startup_confirmed &&
        RunState_ModeNeedsPreparation(MotorControl.ModeNow) && power_stage_hw_phases_disabled())
    {
        power_stage_hw_start();
    }
    (void)RunState_Send(APP_EVENT_START_DONE, 0U, APP_CONTROL_NONE, false, false);
    critical_hw_exit(state);
}

/* 母线电压处于重新使能窗口 [25.6, 33.8] V：过/欠压故障的源恢复判据。 */
static bool RunState_VbusInEnableWindow(void)
{
    if (!isfinite(FOC.Vbus_filt))
    {
        return false;
    }
    return FOC.Vbus_filt >= BUS_VOLTAGE_ENABLE_MIN_V && FOC.Vbus_filt <= BUS_VOLTAGE_ENABLE_MAX_V;
}

/* 每拍更新与清除请求无关的恢复证据：过流回落需持续 100 ms。 */
static void RunState_UpdateRecoveryEvidence(void)
{
    if (MotorControl.ErrorNow != Over_Current)
    {
        overcurrent_recover_ticks = 0U;
        return;
    }
    if (fast_abs(FOC.Ia) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f &&
        fast_abs(FOC.Ib) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f &&
        fast_abs(FOC.Ic) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f)
    {
        if (overcurrent_recover_ticks < RUN_STATE_OC_RECOVER_TICKS)
        {
            ++overcurrent_recover_ticks;
        }
        return;
    }
    overcurrent_recover_ticks = 0U;
}

/* 恢复矩阵（阶段 D1）：逐故障源恢复判据。参数/标定/操作类故障暂无在线判据，
 * 保留操作员确认语义（过渡，随故障保护解耦计划细化）；未知故障一律拒绝。 */
static bool RunState_FaultSourceRecovered(ErrorNow_TypeDef error)
{
    switch (error)
    {
    case Encoder_Error:
        return Encoder_GetBadFrameStreak(&OnBoard_Encoder) == 0U;
    case TemperatureSensor_Error:
        return McuTemperature.valid != 0U;
    case High_Temprature:
        return McuTemperature.valid != 0U && FOC.temp < RUN_STATE_TEMP_RECOVER_C;
    case Over_Voltage:
    case Under_Voltage:
        return RunState_VbusInEnableWindow();
    case Over_Current:
        return overcurrent_recover_ticks >= RUN_STATE_OC_RECOVER_TICKS;
    case CurrentOffset_Error:
    case PolePairs_Error:
    case Large_Phase_Resistance:
    case Large_Phase_Inductance:
    case Encoder_NotCalibrated:
    case MotorParam_Error:
    case Sensorless_Error:
    case FrictionIdentification_Error:
    case ControlOverrun_Error:
    case CoggingCalibration_Error:
        return true;
    default:
        return false;
    }
}

/* 清除资格与兼容值提交分开；整个判定到提交间禁止快环插入新的故障。 */
static bool RunState_TryClearFault(void)
{
    uint32_t state = critical_hw_enter();
    ErrorNow_TypeDef error = MotorControl.ErrorNow;
    AppLifecycleGuards guards;
    AppLifecycleEvent event;
    AppLifecycleResult result;

    if (lifecycle.snapshot.state != APP_FAULT ||
        (error != No_Error && !RunState_FaultSourceRecovered(error)))
    {
        critical_hw_exit(state);
        return false;
    }
    guards = RunState_BuildGuards();
    /* 仅覆盖本次已经按恢复矩阵或操作员确认策略获准的锁存源，不先改 ErrorNow。 */
    guards.active_faults = 0U;
    guards.fault_sources_clear = true;
    memset(&event, 0, sizeof(event));
    event.events = APP_EVENT_CLEAR;
    event.request_epoch = lifecycle.snapshot.epoch;
    result = AppLifecycle_Step(&lifecycle, &event, &guards);
    if (result.rejection == APP_ACCEPTED && result.snapshot.state == APP_SELF_TEST)
    {
        Set_ErrorNow(No_Error);
        MotorControl.ModeNow = Motor_Disable;
    }
    RunState_ApplyActions(result.actions, false);
    critical_hw_exit(state);
    return result.rejection == APP_ACCEPTED;
}

/* 维护目标模式 → 核心操作；FOC-Calibration 五任务已移除待重建，不进会话映射。 */
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
    case Calib_Anticogging:
    case Calib_PhaseResistance:
    case Calib_Friction:
    case Calib_EncoderObserver:
    case Calib_EleAngelOffset:
        return APP_OPERATION_CALIBRATION;
    default:
        return APP_OPERATION_NONE;
    }
}

/* 串行提交维护事件并执行返回动作；完成来源身份仍沿用兼容路径。 */
static bool RunState_SendOperation(uint32_t events,
                                   AppOperation operation,
                                   AppOperationEffect effects,
                                   uint32_t faults)
{
    AppLifecycleEvent event;
    AppLifecycleGuards guards;
    AppLifecycleResult result;
    uint32_t state = critical_hw_enter();

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.faults = faults;
    event.operation = operation;
    event.effects = effects;
    event.control_mode = APP_CONTROL_NONE;
    event.completion_epoch = lifecycle.snapshot.epoch;
    event.request_epoch = lifecycle.snapshot.epoch;
    guards = RunState_BuildGuards();
    if (events == APP_EVENT_CANCEL_DONE && lifecycle.snapshot.state == APP_FAULT)
    {
        /* 原故障保持锁存，释放确认不能再次被同一活动故障事件遮蔽。 */
        guards.active_faults = 0U;
    }
    result = AppLifecycle_Step(&lifecycle, &event, &guards);
    RunState_ApplyActions(result.actions, false);
    critical_hw_exit(state);
    return result.rejection == APP_ACCEPTED;
}

/* 会话服务：SAVE 结果上报、ZERO/DEFAULTS/标定派生完成、被打断会话的兼容取消确认。
 * 尚无 worker 原始会话身份和资源释放查询，不能将当前 epoch 当作陈旧结果隔离证明。
 * 返回 true 表示本拍已上报会话事件（完成/失败/取消确认），调用方推迟一拍再开新会话。 */
static bool RunState_ServiceOperations(ModeNow_TypeDef target, bool worker_stopped)
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
        else if (lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS && target == Save_Param)
        {
            /* 默认值由受理后的 worker 应用，再以结果请求保存。 */
            sent = true;
            (void)RunState_SendOperation(
                APP_EVENT_OPERATION_DONE, APP_OPERATION_DEFAULTS, APP_EFFECT_UNCOMMITTED, 0U);
        }
        else if (lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION &&
                 ((operation_mode == Calib_Anticogging && target == Save_Param) ||
                  (operation_mode == Calib_PhaseResistance && worker_stopped) ||
                  (operation_mode == Calib_Friction && worker_stopped) ||
                  ((operation_mode == Calib_EncoderObserver ||
                    operation_mode == Calib_EleAngelOffset) &&
                   target == Save_Param)))
        {
            /* 标定完成以模块切往 Save_Param 或停机结果表达：未提交完成，随后由 SAVE 提交。 */
            sent = true;
            (void)RunState_SendOperation(
                APP_EVENT_OPERATION_DONE, APP_OPERATION_CALIBRATION, APP_EFFECT_UNCOMMITTED, 0U);
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
        /* 相电阻/摩擦/编码器标定无自管功率：按旧入口条件（自 Disable 且轴配置有效）启相。 */
        if ((target == Calib_PhaseResistance || target == Calib_Friction ||
             target == Calib_EncoderObserver || target == Calib_EleAngelOffset) &&
            ModeLast == Motor_Disable && MotorControl.axis_profile_valid)
        {
            uint32_t state = critical_hw_enter();
            AppLifecycleGuards guards = RunState_BuildGuards();
            if (MotorControl.ModeNow == target && lifecycle.snapshot.state == APP_MAINTENANCE &&
                guards.start_permitted && guards.samples_fresh && guards.fault_sources_clear)
            {
                power_stage_hw_start();
            }
            critical_hw_exit(state);
        }
    }
}

/* 单次复制发布完成的 SWD 请求；全程有界，不等待外部写入者。 */
static void RunState_ServiceBenchRequest(bool worker_idle)
{
    uint32_t state = critical_hw_enter();
    uint32_t seq = FocBenchSpeed.request_seq;
    uint32_t command, duration_ms;
    float iq_limit_a, phase_limit_a, speed_limit_rad_s, target_speed_rad_s;
    AppLifecycleGuards guards;
    uint32_t result = FOC_BENCH_SPEED_DENIED;

    if (seq == 0U || (seq & 1U) != 0U || seq <= bench_speed.last_request_seq)
    {
        critical_hw_exit(state);
        return;
    }
    command = FocBenchSpeed.command;
    duration_ms = FocBenchSpeed.duration_ms;
    iq_limit_a = FocBenchSpeed.iq_limit_a;
    phase_limit_a = FocBenchSpeed.phase_limit_a;
    speed_limit_rad_s = FocBenchSpeed.speed_limit_rad_s;
    target_speed_rad_s = FocBenchSpeed.target_speed_rad_s;
    if (FocBenchSpeed.request_seq != seq)
    {
        critical_hw_exit(state);
        return;
    }
    bench_speed.last_request_seq = seq;
    if (seq <= bench_speed.blocked_seq)
    {
        result = FOC_BENCH_SPEED_INTERRUPTED;
    }
    else if (command == FOC_BENCH_SPEED_STOP)
    {
        (void)FocRunState_RequestRemoteStop();
        RunState_RequestStop(true);
        result = FOC_BENCH_SPEED_ACCEPTED;
    }
    else if (command == FOC_BENCH_SPEED_UPDATE)
    {
        MotorControlCommand goal = {.mode = MOTOR_CONTROL_SPEED, .speed_rad_s = target_speed_rad_s};
        MotorControlResult submitted = bench_speed.active && !bench_speed.stop_requested
                                           ? FocRunState_SubmitControl(&goal)
                                           : MOTOR_CONTROL_DENIED;
        result = submitted == MOTOR_CONTROL_ACCEPTED ? FOC_BENCH_SPEED_ACCEPTED
                 : submitted == MOTOR_CONTROL_BUSY   ? FOC_BENCH_SPEED_BUSY
                 : submitted == MOTOR_CONTROL_INVALID || submitted == MOTOR_CONTROL_OUT_OF_RANGE
                     ? FOC_BENCH_SPEED_INVALID
                     : FOC_BENCH_SPEED_DENIED;
        if (submitted == MOTOR_CONTROL_ACCEPTED)
        {
            pending_control_watch_seq = seq;
        }
    }
    else if (command != FOC_BENCH_SPEED_START || duration_ms == 0U ||
             duration_ms > BENCH_SPEED_MAX_DURATION_MS || !isfinite(iq_limit_a) ||
             !isfinite(phase_limit_a) || !isfinite(speed_limit_rad_s) || iq_limit_a <= 0.0f ||
             phase_limit_a <= iq_limit_a || phase_limit_a > BENCH_SPEED_MAX_PHASE_A ||
             speed_limit_rad_s <= 0.0f || speed_limit_rad_s > BENCH_SPEED_MAX_RAD_S ||
             !isfinite(target_speed_rad_s) || target_speed_rad_s != 0.0f)
    {
        result = FOC_BENCH_SPEED_INVALID;
    }
    else if (bench_speed.active)
    {
        result = FOC_BENCH_SPEED_BUSY;
    }
    else
    {
        guards = RunState_BuildGuards();
        if (worker_idle && lifecycle.snapshot.state == APP_READY &&
            lifecycle.snapshot.operation == APP_OPERATION_NONE &&
            lifecycle.snapshot.operation_result != APP_OPERATION_CANCEL_REQUESTED &&
            operation_mode == Motor_Disable && save_finish == SAVE_FINISH_NONE &&
            MotorControl.ModeNow == Motor_Disable && ModeLast == Motor_Disable &&
            guards.phases_disabled && guards.control_idle && guards.self_test_passed &&
            guards.start_permitted && !MotorControl.isUseSensorless &&
            Encoder_IsOnline(&OnBoard_Encoder) &&
            (Encoder_GetCalibFlag(&OnBoard_Encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
            isfinite(MotorControl.current_limit) && iq_limit_a <= MotorControl.current_limit &&
            isfinite(MotorControl.speed_limit) && speed_limit_rad_s <= MotorControl.speed_limit &&
            isfinite(MotorControl.speedAcc) && MotorControl.speedAcc > 0.0f &&
            isfinite(MotorControl.speedDec) && MotorControl.speedDec > 0.0f &&
            isfinite(MotorControl.speed_Kp) && MotorControl.speed_Kp >= 0.0f &&
            isfinite(MotorControl.speed_Ki) && MotorControl.speed_Ki >= 0.0f &&
            isfinite(Encoder_GetMecVel(&OnBoard_Encoder)) &&
            fabsf(Encoder_GetMecVel(&OnBoard_Encoder)) <= speed_limit_rad_s &&
            fabsf(FOC.Ia) < phase_limit_a && fabsf(FOC.Ib) < phase_limit_a &&
            fabsf(FOC.Ic) < phase_limit_a &&
            (MotorControl.axis_profile.magic == 0U ||
             (isfinite(MotorControl.axis_profile.maximum_speed_rad_s) &&
              speed_limit_rad_s <= MotorControl.axis_profile.maximum_speed_rad_s)) &&
            RunState_VbusInEnableWindow())
        {
            /* 沿用现有准入函数，登记仍发生在 owner；启动不继承旧 PI/目标。 */
            Clear_RunningData();
            if (ModeSwitch_Handle(Speed_Mode))
            {
                bench_speed.saved_current_limit_a = MotorControl.current_limit;
                bench_speed.saved_speed_limit_rad_s = MotorControl.speed_limit;
                bench_speed.phase_limit_a = phase_limit_a;
                bench_speed.speed_limit_rad_s = speed_limit_rad_s;
                bench_speed.remaining_fast_ticks = duration_ms * BENCH_SPEED_FAST_TICKS_PER_MS;
                bench_speed.session_seq = seq;
                bench_speed.applied_seq = 0U;
                bench_speed.stop_reason = FOC_BENCH_SPEED_NO_STOP;
                bench_speed.stop_requested = false;
                MotorControl.current_limit = iq_limit_a;
                MotorControl.speed_limit = speed_limit_rad_s;
                MotorControl.speedRef = 0.0f;
                MotorControl.speedShadow = 0.0f;
                MotorControl.idRef = 0.0f;
                MotorControl.iqRef = 0.0f;
                bench_speed.active = true;
                result = FOC_BENCH_SPEED_ACCEPTED;
            }
        }
    }
    /* 输出区只有 2 kHz 写者；读者用 status_seq 检查一次完整发布。 */
    ++FocBenchSpeed.status_seq;
    FocBenchSpeed.ack_seq = seq;
    FocBenchSpeed.ack_result = result;
    ++FocBenchSpeed.status_seq;
    critical_hw_exit(state);
}

/* 独立于 PC 的快环计时和采样式测试截止，仅在显式台架会话内运行。 */
static void RunState_BenchFastGuard(void)
{
    if (!bench_speed.active)
    {
        return;
    }
    if (MotorControl.ErrorNow == No_Error &&
        (!isfinite(FOC.Ia) || !isfinite(FOC.Ib) || !isfinite(FOC.Ic) ||
         fabsf(FOC.Ia) >= bench_speed.phase_limit_a || fabsf(FOC.Ib) >= bench_speed.phase_limit_a ||
         fabsf(FOC.Ic) >= bench_speed.phase_limit_a))
    {
        bench_speed.stop_reason = FOC_BENCH_SPEED_PHASE_CURRENT;
        Set_ErrorNow(Over_Current);
    }
    if (MotorControl.ErrorNow == No_Error &&
        (!isfinite(Encoder_GetMecVel(&OnBoard_Encoder)) ||
         fabsf(Encoder_GetMecVel(&OnBoard_Encoder)) > bench_speed.speed_limit_rad_s))
    {
        bench_speed.stop_reason = FOC_BENCH_SPEED_OVERSPEED;
        Set_ErrorNow(MotorParam_Error);
    }
    if (bench_speed.remaining_fast_ticks > 0U)
    {
        --bench_speed.remaining_fast_ticks;
    }
    if (bench_speed.remaining_fast_ticks == 0U)
    {
        RunState_BenchCancel(FOC_BENCH_SPEED_TIMEOUT);
        control_pending = false;
        power_stage_hw_stop();
    }
}

/* 仅实际关相并退出运行后恢复 RAM 限值；没有确认时保留更严格限制与 busy。 */
static void RunState_PublishBenchState(void)
{
    uint32_t state = critical_hw_enter();
    uint32_t session_state = FocBenchSpeed.state;
    if (bench_speed.active)
    {
        if (MotorControl.ModeNow == Motor_Disable && power_stage_hw_phases_disabled() &&
            (lifecycle.snapshot.state == APP_READY || lifecycle.snapshot.state == APP_FAULT ||
             lifecycle.snapshot.state == APP_FATAL))
        {
            MotorControl.current_limit = bench_speed.saved_current_limit_a;
            MotorControl.speed_limit = bench_speed.saved_speed_limit_rad_s;
            bench_speed.active = false;
            session_state = FOC_BENCH_SPEED_STOPPED;
        }
        else
        {
            session_state = bench_speed.stop_requested                ? FOC_BENCH_SPEED_STOPPING
                            : lifecycle.snapshot.state == APP_RUNNING ? FOC_BENCH_SPEED_RUNNING
                                                                      : FOC_BENCH_SPEED_STARTING;
        }
    }
    ++FocBenchSpeed.status_seq;
    if (pending_control_watch_seq != 0U && !control_pending)
    {
        if (bench_speed.applied_seq != pending_control_watch_seq &&
            FocBenchSpeed.ack_seq == pending_control_watch_seq)
        {
            FocBenchSpeed.ack_result = FOC_BENCH_SPEED_INTERRUPTED;
        }
        pending_control_watch_seq = 0U;
    }
    FocBenchSpeed.session_seq = bench_speed.session_seq;
    FocBenchSpeed.applied_seq = bench_speed.applied_seq;
    FocBenchSpeed.state = session_state;
    FocBenchSpeed.stop_reason = bench_speed.stop_reason;
    FocBenchSpeed.remaining_ms =
        bench_speed.active
            ? (bench_speed.remaining_fast_ticks + BENCH_SPEED_FAST_TICKS_PER_MS - 1U) /
                  BENCH_SPEED_FAST_TICKS_PER_MS
            : 0U;
    ++FocBenchSpeed.status_seq;
    critical_hw_exit(state);
}

void FocRunState_Init(void)
{
    AppLifecycle_Init(&lifecycle);
    operation_mode = Motor_Disable;
    save_finish = (uint8_t)SAVE_FINISH_NONE;
    overcurrent_recover_ticks = 0U;
    outcome_latched = false;
    control_pending = false;
    pending_control_watch_seq = 0U;
    fast_sample_epoch = 0U;
    observed_fast_sample_epoch = 0U;
    fast_samples_fresh = false;
    memset(&bench_speed, 0, sizeof(bench_speed));
    memset((void *)&FocBenchSpeed, 0, sizeof(FocBenchSpeed));
    FocBenchSpeed.version = FOC_BENCH_SPEED_MAILBOX_VERSION;
}

bool FocRunState_IsMaintenanceActive(ModeNow_TypeDef mode)
{
    return lifecycle.snapshot.state == APP_MAINTENANCE && operation_mode == mode;
}

void FocRunState_SaveFinished(bool committed)
{
    save_finish = committed ? (uint8_t)SAVE_FINISH_COMMITTED : (uint8_t)SAVE_FINISH_FAILED;
}

AppLifecycleState FocRunState_GetState(void)
{
    return lifecycle.snapshot.state;
}

bool FocRunState_RequestRemoteStop(void)
{
    uint32_t state = critical_hw_enter();
    RunState_BenchCancel(FOC_BENCH_SPEED_MANUAL_STOP);
    control_pending = false;
    if (outcome_latched && pending_outcome.result != MOTOR_WORK_FAULT)
    {
        outcome_latched = false;
    }
    MotorControl.ModeNow = Motor_Disable;
    critical_hw_exit(state);
    return true;
}

MotorControlResult FocRunState_SubmitControl(const MotorControlCommand *command)
{
    uint32_t state = critical_hw_enter();
    MotorControlResult result = validate_control(command);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        if (control_pending)
        {
            result = MOTOR_CONTROL_BUSY;
        }
        else
        {
            pending_control = *command;
            pending_control_watch_seq = 0U;
            control_pending = true;
        }
    }
    critical_hw_exit(state);
    return result;
}

void FocRunState_CheckFastFaults(void)
{
    RunState_BenchFastGuard();
    if (Encoder_FeedbackRequired(&MotorControl) &&
        Encoder_GetBadFrameStreak(&OnBoard_Encoder) >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
    {
        Set_ErrorNow(Encoder_Error);
    }

    /* 同时覆盖内部模式赋值，不只通信请求。 */
    if (!MotorControl.axis_profile_valid && MotorControl.ModeNow != Motor_Disable &&
        MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Clear_Error)
    {
        Set_ErrorNow(MotorParam_Error);
    }
}

/**
 * @brief  上报本周期 worker 结果（20 kHz 快环调用，2 kHz 状态机消费）。
 * @param  outcome 模式 worker 的周期结果；MOTOR_WORK_RUNNING 不入箱。
 * @note 与消费端通过短临界区交换；只保留最新的非 RUNNING 结果。
 */
void FocRunState_PostOutcome(MotorWorkOutcome_TypeDef outcome)
{
    uint32_t primask;

    ++fast_sample_epoch;
    if (outcome.result == MOTOR_WORK_RUNNING)
    {
        return;
    }
    primask = critical_hw_enter();
    pending_outcome = outcome;
    outcome_latched = true;
    critical_hw_exit(primask);
}

/**
 * @brief  故障紧急关断快车道：故障已锁存且功率级记录为开时立即关断硬件。
 * @note 仅由 20 kHz 快速中断调用。本板无 TIM1 BKIN 与驱动器故障引脚，软件是
 *       唯一关断路径：该动作把"故障 → 功率级关断"保持在快速环时延内；状态机
 *       在下一 2 kHz 慢拍对齐记录并执行停机清理。
 */
void FocRunState_FastFaultStop(void)
{
    if (MotorControl.ErrorNow != No_Error)
    {
        RunState_BenchCancel(FOC_BENCH_SPEED_FAULT);
        control_pending = false;
    }
    if (MotorControl.ErrorNow != No_Error && !power_stage_hw_phases_disabled())
    {
        power_stage_hw_stop();
    }
}

/* 取走待处理结果；无请求时按 RUNNING 处理。 */
static MotorWorkOutcome_TypeDef FocRunState_TakeOutcome(void)
{
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};
    uint32_t primask = critical_hw_enter();

    if (outcome_latched)
    {
        outcome = pending_outcome;
        outcome_latched = false;
    }
    critical_hw_exit(primask);
    return outcome;
}

void FocRunState_Tick(void)
{
    MotorWorkOutcome_TypeDef outcome = FocRunState_TakeOutcome();
    ModeNow_TypeDef target;
    bool needs_preparation;
    bool defer_start;
    bool operation_event;
    bool clear_requested;

    RunState_UpdateSampleProgress();
    if (bench_speed.active && bench_speed.stop_requested)
    {
        (void)FocRunState_RequestRemoteStop();
        RunState_RequestStop(true);
        if (outcome.result != MOTOR_WORK_FAULT)
        {
            outcome.result = MOTOR_WORK_RUNNING;
        }
    }

    /* 1. worker 结果 → 目标模式与前置动作（与旧实现同序）。 */
    target = MotorControl.ModeNow;
    switch (outcome.result)
    {
    case MOTOR_WORK_SWITCH_MODE:
        if (outcome.power_off)
        {
            power_stage_hw_stop();
        }
        MotorControl.ModeNow = outcome.next_mode;
        target = outcome.next_mode;
        break;
    case MOTOR_WORK_STOP:
        RunState_BenchCancel(FOC_BENCH_SPEED_MANUAL_STOP);
        MotorControl.ModeNow = Motor_Disable;
        target = Motor_Disable;
        break;
    case MOTOR_WORK_FAULT:
        Set_ErrorNow(outcome.error);
        break;
    default:
        break;
    }

    /* 1b. 清除请求：ModeNow==Clear_Error 由模式切换层登记，故障投影前捕获。 */
    clear_requested = MotorControl.ModeNow == Clear_Error;

    /* 2. 故障策略与指示：Save/Default 期间容错保留当前模式。 */
    if (MotorControl.ErrorNow != No_Error)
    {
        if (target != Save_Param && target != Default_Param)
        {
            MotorControl.ModeNow = Motor_Disable;
            target = Motor_Disable;
        }
        indicator_hw_set_led(true, (uint8_t)MotorControl.ErrorNow);
    }
    else
    {
        indicator_hw_set_led(false, (uint8_t)MotorControl.ModeNow);
    }

    /* 3. 启动链、维护会话、恢复证据与故障/清除处理。 */
    RunState_RunBootChain();
    operation_event = RunState_ServiceOperations(target, outcome.result == MOTOR_WORK_STOP);
    RunState_UpdateRecoveryEvidence();
    if (MotorControl.ErrorNow != No_Error)
    {
        /* 核心故障动作立即请求关相，控制清理由下方停止路径按既有条件执行。 */
        (void)RunState_Send(
            APP_EVENT_FAULT, (uint32_t)MotorControl.ErrorNow, APP_CONTROL_NONE, false, false);
    }
    if (clear_requested && lifecycle.snapshot.state == APP_FAULT)
    {
        if (RunState_TryClearFault())
        {
            RunState_RunBootChain();
        }
    }
    else if (lifecycle.snapshot.state == APP_STOPPING || lifecycle.snapshot.state == APP_STARTING ||
             lifecycle.snapshot.state == APP_RUNNING)
    {
        (void)RunState_Send(0U, 0U, APP_CONTROL_NONE, false, false);
    }
    if (lifecycle.snapshot.state == APP_FAULT || lifecycle.snapshot.state == APP_FATAL)
    {
        RunState_BenchCancel(FOC_BENCH_SPEED_FAULT);
        MotorControl.ModeNow = Motor_Disable;
        target = Motor_Disable;
    }

    RunState_ServiceBenchRequest(outcome.result == MOTOR_WORK_RUNNING);
    target = MotorControl.ModeNow;

    /* 故障、worker 停机与外部失能先处理，随后消费目标；复核失败直接丢弃，不重试。 */
    apply_pending_control();

    /* 4. 目标迁移：停止、使能或退出运行态。 */
    needs_preparation = RunState_ModeNeedsPreparation(target);
    if (target == Motor_Disable)
    {
        if (ModeLast != Motor_Disable || lifecycle.snapshot.state == APP_STARTING ||
            !power_stage_hw_phases_disabled())
        {
            RunState_RequestStop(ModeLast != Motor_Disable || !power_stage_hw_phases_disabled());
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
    RunState_PublishBenchState();
}
