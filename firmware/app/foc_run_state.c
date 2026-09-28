#include "foc_run_state.h"

#include <math.h>
#include <string.h>

#include "angle_feedback.h"
#include "app_lifecycle.h"
#include "bus_voltage_profile.h"
#include "boot_hw.h"
#include "image_format.h"
#include "critical_hw.h"
#include "foc_bench_speed.h"
#include "foc_errhandle.h"
#include "foc_algorithm.h"
#include "foc_mode_dispatch.h"
#include "foc_run.h"
#include "foc_sensing.h"
#include "indicator_hw.h"
#include "mcu_temperature.h"
#include "motor_sensing.h"
#include "motor_state.h"
#include "foc_friction_identification.h"
#include "foc_cogging_calibration.h"
#include "foc_pid.h"
#include "foc_encoder_calibration.h"
#include "foc_phase_resistance.h"
#include "foc_sensorless_run.h"
#include "power_stage_hw.h"
#include "utils.h"

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
 * - 本层独占请求受理、模式提交、真实守卫供给与动作执行；
 * - worker 只返回带原始会话身份的结果，不能直接提交模式；
 * - 维护任务进入 MAINTENANCE，前台借用归还后才能确认取消或保存完成。 */

static AppLifecycle lifecycle;
/* 组合层初始化时显式传入，地址终身不变，不导出内部指针。 */
static MotorControl_TypeDef *MotorControl;
static FOC_TypeDef *FOC;
static PI_Controller_TypeDef *PI_Speed;
static Encoder_TypeDef *OnBoard_Encoder;
static SensorlessStartup_TypeDef *SensorlessStartup;
static ModeNow_TypeDef ModeLast;

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
static uint32_t operation_epoch, outcome_epoch, request_epoch, cancel_epoch;
static ModeNow_TypeDef completed_mode;
static MotorMaintenanceProgress last_maintenance;
static struct
{
    MotorMaintenanceRequest request;
    MotorMaintenanceProgress terminal;
    uint32_t ticket;
    bool valid;
} last_maintenance_request;
static FocBackgroundClaim background_claim;
static bool background_busy, stop_pending, configuration_reading;
/* 前台升级交接独占票据；失败后继续禁止运动，只有重新启动初始化才能清空。 */
static uint64_t boot_stop_ticket;
static bool boot_confirmation_pending;
static bool boot_maintenance_only;
/* Over_Current 回落驻留计数；饱和于 RUN_STATE_OC_RECOVER_TICKS。 */
static MotorFaultRecovery recovery;
/* 快环结果邮箱：20 kHz 写入、2 kHz 消费；只保留最新的非 RUNNING 结果。 */
static MotorWorkOutcome_TypeDef pending_outcome;
static volatile bool outcome_latched;
/* 前台写入、监督周期消费的单槽目标邮箱；STOP 与故障优先于目标。 */
static MotorControlCommand pending_control;
static uint32_t control_epoch;
static volatile bool control_pending;
/* 唯一生产调用方位于 ADC2 JEOS 快环末尾，计数表示本次感测处理已经完成。 */
static volatile uint32_t fast_sample_epoch;
static uint32_t observed_fast_sample_epoch;
static bool fast_samples_fresh;
static uint32_t supervisor_epoch;
static bool RunState_VbusInEnableWindow(void);
static MotorMaintenanceKind maintenance_kind(ModeNow_TypeDef mode);
static void clear_running_data(void)
{
    if (ModeLast == Calib_Anticogging)
    {
        FocCogging_Abort();
    }
    if (ModeLast == Calib_Friction)
    {
        FocFrictionIdentification_Abort(MotorControl, PI_Speed);
    }
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = 0.0f;
    MotorControl->vqRef = 0.0f;
    MotorControl->speedRef = 0.0f;
    MotorControl->speedShadow = 0.0f;
    MotorControl->posShadow = 0.0f;
    MotorControl->isReachTargetPos = false;
    MotorControl->pos_vel_filtered = 0.0f;
    FOC->Id = 0.0f;
    FOC->Iq = 0.0f;
    FOC_CurrentController_Reset(FOC);
    PI_Controller_Reset(PI_Speed);
    SensorlessStartup_Reset(SensorlessStartup);
}

/* STOP/FAULT 撤销已提交或正在写入的台架请求；不因迟到提交重新启动。 */
static void RunState_BenchCancel(uint32_t reason)
{
    uint32_t seq = FocBenchSpeed.request_seq;
    uint32_t committed_seq = (seq & 1U) != 0U && seq != UINT32_MAX ? seq + 1U : seq;
    if (committed_seq > bench_speed.blocked_seq &&
        (reason != FOC_BENCH_SPEED_FAULT ||
         (lifecycle.snapshot.state != APP_FAULT && lifecycle.snapshot.state != APP_FATAL)))
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
    if (boot_stop_ticket != 0U || boot_confirmation_pending || boot_maintenance_only)
    {
        return MOTOR_CONTROL_DENIED;
    }
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
    if (MotorControl->ErrorNow != No_Error)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (power_stage_hw_phases_disabled() || lifecycle.snapshot.state != APP_RUNNING ||
        MotorControl->ModeNow != mode || !MotorControl->axis_profile_valid ||
        MotorControl->isUseSensorless || !Encoder_IsOnline(OnBoard_Encoder) ||
        (Encoder_GetCalibFlag(OnBoard_Encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        !RunState_VbusInEnableWindow() || !isfinite(FOC->Vbus) ||
        FOC->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        return MOTOR_CONTROL_DENIED;
    }
    switch (command->mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return isfinite(MotorControl->current_limit) && MotorControl->current_limit > 0.0f &&
                       fabsf(command->iq_a) <= MotorControl->current_limit
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_SPEED:
        return isfinite(MotorControl->speed_limit) && MotorControl->speed_limit > 0.0f &&
                       fabsf(command->speed_rad_s) <= MotorControl->speed_limit &&
                       (MotorControl->axis_profile.magic == 0U ||
                        (isfinite(MotorControl->axis_profile.maximum_speed_rad_s) &&
                         MotorControl->axis_profile.maximum_speed_rad_s > 0.0f &&
                         fabsf(command->speed_rad_s) <=
                             MotorControl->axis_profile.maximum_speed_rad_s))
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_POSITION:
        return MotorAxisProfile_AllowsPosition(&MotorControl->axis_profile,
                                               MotorControl->axis_profile_valid,
                                               Encoder_GetMecPos(OnBoard_Encoder),
                                               command->position_rad)
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_VQ:
        return fabsf(command->vq_v) <= FOC_MaximumVoltage(FOC->Vbus_filt)
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
    if (control_epoch != lifecycle.snapshot.epoch || stop_pending ||
        validate_control(&pending_control) != MOTOR_CONTROL_ACCEPTED)
    {
        critical_hw_exit(state);
        return;
    }
    switch (pending_control.mode)
    {
    case MOTOR_CONTROL_CURRENT:
        MotorControl->iqRef = pending_control.iq_a;
        break;
    case MOTOR_CONTROL_SPEED:
        MotorControl->speedRef = pending_control.speed_rad_s;
        break;
    case MOTOR_CONTROL_POSITION:
        MotorControl->posRef = pending_control.position_rad;
        break;
    case MOTOR_CONTROL_VQ:
        MotorControl->vqRef = pending_control.vq_v;
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
           MotorControl->ModeNow == Vq_Mode || MotorControl->ModeNow == Calib_EncoderOffset ||
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
    ModeNow_TypeDef mode = MotorControl->ModeNow;

    guards.active_faults = (uint32_t)MotorControl->ErrorNow;
    guards.phases_disabled = power_stage_hw_phases_disabled();
    guards.control_idle = !control_pending && !outcome_latched;
    /* 取消确认必须同时取得每个 worker 和前台的实际释放证明。 */
    guards.operation_idle = !background_busy && !outcome_latched;
    guards.maintenance_released =
        !background_busy && (!configuration_reading || lifecycle.snapshot.state == APP_RUNNING) &&
        FocEncoderCalibration_IsReleased() && FocCogging_IsReleased() &&
        FocFrictionIdentification_IsReleased() && PhaseResistanceMode_IsReleased();
    guards.samples_fresh = fast_samples_fresh && isfinite(FOC->Vbus) && isfinite(FOC->Vbus_filt) &&
                           isfinite(FOC->Ia) && isfinite(FOC->Ib) && isfinite(FOC->Ic) &&
                           MotorTemperature_IsFresh() && isfinite(FOC->temp);
    guards.fault_sources_clear = MotorControl->ErrorNow == No_Error;
    guards.self_test_passed =
        MotorControl->axis_profile_valid && guards.samples_fresh && guards.fault_sources_clear;
    guards.start_permitted =
        MotorControl->axis_profile_valid && !boot_confirmation_pending && !boot_maintenance_only;
    if (lifecycle.snapshot.state != APP_RUNNING)
    {
        guards.start_permitted = guards.start_permitted && RunState_VbusInEnableWindow() &&
                                 isfinite(FOC->Vbus) && FOC->Vbus < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
    }
    if (Encoder_FeedbackRequired(MotorControl))
    {
        guards.start_permitted = guards.start_permitted && Encoder_IsOnline(OnBoard_Encoder);
    }
    if (mode == Speed_Mode || mode == Position_Mode || mode == Vq_Mode ||
        mode == Calib_Anticogging || mode == Calib_Friction ||
        (mode == Current_Mode && !MotorControl->isUseSensorless))
    {
        guards.start_permitted = guards.start_permitted && (Encoder_GetCalibFlag(OnBoard_Encoder) &
                                                            ENC_CALIB_ALL) == ENC_CALIB_ALL;
    }
    guards.operation_permitted = MotorControl->axis_profile_valid &&
                                 (lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION ||
                                  power_stage_hw_phases_disabled());
    /* 位置/速度模式需要外环控制器就绪后才确认启动，其余模式可同拍确认。 */
    guards.startup_confirmed =
        !RunState_ModeNeedsPreparation(MotorControl->ModeNow) || MotorOuterLoop_IsReady();
    guards.request_fresh = request_epoch == lifecycle.snapshot.epoch && !stop_pending;
    critical_hw_exit(state);
    return guards;
}

/* 功率事实来自平台回读；取消控制数据与数字关相请求分别处理。 */
static void RunState_ApplyActions(uint32_t actions, bool clear_control)
{
    if (actions & APP_ACTION_CANCEL_OPERATION)
    {
        FocEncoderCalibration_Cancel();
        PhaseResistanceMode_Cancel(FOC, MotorControl);
        FocFrictionIdentification_Abort(MotorControl, PI_Speed);
        if (lifecycle.snapshot.operation_result == APP_OPERATION_CANCEL_REQUESTED ||
            lifecycle.snapshot.state == APP_FAULT || lifecycle.snapshot.state == APP_FATAL)
        {
            cancel_epoch = lifecycle.snapshot.epoch;
            FocCogging_Abort();
        }
    }

    if (actions & APP_ACTION_REVOKE_REQUESTS)
    {
        control_pending = false;
    }
    if (actions & APP_ACTION_DISABLE_POWER)
    {
        if (clear_control)
        {
            clear_running_data();
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

static void project_lifecycle_fault(void)
{
    if ((lifecycle.snapshot.state == APP_FAULT || lifecycle.snapshot.state == APP_FATAL) &&
        MotorControl->ErrorNow == No_Error)
    {
        ErrorNow_TypeDef error = ControlOverrun_Error;
        if (!MotorTemperature_IsFresh())
        {
            error = TemperatureSensor_Error;
        }
        else if (!MotorControl->axis_profile_valid)
        {
            error = MotorParam_Error;
        }
        else if (!RunState_VbusInEnableWindow())
        {
            error = isfinite(FOC->Vbus_filt) && FOC->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V
                        ? Under_Voltage
                        : Over_Voltage;
        }
        else if (Encoder_FeedbackRequired(MotorControl) && !Encoder_IsOnline(OnBoard_Encoder))
        {
            error = Encoder_Error;
        }
        MotorFault_Report(MotorControl, error);
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
    uint32_t state = critical_hw_enter();

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.faults = faults;
    event.control_mode = control_mode;
    event.completion_epoch = lifecycle.snapshot.epoch;
    event.request_epoch = lifecycle.snapshot.epoch;
    AppLifecycleGuards guards = RunState_BuildGuards();
    AppLifecycleResult result = AppLifecycle_Step(&lifecycle, &event, &guards);
    project_lifecycle_fault();
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
    if (boot_stop_ticket == 0U && !boot_confirmation_pending &&
        lifecycle.snapshot.state == APP_STARTING && guards.start_permitted &&
        guards.samples_fresh && guards.fault_sources_clear && guards.startup_confirmed &&
        RunState_ModeNeedsPreparation(MotorControl->ModeNow) && power_stage_hw_phases_disabled())
    {
        power_stage_hw_start();
    }
    (void)RunState_Send(APP_EVENT_START_DONE, 0U, APP_CONTROL_NONE, false, false);
    critical_hw_exit(state);
}

/* 母线电压处于重新使能窗口 [25.6, 33.8] V：过/欠压故障的源恢复判据。 */
static bool RunState_VbusInEnableWindow(void)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return FOC->Vbus_filt >= BUS_VOLTAGE_ENABLE_MIN_V && FOC->Vbus_filt <= BUS_VOLTAGE_ENABLE_MAX_V;
}

/* 恢复证据在故障模块维护；保留原实际监督频率及计数。 */

/* 恢复矩阵（阶段 D1）：逐故障源恢复判据。参数/标定/操作类故障暂无在线判据，
 * 保留操作员确认语义（过渡，随故障保护解耦计划细化）；未知故障一律拒绝。 */

/* 清除资格与模式提交分开；整个判定到提交间禁止快环插入新的故障。 */
static bool RunState_TryClearFault(void)
{
    uint32_t state = critical_hw_enter();
    ErrorNow_TypeDef error = MotorControl->ErrorNow;
    AppLifecycleEvent event;

    if (lifecycle.snapshot.state != APP_FAULT ||
        (error != No_Error &&
         !MotorFault_SourceRecovered(&recovery,
                                     MotorControl,
                                     FOC,
                                     OnBoard_Encoder,
                                     error,
                                     fast_samples_fresh && RunState_BuildGuards().samples_fresh,
                                     RunState_BuildGuards().maintenance_released,
                                     MotorTemperature_IsFresh())))
    {
        critical_hw_exit(state);
        return false;
    }
    AppLifecycleGuards guards = RunState_BuildGuards();
    /* 仅覆盖本次已经按恢复矩阵或操作员确认策略获准的锁存源，不先改 ErrorNow。 */
    guards.active_faults = 0U;
    guards.fault_sources_clear = true;
    memset(&event, 0, sizeof(event));
    event.events = APP_EVENT_CLEAR;
    event.request_epoch = lifecycle.snapshot.epoch;
    AppLifecycleResult result = AppLifecycle_Step(&lifecycle, &event, &guards);
    project_lifecycle_fault();
    if (result.rejection == APP_ACCEPTED && result.snapshot.state == APP_SELF_TEST)
    {
        MotorControl->ErrorNow = No_Error;
        MotorControl->ModeNow = Motor_Disable;
    }
    RunState_ApplyActions(result.actions, false);
    critical_hw_exit(state);
    return result.rejection == APP_ACCEPTED;
}

/* 维护目标模式映射为核心操作，包含已恢复的 Mode13/15；其余旧编号仍拒绝。 */
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

/* 串行提交维护事件并执行返回动作；事件只携带创建时的会话身份。 */
static bool RunState_SendOperation(uint32_t events,
                                   AppOperation operation,
                                   AppOperationEffect effects,
                                   uint32_t faults)
{
    AppLifecycleEvent event;
    uint32_t state = critical_hw_enter();

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.faults = faults;
    event.operation = operation;
    event.effects = effects;
    event.control_mode = APP_CONTROL_NONE;
    event.completion_epoch = events == APP_EVENT_CANCEL_DONE ? cancel_epoch : operation_epoch;
    event.request_epoch = lifecycle.snapshot.epoch;
    AppLifecycleGuards guards = RunState_BuildGuards();
    AppLifecycleResult result = AppLifecycle_Step(&lifecycle, &event, &guards);
    if (result.rejection == APP_ACCEPTED &&
        (events == APP_EVENT_OPERATION_DONE || events == APP_EVENT_OPERATION_FAILED ||
         events == APP_EVENT_CANCEL_DONE))
    {
        last_maintenance = (MotorMaintenanceProgress){operation_epoch,
                                                      maintenance_kind(operation_mode),
                                                      result.snapshot.operation_result,
                                                      result.snapshot.operation_effects};
        if (last_maintenance_request.valid && last_maintenance_request.ticket == operation_epoch)
        {
            /* 派生 SAVE 有新身份，不得覆盖原始标定请求的终态。 */
            last_maintenance_request.terminal = last_maintenance;
        }
    }
    project_lifecycle_fault();
    RunState_ApplyActions(result.actions, false);
    critical_hw_exit(state);
    return result.rejection == APP_ACCEPTED;
}

/* 会话服务：SAVE 结果上报、ZERO/DEFAULTS/标定派生完成、被打断会话的实际释放确认。
 * 完成使用创建时身份，取消确认等待全部 worker 和前台资源归还。
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
        else if ((lifecycle.snapshot.operation == APP_OPERATION_ZERO ||
                  lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS) &&
                 target == Save_Param)
        {
            /* ZERO/DEFAULTS 由受理后的 worker 应用，再以结果请求保存；两者终态相同。 */
            sent = true;
            (void)RunState_SendOperation(
                APP_EVENT_OPERATION_DONE, lifecycle.snapshot.operation, APP_EFFECT_UNCOMMITTED, 0U);
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
             lifecycle.snapshot.operation != APP_OPERATION_NONE &&
             (lifecycle.snapshot.operation_result == APP_OPERATION_CANCEL_REQUESTED ||
              (state == APP_FAULT &&
               lifecycle.snapshot.operation_result == APP_OPERATION_FAILED)) &&
             RunState_BuildGuards().maintenance_released && !background_busy)
    {
        /* FAILED仍需确认真实归还；核心保留FAILED终态并清除操作占用，才能显式CLEAR。 */
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

    if (boot_stop_ticket != 0U || boot_confirmation_pending)
    {
        return;
    }

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
    if (lifecycle.snapshot.state == APP_READY)
    {
        request_epoch = lifecycle.snapshot.epoch;
    }
    if (lifecycle.snapshot.state == APP_READY &&
        RunState_SendOperation(APP_EVENT_BEGIN_OPERATION, operation, APP_EFFECT_UNCOMMITTED, 0U))
    {
        operation_mode = target;
        operation_epoch = lifecycle.snapshot.epoch;
        /* 相电阻/摩擦/编码器标定无自管功率：按旧入口条件（自 Disable 且轴配置有效）启相。 */
        if ((target == Calib_PhaseResistance || target == Calib_Friction ||
             target == Calib_EncoderObserver || target == Calib_EleAngelOffset) &&
            ModeLast == Motor_Disable && MotorControl->axis_profile_valid)
        {
            uint32_t state = critical_hw_enter();
            AppLifecycleGuards guards = RunState_BuildGuards();
            if (MotorControl->ModeNow == target && lifecycle.snapshot.state == APP_MAINTENANCE &&
                guards.start_permitted && guards.samples_fresh && guards.fault_sources_clear)
            {
                power_stage_hw_start();
            }
            critical_hw_exit(state);
        }
    }
}

static ModeNow_TypeDef maintenance_mode(MotorMaintenanceKind kind)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
        return Calib_EncoderObserver;
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return Calib_EleAngelOffset;
    case MOTOR_MAINTENANCE_SAVE:
        return Save_Param;
    case MOTOR_MAINTENANCE_COGGING:
        return Calib_Anticogging;
    case MOTOR_MAINTENANCE_PHASE_RESISTANCE:
        return Calib_PhaseResistance;
    case MOTOR_MAINTENANCE_FRICTION:
        return Calib_Friction;
    case MOTOR_MAINTENANCE_MECHANICAL_ZERO:
        return Set_ZeroPosition;
    case MOTOR_MAINTENANCE_DEFAULTS:
        return Default_Param;
    default:
        return Motor_Disable;
    }
}
static MotorMaintenanceKind maintenance_kind(ModeNow_TypeDef mode)
{
    switch (mode)
    {
    case Calib_EncoderObserver:
        return MOTOR_MAINTENANCE_ENCODER_LUT;
    case Calib_EleAngelOffset:
        return MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
    case Calib_Anticogging:
        return MOTOR_MAINTENANCE_COGGING;
    case Calib_PhaseResistance:
        return MOTOR_MAINTENANCE_PHASE_RESISTANCE;
    case Calib_Friction:
        return MOTOR_MAINTENANCE_FRICTION;
    case Set_ZeroPosition:
        return MOTOR_MAINTENANCE_MECHANICAL_ZERO;
    case Default_Param:
        return MOTOR_MAINTENANCE_DEFAULTS;
    default:
        return MOTOR_MAINTENANCE_SAVE;
    }
}

/* 运行所有者已静止：模式已失能、无目标/占用、资源已释放；调用方负责状态与角色前提。 */
static bool RunState_OwnerQuiescent(const AppLifecycleGuards *guards)
{
    return MotorControl->ModeNow == Motor_Disable && !stop_pending && !bench_speed.active &&
           !control_pending && !configuration_reading && !background_busy && !outcome_latched &&
           lifecycle.snapshot.operation == APP_OPERATION_NONE && guards->phases_disabled &&
           guards->control_idle && guards->operation_idle && guards->maintenance_released;
}

static bool configuration_idle(void)
{
    AppLifecycleGuards guards = RunState_BuildGuards();
    /* 纯布尔合取：ModeNow 在辅助函数内部检查，交换与 ErrorNow 的读取次序不影响结果。 */
    return boot_stop_ticket == 0U && lifecycle.snapshot.state == APP_READY &&
           MotorControl->ErrorNow == No_Error && RunState_OwnerQuiescent(&guards);
}

bool FocRunState_ConfigurationAllowed(void)
{
    return !boot_confirmation_pending && configuration_idle();
}

/* 调用方已持有临界区；只读共同占用标志，故障优先级和资源释放由各入口判断。 */
static bool request_busy(void)
{
    return boot_stop_ticket != 0U || background_busy || configuration_reading || outcome_latched ||
           control_pending || bench_speed.active ||
           lifecycle.snapshot.operation != APP_OPERATION_NONE;
}

MotorControlResult FocRunState_RequestEnable(MotorControlMode mode)
{
    if (boot_maintenance_only)
    {
        return MOTOR_CONTROL_DENIED;
    }
    ModeNow_TypeDef target;
    switch (mode)
    {
    case MOTOR_CONTROL_CURRENT:
        target = Current_Mode;
        break;
    case MOTOR_CONTROL_SPEED:
        target = Speed_Mode;
        break;
    case MOTOR_CONTROL_POSITION:
        target = Position_Mode;
        break;
    case MOTOR_CONTROL_VQ:
        target = Vq_Mode;
        break;
    default:
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (MotorControl->ErrorNow != No_Error)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (request_busy())
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (FocRunState_ConfigurationAllowed())
    {
        AppLifecycleGuards guards = RunState_BuildGuards();
        if (guards.self_test_passed && guards.start_permitted && guards.samples_fresh &&
            guards.fault_sources_clear && !MotorControl->isUseSensorless)
        {
            clear_running_data();
            request_epoch = lifecycle.snapshot.epoch;
            result = FocMode_Admit(MotorControl, FOC, OnBoard_Encoder, target)
                         ? MOTOR_CONTROL_ACCEPTED
                         : MOTOR_CONTROL_DENIED;
        }
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult FocRunState_RequestClear(void)
{
    uint32_t mask = critical_hw_enter();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (request_busy() || !RunState_BuildGuards().maintenance_released)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else
    {
        request_epoch = lifecycle.snapshot.epoch;
        if ((lifecycle.snapshot.state == APP_READY && MotorControl->ErrorNow == No_Error) ||
            RunState_TryClearFault())
        {
            result = MOTOR_CONTROL_ACCEPTED;
        }
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult FocRunState_RequestMaintenance(MotorMaintenanceKind kind, uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return MOTOR_CONTROL_INVALID;
    }
    *ticket = 0U;
    ModeNow_TypeDef mode = maintenance_mode(kind);
    if (mode == Motor_Disable)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (MotorControl->ErrorNow != No_Error)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (request_busy() || !RunState_BuildGuards().maintenance_released)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (FocRunState_ConfigurationAllowed() && RunState_BuildGuards().self_test_passed &&
             (kind == MOTOR_MAINTENANCE_SAVE || MotorControl_ConfigurationRevisionAvailable()))
    {
        request_epoch = lifecycle.snapshot.epoch;
        clear_running_data();
        if (FocMode_Admit(MotorControl, FOC, OnBoard_Encoder, mode))
        {
            RunState_BeginOperationIfRequested(mode);
            if (FocRunState_IsMaintenanceActive(mode))
            {
                *ticket = operation_epoch;
                completed_mode = Motor_Disable;
                result = MOTOR_CONTROL_ACCEPTED;
            }
            else
            {
                MotorControl->ModeNow = Motor_Disable;
            }
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool FocRunState_ReadMaintenance(uint32_t ticket, MotorMaintenanceProgress *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *progress = (MotorMaintenanceProgress){0};
    bool found = ticket != 0U && ticket == last_maintenance.ticket;
    if (found)
    {
        *progress = last_maintenance;
    }
    else if (last_maintenance_request.valid && ticket == last_maintenance_request.terminal.ticket &&
             ticket != 0U)
    {
        *progress = last_maintenance_request.terminal;
        found = true;
    }
    else if (ticket != 0U && ticket == operation_epoch)
    {
        progress->ticket = ticket;
        progress->kind = maintenance_kind(operation_mode);
        progress->result = lifecycle.snapshot.operation_result;
        progress->effects = lifecycle.snapshot.operation_effects;
        found = true;
    }
    critical_hw_exit(mask);
    return found;
}

MotorControlResult FocRunState_RequestIdentifiedMaintenance(const MotorMaintenanceRequest *request,
                                                            uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return MOTOR_CONTROL_INVALID;
    }
    *ticket = 0U;
    if (request == NULL || request->reserved != 0U || request->source_node == UINT8_MAX ||
        maintenance_mode(request->kind) == Motor_Disable)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result;
    if (last_maintenance_request.valid &&
        request->source_node == last_maintenance_request.request.source_node)
    {
        uint16_t delta = (uint16_t)(request->sequence - last_maintenance_request.request.sequence);
        if (delta == 0U)
        {
            result = request->kind == last_maintenance_request.request.kind
                         ? MOTOR_CONTROL_ACCEPTED
                         : MOTOR_CONTROL_CONFLICTING_RETRY;
            if (result == MOTOR_CONTROL_ACCEPTED)
            {
                *ticket = last_maintenance_request.ticket;
            }
            critical_hw_exit(mask);
            return result;
        }
        if (delta >= UINT16_C(0x8000))
        {
            critical_hw_exit(mask);
            return MOTOR_CONTROL_STALE_SEQUENCE;
        }
    }
    result = FocRunState_RequestMaintenance(request->kind, ticket);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        last_maintenance_request.request = *request;
        last_maintenance_request.terminal = (MotorMaintenanceProgress){0};
        last_maintenance_request.ticket = *ticket;
        last_maintenance_request.valid = true;
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult FocRunState_CancelMaintenance(uint32_t ticket)
{
    if (ticket == 0U)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result = MOTOR_CONTROL_INVALID;
    if (ticket == last_maintenance.ticket ||
        (last_maintenance_request.valid && ticket == last_maintenance_request.terminal.ticket))
    {
        /* 最近终态的重复取消只确认身份，绝不停止当前其他任务。 */
        result = MOTOR_CONTROL_ACCEPTED;
    }
    else if (ticket == operation_epoch && lifecycle.snapshot.operation != APP_OPERATION_NONE)
    {
        if (lifecycle.snapshot.operation_result == APP_OPERATION_CANCEL_REQUESTED)
        {
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (lifecycle.snapshot.state == APP_MAINTENANCE)
        {
            control_pending = false;
            if (outcome_latched)
            {
                if (pending_outcome.result == MOTOR_WORK_FAULT && outcome_epoch == operation_epoch)
                {
                    /* 同代故障先锁存，再取消；不能让CANCEL递增代号后丢弃故障。 */
                    MotorFault_Report(MotorControl, pending_outcome.error);
                }
                outcome_latched = false;
            }
            bool accepted = RunState_SendOperation(
                APP_EVENT_CANCEL, lifecycle.snapshot.operation, APP_EFFECT_UNKNOWN, 0U);
            MotorControl->ModeNow = Motor_Disable;
            result = accepted ? MOTOR_CONTROL_ACCEPTED : MOTOR_CONTROL_FAULT;
        }
        else
        {
            result = MOTOR_CONTROL_DENIED;
        }
    }
    critical_hw_exit(mask);
    return result;
}

static uint32_t bench_result(MotorControlResult result)
{
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        return FOC_BENCH_SPEED_ACCEPTED;
    }
    if (result == MOTOR_CONTROL_BUSY)
    {
        return FOC_BENCH_SPEED_BUSY;
    }
    if (result == MOTOR_CONTROL_INVALID || result == MOTOR_CONTROL_OUT_OF_RANGE)
    {
        return FOC_BENCH_SPEED_INVALID;
    }
    return FOC_BENCH_SPEED_DENIED;
}

static uint32_t submit_maintenance(uint32_t command, uint32_t seq)
{
    if (command == FOC_BENCH_CLEAR)
    {
        return bench_result(FocRunState_RequestClear());
    }
    MotorMaintenanceKind kind;
    switch (command)
    {
    case FOC_BENCH_ENCODER_LUT:
        kind = MOTOR_MAINTENANCE_ENCODER_LUT;
        break;
    case FOC_BENCH_ELECTRICAL_ZERO:
        kind = MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
        break;
    case FOC_BENCH_SAVE:
        kind = MOTOR_MAINTENANCE_SAVE;
        break;
    case FOC_BENCH_COGGING:
        kind = MOTOR_MAINTENANCE_COGGING;
        break;
    case FOC_BENCH_PHASE_RESISTANCE:
        kind = MOTOR_MAINTENANCE_PHASE_RESISTANCE;
        break;
    case FOC_BENCH_FRICTION:
        kind = MOTOR_MAINTENANCE_FRICTION;
        break;
    case FOC_BENCH_MECHANICAL_ZERO:
        kind = MOTOR_MAINTENANCE_MECHANICAL_ZERO;
        break;
    case FOC_BENCH_DEFAULTS:
        kind = MOTOR_MAINTENANCE_DEFAULTS;
        break;
    default:
        return FOC_BENCH_SPEED_INVALID;
    }
    uint32_t ticket;
    MotorControlResult result = FocRunState_RequestMaintenance(kind, &ticket);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        bench_speed.session_seq = seq;
    }
    return bench_result(result);
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
        result = bench_result(submitted);
        if (submitted == MOTOR_CONTROL_ACCEPTED)
        {
            pending_control_watch_seq = seq;
        }
    }
    else if (command >= FOC_BENCH_ENCODER_LUT && command <= FOC_BENCH_DEFAULTS)
    {
        result = worker_idle ? submit_maintenance(command, seq) : FOC_BENCH_SPEED_BUSY;
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
            MotorControl->ModeNow == Motor_Disable && ModeLast == Motor_Disable &&
            guards.phases_disabled && guards.control_idle && guards.operation_idle &&
            guards.maintenance_released && guards.self_test_passed && guards.start_permitted &&
            !MotorControl->isUseSensorless && Encoder_IsOnline(OnBoard_Encoder) &&
            (Encoder_GetCalibFlag(OnBoard_Encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
            isfinite(MotorControl->current_limit) && iq_limit_a <= MotorControl->current_limit &&
            isfinite(MotorControl->speed_limit) && speed_limit_rad_s <= MotorControl->speed_limit &&
            isfinite(MotorControl->speedAcc) && MotorControl->speedAcc > 0.0f &&
            isfinite(MotorControl->speedDec) && MotorControl->speedDec > 0.0f &&
            isfinite(MotorControl->speed_Kp) && MotorControl->speed_Kp >= 0.0f &&
            isfinite(MotorControl->speed_Ki) && MotorControl->speed_Ki >= 0.0f &&
            isfinite(Encoder_GetMecVel(OnBoard_Encoder)) &&
            fabsf(Encoder_GetMecVel(OnBoard_Encoder)) <= speed_limit_rad_s &&
            fabsf(FOC->Ia) < phase_limit_a && fabsf(FOC->Ib) < phase_limit_a &&
            fabsf(FOC->Ic) < phase_limit_a &&
            (MotorControl->axis_profile.magic == 0U ||
             (isfinite(MotorControl->axis_profile.maximum_speed_rad_s) &&
              speed_limit_rad_s <= MotorControl->axis_profile.maximum_speed_rad_s)) &&
            RunState_VbusInEnableWindow())
        {
            /* 沿用现有准入函数，登记仍发生在 owner；启动不继承旧 PI/目标。 */
            clear_running_data();
            request_epoch = lifecycle.snapshot.epoch;
            if (FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED)
            {
                bench_speed.saved_current_limit_a = MotorControl->current_limit;
                bench_speed.saved_speed_limit_rad_s = MotorControl->speed_limit;
                bench_speed.phase_limit_a = phase_limit_a;
                bench_speed.speed_limit_rad_s = speed_limit_rad_s;
                bench_speed.remaining_fast_ticks = duration_ms * BENCH_SPEED_FAST_TICKS_PER_MS;
                bench_speed.session_seq = seq;
                bench_speed.applied_seq = 0U;
                bench_speed.stop_reason = FOC_BENCH_SPEED_NO_STOP;
                bench_speed.stop_requested = false;
                MotorControl->current_limit = iq_limit_a;
                MotorControl->speed_limit = speed_limit_rad_s;
                MotorControl->speedRef = 0.0f;
                MotorControl->speedShadow = 0.0f;
                MotorControl->idRef = 0.0f;
                MotorControl->iqRef = 0.0f;
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
    if (MotorControl->ErrorNow == No_Error &&
        (!isfinite(FOC->Ia) || !isfinite(FOC->Ib) || !isfinite(FOC->Ic) ||
         fabsf(FOC->Ia) >= bench_speed.phase_limit_a ||
         fabsf(FOC->Ib) >= bench_speed.phase_limit_a ||
         fabsf(FOC->Ic) >= bench_speed.phase_limit_a))
    {
        bench_speed.stop_reason = FOC_BENCH_SPEED_PHASE_CURRENT;
        MotorFault_Report(MotorControl, Over_Current);
    }
    if (MotorControl->ErrorNow == No_Error &&
        (!isfinite(Encoder_GetMecVel(OnBoard_Encoder)) ||
         fabsf(Encoder_GetMecVel(OnBoard_Encoder)) > bench_speed.speed_limit_rad_s))
    {
        bench_speed.stop_reason = FOC_BENCH_SPEED_OVERSPEED;
        MotorFault_Report(MotorControl, MotorParam_Error);
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
        if (MotorControl->ModeNow == Motor_Disable && power_stage_hw_phases_disabled() &&
            (lifecycle.snapshot.state == APP_READY || lifecycle.snapshot.state == APP_FAULT ||
             lifecycle.snapshot.state == APP_FATAL))
        {
            MotorControl->current_limit = bench_speed.saved_current_limit_a;
            MotorControl->speed_limit = bench_speed.saved_speed_limit_rad_s;
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
    FocBenchSpeed.operation_epoch = operation_epoch;
    FocBenchSpeed.operation_mode = operation_mode;
    FocBenchSpeed.operation_result = lifecycle.snapshot.operation_result;
    FocBenchSpeed.operation_effects = lifecycle.snapshot.operation_effects;
    FocBenchSpeed.completed_mode = completed_mode;
    FocBenchSpeed.calibration_flags = Encoder_GetCalibFlag(OnBoard_Encoder);
    FocBenchSpeed.calibration_step = FocEncoderCalibration_GetStep();
    ++FocBenchSpeed.status_seq;
    critical_hw_exit(state);
}

void FocRunState_Init(MotorControl_TypeDef *motor,
                      FOC_TypeDef *foc,
                      PI_Controller_TypeDef *pi,
                      Encoder_TypeDef *encoder,
                      SensorlessStartup_TypeDef *startup)
{
    MotorControl = motor;
    FOC = foc;
    PI_Speed = pi;
    OnBoard_Encoder = encoder;
    SensorlessStartup = startup;
    ModeLast = Motor_Disable;
    AppLifecycle_Init(&lifecycle);
    operation_epoch = outcome_epoch = cancel_epoch = 0U;
    completed_mode = Motor_Disable;
    last_maintenance = (MotorMaintenanceProgress){0};
    memset(&last_maintenance_request, 0, sizeof(last_maintenance_request));
    request_epoch = lifecycle.snapshot.epoch;
    background_busy = stop_pending = configuration_reading = false;
    boot_stop_ticket = 0U;
    BootHwInfo boot_info = {0};
    BootHw_GetInfo(&boot_info);
    boot_confirmation_pending = boot_info.available && !boot_info.confirmed;
    boot_maintenance_only = boot_info.available && boot_info.image_role == YGI_ROLE_MAINTENANCE;
    supervisor_epoch = 0U;
    background_claim = (FocBackgroundClaim){0};
    operation_mode = Motor_Disable;
    save_finish = (uint8_t)SAVE_FINISH_NONE;
    recovery.overcurrent_ticks = 0U;
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

AppLifecycleState FocRunState_GetState(void)
{
    return lifecycle.snapshot.state;
}

bool FocRunState_RequestRemoteStop(void)
{
    uint32_t state = critical_hw_enter();
    RunState_BenchCancel(FOC_BENCH_SPEED_MANUAL_STOP);
    control_pending = false;
    if (outcome_latched)
    {
        if (pending_outcome.result == MOTOR_WORK_FAULT && outcome_epoch == FocRunState_WorkEpoch())
        {
            /* STOP撤销目标，但不能撤销当前worker已报告的故障。 */
            MotorFault_Report(MotorControl, pending_outcome.error);
        }
        outcome_latched = false;
    }
    MotorControl->ModeNow = Motor_Disable;
    stop_pending = true;
    critical_hw_exit(state);
    return true;
}

MotorControlResult FocRunState_RequestBootStop(uint64_t ticket)
{
    if (ticket == 0U)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result;
    if (boot_stop_ticket != 0U)
    {
        result = boot_stop_ticket == ticket ? MOTOR_CONTROL_ACCEPTED : MOTOR_CONTROL_BUSY;
    }
    else if (lifecycle.snapshot.state == APP_FATAL)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else
    {
        boot_stop_ticket = ticket;
        (void)FocRunState_RequestRemoteStop();
        result = MOTOR_CONTROL_ACCEPTED;
    }
    critical_hw_exit(mask);
    return result;
}

bool FocRunState_BootHandoffReady(uint64_t ticket)
{
    uint32_t mask = critical_hw_enter();
    AppLifecycleGuards guards = RunState_BuildGuards();
    bool ready = ticket != 0U && ticket == boot_stop_ticket &&
                 (lifecycle.snapshot.state == APP_READY || lifecycle.snapshot.state == APP_FAULT) &&
                 RunState_OwnerQuiescent(&guards);
    critical_hw_exit(mask);
    return ready;
}

bool FocRunState_ReadBootHealth(uint32_t *progress, bool *confirmation_ready)
{
    if (progress == NULL || confirmation_ready == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    AppLifecycleGuards guards = RunState_BuildGuards();
    *progress = supervisor_epoch;
    bool healthy = guards.samples_fresh && lifecycle.snapshot.state != APP_FATAL;
    *confirmation_ready = healthy && configuration_idle() && guards.self_test_passed &&
                          Encoder_IsOnline(OnBoard_Encoder);
    critical_hw_exit(mask);
    return healthy;
}

bool FocRunState_AcknowledgeBootConfirmation(uint64_t boot_epoch, uint32_t image_token)
{
    uint32_t mask = critical_hw_enter();
    BootHwInfo info = {0};
    BootHw_GetInfo(&info);
    bool confirmed = info.available && info.confirmed && !info.trial &&
                     info.boot_epoch == boot_epoch && info.image_token == image_token &&
                     boot_epoch != 0U && image_token != 0U;
    if (confirmed)
    {
        boot_confirmation_pending = false;
    }
    critical_hw_exit(mask);
    return confirmed;
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
            control_epoch = lifecycle.snapshot.epoch;
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
    if (Encoder_FeedbackRequired(MotorControl) &&
        Encoder_GetBadFrameStreak(OnBoard_Encoder) >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
    {
        MotorFault_Report(MotorControl, Encoder_Error);
    }

    /* 同时覆盖内部模式赋值，不只通信请求。 */
    if (!MotorControl->axis_profile_valid && MotorControl->ModeNow != Motor_Disable &&
        MotorControl->ModeNow != Save_Param && MotorControl->ModeNow != Clear_Error)
    {
        MotorFault_Report(MotorControl, MotorParam_Error);
    }
}

/**
 * @brief  上报本周期 worker 结果（20 kHz 快环调用，2 kHz 状态机消费）。
 * @param  outcome 模式 worker 的周期结果；MOTOR_WORK_RUNNING 不入箱。
 * @note 与消费端通过短临界区交换；只保留最新的非 RUNNING 结果。
 */
void FocRunState_PostOutcome(MotorWorkOutcome_TypeDef outcome, uint32_t epoch)
{
    uint32_t primask;

    ++fast_sample_epoch;
    if (outcome.result == MOTOR_WORK_RUNNING)
    {
        return;
    }
    primask = critical_hw_enter();
    if (epoch != FocRunState_WorkEpoch() || stop_pending)
    {
        critical_hw_exit(primask);
        return;
    }
    outcome_epoch = epoch;
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
    if (MotorControl->ErrorNow != No_Error)
    {
        RunState_BenchCancel(FOC_BENCH_SPEED_FAULT);
        control_pending = false;
    }
    if (MotorControl->ErrorNow != No_Error && !power_stage_hw_phases_disabled())
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
        if (outcome_epoch == FocRunState_WorkEpoch() && !stop_pending)
        {
            outcome = pending_outcome;
        }
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
    if (stop_pending)
    {
        stop_pending = false;
        RunState_RequestStop(true);
    }
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
    target = MotorControl->ModeNow;
    switch (outcome.result)
    {
    case MOTOR_WORK_SWITCH_MODE:
        completed_mode = operation_mode;
        if (outcome.power_off)
        {
            power_stage_hw_stop();
        }
        MotorControl->ModeNow = outcome.next_mode;
        target = outcome.next_mode;
        break;
    case MOTOR_WORK_STOP:
        RunState_BenchCancel(FOC_BENCH_SPEED_MANUAL_STOP);
        MotorControl->ModeNow = Motor_Disable;
        target = Motor_Disable;
        break;
    case MOTOR_WORK_FAULT:
        MotorFault_Report(MotorControl, outcome.error);
        break;
    default:
        break;
    }

    /* 1b. 清除请求：ModeNow==Clear_Error 由模式切换层登记，故障投影前捕获。 */
    clear_requested = MotorControl->ModeNow == Clear_Error;

    /* 2. 故障策略与指示：Save/Default 期间容错保留当前模式。 */
    if (MotorControl->ErrorNow != No_Error)
    {
        MotorControl->ModeNow = Motor_Disable;
        target = Motor_Disable;
        indicator_hw_set_led(true, (uint8_t)MotorControl->ErrorNow);
    }
    else
    {
        indicator_hw_set_led(false, (uint8_t)MotorControl->ModeNow);
    }

    /* 3. 启动链、维护会话、恢复证据与故障/清除处理。 */
    RunState_RunBootChain();
    operation_event = RunState_ServiceOperations(target, outcome.result == MOTOR_WORK_STOP);
    MotorFault_UpdateRecovery(&recovery, FOC, MotorControl->ErrorNow);
    if (MotorControl->ErrorNow != No_Error)
    {
        /* 核心故障动作立即请求关相，控制清理由下方停止路径按既有条件执行。 */
        (void)RunState_Send(
            APP_EVENT_FAULT, (uint32_t)MotorControl->ErrorNow, APP_CONTROL_NONE, false, false);
    }
    if (clear_requested && lifecycle.snapshot.state == APP_FAULT)
    {
        request_epoch = lifecycle.snapshot.epoch;
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
        MotorControl->ModeNow = Motor_Disable;
        target = Motor_Disable;
    }

    RunState_ServiceBenchRequest(outcome.result == MOTOR_WORK_RUNNING);
    target = MotorControl->ModeNow;

    /* 故障、worker 停机与外部失能先处理，随后消费目标；复核失败直接丢弃，不重试。 */
    apply_pending_control();

    /* 4. 目标迁移：停止、使能或退出运行态。 */
    needs_preparation = RunState_ModeNeedsPreparation(target);
    if (target == Motor_Disable &&
        !(lifecycle.snapshot.state == APP_MAINTENANCE && operation_mode == Calib_Anticogging &&
          (FocCogging_NeedsFinalize() || background_busy)))
    {
        if (ModeLast != Motor_Disable || lifecycle.snapshot.state == APP_STARTING ||
            !power_stage_hw_phases_disabled())
        {
            RunState_RequestStop(ModeLast != Motor_Disable || !power_stage_hw_phases_disabled());
        }
    }
    else if (RunState_ModeIsAutoStartable(target))
    {
        if (ModeLast == Motor_Disable && MotorControl->axis_profile_valid)
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
        /* 维护目标先退出运行，再由 READY 会话受理。 */
        RunState_RequestStop(false);
    }
    if (!operation_event)
    {
        RunState_BeginOperationIfRequested(target);
    }

    /* 5. 提交模式投影：STARTING 等待准备完成后才输出。 */
    defer_start = lifecycle.snapshot.state == APP_STARTING;

    if (!defer_start)
    {
        ModeLast = MotorControl->ModeNow;
    }

    MotorControl->ModeNow_f = MotorControl->ModeNow;
    MotorControl->ErrorNow_f = MotorControl->ErrorNow;
    RunState_PublishBenchState();
    ++supervisor_epoch;
}

uint32_t FocRunState_WorkEpoch(void)
{
    return lifecycle.snapshot.state == APP_MAINTENANCE ? operation_epoch : lifecycle.snapshot.epoch;
}
bool FocRunState_BackgroundValid(const FocBackgroundClaim *claim)
{
    return claim != NULL && background_busy && claim->epoch == background_claim.epoch &&
           claim->kind == background_claim.kind && claim->epoch == operation_epoch &&
           claim->epoch == lifecycle.snapshot.epoch &&
           lifecycle.snapshot.state == APP_MAINTENANCE && !stop_pending &&
           MotorControl->ErrorNow == No_Error && power_stage_hw_phases_disabled();
}
bool FocRunState_BeginBackground(FocBackgroundClaim *claim)
{
    uint32_t mask = critical_hw_enter();
    bool accepted = false;
    if (claim != NULL && !background_busy && !stop_pending && !outcome_latched &&
        lifecycle.snapshot.state == APP_MAINTENANCE && MotorControl->ErrorNow == No_Error &&
        power_stage_hw_phases_disabled() && FocEncoderCalibration_IsReleased())
    {
        FocBackgroundKind kind = lifecycle.snapshot.operation == APP_OPERATION_SAVE
                                     ? FOC_BACKGROUND_SAVE
                                 : operation_mode == Calib_Anticogging && FocCogging_BeginFinalize()
                                     ? FOC_BACKGROUND_COGGING
                                     : FOC_BACKGROUND_NONE;
        if (kind != FOC_BACKGROUND_NONE &&
            (kind != FOC_BACKGROUND_SAVE || save_finish == SAVE_FINISH_NONE))
        {
            background_claim = (FocBackgroundClaim){operation_epoch, kind};
            background_busy = true;
            *claim = background_claim;
            accepted = true;
        }
    }
    critical_hw_exit(mask);
    return accepted;
}
void FocRunState_FinishBackground(const FocBackgroundClaim *claim, bool success)
{
    uint32_t mask = critical_hw_enter();
    if (claim == NULL || !background_busy || claim->epoch != background_claim.epoch ||
        claim->kind != background_claim.kind)
    {
        critical_hw_exit(mask);
        return;
    }
    bool current = FocRunState_BackgroundValid(claim);
    background_busy = false;
    if (claim->kind == FOC_BACKGROUND_COGGING)
    {
        FocCogging_ReleaseFinalize(success && current);
    }
    if (current)
    {
        if (claim->kind == FOC_BACKGROUND_SAVE)
        {
            if (!success)
            {
                MotorFault_Report(MotorControl, MotorParam_Error);
            }
            FocCogging_SaveResult(success);
            save_finish = success ? SAVE_FINISH_COMMITTED : SAVE_FINISH_FAILED;
            MotorControl->ModeNow = Motor_Disable;
        }
        else if (success)
        {
            MotorControl->ModeNow = Save_Param;
        }
        else
        {
            MotorFault_Report(MotorControl, CoggingCalibration_Error);
        }
    }
    else if (claim->kind == FOC_BACKGROUND_SAVE)
    {
        FocCogging_SaveResult(false);
    }
    critical_hw_exit(mask);
}

void FocRunState_ServiceCompensation(void)
{
    uint32_t mask = critical_hw_enter();
    bool permitted =
        boot_stop_ticket == 0U && !boot_confirmation_pending && !background_busy && !stop_pending &&
        !outcome_latched && MotorControl->ErrorNow == No_Error &&
        (lifecycle.snapshot.state == APP_READY || lifecycle.snapshot.state == APP_RUNNING);
    if (permitted)
    {
        configuration_reading = true;
    }
    critical_hw_exit(mask);
    if (!permitted)
    {
        return;
    }
    FocCogging_Service(MotorControl, OnBoard_Encoder);
    mask = critical_hw_enter();
    configuration_reading = false;
    critical_hw_exit(mask);
}
