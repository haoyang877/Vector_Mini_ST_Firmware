#include "motor_state_machine.h"

#include <math.h>
#include <string.h>

#include "angle_feedback.h"
#include "motor_state_internal.h"
#include "bus_voltage_profile.h"
#include "critical_hw.h"
#include "foc_errhandle.h"
#include "foc_algorithm.h"
#include "motor_maintenance.h"
#include "motor_execution.h"
#include "foc_sensing.h"
#include "indicator_hw.h"
#include "mcu_temperature.h"
#include "motor_sensing.h"
#include "motor_axis.h"
#include "foc_cogging_calibration.h"
#include "foc_pid.h"
#include "encoder_calibration_task.h"
#include "phase_resistance_task.h"
#include "foc_sensorless_run.h"
#include "power_stage_hw.h"
#include "control_math.h"
#include "time_hw.h"
#include "parameter_value_service.h"

/* 普通自检仍为500ms；首次上电额外允许历史1秒零点采样，监督停摆不在此保证内。 */
#define MOTOR_SELF_TEST_TIMEOUT_MS 500U
#define MOTOR_SELF_TEST_STABLE_TICKS 4U

/* 唯一运行上下文：前台受理、20 kHz 结果与 2 kHz 监督通过原有短临界区交接。
 * 规则不拥有独立实例；具体维护任务是分类和取消动作的唯一身份来源。 */
typedef struct
{
    MotorStateRulesContext rules;
    MotorControlState *motor;
    FocState *foc;
    PiController *speed_pi;
    EncoderState *rotor_encoder;
    SensorlessStartupState *sensorless_startup;
    bool enable_pending, maintenance_runnable, save_requested;
    volatile uint8_t save_finish;
    uint32_t operation_epoch, outcome_epoch, cancel_epoch;
    MotorMaintenanceProgress last_maintenance;
    struct
    {
        MotorMaintenanceRequest request;
        MotorMaintenanceProgress terminal;
        uint32_t ticket;
    } last_maintenance_request;
    FocBackgroundClaim background_claim;
    bool background_busy, stop_pending, configuration_reading;
    uint64_t boot_stop_ticket;
    bool boot_maintenance_only;
    MotorFaultRecovery recovery;
    MotorTaskResult pending_outcome;
    volatile bool has_latched_outcome;
    MotorControlCommand pending_control;
    uint32_t control_epoch;
    volatile bool has_pending_control;
    MotorControlProgress control_progress;
    MotorControlStartProgress latest_start, active_start;
    uint16_t stop_sequence;
    uint8_t remote_source;
    bool remote_source_valid, stop_sequence_valid;
    volatile uint32_t fast_sample_epoch;
    uint32_t observed_fast_sample_epoch;
    bool fast_samples_fresh;
    uint32_t supervisor_epoch;
    uint32_t self_test_started_ms, self_test_timeout_ms;
    uint8_t self_test_stable_ticks;
    bool self_test_timing_started;
} MotorStateContext;

/* 主状态机全局符号供 Watch 观测；状态转换仍由本模块的受理和监督入口执行。 */
MotorStateContext s_context;

/* 前台保存结果由监督拍消费；COMMITTED 要求既有回读验证成功。 */
enum
{
    SAVE_FINISH_NONE = 0,
    SAVE_FINISH_COMMITTED,
    SAVE_FINISH_FAILED
};

static bool state_vbus_in_enable_window(void);

/* 仅汇总既有模块事实，不做 I/O、标定、CRC 或转矩试验。 */
static bool bus_measurements_finite(void)
{
    return isfinite(s_context.foc->bus_voltage_v) &&
           isfinite(s_context.foc->bus_voltage_filtered_v);
}

static bool phase_currents_finite(void)
{
    return isfinite(s_context.foc->phase_a_current_a) &&
           isfinite(s_context.foc->phase_b_current_a) && isfinite(s_context.foc->phase_c_current_a);
}

static bool temperature_measurement_fresh(void)
{
    return MotorTemperature_IsFresh() && isfinite(s_context.foc->temperature_c);
}

/* 感测自检：允许无功率母线，不要求标定；运行保护与启相窗口另有契约。 */
static uint32_t read_self_test_measurements(void)
{
    uint32_t checks = s_context.fast_samples_fresh ? MOTOR_SELF_TEST_SAMPLES_PROGRESSING : 0U;
    if (bus_measurements_finite() && s_context.foc->bus_voltage_v >= 0.0f &&
        s_context.foc->bus_voltage_filtered_v >= 0.0f &&
        s_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        checks |= MOTOR_SELF_TEST_BUS_VALID;
    }
    if (MotorCurrentOffsets_Valid(s_context.motor) && phase_currents_finite())
    {
        checks |= MOTOR_SELF_TEST_CURRENTS_VALID;
    }
    if (temperature_measurement_fresh() && s_context.foc->temperature_c < MCU_TEMPERATURE_TRIP_C)
    {
        checks |= MOTOR_SELF_TEST_TEMPERATURE_VALID;
    }
    if (Encoder_IsOnline(s_context.rotor_encoder) &&
        Encoder_GetBadFrameStreak(s_context.rotor_encoder) == 0U)
    {
        checks |= MOTOR_SELF_TEST_ENCODER_VALID;
    }
    return checks;
}

/* 三组事实：感测、参数、停机确认；复用当前事实，仍保留八个线上诊断位。 */
static uint32_t build_self_test_checks(const MotorStateFacts *facts)
{
    uint32_t checks = read_self_test_measurements();
    if (facts->configuration_valid)
    {
        checks |= MOTOR_SELF_TEST_CONFIGURATION_VALID;
    }
    if (facts->phases_disabled)
    {
        checks |= MOTOR_SELF_TEST_OUTPUT_DISABLED;
    }
    if (facts->control_idle && facts->operation_idle && facts->maintenance_released &&
        !s_context.configuration_reading)
    {
        checks |= MOTOR_SELF_TEST_RESOURCES_RELEASED;
    }
    return checks;
}

/* 只能终结尚未提交的远程身份；STOP/故障不能把旧终态改写成新成功。 */
static void reject_pending_control(MotorControlResult reason)
{
    s_context.has_pending_control = false;
    if (s_context.control_progress.state == MOTOR_CONTROL_PROGRESS_PENDING)
    {
        s_context.control_progress.state = MOTOR_CONTROL_PROGRESS_REJECTED;
        s_context.control_progress.result = reason;
    }
}

/* 只改变尚未确认启相的原身份；已启动结果保留为历史，不表示当前仍有输出。 */
static void reject_pending_start(MotorControlResult reason)
{
    if (s_context.active_start.state != MOTOR_CONTROL_START_PENDING)
    {
        return;
    }
    s_context.active_start.state = MOTOR_CONTROL_START_REJECTED;
    s_context.active_start.result = reason;
    if (s_context.latest_start.state != MOTOR_CONTROL_START_NONE &&
        s_context.latest_start.request.source_node == s_context.active_start.request.source_node &&
        s_context.latest_start.request.sequence == s_context.active_start.request.sequence)
    {
        s_context.latest_start = s_context.active_start;
    }
}

static bool same_start_identity(const MotorControlStartProgress *progress,
                                const MotorControlStartRequest *request)
{
    return progress->state != MOTOR_CONTROL_START_NONE &&
           progress->request.source_node == request->source_node &&
           progress->request.sequence == request->sequence;
}

/* 16 位半范围序号：0 是相同身份，1..32767 是新请求，其余是陈旧请求。 */
static bool start_sequence_newer(uint16_t sequence, uint16_t baseline)
{
    uint16_t delta = (uint16_t)(sequence - baseline);
    return delta != 0U && delta < UINT16_C(0x8000);
}

/* 停机清理：清零参考、影子和滤波量；任务取消独立按当前身份执行。 */
static void clear_running_data(void)
{
    s_context.motor->id_target_a = 0.0f;
    s_context.motor->iq_target_a = 0.0f;
    s_context.motor->vq_target_v = 0.0f;
    s_context.motor->speed_target_rad_s = 0.0f;
    s_context.motor->speed_ramped_rad_s = 0.0f;
    s_context.motor->position_reference_rad = 0.0f;
    s_context.motor->position_target_reached = false;
    s_context.motor->position_velocity_filtered_rad_s = 0.0f;
    s_context.foc->id_a = 0.0f;
    s_context.foc->iq_a = 0.0f;
    FOC_CurrentController_Reset(s_context.foc);
    PiController_Reset(s_context.speed_pi);
    SensorlessStartup_Reset(s_context.sensorless_startup);
}

/* 状态准入：锁定期、故障、实际输出与反馈许可；目标格式和范围由控制模块拥有。 */
static MotorControlResult admit_control_target(const MotorControlCommand *command)
{
    if (s_context.boot_stop_ticket != 0U || s_context.boot_maintenance_only)
    {
        return MOTOR_CONTROL_DENIED;
    }
    MotorControlResult target_result = MotorExecution_ValidateTarget(
        command, s_context.motor, s_context.foc, s_context.rotor_encoder);
    /* 保持原拒绝优先级：格式错误先于故障，范围结果后于运行准入。 */
    if (target_result == MOTOR_CONTROL_INVALID)
    {
        return target_result;
    }
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (power_stage_hw_phases_disabled() || s_context.rules.state != MOTOR_STATE_RUNNING ||
        s_context.motor->control_mode != command->mode ||
        !s_context.motor->application.axis_profile_valid ||
        s_context.motor->uses_sensorless_feedback || !Encoder_IsOnline(s_context.rotor_encoder) ||
        (Encoder_GetCalibFlag(s_context.rotor_encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        !state_vbus_in_enable_window() || !isfinite(s_context.foc->bus_voltage_v) ||
        s_context.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        return MOTOR_CONTROL_DENIED;
    }
    return target_result;
}

/* 监督拍消费单槽目标邮箱：会话失效、STOP 或复核失败即丢弃，受理后写入对应参考。 */
static void apply_pending_control(void)
{
    /* 防止快环在状态复核与目标写入之间锁存故障；临界区只做有界值检查和一次赋值。 */
    uint32_t state = critical_hw_enter();
    if (!s_context.has_pending_control)
    {
        critical_hw_exit(state);
        return;
    }
    MotorControlResult result;
    if (s_context.control_epoch != s_context.rules.epoch || s_context.stop_pending)
    {
        result = s_context.motor->fault_code != MOTOR_FAULT_NONE ? MOTOR_CONTROL_FAULT
                                                                 : MOTOR_CONTROL_DENIED;
    }
    else
    {
        result = admit_control_target(&s_context.pending_control);
    }
    if (result != MOTOR_CONTROL_ACCEPTED)
    {
        reject_pending_control(result);
        critical_hw_exit(state);
        return;
    }
    s_context.has_pending_control = false;
    switch (s_context.pending_control.mode)
    {
    case MOTOR_CONTROL_CURRENT:
        s_context.motor->iq_target_a = s_context.pending_control.iq_a;
        break;
    case MOTOR_CONTROL_SPEED:
        s_context.motor->speed_target_rad_s = s_context.pending_control.speed_rad_s;
        break;
    case MOTOR_CONTROL_POSITION:
        s_context.motor->position_target_rad = s_context.pending_control.position_rad;
        break;
    case MOTOR_CONTROL_VQ:
        s_context.motor->vq_target_v = s_context.pending_control.vq_v;
        break;
    default:
        break;
    }
    if (s_context.pending_control.source_node != 0U)
    {
        s_context.control_progress.state = MOTOR_CONTROL_PROGRESS_APPLIED;
        s_context.control_progress.result = MOTOR_CONTROL_ACCEPTED;
    }
    critical_hw_exit(state);
}

/* 需要编码器反馈的模式在坏帧超限后立即置编码器故障。 */
static bool encoder_feedback_required(const MotorControlState *motor)
{
    if (motor->control_mode != MOTOR_CONTROL_NONE)
    {
        return motor->control_mode != MOTOR_CONTROL_CURRENT || !motor->uses_sensorless_feedback;
    }
    return s_context.rules.state == MOTOR_STATE_MAINTENANCE &&
           s_context.rules.maintenance_kind != MOTOR_MAINTENANCE_PHASE_RESISTANCE &&
           s_context.rules.maintenance_kind != MOTOR_MAINTENANCE_SAVE &&
           s_context.rules.maintenance_kind != MOTOR_MAINTENANCE_DEFAULTS;
}

/* 位置/速度模式首个周期先在功率输出关闭时校验控制器，下一快周期再使能。 */
/* 同一监督拍共用采样进展，不能让多次核心 Step 把同一帧误判为陈旧。 */
static void state_update_sample_progress(void)
{
    uint32_t state = critical_hw_enter();
    uint32_t current_epoch = s_context.fast_sample_epoch;
    critical_hw_exit(state);
    s_context.fast_samples_fresh = current_epoch != s_context.observed_fast_sample_epoch;
    s_context.observed_fast_sample_epoch = current_epoch;
}

/* 基础健康不要求编码器标定；各模式的标定准入仍由既有模式校验拥有。 */
static MotorStateFacts read_state_conditions(void)
{
    MotorStateFacts facts;
    uint32_t state = critical_hw_enter();
    MotorControlMode mode = s_context.motor->control_mode;

    facts.active_faults = (uint32_t)s_context.motor->fault_code;
    facts.phases_disabled = power_stage_hw_phases_disabled();
    facts.control_idle = !s_context.has_pending_control && !s_context.has_latched_outcome;
    /* 取消确认必须同时取得每个 worker 和前台的实际释放证明。 */
    facts.operation_idle = !s_context.background_busy && !s_context.has_latched_outcome;
    facts.maintenance_released =
        !s_context.background_busy &&
        (!s_context.configuration_reading || s_context.rules.state == MOTOR_STATE_RUNNING) &&
        MotorMaintenance_IsReleased();
    facts.inputs_valid = s_context.fast_samples_fresh && bus_measurements_finite() &&
                         phase_currents_finite() && temperature_measurement_fresh();
    facts.configuration_valid = ParameterValue_ConfigurationValid(s_context.motor);
    /* tested 是唯一完成记录；稳定计数只供 SELF_TEST_DONE 提交时证明窗口。 */
    bool stable_self_test = s_context.rules.state == MOTOR_STATE_SELF_TEST &&
                            s_context.self_test_stable_ticks == MOTOR_SELF_TEST_STABLE_TICKS;
    facts.self_test_ready = (s_context.rules.tested || stable_self_test) &&
                            facts.configuration_valid && facts.inputs_valid &&
                            facts.active_faults == 0U;
    facts.start_permitted =
        s_context.motor->application.axis_profile_valid && !s_context.boot_maintenance_only;
    if (mode != MOTOR_CONTROL_NONE)
    {
        facts.start_permitted = facts.start_permitted && s_context.motor->configuration_confirmed &&
                                facts.configuration_valid;
    }
    if (s_context.rules.state != MOTOR_STATE_RUNNING)
    {
        facts.start_permitted = facts.start_permitted && state_vbus_in_enable_window() &&
                                isfinite(s_context.foc->bus_voltage_v) &&
                                s_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
    }
    if (encoder_feedback_required(s_context.motor))
    {
        facts.start_permitted = facts.start_permitted && Encoder_IsOnline(s_context.rotor_encoder);
    }
    if (mode == MOTOR_CONTROL_SPEED || mode == MOTOR_CONTROL_POSITION || mode == MOTOR_CONTROL_VQ ||
        (s_context.rules.state == MOTOR_STATE_MAINTENANCE &&
         (s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_COGGING ||
          s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_FRICTION)) ||
        (mode == MOTOR_CONTROL_CURRENT && !s_context.motor->uses_sensorless_feedback))
    {
        facts.start_permitted =
            facts.start_permitted &&
            (Encoder_GetCalibFlag(s_context.rotor_encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL;
    }
    facts.operation_permitted = s_context.motor->application.axis_profile_valid &&
                                ((s_context.rules.maintenance_active &&
                                  MotorStateRules_Classify(s_context.rules.maintenance_kind) ==
                                      MOTOR_MAINTENANCE_CLASS_CALIBRATION) ||
                                 power_stage_hw_phases_disabled());
    /* 位置/速度模式需要外环控制器就绪后才确认启动，其余模式可同拍确认。 */
    facts.startup_confirmed =
        (mode != MOTOR_CONTROL_POSITION && mode != MOTOR_CONTROL_SPEED) || MotorOuterLoop_IsReady();
    /* 本地事件在提交临界区使用当前代际；异步结果仍保留创建时的身份。 */
    facts.request_fresh = !s_context.stop_pending;
    critical_hw_exit(state);
    return facts;
}

/* 功率事实来自平台回读；取消控制数据与数字关相请求分别处理。 */
static void state_apply_actions(uint32_t actions, MotorMaintenanceKind cancelled_kind)
{
    /* 关断先于功能取消；即使取消函数失效也不延后关相请求。 */
    if ((actions & MOTOR_STATE_ACTION_DISABLE_POWER) && !power_stage_hw_phases_disabled())
    {
        power_stage_hw_stop();
    }
    if (actions & MOTOR_STATE_ACTION_CANCEL_CONTROL)
    {
        s_context.enable_pending = false;
        s_context.motor->control_mode = MOTOR_CONTROL_NONE;
        clear_running_data();
    }
    if (actions & MOTOR_STATE_ACTION_CANCEL_OPERATION)
    {
        s_context.maintenance_runnable = false;
        if (s_context.rules.operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED ||
            s_context.rules.operation_result == MOTOR_MAINTENANCE_FAILED ||
            s_context.rules.state == MOTOR_STATE_FAULT ||
            s_context.rules.state == MOTOR_STATE_FATAL)
        {
            s_context.cancel_epoch = s_context.rules.epoch;
            s_context.save_requested = false;
            MotorMaintenance_Cancel(
                cancelled_kind, s_context.foc, s_context.motor, s_context.speed_pi);
        }
    }
    if (actions & MOTOR_STATE_ACTION_REVOKE_REQUESTS)
    {
        reject_pending_control(s_context.motor->fault_code != MOTOR_FAULT_NONE
                                   ? MOTOR_CONTROL_FAULT
                                   : MOTOR_CONTROL_DENIED);
    }
    if (actions & MOTOR_STATE_ACTION_SELF_TEST)
    {
        if (actions & MOTOR_STATE_ACTION_FAULT_CLEARED)
        {
            s_context.self_test_timeout_ms = MOTOR_SELF_TEST_TIMEOUT_MS;
        }
        s_context.self_test_started_ms = time_hw_now_ms();
        s_context.self_test_timing_started = true;
        s_context.self_test_stable_ticks = 0U;
    }
}

/* 转换已锁入故障但源模块尚未报告时，按既有顺序提供具体诊断，不改变恢复策略。 */
static void publish_transition_fault(void)
{
    if ((s_context.rules.state == MOTOR_STATE_FAULT ||
         s_context.rules.state == MOTOR_STATE_FATAL) &&
        s_context.motor->fault_code == MOTOR_FAULT_NONE)
    {
        MotorFaultCode error = MOTOR_FAULT_CONTROL_OVERRUN;
        if (!MotorTemperature_IsFresh())
        {
            error = MOTOR_FAULT_TEMPERATURE_SENSOR;
        }
        else if (!s_context.motor->application.axis_profile_valid)
        {
            error = MOTOR_FAULT_INVALID_PARAMETERS;
        }
        else if (!state_vbus_in_enable_window())
        {
            error = isfinite(s_context.foc->bus_voltage_filtered_v) &&
                            s_context.foc->bus_voltage_filtered_v < BUS_VOLTAGE_ENABLE_MIN_V
                        ? MOTOR_FAULT_UNDERVOLTAGE
                        : MOTOR_FAULT_OVERVOLTAGE;
        }
        else if (encoder_feedback_required(s_context.motor) &&
                 !Encoder_IsOnline(s_context.rotor_encoder))
        {
            error = MOTOR_FAULT_ENCODER;
        }
        MotorFault_Report(s_context.motor, error);
    }
}

/* 唯一事件提交路径；调用者显式保留完成身份，CLEAR 只接收已核对的恢复事实。
 * 普通受理只进入 STARTING；真实启相仍由监督拍末尾按原顺序复核后执行。 */
static bool dispatch_state_event(uint32_t events,
                                 MotorMaintenanceEffect effects,
                                 uint32_t faults,
                                 const MotorStateFacts *recovery_facts)
{
    uint32_t mask = critical_hw_enter();
    /* 完成使用创建时保存的身份；禁止用当前规则 epoch 重建维护完成身份。
     * worker/前台结果在进入本路径前仍须通过原始邮箱或认领身份复核。 */
    uint32_t completion_epoch = s_context.rules.epoch;
    if (events == MOTOR_STATE_EVENT_CANCEL_DONE)
    {
        completion_epoch = s_context.cancel_epoch;
    }
    else if (events & (MOTOR_STATE_EVENT_BEGIN_OPERATION | MOTOR_STATE_EVENT_OPERATION_DONE |
                       MOTOR_STATE_EVENT_OPERATION_FAILED | MOTOR_STATE_EVENT_CANCEL))
    {
        completion_epoch = s_context.operation_epoch;
    }
    MotorStateEvent event = {.events = events,
                             .faults = faults,
                             .request_epoch = s_context.rules.epoch,
                             .completion_epoch = completion_epoch,
                             .control_mode = s_context.motor->control_mode,
                             .maintenance_kind = s_context.rules.maintenance_kind,
                             .effects = effects};
    MotorStateFacts facts = recovery_facts != NULL ? *recovery_facts : read_state_conditions();
    MotorMaintenanceKind previous_kind = s_context.rules.maintenance_kind;
    MotorStateDecision decision = MotorStateRules_Step(&s_context.rules, &event, &facts);

    if (decision.rejection == MOTOR_STATE_ACCEPTED &&
        (event.events == MOTOR_STATE_EVENT_OPERATION_DONE ||
         event.events == MOTOR_STATE_EVENT_OPERATION_FAILED ||
         event.events == MOTOR_STATE_EVENT_CANCEL_DONE))
    {
        s_context.last_maintenance = (MotorMaintenanceProgress){s_context.operation_epoch,
                                                                previous_kind,
                                                                s_context.rules.operation_result,
                                                                s_context.rules.operation_effects};
        if (s_context.last_maintenance_request.ticket != 0U &&
            s_context.last_maintenance_request.ticket == s_context.operation_epoch)
        {
            s_context.last_maintenance_request.terminal = s_context.last_maintenance;
        }
    }
    publish_transition_fault();
    if (event.events == MOTOR_STATE_EVENT_CLEAR && decision.rejection == MOTOR_STATE_ACCEPTED &&
        s_context.rules.state == MOTOR_STATE_SELF_TEST)
    {
        s_context.motor->fault_code = MOTOR_FAULT_NONE;
        s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    }
    state_apply_actions(decision.actions, previous_kind);
    critical_hw_exit(mask);
    return decision.rejection == MOTOR_STATE_ACCEPTED;
}

/* 启动链：BOOT → SELF_TEST → READY（无对外动作）。 */
static void advance_boot_and_self_test(void)
{
    uint32_t now_ms = time_hw_now_ms();
    if (!s_context.self_test_timing_started)
    {
        s_context.self_test_started_ms = now_ms;
        s_context.self_test_timing_started = true;
    }
    if (!power_stage_hw_phases_disabled())
    {
        power_stage_hw_stop();
        if (!power_stage_hw_phases_disabled())
        {
            (void)dispatch_state_event(MOTOR_STATE_EVENT_FATAL,
                                       MOTOR_MAINTENANCE_EFFECT_UNKNOWN,
                                       MOTOR_STATE_FAULT_INTERNAL,
                                       NULL);
            return;
        }
    }
    if (s_context.rules.state == MOTOR_STATE_BOOT)
    {
        (void)dispatch_state_event(
            MOTOR_STATE_EVENT_BOOT_DONE, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
    }
    now_ms = time_hw_now_ms();
    if ((uint32_t)(now_ms - s_context.self_test_started_ms) >= s_context.self_test_timeout_ms)
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_SELF_TEST_TIMEOUT);
        (void)dispatch_state_event(MOTOR_STATE_EVENT_FAULT,
                                   MOTOR_MAINTENANCE_EFFECT_UNKNOWN,
                                   (uint32_t)s_context.motor->fault_code,
                                   NULL);
        return;
    }
    if (s_context.rules.state == MOTOR_STATE_SELF_TEST)
    {
        MotorStateFacts facts = read_state_conditions();
        uint32_t checks = build_self_test_checks(&facts);
        if (checks == MOTOR_SELF_TEST_ALL && s_context.motor->fault_code == MOTOR_FAULT_NONE)
        {
            if (s_context.self_test_stable_ticks < MOTOR_SELF_TEST_STABLE_TICKS)
            {
                ++s_context.self_test_stable_ticks;
            }
            if (s_context.self_test_stable_ticks == MOTOR_SELF_TEST_STABLE_TICKS)
            {
                (void)dispatch_state_event(
                    MOTOR_STATE_EVENT_SELF_TEST_DONE, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
                return;
            }
        }
        else
        {
            s_context.self_test_stable_ticks = 0U;
        }
    }
}

/* 停止请求：关相并尝试确认；确认可能延后一拍（quiet 需要功率级真实关闭）。 */
static void state_request_stop(void)
{
    (void)dispatch_state_event(MOTOR_STATE_EVENT_STOP, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
    (void)dispatch_state_event(0U, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
}

/* 母线电压处于重新使能窗口 [25.6, 33.8] V：过/欠压故障的源恢复判据。 */
static bool state_vbus_in_enable_window(void)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return s_context.foc->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V &&
           s_context.foc->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V;
}

/* 会话服务：SAVE 结果上报、ZERO/DEFAULTS/标定派生完成、被打断会话的实际释放确认。
 * 完成使用创建时身份，取消确认等待全部 worker 和前台资源归还。
 * 返回 true 表示本拍已上报会话事件（完成/失败/取消确认），调用方推迟一拍再开新会话。 */
static bool finish_maintenance_or_release(void)
{
    MotorState state = s_context.rules.state;
    if (state == MOTOR_STATE_MAINTENANCE &&
        s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_SAVE)
    {
        if (s_context.save_finish != SAVE_FINISH_NONE)
        {
            bool success = s_context.save_finish == SAVE_FINISH_COMMITTED;
            s_context.save_finish = SAVE_FINISH_NONE;
            return dispatch_state_event(
                success ? MOTOR_STATE_EVENT_OPERATION_DONE : MOTOR_STATE_EVENT_OPERATION_FAILED,
                success ? MOTOR_MAINTENANCE_EFFECT_COMMITTED : MOTOR_MAINTENANCE_EFFECT_UNKNOWN,
                success ? 0U : (uint32_t)MOTOR_FAULT_INVALID_PARAMETERS,
                NULL);
        }
    }
    else if ((state == MOTOR_STATE_STOPPING || state == MOTOR_STATE_FAULT) &&
             s_context.rules.maintenance_active &&
             (s_context.rules.operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED ||
              s_context.rules.operation_result == MOTOR_MAINTENANCE_FAILED) &&
             read_state_conditions().maintenance_released && !s_context.background_busy)
    {
        return dispatch_state_event(
            MOTOR_STATE_EVENT_CANCEL_DONE, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
    }
    return false;
}

/* READY 受理实际任务；每次显式请求创建独立身份。 */
static bool begin_operation(MotorMaintenanceKind kind)
{
    if (s_context.rules.state != MOTOR_STATE_READY || s_context.boot_stop_ticket != 0U)
    {
        return false;
    }
    MotorFaultCode admission =
        MotorMaintenance_Admit(kind, s_context.motor, s_context.foc, s_context.rotor_encoder);
    if (admission != MOTOR_FAULT_NONE)
    {
        return false;
    }
    s_context.rules.maintenance_kind = kind;
    if (!dispatch_state_event(
            MOTOR_STATE_EVENT_BEGIN_OPERATION, MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED, 0U, NULL))
    {
        s_context.rules.maintenance_kind = MOTOR_MAINTENANCE_NONE;
        return false;
    }
    s_context.operation_epoch = s_context.rules.epoch;
    if (kind == MOTOR_MAINTENANCE_PHASE_RESISTANCE)
    {
        /* 保留已完成诊断直到新请求；新会话显式清除该功能的旧终态。 */
        PhaseResistanceTask_Cancel(s_context.foc, s_context.motor);
    }
    s_context.maintenance_runnable = true;
    /* 齿槽需要先由功能准备零矢量，再返回 PREPARED；其余现有标定保持启相条件。 */
    if (kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO ||
        kind == MOTOR_MAINTENANCE_PHASE_RESISTANCE || kind == MOTOR_MAINTENANCE_FRICTION)
    {
        MotorStateFacts facts = read_state_conditions();
        if (!s_context.stop_pending && facts.start_permitted && facts.inputs_valid &&
            facts.active_faults == 0U)
        {
            power_stage_hw_start();
            if (!power_stage_hw_phases_disabled())
            {
                return true;
            }
        }
        MotorFault_Report(s_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        (void)dispatch_state_event(MOTOR_STATE_EVENT_FAULT,
                                   MOTOR_MAINTENANCE_EFFECT_UNKNOWN,
                                   (uint32_t)s_context.motor->fault_code,
                                   NULL);
        return false;
    }
    return true;
}

/* 运行所有者已静止：模式已失能、无目标/占用、资源已释放；调用方负责状态与角色前提。 */
static bool output_and_owner_released(const MotorStateFacts *facts)
{
    return s_context.motor->control_mode == MOTOR_CONTROL_NONE && !s_context.stop_pending &&
           !s_context.configuration_reading && !s_context.rules.maintenance_active &&
           facts->phases_disabled && facts->control_idle && facts->operation_idle &&
           facts->maintenance_released;
}

/* READY 且无票据、无故障、无占用：停机配置写入的静态前提。 */
static bool configuration_idle(const MotorStateFacts *facts)
{
    return s_context.boot_stop_ticket == 0U && s_context.rules.state == MOTOR_STATE_READY &&
           s_context.motor->fault_code == MOTOR_FAULT_NONE && output_and_owner_released(facts);
}

bool MotorStateMachine_ConfigurationAllowed(void)
{
    MotorStateFacts facts = read_state_conditions();
    return configuration_idle(&facts);
}

/* 调用方已持有临界区；只读共同占用标志，故障优先级和资源释放由各入口判断。 */
static bool request_busy(void)
{
    return s_context.boot_stop_ticket != 0U || s_context.save_requested ||
           s_context.background_busy || s_context.configuration_reading ||
           s_context.has_latched_outcome || s_context.has_pending_control ||
           s_context.rules.maintenance_active;
}

/* 公开入口与状态反馈共用准入；拒绝只是准备未满足，不清目标、不锁故障。 */
static uint32_t state_start_blockers(const MotorStateFacts *facts)
{
    uint32_t reasons = 0U;
    if (s_context.motor->uses_sensorless_feedback)
    {
        reasons |= MOTOR_START_MODE_UNSUPPORTED;
    }
    if (!s_context.motor->configuration_confirmed || !facts->configuration_valid)
    {
        reasons |= MOTOR_START_CONFIGURATION_REQUIRED;
    }
    if ((Encoder_GetCalibFlag(s_context.rotor_encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        reasons |= MOTOR_START_CALIBRATION_REQUIRED;
    }
    if (!Encoder_IsOnline(s_context.rotor_encoder))
    {
        reasons |= MOTOR_START_ENCODER_UNAVAILABLE;
    }
    if (!state_vbus_in_enable_window() || !isfinite(s_context.foc->bus_voltage_v) ||
        s_context.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        reasons |= MOTOR_START_BUS_UNAVAILABLE;
    }
    if (!facts->self_test_ready)
    {
        reasons |= MOTOR_START_HEALTH_UNAVAILABLE;
    }
    if (s_context.rules.state != MOTOR_STATE_READY || s_context.boot_maintenance_only)
    {
        reasons |= MOTOR_START_STATE_UNAVAILABLE;
    }
    if (!configuration_idle(facts) || request_busy() || s_context.enable_pending)
    {
        reasons |= MOTOR_START_OWNER_BUSY;
    }
    return reasons;
}

/* 位置边界是模式附加条件，其他模式共用同一组启动资格。 */
static bool state_position_start_allowed(void)
{
    float position = Encoder_GetMecPos(s_context.rotor_encoder);
    return MotorAxisProfile_AllowsPosition(&s_context.motor->application.axis_profile,
                                           s_context.motor->application.axis_profile_valid,
                                           position,
                                           position);
}

void MotorStateMachine_ReadReadiness(MotorReadiness *readiness)
{
    if (readiness == NULL)
    {
        return;
    }
    uint32_t mask = critical_hw_enter();
    MotorStateFacts facts = read_state_conditions();
    *readiness = (MotorReadiness){0};
    readiness->start_blockers = state_start_blockers(&facts);
    readiness->self_test_checks = build_self_test_checks(&facts);
    readiness->calibration_flags = Encoder_GetCalibFlag(s_context.rotor_encoder);
    for (unsigned mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_VQ; ++mode)
    {
        if (readiness->start_blockers == 0U &&
            (mode != MOTOR_CONTROL_POSITION || state_position_start_allowed()))
        {
            readiness->startable_modes |= (uint8_t)(1U << mode);
        }
    }
    critical_hw_exit(mask);
}

MotorControlResult MotorStateMachine_RequestEnable(MotorControlMode mode)
{
    if ((unsigned)mode > MOTOR_CONTROL_VQ)
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (s_context.boot_maintenance_only)
    {
        return MOTOR_CONTROL_DENIED;
    }
    uint32_t mask = critical_hw_enter();
    MotorStateFacts facts = read_state_conditions();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (request_busy() || s_context.enable_pending)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (state_start_blockers(&facts) == 0U &&
             (mode != MOTOR_CONTROL_POSITION || state_position_start_allowed()))
    {
        clear_running_data();
        if (mode == MOTOR_CONTROL_POSITION)
        {
            s_context.motor->position_target_rad = s_context.motor->position_reference_rad =
                Encoder_GetMecPos(s_context.rotor_encoder);
        }
        s_context.motor->control_mode = mode;
        s_context.enable_pending = true;
        result = MOTOR_CONTROL_ACCEPTED;
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult
MotorStateMachine_RequestIdentifiedEnable(const MotorControlStartRequest *request)
{
    if (request == NULL || request->reserved != 0U || request->source_node == 0U ||
        request->source_node == UINT8_MAX)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result;
    if (s_context.remote_source_valid && s_context.remote_source != request->source_node)
    {
        result = MOTOR_CONTROL_DENIED;
    }
    else if (same_start_identity(&s_context.latest_start, request))
    {
        result = s_context.latest_start.request.mode == request->mode
                     ? s_context.latest_start.result
                     : MOTOR_CONTROL_CONFLICTING_RETRY;
    }
    else if (same_start_identity(&s_context.active_start, request))
    {
        result = s_context.active_start.request.mode == request->mode
                     ? s_context.active_start.result
                     : MOTOR_CONTROL_CONFLICTING_RETRY;
    }
    else if ((s_context.stop_sequence_valid &&
              !start_sequence_newer(request->sequence, s_context.stop_sequence)) ||
             (s_context.latest_start.state != MOTOR_CONTROL_START_NONE &&
              !start_sequence_newer(request->sequence, s_context.latest_start.request.sequence)))
    {
        result = MOTOR_CONTROL_STALE_SEQUENCE;
    }
    else
    {
        if (!s_context.remote_source_valid)
        {
            s_context.remote_source = request->source_node;
            s_context.remote_source_valid = true;
        }
        result = request->mode == MOTOR_CONTROL_SPEED
                     ? MotorStateMachine_RequestEnable(request->mode)
                     : MOTOR_CONTROL_INVALID;
        s_context.latest_start = (MotorControlStartProgress){
            .request = *request,
            .state = result == MOTOR_CONTROL_ACCEPTED ? MOTOR_CONTROL_START_PENDING
                                                      : MOTOR_CONTROL_START_REJECTED,
            .result = result};
        if (result == MOTOR_CONTROL_ACCEPTED)
        {
            s_context.active_start = s_context.latest_start;
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorStateMachine_ReadEnableProgress(uint8_t source_node,
                                          uint16_t sequence,
                                          MotorControlStartProgress *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *progress = (MotorControlStartProgress){.state = MOTOR_CONTROL_START_NONE};
    const MotorControlStartProgress *found = NULL;
    if (s_context.latest_start.state != MOTOR_CONTROL_START_NONE &&
        s_context.latest_start.request.source_node == source_node &&
        s_context.latest_start.request.sequence == sequence)
    {
        found = &s_context.latest_start;
    }
    else if (s_context.active_start.state != MOTOR_CONTROL_START_NONE &&
             s_context.active_start.request.source_node == source_node &&
             s_context.active_start.request.sequence == sequence)
    {
        found = &s_context.active_start;
    }
    if (found != NULL)
    {
        *progress = *found;
    }
    critical_hw_exit(mask);
    return found != NULL;
}

MotorControlResult MotorStateMachine_RequestClear(void)
{
    uint32_t mask = critical_hw_enter();
    MotorStateFacts facts = read_state_conditions();
    MotorFaultCode error = s_context.motor->fault_code;
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (request_busy() || s_context.enable_pending || !facts.maintenance_released)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (s_context.rules.state == MOTOR_STATE_READY && error == MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    else if (s_context.rules.state == MOTOR_STATE_FAULT &&
             (error != MOTOR_FAULT_SELF_TEST_TIMEOUT ||
              build_self_test_checks(&facts) == MOTOR_SELF_TEST_ALL) &&
             (error == MOTOR_FAULT_NONE || MotorFault_SourceRecovered(&s_context.recovery,
                                                                      s_context.motor,
                                                                      s_context.foc,
                                                                      s_context.rotor_encoder,
                                                                      error,
                                                                      facts.inputs_valid,
                                                                      facts.maintenance_released,
                                                                      MotorTemperature_IsFresh())))
    {
        /* 恢复事实与 CLEAR 同处一个临界区；提交前不提前清除故障。 */
        facts.active_faults = 0U;
        bool accepted = dispatch_state_event(
            MOTOR_STATE_EVENT_CLEAR, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, &facts);
        result = accepted ? MOTOR_CONTROL_ACCEPTED : MOTOR_CONTROL_DENIED;
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult MotorStateMachine_RequestMaintenance(MotorMaintenanceKind kind, uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return MOTOR_CONTROL_INVALID;
    }
    *ticket = 0U;
    if ((unsigned)kind >= MOTOR_MAINTENANCE_NONE)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorStateFacts facts = read_state_conditions();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (request_busy() || s_context.enable_pending || !facts.maintenance_released)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (configuration_idle(&facts) && facts.self_test_ready &&
             (kind == MOTOR_MAINTENANCE_SAVE || MotorAxis_ConfigurationRevisionAvailable()))
    {
        clear_running_data();
        if (begin_operation(kind))
        {
            *ticket = s_context.operation_epoch;
            result = MOTOR_CONTROL_ACCEPTED;
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorStateMachine_ReadMaintenance(uint32_t ticket, MotorMaintenanceProgress *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *progress = (MotorMaintenanceProgress){0};
    bool found = ticket != 0U && ticket == s_context.last_maintenance.ticket;
    if (found)
    {
        *progress = s_context.last_maintenance;
    }
    else if (s_context.last_maintenance_request.ticket != 0U &&
             ticket == s_context.last_maintenance_request.terminal.ticket && ticket != 0U)
    {
        *progress = s_context.last_maintenance_request.terminal;
        found = true;
    }
    else if (ticket != 0U && ticket == s_context.operation_epoch)
    {
        progress->ticket = ticket;
        progress->kind = s_context.rules.maintenance_kind;
        progress->result = s_context.rules.operation_result;
        progress->effects = s_context.rules.operation_effects;
        found = true;
    }
    critical_hw_exit(mask);
    return found;
}

MotorControlResult
MotorStateMachine_RequestIdentifiedMaintenance(const MotorMaintenanceRequest *request,
                                               uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return MOTOR_CONTROL_INVALID;
    }
    *ticket = 0U;
    if (request == NULL || request->reserved != 0U || request->source_node == UINT8_MAX ||
        (unsigned)request->kind >= MOTOR_MAINTENANCE_NONE)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result;
    if (s_context.last_maintenance_request.ticket != 0U &&
        request->source_node == s_context.last_maintenance_request.request.source_node)
    {
        uint16_t delta =
            (uint16_t)(request->sequence - s_context.last_maintenance_request.request.sequence);
        if (delta == 0U)
        {
            result = request->kind == s_context.last_maintenance_request.request.kind
                         ? MOTOR_CONTROL_ACCEPTED
                         : MOTOR_CONTROL_CONFLICTING_RETRY;
            if (result == MOTOR_CONTROL_ACCEPTED)
            {
                *ticket = s_context.last_maintenance_request.ticket;
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
    result = MotorStateMachine_RequestMaintenance(request->kind, ticket);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        s_context.last_maintenance_request.request = *request;
        s_context.last_maintenance_request.terminal = (MotorMaintenanceProgress){0};
        s_context.last_maintenance_request.ticket = *ticket;
    }
    critical_hw_exit(mask);
    return result;
}

MotorControlResult MotorStateMachine_CancelMaintenance(uint32_t ticket)
{
    if (ticket == 0U)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result = MOTOR_CONTROL_INVALID;
    if (ticket == s_context.last_maintenance.ticket ||
        (s_context.last_maintenance_request.ticket != 0U &&
         ticket == s_context.last_maintenance_request.terminal.ticket))
    {
        /* 最近终态的重复取消只确认身份，绝不停止当前其他任务。 */
        result = MOTOR_CONTROL_ACCEPTED;
    }
    else if (ticket == s_context.operation_epoch && s_context.rules.maintenance_active)
    {
        if (s_context.rules.operation_result == MOTOR_MAINTENANCE_CANCEL_REQUESTED)
        {
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (s_context.rules.state == MOTOR_STATE_MAINTENANCE)
        {
            reject_pending_control(MOTOR_CONTROL_DENIED);
            if (s_context.has_latched_outcome)
            {
                if (s_context.pending_outcome.result == MOTOR_TASK_FAILED &&
                    s_context.outcome_epoch == s_context.operation_epoch)
                {
                    /* 同代故障先锁存，再取消；不能让CANCEL递增代号后丢弃故障。 */
                    MotorFault_Report(s_context.motor, s_context.pending_outcome.error);
                }
                s_context.has_latched_outcome = false;
            }
            bool accepted = dispatch_state_event(
                MOTOR_STATE_EVENT_CANCEL, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
            s_context.motor->control_mode = MOTOR_CONTROL_NONE;
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

void MotorStateMachine_Init(MotorControlState *motor,
                            FocState *foc,
                            PiController *pi,
                            EncoderState *encoder,
                            SensorlessStartupState *startup)
{
    /* 初始化发生在中断使能前，统一清空避免新增字段遗漏复位。 */
    memset(&s_context, 0, sizeof(s_context));
    s_context.self_test_timeout_ms =
        MOTOR_SELF_TEST_TIMEOUT_MS +
        (MotorCurrentOffsets_Valid(motor) ? 0U : MOTOR_CURRENT_OFFSET_SAMPLE_TIME_MS);
    s_context.motor = motor;
    s_context.foc = foc;
    s_context.speed_pi = pi;
    s_context.rotor_encoder = encoder;
    s_context.sensorless_startup = startup;
    s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    MotorStateRules_Init(&s_context.rules);
}

MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void)
{
    return s_context.rules.state == MOTOR_STATE_MAINTENANCE ? s_context.rules.maintenance_kind
                                                            : MOTOR_MAINTENANCE_NONE;
}

bool MotorStateMachine_MaintenanceRunnable(void)
{
    return s_context.maintenance_runnable && !s_context.has_latched_outcome &&
           !s_context.stop_pending && s_context.rules.state == MOTOR_STATE_MAINTENANCE &&
           s_context.motor->fault_code == MOTOR_FAULT_NONE;
}

bool MotorStateMachine_OutputExpected(void)
{
    return !power_stage_hw_phases_disabled() ||
           s_context.motor->control_mode != MOTOR_CONTROL_NONE ||
           (s_context.rules.state == MOTOR_STATE_MAINTENANCE && s_context.maintenance_runnable &&
            s_context.rules.maintenance_active &&
            MotorStateRules_Classify(s_context.rules.maintenance_kind) ==
                MOTOR_MAINTENANCE_CLASS_CALIBRATION);
}

MotorState MotorStateMachine_GetState(void)
{
    return s_context.rules.state;
}

/* 前台持锁：STOP 总是撤销本代待启动与目标，序号只影响后续 START 准入。 */
static void request_remote_stop_locked(void)
{
    reject_pending_control(s_context.motor->fault_code != MOTOR_FAULT_NONE ? MOTOR_CONTROL_FAULT
                                                                           : MOTOR_CONTROL_DENIED);
    reject_pending_start(s_context.motor->fault_code != MOTOR_FAULT_NONE ? MOTOR_CONTROL_FAULT
                                                                         : MOTOR_CONTROL_DENIED);
    if (s_context.has_latched_outcome)
    {
        if (s_context.pending_outcome.result == MOTOR_TASK_FAILED &&
            s_context.outcome_epoch == MotorStateMachine_WorkEpoch())
        {
            /* STOP撤销目标，但不能撤销当前worker已报告的故障。 */
            MotorFault_Report(s_context.motor, s_context.pending_outcome.error);
        }
        s_context.has_latched_outcome = false;
    }
    s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    s_context.enable_pending = s_context.maintenance_runnable = s_context.save_requested = false;
    s_context.stop_pending = true;
}

bool MotorStateMachine_RequestRemoteStop(void)
{
    uint32_t mask = critical_hw_enter();
    request_remote_stop_locked();
    critical_hw_exit(mask);
    return true;
}

bool MotorStateMachine_RequestIdentifiedStop(uint8_t source_node, uint16_t sequence)
{
    uint32_t mask = critical_hw_enter();
    if (source_node != 0U && source_node != UINT8_MAX &&
        (!s_context.remote_source_valid || s_context.remote_source == source_node))
    {
        s_context.remote_source = source_node;
        s_context.remote_source_valid = true;
        if (!s_context.stop_sequence_valid ||
            start_sequence_newer(sequence, s_context.stop_sequence))
        {
            s_context.stop_sequence = sequence;
            s_context.stop_sequence_valid = true;
        }
    }
    request_remote_stop_locked();
    critical_hw_exit(mask);
    return true;
}

MotorControlResult MotorStateMachine_RequestBootStop(uint64_t ticket)
{
    if (ticket == 0U)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = critical_hw_enter();
    MotorControlResult result;
    if (s_context.boot_stop_ticket != 0U)
    {
        result = s_context.boot_stop_ticket == ticket ? MOTOR_CONTROL_ACCEPTED : MOTOR_CONTROL_BUSY;
    }
    else if (s_context.rules.state == MOTOR_STATE_FATAL)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else
    {
        s_context.boot_stop_ticket = ticket;
        (void)MotorStateMachine_RequestRemoteStop();
        result = MOTOR_CONTROL_ACCEPTED;
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorStateMachine_BootHandoffReady(uint64_t ticket)
{
    uint32_t mask = critical_hw_enter();
    MotorStateFacts facts = read_state_conditions();
    bool ready = ticket != 0U && ticket == s_context.boot_stop_ticket &&
                 (s_context.rules.state == MOTOR_STATE_READY ||
                  s_context.rules.state == MOTOR_STATE_FAULT) &&
                 output_and_owner_released(&facts);
    critical_hw_exit(mask);
    return ready;
}

bool MotorStateMachine_ReadBootHealth(uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *progress = s_context.supervisor_epoch;
    bool healthy =
        read_state_conditions().inputs_valid && s_context.rules.state != MOTOR_STATE_FATAL;
    critical_hw_exit(mask);
    return healthy;
}

MotorControlResult MotorStateMachine_SubmitControl(const MotorControlCommand *command)
{
    uint32_t state = critical_hw_enter();
    MotorControlResult result = admit_control_target(command);
    if (result == MOTOR_CONTROL_ACCEPTED)
    {
        if (s_context.has_pending_control)
        {
            result = MOTOR_CONTROL_BUSY;
        }
        else
        {
            s_context.pending_control = *command;
            s_context.control_epoch = s_context.rules.epoch;
            s_context.has_pending_control = true;
            if (command->source_node != 0U)
            {
                s_context.control_progress =
                    (MotorControlProgress){.state = MOTOR_CONTROL_PROGRESS_PENDING,
                                           .result = MOTOR_CONTROL_ACCEPTED,
                                           .source_node = command->source_node,
                                           .sequence = command->sequence};
            }
        }
    }
    critical_hw_exit(state);
    return result;
}

void MotorStateMachine_ReadControlProgress(MotorControlProgress *progress)
{
    if (progress == NULL)
    {
        return;
    }
    uint32_t mask = critical_hw_enter();
    *progress = s_context.control_progress;
    critical_hw_exit(mask);
}

void MotorStateMachine_CheckFastFaults(void)
{
    if (s_context.motor->control_mode == MOTOR_CONTROL_NONE &&
        isfinite(s_context.foc->bus_voltage_v) &&
        s_context.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    if ((encoder_feedback_required(s_context.motor) ||
         s_context.rules.state == MOTOR_STATE_READY) &&
        Encoder_GetBadFrameStreak(s_context.rotor_encoder) >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_ENCODER);
    }

    /* 检查实际输出和已受理控制，不依赖通信入口。 */
    if (!s_context.motor->application.axis_profile_valid && MotorStateMachine_OutputExpected())
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}

void MotorStateMachine_PostOutcome(MotorTaskResult outcome, uint32_t epoch)
{
    uint32_t primask;

    ++s_context.fast_sample_epoch;
    if (outcome.result == MOTOR_TASK_RUNNING)
    {
        return;
    }
    primask = critical_hw_enter();
    if (epoch != MotorStateMachine_WorkEpoch() || s_context.stop_pending)
    {
        critical_hw_exit(primask);
        return;
    }
    if (outcome.result == MOTOR_TASK_PREPARED)
    {
        if (!MotorStateMachine_MaintenanceRunnable() ||
            s_context.rules.maintenance_kind != MOTOR_MAINTENANCE_COGGING)
        {
            critical_hw_exit(primask);
            return;
        }
        /* 重复就绪事实不重复启相；首次启相失败必须终止功能，不能继续无输出标定。 */
        if (!power_stage_hw_phases_disabled())
        {
            critical_hw_exit(primask);
            return;
        }
        MotorStateFacts facts = read_state_conditions();
        if (facts.start_permitted && facts.inputs_valid && facts.active_faults == 0U)
        {
            power_stage_hw_start();
            if (!power_stage_hw_phases_disabled())
            {
                critical_hw_exit(primask);
                return;
            }
        }
        outcome = (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_CONTROL_OVERRUN};
    }
    if (outcome.result == MOTOR_TASK_FAILED)
    {
        if (outcome.error == MOTOR_FAULT_NONE)
        {
            outcome.error = MOTOR_FAULT_CONTROL_OVERRUN;
        }
        MotorFault_Report(s_context.motor, outcome.error);
        MotorStateMachine_FastFaultStop();
    }
    if (!s_context.has_latched_outcome || outcome.result == MOTOR_TASK_FAILED)
    {
        s_context.outcome_epoch = epoch;
        s_context.pending_outcome = outcome;
        s_context.has_latched_outcome = true;
    }
    critical_hw_exit(primask);
}

void MotorStateMachine_FastFaultStop(void)
{
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        reject_pending_control(MOTOR_CONTROL_FAULT);
        reject_pending_start(MOTOR_CONTROL_FAULT);
    }
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE && !power_stage_hw_phases_disabled())
    {
        power_stage_hw_stop();
    }
}

/* 取走待处理结果；无请求时按 RUNNING 处理。 */
static MotorTaskResult take_current_task_result(void)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    uint32_t primask = critical_hw_enter();

    if (s_context.has_latched_outcome)
    {
        if (s_context.outcome_epoch == MotorStateMachine_WorkEpoch() && !s_context.stop_pending)
        {
            outcome = s_context.pending_outcome;
            if (outcome.result != MOTOR_TASK_RUNNING)
            {
                s_context.maintenance_runnable = false;
            }
        }
        s_context.has_latched_outcome = false;
    }
    critical_hw_exit(primask);
    return outcome;
}

/* 消费监督拍开始时可见的 STOP；本拍后续仍必须接收更高优先级故障。 */
static bool state_consume_stop_request(void)
{
    bool stopped = s_context.stop_pending;
    if (stopped)
    {
        s_context.stop_pending = false;
        state_request_stop();
    }
    return stopped;
}

/* 结果消费保持故障优先；完成先关相，再发布任务终态及派生保存意图。 */
static void state_process_task_outcome(bool stopped)
{
    MotorTaskResult outcome = take_current_task_result();
    if (outcome.result == MOTOR_TASK_FAILED)
    {
        MotorFault_Report(s_context.motor, outcome.error);
    }
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        s_context.save_requested = false;
        (void)dispatch_state_event(MOTOR_STATE_EVENT_FAULT,
                                   MOTOR_MAINTENANCE_EFFECT_UNKNOWN,
                                   (uint32_t)s_context.motor->fault_code,
                                   NULL);
    }
    else if (!stopped && outcome.result == MOTOR_TASK_STOP_REQUESTED)
    {
        state_request_stop();
    }
    else if (!stopped && s_context.rules.state == MOTOR_STATE_MAINTENANCE &&
             (outcome.result == MOTOR_TASK_COMPLETED || outcome.result == MOTOR_TASK_FINALIZE))
    {
        s_context.maintenance_runnable = false;
        power_stage_hw_stop();
        clear_running_data();
        if (outcome.result == MOTOR_TASK_COMPLETED)
        {
            s_context.save_requested =
                s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_ENCODER_LUT ||
                s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO ||
                s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_MECHANICAL_ZERO ||
                s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_DEFAULTS;
            (void)dispatch_state_event(
                MOTOR_STATE_EVENT_OPERATION_DONE, MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED, 0U, NULL);
        }
    }
}

/* 动作后重新读取事实：持续运行守卫与停止确认不能使用本拍早先的旧快照。 */
static void advance_current_state(void)
{
    switch (s_context.rules.state)
    {
    case MOTOR_STATE_BOOT:
    case MOTOR_STATE_SELF_TEST:
        advance_boot_and_self_test();
        break;
    case MOTOR_STATE_STARTING:
    case MOTOR_STATE_RUNNING:
    case MOTOR_STATE_STOPPING:
        /* 重新采集当前条件，执行运行守卫或停止确认。 */
        (void)dispatch_state_event(0U, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_MAINTENANCE:
    case MOTOR_STATE_FAULT:
    case MOTOR_STATE_FATAL:
        /* 请求和结果由同拍前后明确阶段处理，不新增迁移或自动恢复。 */
        break;
    }
}

/* 维护终结/取消确认当拍不启动派生保存；保存受理后再检查显式使能。 */
static void start_requested_save(bool stopped, bool operation_finished)
{
    if (!stopped && !operation_finished && s_context.save_requested &&
        s_context.rules.state == MOTOR_STATE_READY)
    {
        uint32_t mask = critical_hw_enter();
        s_context.save_requested = false;
        (void)begin_operation(MOTOR_MAINTENANCE_SAVE);
        critical_hw_exit(mask);
    }
}

/* 请求受理只进入 STARTING；功率输出在准备完成后独立复核。 */
static void begin_requested_control(bool stopped)
{
    if (s_context.enable_pending && s_context.rules.state == MOTOR_STATE_READY && !stopped)
    {
        s_context.enable_pending = false;
        (void)dispatch_state_event(
            MOTOR_STATE_EVENT_ENABLE, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
    }
}

/* 实际启相前在原临界区再次读取许可；准备就绪本身不能作为延迟启相授权。 */
static void enable_prepared_control(void)
{
    if (s_context.rules.state == MOTOR_STATE_STARTING && read_state_conditions().startup_confirmed)
    {
        uint32_t mask = critical_hw_enter();
        MotorStateFacts facts = read_state_conditions();
        if (!s_context.stop_pending && s_context.boot_stop_ticket == 0U && facts.start_permitted &&
            facts.inputs_valid && facts.active_faults == 0U)
        {
            power_stage_hw_start();
            (void)dispatch_state_event(
                MOTOR_STATE_EVENT_START_DONE, MOTOR_MAINTENANCE_EFFECT_UNKNOWN, 0U, NULL);
        }
        critical_hw_exit(mask);
    }
}

/* 原代启相事实经监督拍确认；STOP/故障可在此之前同步撤销身份。 */
static void update_start_progress(void)
{
    uint32_t mask = critical_hw_enter();
    if (s_context.active_start.state == MOTOR_CONTROL_START_PENDING)
    {
        if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
        {
            reject_pending_start(MOTOR_CONTROL_FAULT);
        }
        else if (s_context.rules.state == MOTOR_STATE_RUNNING &&
                 s_context.motor->control_mode == s_context.active_start.request.mode &&
                 !s_context.stop_pending && !power_stage_hw_phases_disabled())
        {
            s_context.active_start.state = MOTOR_CONTROL_START_STARTED;
            if (same_start_identity(&s_context.latest_start, &s_context.active_start.request))
            {
                s_context.latest_start = s_context.active_start;
            }
        }
        else if ((s_context.rules.state == MOTOR_STATE_READY && !s_context.enable_pending) ||
                 (s_context.rules.state != MOTOR_STATE_READY &&
                  s_context.rules.state != MOTOR_STATE_STARTING))
        {
            reject_pending_start(MOTOR_CONTROL_DENIED);
        }
    }
    critical_hw_exit(mask);
}

void MotorStateMachine_Tick(void)
{
    state_update_sample_progress();
    bool stopped = state_consume_stop_request();
    state_process_task_outcome(stopped);
    bool operation_event = finish_maintenance_or_release();
    MotorFault_UpdateRecovery(&s_context.recovery, s_context.foc, s_context.motor->fault_code);

    advance_current_state();
    start_requested_save(stopped, operation_event);
    begin_requested_control(stopped);
    enable_prepared_control();
    apply_pending_control();
    update_start_progress();

    indicator_hw_set_led(s_context.motor->fault_code != MOTOR_FAULT_NONE,
                         (uint8_t)(s_context.motor->fault_code != MOTOR_FAULT_NONE
                                       ? s_context.motor->fault_code
                                       : (unsigned)s_context.motor->control_mode));
    ++s_context.supervisor_epoch;
}

uint32_t MotorStateMachine_WorkEpoch(void)
{
    return s_context.rules.state == MOTOR_STATE_MAINTENANCE ? s_context.operation_epoch
                                                            : s_context.rules.epoch;
}
bool MotorStateMachine_BackgroundValid(const FocBackgroundClaim *claim)
{
    return claim != NULL && s_context.background_busy &&
           claim->epoch == s_context.background_claim.epoch &&
           claim->kind == s_context.background_claim.kind &&
           claim->epoch == s_context.operation_epoch && claim->epoch == s_context.rules.epoch &&
           s_context.rules.state == MOTOR_STATE_MAINTENANCE && !s_context.stop_pending &&
           s_context.motor->fault_code == MOTOR_FAULT_NONE && power_stage_hw_phases_disabled();
}
bool MotorStateMachine_BeginBackground(FocBackgroundClaim *claim)
{
    uint32_t mask = critical_hw_enter();
    bool accepted = false;
    if (claim != NULL && !s_context.background_busy && !s_context.stop_pending &&
        !s_context.has_latched_outcome && s_context.rules.state == MOTOR_STATE_MAINTENANCE &&
        s_context.motor->fault_code == MOTOR_FAULT_NONE && power_stage_hw_phases_disabled() &&
        EncoderCalibrationTask_IsReleased())
    {
        FocBackgroundKind kind = s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_SAVE
                                     ? FOC_BACKGROUND_SAVE
                                 : s_context.rules.maintenance_kind == MOTOR_MAINTENANCE_COGGING &&
                                         FocCogging_BeginFinalize()
                                     ? FOC_BACKGROUND_COGGING
                                     : FOC_BACKGROUND_NONE;
        if (kind != FOC_BACKGROUND_NONE &&
            (kind != FOC_BACKGROUND_SAVE || s_context.save_finish == SAVE_FINISH_NONE))
        {
            s_context.background_claim = (FocBackgroundClaim){s_context.operation_epoch, kind};
            s_context.background_busy = true;
            *claim = s_context.background_claim;
            accepted = true;
        }
    }
    critical_hw_exit(mask);
    return accepted;
}
void MotorStateMachine_FinishBackground(const FocBackgroundClaim *claim, bool success)
{
    uint32_t mask = critical_hw_enter();
    if (claim == NULL || !s_context.background_busy ||
        claim->epoch != s_context.background_claim.epoch ||
        claim->kind != s_context.background_claim.kind)
    {
        critical_hw_exit(mask);
        return;
    }
    bool current = MotorStateMachine_BackgroundValid(claim);
    s_context.background_busy = false;
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
                MotorFault_Report(s_context.motor, MOTOR_FAULT_INVALID_PARAMETERS);
            }
            FocCogging_SaveResult(success);
            s_context.save_finish = success ? SAVE_FINISH_COMMITTED : SAVE_FINISH_FAILED;
            s_context.motor->control_mode = MOTOR_CONTROL_NONE;
        }
        else if (success)
        {
            s_context.maintenance_runnable = false;
            s_context.save_requested = true;
            (void)dispatch_state_event(
                MOTOR_STATE_EVENT_OPERATION_DONE, MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED, 0U, NULL);
        }
        else
        {
            MotorFault_Report(s_context.motor, MOTOR_FAULT_COGGING_CALIBRATION);
        }
    }
    else if (claim->kind == FOC_BACKGROUND_SAVE)
    {
        FocCogging_SaveResult(false);
    }
    critical_hw_exit(mask);
}

void MotorStateMachine_ServiceCompensation(void)
{
    uint32_t mask = critical_hw_enter();
    bool permitted = s_context.boot_stop_ticket == 0U && !s_context.background_busy &&
                     !s_context.enable_pending && !s_context.stop_pending &&
                     !s_context.has_latched_outcome &&
                     s_context.motor->fault_code == MOTOR_FAULT_NONE &&
                     (s_context.rules.state == MOTOR_STATE_READY ||
                      s_context.rules.state == MOTOR_STATE_RUNNING);
    if (permitted)
    {
        s_context.configuration_reading = true;
    }
    critical_hw_exit(mask);
    if (!permitted)
    {
        return;
    }
    FocCogging_Service(s_context.motor, s_context.rotor_encoder);
    mask = critical_hw_enter();
    s_context.configuration_reading = false;
    critical_hw_exit(mask);
}
