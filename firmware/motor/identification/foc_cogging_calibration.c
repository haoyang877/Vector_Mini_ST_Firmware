#include "foc_cogging_calibration.h"
#include <math.h>
#include <string.h>
#include "cogging_compensation.h"
#include "foc_errhandle.h"
#include "control_config.h"
#include "critical_hw.h"
#include "current_sense_profile.h"
#include "utils.h"
#include "bus_voltage_profile.h"

/* 齿槽功能适配层：运行补偿采集、台架保护、标定会话。
 * 参数 ABI 保持既有约定；私有表与会话只通过显式边界交互，完成/失败经结果协议上报
 * （MotorTaskResult），停相与模式回退由运行状态机执行。 */

static CoggingMapRecord s_cogging_map;
/* 前台与快速 ISR 共享该私有控制状态，保留原有 volatile 访问语义。 */
static volatile CoggingCompensationControl s_cogging_compensation;
static volatile uint8_t s_compensation_request;
static CoggingCalibration s_cogging_calib;
static volatile CoggingTorqueGuard s_torque_guard;
static volatile uint32_t s_torque_sample_state;
static volatile CoggingTorqueFrame s_torque_sample;

static bool s_session_active;
static volatile bool s_finalize_pending, s_save_pending, s_finalize_claimed, s_finalize_cancelled;
/* 台架保护跳闸挂起：置位后由模式调度转为结果协议（见 FocCogging_TakeTorqueTrip）。 */
static volatile bool s_torque_trip_pending;
static uint32_t s_divider;
static float s_start_mech_rad;
static float s_start_angle, s_calibration_current;
static float s_hold_position, s_previous_position, s_hold_velocity;
static float s_reference_velocity, s_iq_block_sum;
static uint8_t s_start_reverse;
static uint16_t s_start_zero;
static int32_t s_start_pole_pairs;
static uint32_t s_torque_tick;

/* ---------------------------------------------------------------- 运行补偿 */

bool FocCogging_SetCompensation(bool enabled,
                                const MotorControlState *motor_state,
                                const EncoderState *rotor_encoder)
{
    CoggingCompensationAdmitInfo info;
    if (!enabled)
    {
        CoggingCompensation_Disable(&s_cogging_compensation);
        return true;
    }
    info.mode_accepts_request = motor_state->control_mode == MOTOR_CONTROL_NONE ||
                                motor_state->control_mode == MOTOR_CONTROL_CURRENT;
    info.error_clear = motor_state->fault_code == MOTOR_FAULT_NONE;
    info.sensorless_off = !motor_state->uses_sensorless_feedback;
    info.table_valid = FocCogging_TableValid(motor_state, rotor_encoder);
    info.scale_matches = s_cogging_map.full_scale_a == CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    info.identity.reverse = Encoder_GetReverse(rotor_encoder);
    info.identity.electrical_zero_q15 = rotor_encoder->electrical_zero_q15;
    info.identity.pole_pairs = motor_state->motor_pole_pairs;
    info.identity.table_crc = s_cogging_map.crc32;
    /* CRC 在开中断状态计算；提交前重读可被 STOP/FAULT 改变的资格。 */
    uint32_t mask = critical_hw_enter();
    info.mode_accepts_request = motor_state->control_mode == MOTOR_CONTROL_NONE ||
                                motor_state->control_mode == MOTOR_CONTROL_CURRENT;
    info.error_clear = motor_state->fault_code == MOTOR_FAULT_NONE;
    bool accepted = CoggingCompensation_Admit(&s_cogging_compensation, &info);
    critical_hw_exit(mask);
    return accepted;
}

float FocCogging_Apply(const MotorControlState *m, const EncoderState *e)
{
    CoggingCompensationTick tick;
    tick.mode_is_current = m->control_mode == MOTOR_CONTROL_CURRENT;
    tick.error_clear = m->fault_code == MOTOR_FAULT_NONE;
    tick.sensorless_off = !m->uses_sensorless_feedback;
    tick.encoder_usable = Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL;
    tick.command_a = m->iq_target_a;
    tick.limit_a = m->current_limit_a;
    tick.tick_s = Current_Ts;
    tick.position_q15 = e->linearized_q15;
    tick.identity.reverse = e->reverse;
    tick.identity.electrical_zero_q15 = e->electrical_zero_q15;
    tick.identity.pole_pairs = m->motor_pole_pairs;
    tick.identity.table_crc = s_cogging_map.crc32;
    return CoggingCompensation_Update(&s_cogging_compensation, &s_cogging_map, &tick);
}

/* ---------------------------------------------------------------- 台架保护 */

bool FocCogging_SetTorqueGuard(uint32_t lease_ticks, float speed_limit_rad_s)
{
    if (lease_ticks != 0U && (!isfinite(speed_limit_rad_s) || speed_limit_rad_s <= 0.0f))
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    s_torque_guard.lease_ticks = lease_ticks;
    s_torque_guard.speed_limit_rad_s = lease_ticks != 0U ? speed_limit_rad_s : 0.0f;
    s_torque_guard.enabled = lease_ticks != 0U;
    critical_hw_exit(mask);
    return true;
}

bool FocCogging_ReadTorqueGuard(CoggingTorqueGuard *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *snapshot = s_torque_guard;
    critical_hw_exit(mask);
    return true;
}

bool FocCogging_RequestTorqueSample(void)
{
    uint32_t mask = critical_hw_enter();
    bool accepted = s_torque_sample_state == 0U;
    if (accepted)
    {
        s_torque_sample_state = 1U;
    }
    critical_hw_exit(mask);
    return accepted;
}

bool FocCogging_ReadTorqueSample(CoggingTorqueFrame *sample)
{
    if (sample == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    bool ready = s_torque_sample_state == 2U;
    if (ready)
    {
        *sample = s_torque_sample;
        s_torque_sample_state = 0U;
    }
    critical_hw_exit(mask);
    return ready;
}

bool FocCogging_TorqueGuard(MotorControlState *m, const EncoderState *e)
{
    uint32_t trip = 0U;
    if (!s_torque_guard.enabled)
    {
        return true;
    }
    /* 租约：主机必须持续续租，进程失联即到期停机。 */
    if (s_torque_guard.lease_ticks == 0U)
    {
        trip = 1U;
    }
    else
    {
        --s_torque_guard.lease_ticks;
    }
    /* 测速上限：覆盖主机存活但脚本下发错误指令的情形。 */
    if (!isfinite(s_torque_guard.speed_limit_rad_s) || s_torque_guard.speed_limit_rad_s <= 0.0f ||
        !isfinite(e->mechanical_velocity_continuous_rad_s) ||
        fabsf(e->mechanical_velocity_continuous_rad_s) > s_torque_guard.speed_limit_rad_s)
    {
        trip = 2U;
    }
    if (trip == 0U)
    {
        return true;
    }
    s_torque_guard.trip = trip;
    s_torque_trip_pending = true;
    m->iq_target_a = m->id_target_a = 0.0f;
    CoggingCompensation_Disable(&s_cogging_compensation);
    CoggingCompensation_ZeroBlend(&s_cogging_compensation);
    return false;
}

bool FocCogging_TakeTorqueTrip(void)
{
    bool pending = s_torque_trip_pending;
    s_torque_trip_pending = false;
    return pending;
}

void FocCogging_TorqueObserve(const FocState *f, const MotorControlState *m, const EncoderState *e)
{
    ++s_torque_tick;
    if (s_torque_sample_state != 1U)
    {
        return;
    }
    s_torque_sample.tick = s_torque_tick;
    s_torque_sample.position_rad = Encoder_GetMecPos(e);
    s_torque_sample.velocity_rad_s = e->mechanical_velocity_continuous_rad_s;
    s_torque_sample.command_a = m->iq_target_a;
    s_torque_sample.compensation_a = s_cogging_compensation.applied_a;
    s_torque_sample.total_a = s_cogging_compensation.total_a;
    s_torque_sample.feedback_a = f->iq_a;
    s_torque_sample.vbus_v = f->bus_voltage_filtered_v;
    s_torque_sample.blend = s_cogging_compensation.blend;
    s_torque_sample_state = 2U;
}

/* ---------------------------------------------------------------- 标定会话 */

bool FocCogging_CanStart(const MotorControlState *m, const EncoderState *e)
{
    /* 整圈旋转绝不能在带行程约束的 roll/pitch 轴上尝试。 */
    return m != NULL && e != NULL && m->axis_profile_valid && m->axis_profile.magic == 0U &&
           m->fault_code == MOTOR_FAULT_NONE && Encoder_IsOnline(e) && e->velocity_ready &&
           (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL && isfinite(m->current_limit_a) &&
           m->current_limit_a > 0.0f && isfinite(m->calibration_current_a) &&
           m->calibration_current_a > 0.0f && isfinite(m->speed_limit_rad_s) &&
           m->speed_limit_rad_s > 0.0f && isfinite(m->speed_kp) && m->speed_kp > 0.0f &&
           isfinite(m->speed_ki) && m->speed_ki > 0.0f &&
           isfinite(Encoder_GetMecVelContinuous(e)) &&
           fabsf(Encoder_GetMecVelContinuous(e)) < 0.05f && !s_finalize_pending && !s_save_pending;
}

/* 本地停止：只清理控制状态，不动功率级与模式（由运行状态机按结果协议执行）。 */
static void local_stop(FocState *f, MotorControlState *m, PiController *pi)
{
    m->id_target_a = m->iq_target_a = m->speed_target_rad_s = m->speed_ramped_rad_s = 0.0f;
    PiController_Reset(pi);
    FOC_CurrentController_Reset(f);
    s_session_active = false;
}

void FocCogging_Abort(void)
{
    if (s_finalize_claimed)
    {
        s_finalize_cancelled = true;
        return;
    }
    s_finalize_pending = s_save_pending = false;
    CoggingCalibration_Abort(&s_cogging_calib, COGGING_CANCELLED);
    s_session_active = false;
}

/* 每拍安全门：会话身份、输入有限性、母线/温度、电流与限流配置。 */
static bool session_is_safe(const FocState *f,
                            const MotorControlState *m,
                            const EncoderState *e,
                            float position_rad,
                            float velocity_rad_s)
{
    return Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           e->reverse == s_start_reverse && e->electrical_zero_q15 == s_start_zero &&
           m->motor_pole_pairs == s_start_pole_pairs && isfinite(Encoder_GetElePhase(e)) &&
           isfinite(Encoder_GetEleVel(e)) && isfinite(position_rad) && isfinite(velocity_rad_s) &&
           fabsf(velocity_rad_s) <= 1.5f && isfinite(f->bus_voltage_v) &&
           isfinite(f->bus_voltage_filtered_v) &&
           f->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           f->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V &&
           f->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V && isfinite(f->temperature_c) &&
           f->temperature_c < 100.0f && isfinite(f->iq_a) &&
           fabsf(f->iq_a) <= s_calibration_current + 0.5f && isfinite(m->current_limit_a) &&
           m->current_limit_a >= s_calibration_current && isfinite(m->speed_limit_rad_s) &&
           m->speed_limit_rad_s > 0.0f && isfinite(m->speed_kp) && m->speed_kp > 0.0f &&
           isfinite(m->speed_ki) && m->speed_ki > 0.0f && m->fault_code == MOTOR_FAULT_NONE;
}

/* 启动会话：安全门 + 状态初始化 + 接管功率许可；失败时已停机并置错。 */
static bool start_session(FocState *f, MotorControlState *m, PiController *pi, EncoderState *e)
{
    if (!FocCogging_CanStart(m, e) || !isfinite(f->bus_voltage_v) ||
        !isfinite(f->bus_voltage_filtered_v) ||
        f->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
        f->bus_voltage_filtered_v > BUS_VOLTAGE_ENABLE_MAX_V ||
        f->bus_voltage_filtered_v < BUS_VOLTAGE_ENABLE_MIN_V || !isfinite(f->temperature_c) ||
        f->temperature_c >= 100.0f || !isfinite(f->iq_a) || fabsf(f->iq_a) > 1.5f ||
        !isfinite(Encoder_GetElePhase(e)) || !isfinite(Encoder_GetEleVel(e)))
    {
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason = COGGING_INVALID_INPUT;
        MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
        local_stop(f, m, pi);
        return false;
    }
    s_start_mech_rad = Encoder_GetMecPos(e);
    s_start_angle = (float)e->linearized_q15 * (_2PI / 65536.0f);
    s_hold_position = s_previous_position = s_start_angle;
    s_hold_velocity = 0.0f;
    s_reference_velocity = s_iq_block_sum = 0.0f;
    s_start_reverse = e->reverse;
    s_start_zero = e->electrical_zero_q15;
    s_start_pole_pairs = m->motor_pole_pairs;
    s_calibration_current = fminf(1.0f, fminf(m->calibration_current_a, m->current_limit_a));
    (void)CoggingCalibration_Start(
        &s_cogging_calib, s_start_angle, CURRENT_SENSE_PROFILE_FULL_SCALE_A);
    s_divider = 0U;
    PiController_Reset(pi);
    FOC_CurrentController_Reset(f);
    m->id_target_a = m->iq_target_a = m->speed_target_rad_s = m->speed_ramped_rad_s = 0.0f;
    s_session_active = true;
    /* 本模式自行接管功率许可；通用模式边沿不包含它。 */
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return true;
}

/* 20 kHz 采样与 2 kHz 服务：抽点、标定更新与位置保持。
 * 返回 false 表示会话已结束（已停机），调用方不得再驱动电流环。 */
static void hold_current_point(MotorControlState *m, PiController *pi, float position_rad);

static bool service_tick2k_hz(
    FocState *f, MotorControlState *m, PiController *pi, float position_rad, float velocity_rad_s)
{
    /* 先积分全部 20 kHz 反馈样本，再抽取到 2 kHz。 */
    s_iq_block_sum += f->iq_a;
    if (++s_divider < SPEED_LOOP_DIVIDER)
    {
        return true;
    }
    const float iq_average = s_iq_block_sum / (float)SPEED_LOOP_DIVIDER;
    s_iq_block_sum = 0.0f;
    s_divider = 0U;
    /* 单个原始码宽的迟滞跟踪，抑制静止时的量化往返。 */
    const float deadband = _2PI / 32768.0f;
    float delta = position_rad - s_previous_position;
    delta = delta > deadband ? delta - deadband : (delta < -deadband ? delta + deadband : 0.0f);
    s_previous_position += delta;
    s_hold_velocity += 0.25f * (delta / Speed_Ts - s_hold_velocity);
    CoggingCalibration_Update(&s_cogging_calib,
                              position_rad,
                              velocity_rad_s,
                              iq_average,
                              fabsf(m->iq_target_a) >= s_calibration_current * 0.98f);
    if (s_cogging_calib.state == COGGING_FAILED)
    {
        MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
        local_stop(f, m, pi);
        return false;
    }
    if (s_cogging_calib.state == COGGING_COMPLETE)
    {
        local_stop(f, m, pi);
        s_finalize_pending = true;
        return false;
    }
    hold_current_point(m, pi, position_rad);
    return true;
}

/* 位置保持：受限斜坡 + 约 90 Hz 微分阻尼，PI 限幅随阻尼移动。 */
static void hold_current_point(MotorControlState *m, PiController *pi, float position_rad)
{
    const float distance = s_cogging_calib.target_rad - s_hold_position;
    const float braking_speed = sqrtf(4.0f * fabsf(distance)); /* a = 2 rad/s^2 */
    const float speed_target =
        copysignf(fminf(braking_speed, fminf(0.1f, m->speed_limit_rad_s)), distance);
    s_reference_velocity +=
        Math_Constrain(speed_target - s_reference_velocity, -2.0f * Speed_Ts, 2.0f * Speed_Ts);
    const float step = s_reference_velocity * Speed_Ts;
    if (distance * step >= 0.0f && fabsf(step) >= fabsf(distance))
    {
        s_hold_position = s_cogging_calib.target_rad;
        s_reference_velocity = 0.0f;
    }
    else
    {
        s_hold_position += step;
    }
    const float damping = 0.12f * s_hold_velocity;
    PiController_Configure(pi,
                           60.0f,
                           240.0f,
                           Speed_Ts,
                           -s_calibration_current + damping,
                           s_calibration_current + damping);
    m->speed_target_rad_s = m->speed_ramped_rad_s = s_reference_velocity;
    m->id_target_a = 0.0f;
    m->iq_target_a = Math_Constrain(PiController_Run(pi, s_hold_position, position_rad) - damping,
                                    -s_calibration_current,
                                    s_calibration_current);
}

MotorTaskResult
FocCogging_Task(FocState *f, MotorControlState *m, PiController *pi, EncoderState *e)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    float position, velocity;

    if (!s_session_active)
    {
        if (start_session(f, m, pi, e))
        {
            return (MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE};
        }
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error =
            (m->fault_code != MOTOR_FAULT_NONE) ? m->fault_code : MOTOR_FAULT_COGGING_CALIBRATION;
        return outcome;
    }
    position = s_start_angle + (Encoder_GetMecPos(e) - s_start_mech_rad);
    velocity = Encoder_GetMecVelContinuous(e);
    if (!session_is_safe(f, m, e, position, velocity))
    {
        CoggingCalibration_Abort(&s_cogging_calib, COGGING_SAFETY_FAULT);
        if (m->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
        }
        local_stop(f, m, pi);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = m->fault_code;
        return outcome;
    }
    if (!service_tick2k_hz(f, m, pi, position, velocity))
    {
        /* 会话已结束：完成报告前台整理需求；
         * 失败时经故障结果停机；本函数不再触碰功率级与模式。 */
        if (s_cogging_calib.state == COGGING_COMPLETE)
        {
            outcome.result = MOTOR_TASK_FINALIZE;
        }
        else
        {
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = (m->fault_code != MOTOR_FAULT_NONE) ? m->fault_code
                                                                : MOTOR_FAULT_COGGING_CALIBRATION;
        }
        return outcome;
    }
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return outcome;
}

static uint32_t signature(const MotorControlState *motor_state, const EncoderState *rotor_encoder)
{
    return Cogging_EncoderSignature(Encoder_GetReverse(rotor_encoder),
                                    rotor_encoder->electrical_zero_q15,
                                    motor_state->motor_pole_pairs,
                                    rotor_encoder->linearization_lut_q15);
}

bool FocCogging_TableValid(const MotorControlState *motor_state, const EncoderState *rotor_encoder)
{
    return (Encoder_GetCalibFlag(rotor_encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           CoggingMap_Valid(&s_cogging_map, signature(motor_state, rotor_encoder));
}

void FocCogging_RequestCompensation(bool enabled)
{
    uint32_t mask = critical_hw_enter();
    s_compensation_request = enabled ? 1U : 2U;
    critical_hw_exit(mask);
}

void FocCogging_Service(MotorControlState *motor_state, EncoderState *rotor_encoder)
{
    uint32_t mask = critical_hw_enter();
    if (s_compensation_request != 0U)
    {
        s_cogging_compensation.request = s_compensation_request == 1U ? 1U : 0U;
        s_compensation_request = 0U;
    }
    critical_hw_exit(mask);
    if (motor_state->control_mode != MOTOR_CONTROL_NONE &&
        motor_state->control_mode != MOTOR_CONTROL_CURRENT)
    {
        /* 离开允许模式立即关断补偿；停机预开启由 request 位保持。 */
        CoggingCompensation_Disable(&s_cogging_compensation);
        CoggingCompensation_ZeroBlend(&s_cogging_compensation);
    }
    else
    {
        if (motor_state->control_mode == MOTOR_CONTROL_NONE)
        {
            CoggingCompensation_ZeroBlend(&s_cogging_compensation);
        }
        if (s_cogging_compensation.request != s_cogging_compensation.enabled)
        {
            (void)FocCogging_SetCompensation(
                s_cogging_compensation.request == 1U, motor_state, rotor_encoder);
        }
    }
}

CoggingState FocCogging_GetState(void)
{
    return (s_finalize_pending || s_save_pending) ? COGGING_SAVING : s_cogging_calib.state;
}

void FocCogging_SaveResult(bool success)
{
    if (!s_save_pending)
    {
        return;
    }
    s_save_pending = false;
    if (!success)
    {
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason = COGGING_SAVE_FAILED;
    }
}

void FocCogging_ResetMap(void)
{
    memset(&s_cogging_map, 0, sizeof(s_cogging_map));
}
void FocCogging_ImportMap(const CoggingMapRecord *record)
{
    s_cogging_map = *record;
}
void FocCogging_ExportMap(CoggingMapRecord *record,
                          const MotorControlState *motor,
                          const EncoderState *encoder)
{
    memset(record, 0, sizeof(*record));
    if (FocCogging_TableValid(motor, encoder))
    {
        *record = s_cogging_map;
    }
}

bool FocCogging_NeedsFinalize(void)
{
    return s_finalize_pending;
}
bool FocCogging_IsReleased(void)
{
    return !s_session_active && !s_finalize_pending && !s_finalize_claimed;
}
bool FocCogging_BeginFinalize(void)
{
    if (!s_finalize_pending || s_finalize_claimed || s_session_active)
    {
        return false;
    }
    s_finalize_pending = false;
    s_finalize_claimed = true;
    s_finalize_cancelled = false;
    return true;
}
bool FocCogging_BuildCandidate(CoggingMapRecord *record,
                               const MotorControlState *motor,
                               const EncoderState *encoder)
{
    return s_finalize_claimed && !s_finalize_cancelled &&
           s_cogging_calib.state == COGGING_COMPLETE &&
           CoggingMap_Build(s_cogging_calib.iq_q15,
                            s_cogging_calib.full_scale_a,
                            signature(motor, encoder),
                            record);
}
bool FocCogging_PublishCandidate(const CoggingMapRecord *record)
{
    if (!s_finalize_claimed || s_finalize_cancelled)
    {
        return false;
    }
    s_cogging_map = *record;
    s_save_pending = true;
    return true;
}
void FocCogging_ReleaseFinalize(bool published)
{
    if (!s_finalize_claimed)
    {
        return;
    }
    if (!published || s_finalize_cancelled)
    {
        s_save_pending = false;
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason = s_finalize_cancelled ? COGGING_CANCELLED : COGGING_INVALID_INPUT;
    }
    s_finalize_claimed = false;
}
