#include "foc_cogging_calibration.h"
#include <math.h>
#include "foc_errhandle.h"
#include "control_config.h"
#include "current_sense_profile.h"
#include "control_math.h"
#include "bus_voltage_profile.h"

/* 标定任务适配：20 kHz 安全观察/块平均、2 kHz 采样推进与保持控制。
 * 暂存表及认领/保存通知归本模块；活动记录与运行补偿归 foc_cogging_runtime。
 * 功率许可、关相、工作区和真实存储提交始终归 APP 所有者。 */

static CoggingCalibration s_cogging_calib;

static bool s_session_active;
static volatile bool s_finalize_pending, s_save_pending, s_finalize_claimed, s_finalize_cancelled;
static uint32_t s_divider;
static float s_start_mech_rad;
static float s_start_angle, s_calibration_current;
static float s_hold_position, s_previous_position, s_hold_velocity;
static float s_reference_velocity, s_iq_block_sum;
static uint8_t s_start_reverse;
static uint16_t s_start_zero;
static int32_t s_start_pole_pairs;

/* ---------------------------------------------------------------- 标定会话 */

bool FocCogging_CanStart(const MotorControlState *m, const EncoderState *e)
{
    /* 整圈旋转绝不能在带行程约束的 roll/pitch 轴上尝试。 */
    return m != NULL && e != NULL && m->application.axis_profile_valid &&
           m->application.axis_profile.magic == 0U && m->fault_code == MOTOR_FAULT_NONE &&
           Encoder_IsOnline(e) && e->velocity_ready &&
           (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           isfinite(m->application.current_limit_a) && m->application.current_limit_a > 0.0f &&
           isfinite(m->model.calibration_current_a) && m->model.calibration_current_a > 0.0f &&
           isfinite(m->application.speed_limit_rad_s) && m->application.speed_limit_rad_s > 0.0f &&
           isfinite(m->application.speed_kp) && m->application.speed_kp > 0.0f &&
           isfinite(m->application.speed_ki) && m->application.speed_ki > 0.0f &&
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
           m->model.motor_pole_pairs == s_start_pole_pairs && isfinite(Encoder_GetElePhase(e)) &&
           isfinite(Encoder_GetEleVel(e)) && isfinite(position_rad) && isfinite(velocity_rad_s) &&
           fabsf(velocity_rad_s) <= 1.5f && isfinite(f->bus_voltage_v) &&
           isfinite(f->bus_voltage_filtered_v) &&
           f->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           f->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V &&
           f->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V && isfinite(f->temperature_c) &&
           f->temperature_c < 100.0f && isfinite(f->iq_a) &&
           fabsf(f->iq_a) <= s_calibration_current + 0.5f &&
           isfinite(m->application.current_limit_a) &&
           m->application.current_limit_a >= s_calibration_current &&
           isfinite(m->application.speed_limit_rad_s) && m->application.speed_limit_rad_s > 0.0f &&
           isfinite(m->application.speed_kp) && m->application.speed_kp > 0.0f &&
           isfinite(m->application.speed_ki) && m->application.speed_ki > 0.0f &&
           m->fault_code == MOTOR_FAULT_NONE;
}

/* 启动会话：安全门、状态初始化与零矢量预载；失败由任务统一清理。 */
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
        return false;
    }
    s_start_mech_rad = Encoder_GetMecPos(e);
    s_start_angle = (float)e->linearized_q15 * (_2PI / 65536.0f);
    s_hold_position = s_previous_position = s_start_angle;
    s_hold_velocity = 0.0f;
    s_reference_velocity = s_iq_block_sum = 0.0f;
    s_start_reverse = e->reverse;
    s_start_zero = e->electrical_zero_q15;
    s_start_pole_pairs = m->model.motor_pole_pairs;
    s_calibration_current =
        fminf(1.0f, fminf(m->model.calibration_current_a, m->application.current_limit_a));
    (void)CoggingCalibration_Start(
        &s_cogging_calib, s_start_angle, CURRENT_SENSE_PROFILE_FULL_SCALE_A);
    s_divider = 0U;
    PiController_Reset(pi);
    FOC_CurrentController_Reset(f);
    m->id_target_a = m->iq_target_a = m->speed_target_rad_s = m->speed_ramped_rad_s = 0.0f;
    s_session_active = true;
    /* 只预载零矢量；功率许可由所有者消费 PREPARED 后授予。 */
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return true;
}

/* 20 kHz 采样与 2 kHz 服务：抽点、标定更新与位置保持。
 * 返回 false 表示采集已结束，任务必须清理控制状态且不得再驱动电流环。 */
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
        return false;
    }
    if (s_cogging_calib.state == COGGING_COMPLETE)
    {
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
        copysignf(fminf(braking_speed, fminf(0.1f, m->application.speed_limit_rad_s)), distance);
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
    bool complete = false;

    if (!s_session_active)
    {
        if (start_session(f, m, pi, e))
        {
            return (MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE};
        }
    }
    else
    {
        float position = s_start_angle + (Encoder_GetMecPos(e) - s_start_mech_rad);
        float velocity = Encoder_GetMecVelContinuous(e);
        if (!session_is_safe(f, m, e, position, velocity))
        {
            CoggingCalibration_Abort(&s_cogging_calib, COGGING_SAFETY_FAULT);
            if (m->fault_code == MOTOR_FAULT_NONE)
            {
                MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
            }
        }
        else if (service_tick2k_hz(f, m, pi, position, velocity))
        {
            FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
            return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        }
        else
        {
            complete = s_cogging_calib.state == COGGING_COMPLETE;
        }
    }
    /* 所有终止路径先清零目标与控制器，再报告结束；关相仍由主状态机执行。 */
    local_stop(f, m, pi);
    if (complete)
    {
        s_finalize_pending = true;
        return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
    }
    return (MotorTaskResult){MOTOR_TASK_FAILED,
                             m->fault_code != MOTOR_FAULT_NONE ? m->fault_code
                                                               : MOTOR_FAULT_COGGING_CALIBRATION};
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
                            Cogging_EncoderSignature(Encoder_GetReverse(encoder),
                                                     encoder->electrical_zero_q15,
                                                     motor->model.motor_pole_pairs,
                                                     encoder->linearization_lut_q15),
                            record);
}
bool FocCogging_PublishCandidate(const CoggingMapRecord *record)
{
    if (!s_finalize_claimed || s_finalize_cancelled)
    {
        return false;
    }
    FocCogging_ImportMap(record);
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
