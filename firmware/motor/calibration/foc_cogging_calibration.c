#include "foc_cogging_calibration.h"

#include <math.h>

#include "foc_errhandle.h"
#include "control_config.h"
#include "current_sense_profile.h"
#include "control_math.h"
#include "bus_voltage_profile.h"

/* 最小标定任务：启动、每拍保护、定位采样、统一结束与前台候选交接。
 * 算法/运行表/功率/工作区/Flash 各归原所有者；任务不增加生命周期。 */
typedef enum
{
    COGGING_PENDING_FINALIZE = 1,
    COGGING_PENDING_SAVE,
    COGGING_PENDING_NONE
} CoggingPending;

typedef enum
{
    COGGING_CLAIM_ACTIVE = 1,
    COGGING_CLAIM_CANCELLED,
    COGGING_CLAIM_NONE
} CoggingClaim;

/* active 直到目标和控制器都清理才归零；算法 COMPLETE 不等于资源已释放。 */
typedef struct
{
    bool active;
    volatile CoggingPending pending;
    volatile CoggingClaim claim;
    uint32_t fast_ticks;
    float start_mechanical_rad, start_angle_rad, current_limit_a;
    float hold_position_rad, previous_position_rad, hold_velocity_rad_s;
    float reference_velocity_rad_s, iq_block_sum_a;
    uint8_t reverse;
    uint16_t electrical_zero_q15;
    int32_t pole_pairs;
} CoggingTaskContext;

/* 非零空闲编码配合显式初值，保持既有 RW 初始化压缩；编码不进入公开 ABI。 */
static CoggingTaskContext s_task = {.pending = COGGING_PENDING_NONE, .claim = COGGING_CLAIM_NONE};
/* 保留算法观察符号；只有本任务向已有采样核心传入这个状态对象。 */
static CoggingCalibration s_cogging_calib;

static bool motion_parameters_valid(const MotorControlState *m)
{
    return isfinite(m->application.speed_limit_rad_s) && m->application.speed_limit_rad_s > 0.0f &&
           isfinite(m->application.speed_kp) && m->application.speed_kp > 0.0f &&
           isfinite(m->application.speed_ki) && m->application.speed_ki > 0.0f;
}

static bool environment_is_safe(const FocState *f)
{
    return isfinite(f->bus_voltage_v) && isfinite(f->bus_voltage_filtered_v) &&
           f->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           f->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V &&
           f->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V && isfinite(f->temperature_c) &&
           f->temperature_c < 100.0f;
}

bool FocCogging_CanStart(const MotorControlState *m, const EncoderState *e)
{
    /* 整圈旋转绝不能在带行程约束的 roll/pitch 轴上尝试。 */
    return m != NULL && e != NULL && m->application.axis_profile_valid &&
           m->application.axis_profile.magic == 0U && m->fault_code == MOTOR_FAULT_NONE &&
           Encoder_IsOnline(e) && e->velocity_ready &&
           (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           isfinite(m->application.current_limit_a) && m->application.current_limit_a > 0.0f &&
           isfinite(m->model.calibration_current_a) && m->model.calibration_current_a > 0.0f &&
           motion_parameters_valid(m) && isfinite(Encoder_GetMecVelContinuous(e)) &&
           fabsf(Encoder_GetMecVelContinuous(e)) < 0.05f && s_task.pending == COGGING_PENDING_NONE;
}

static bool session_is_safe(const FocState *f,
                            const MotorControlState *m,
                            const EncoderState *e,
                            float position_rad,
                            float velocity_rad_s)
{
    return Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           e->reverse == s_task.reverse && e->electrical_zero_q15 == s_task.electrical_zero_q15 &&
           m->model.motor_pole_pairs == s_task.pole_pairs && isfinite(Encoder_GetElePhase(e)) &&
           isfinite(Encoder_GetEleVel(e)) && isfinite(position_rad) && isfinite(velocity_rad_s) &&
           fabsf(velocity_rad_s) <= 1.5f && environment_is_safe(f) && isfinite(f->iq_a) &&
           fabsf(f->iq_a) <= s_task.current_limit_a + 0.5f &&
           isfinite(m->application.current_limit_a) &&
           m->application.current_limit_a >= s_task.current_limit_a && motion_parameters_valid(m) &&
           m->fault_code == MOTOR_FAULT_NONE;
}

static bool start_session(FocState *f, MotorControlState *m, PiController *pi, EncoderState *e)
{
    if (!FocCogging_CanStart(m, e) || !environment_is_safe(f) || !isfinite(f->iq_a) ||
        fabsf(f->iq_a) > 1.5f || !isfinite(Encoder_GetElePhase(e)) ||
        !isfinite(Encoder_GetEleVel(e)))
    {
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason = COGGING_INVALID_INPUT;
        MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
        return false;
    }
    s_task.start_mechanical_rad = Encoder_GetMecPos(e);
    s_task.start_angle_rad = (float)e->linearized_q15 * (_2PI / 65536.0f);
    s_task.hold_position_rad = s_task.previous_position_rad = s_task.start_angle_rad;
    s_task.hold_velocity_rad_s = 0.0f;
    s_task.reference_velocity_rad_s = s_task.iq_block_sum_a = 0.0f;
    s_task.reverse = e->reverse;
    s_task.electrical_zero_q15 = e->electrical_zero_q15;
    s_task.pole_pairs = m->model.motor_pole_pairs;
    s_task.current_limit_a =
        fminf(1.0f, fminf(m->model.calibration_current_a, m->application.current_limit_a));
    (void)CoggingCalibration_Start(
        &s_cogging_calib, s_task.start_angle_rad, CURRENT_SENSE_PROFILE_FULL_SCALE_A);
    s_task.fast_ticks = 0U;
    PiController_Reset(pi);
    FOC_CurrentController_Reset(f);
    m->id_target_a = m->iq_target_a = m->speed_target_rad_s = m->speed_ramped_rad_s = 0.0f;
    s_task.active = true;
    /* 只预载零矢量；功率许可由所有者消费 PREPARED 后授予。 */
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return true;
}

static void hold_current_point(MotorControlState *m, PiController *pi, float position_rad)
{
    const float distance = s_cogging_calib.target_rad - s_task.hold_position_rad;
    const float braking_speed = sqrtf(4.0f * fabsf(distance)); /* a = 2 rad/s^2 */
    const float speed_target =
        copysignf(fminf(braking_speed, fminf(0.1f, m->application.speed_limit_rad_s)), distance);
    s_task.reference_velocity_rad_s += Math_Constrain(
        speed_target - s_task.reference_velocity_rad_s, -2.0f * Speed_Ts, 2.0f * Speed_Ts);
    const float step = s_task.reference_velocity_rad_s * Speed_Ts;
    if (distance * step >= 0.0f && fabsf(step) >= fabsf(distance))
    {
        s_task.hold_position_rad = s_cogging_calib.target_rad;
        s_task.reference_velocity_rad_s = 0.0f;
    }
    else
    {
        s_task.hold_position_rad += step;
    }
    const float damping = 0.12f * s_task.hold_velocity_rad_s;
    PiController_Configure(pi,
                           60.0f,
                           240.0f,
                           Speed_Ts,
                           -s_task.current_limit_a + damping,
                           s_task.current_limit_a + damping);
    m->speed_target_rad_s = m->speed_ramped_rad_s = s_task.reference_velocity_rad_s;
    m->id_target_a = 0.0f;
    m->iq_target_a =
        Math_Constrain(PiController_Run(pi, s_task.hold_position_rad, position_rad) - damping,
                       -s_task.current_limit_a,
                       s_task.current_limit_a);
}

/* 2 kHz 只做迟滞测速与显式采样推进；终态由主任务直接消费。 */
static void sample_current_point(MotorControlState *m,
                                 float position_rad,
                                 float velocity_rad_s,
                                 float iq_average)
{
    /* 单个原始码宽的迟滞跟踪，抑制静止时的量化往返。 */
    const float deadband = _2PI / 32768.0f;
    float delta = position_rad - s_task.previous_position_rad;
    delta = delta > deadband ? delta - deadband : (delta < -deadband ? delta + deadband : 0.0f);
    s_task.previous_position_rad += delta;
    s_task.hold_velocity_rad_s += 0.25f * (delta / Speed_Ts - s_task.hold_velocity_rad_s);
    CoggingCalibration_Update(&s_cogging_calib,
                              position_rad,
                              velocity_rad_s,
                              iq_average,
                              fabsf(m->iq_target_a) >= s_task.current_limit_a * 0.98f);
}

/* 每条终止路径先清理，再发布事实；真实关相仍由 APP 结果消费者执行。 */
static MotorTaskResult
finish_task(FocState *f, MotorControlState *m, PiController *pi, MotorTaskState result)
{
    m->id_target_a = m->iq_target_a = m->speed_target_rad_s = m->speed_ramped_rad_s = 0.0f;
    PiController_Reset(pi);
    FOC_CurrentController_Reset(f);
    s_task.active = false;
    if (result == MOTOR_TASK_FINALIZE)
    {
        s_task.pending = COGGING_PENDING_FINALIZE;
        return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
    }
    return (MotorTaskResult){MOTOR_TASK_FAILED,
                             m->fault_code != MOTOR_FAULT_NONE ? m->fault_code
                                                               : MOTOR_FAULT_COGGING_CALIBRATION};
}

MotorTaskResult
FocCogging_Task(FocState *f, MotorControlState *m, PiController *pi, EncoderState *e)
{
    if (!s_task.active)
    {
        if (start_session(f, m, pi, e))
        {
            return (MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE};
        }
        return finish_task(f, m, pi, MOTOR_TASK_FAILED);
    }

    const float position_rad =
        s_task.start_angle_rad + (Encoder_GetMecPos(e) - s_task.start_mechanical_rad);
    const float velocity_rad_s = Encoder_GetMecVelContinuous(e);
    if (!session_is_safe(f, m, e, position_rad, velocity_rad_s))
    {
        CoggingCalibration_Abort(&s_cogging_calib, COGGING_SAFETY_FAULT);
        if (m->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
        }
        return finish_task(f, m, pi, MOTOR_TASK_FAILED);
    }

    s_task.iq_block_sum_a += f->iq_a;
    if (++s_task.fast_ticks >= SPEED_LOOP_DIVIDER)
    {
        const float iq_average = s_task.iq_block_sum_a / (float)SPEED_LOOP_DIVIDER;
        s_task.iq_block_sum_a = 0.0f;
        s_task.fast_ticks = 0U;
        sample_current_point(m, position_rad, velocity_rad_s, iq_average);
        if (s_cogging_calib.state == COGGING_FAILED)
        {
            MotorFault_Report(m, MOTOR_FAULT_COGGING_CALIBRATION);
            return finish_task(f, m, pi, MOTOR_TASK_FAILED);
        }
        if (s_cogging_calib.state == COGGING_COMPLETE)
        {
            return finish_task(f, m, pi, MOTOR_TASK_FINALIZE);
        }
        hold_current_point(m, pi, position_rad);
    }
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

void FocCogging_Abort(void)
{
    if (s_task.claim != COGGING_CLAIM_NONE)
    {
        s_task.claim = COGGING_CLAIM_CANCELLED;
        return;
    }
    s_task.pending = COGGING_PENDING_NONE;
    CoggingCalibration_Abort(&s_cogging_calib, COGGING_CANCELLED);
    s_task.active = false;
}

CoggingState FocCogging_GetState(void)
{
    return s_task.pending != COGGING_PENDING_NONE ? COGGING_SAVING : s_cogging_calib.state;
}

void FocCogging_SaveResult(bool success)
{
    if (s_task.pending != COGGING_PENDING_SAVE)
    {
        return;
    }
    s_task.pending = COGGING_PENDING_NONE;
    if (!success)
    {
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason = COGGING_SAVE_FAILED;
    }
}

bool FocCogging_NeedsFinalize(void)
{
    return s_task.pending == COGGING_PENDING_FINALIZE;
}

bool FocCogging_IsReleased(void)
{
    return !s_task.active && s_task.pending != COGGING_PENDING_FINALIZE &&
           s_task.claim == COGGING_CLAIM_NONE;
}

bool FocCogging_BeginFinalize(void)
{
    if (!FocCogging_NeedsFinalize() || s_task.claim != COGGING_CLAIM_NONE || s_task.active)
    {
        return false;
    }
    s_task.pending = COGGING_PENDING_NONE;
    s_task.claim = COGGING_CLAIM_ACTIVE;
    return true;
}

bool FocCogging_BuildCandidate(CoggingMapRecord *record,
                               const MotorControlState *motor,
                               const EncoderState *encoder)
{
    return s_task.claim == COGGING_CLAIM_ACTIVE && s_cogging_calib.state == COGGING_COMPLETE &&
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
    if (s_task.claim != COGGING_CLAIM_ACTIVE)
    {
        return false;
    }
    FocCogging_ImportMap(record);
    s_task.pending = COGGING_PENDING_SAVE;
    return true;
}

void FocCogging_ReleaseFinalize(bool published)
{
    if (s_task.claim == COGGING_CLAIM_NONE)
    {
        return;
    }
    if (!published || s_task.claim == COGGING_CLAIM_CANCELLED)
    {
        s_task.pending = COGGING_PENDING_NONE;
        s_cogging_calib.state = COGGING_FAILED;
        s_cogging_calib.reason =
            s_task.claim == COGGING_CLAIM_CANCELLED ? COGGING_CANCELLED : COGGING_INVALID_INPUT;
    }
    s_task.claim = COGGING_CLAIM_NONE;
}
