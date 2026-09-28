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

static CoggingMapRecord CoggingMap;
/* 前台与快速 ISR 共享该私有控制状态，保留原有 volatile 访问语义。 */
static volatile CoggingCompensationControl CoggingCompensation;
static volatile uint8_t compensation_request;
static CoggingCalibration CoggingCalib;
static volatile CoggingTorqueGuard torque_guard;
static volatile uint32_t torque_sample_state;
static volatile CoggingTorqueFrame torque_sample;

static bool session_started;
static volatile bool finalize_pending, save_pending, finalize_claimed, finalize_cancelled;
/* 台架保护跳闸挂起：置位后由模式调度转为结果协议（见 FocCogging_TakeTorqueTrip）。 */
static volatile bool torque_trip_pending;
static uint32_t divider;
static float start_mech_rad;
static float start_angle, calibration_current;
static float hold_position, previous_position, hold_velocity;
static float reference_velocity, iq_block_sum;
static uint8_t start_reverse;
static uint16_t start_zero;
static int32_t start_pole_pairs;
static uint32_t torque_tick;

/* ---------------------------------------------------------------- 运行补偿 */

bool FocCogging_SetCompensation(bool enabled,
                                const MotorControl_TypeDef *MotorControl,
                                const Encoder_TypeDef *OnBoard_Encoder)
{
    CoggingCompensationAdmitInfo info;
    if (!enabled)
    {
        CoggingCompensation_Disable(&CoggingCompensation);
        return true;
    }
    info.mode_accepts_request = MotorControl->control_mode == MOTOR_CONTROL_NONE ||
                                MotorControl->control_mode == MOTOR_CONTROL_CURRENT;
    info.error_clear = MotorControl->ErrorNow == No_Error;
    info.sensorless_off = !MotorControl->isUseSensorless;
    info.table_valid = FocCogging_TableValid(MotorControl, OnBoard_Encoder);
    info.scale_matches = CoggingMap.full_scale_a == CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    info.identity.reverse = Encoder_GetReverse(OnBoard_Encoder);
    info.identity.electrical_zero_q15 = OnBoard_Encoder->electrical_zero_q15;
    info.identity.pole_pairs = MotorControl->motor_pole_pairs;
    info.identity.table_crc = CoggingMap.crc32;
    /* CRC 在开中断状态计算；提交前重读可被 STOP/FAULT 改变的资格。 */
    uint32_t mask = critical_hw_enter();
    info.mode_accepts_request = MotorControl->control_mode == MOTOR_CONTROL_NONE ||
                                MotorControl->control_mode == MOTOR_CONTROL_CURRENT;
    info.error_clear = MotorControl->ErrorNow == No_Error;
    bool accepted = CoggingCompensation_Admit(&CoggingCompensation, &info);
    critical_hw_exit(mask);
    return accepted;
}

float FocCogging_Apply(const MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    CoggingCompensationTick tick;
    tick.mode_is_current = m->control_mode == MOTOR_CONTROL_CURRENT;
    tick.error_clear = m->ErrorNow == No_Error;
    tick.sensorless_off = !m->isUseSensorless;
    tick.encoder_usable = Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL;
    tick.command_a = m->iqRef;
    tick.limit_a = m->current_limit;
    tick.tick_s = Current_Ts;
    tick.position_q15 = e->linearized_q15;
    tick.identity.reverse = e->reverse;
    tick.identity.electrical_zero_q15 = e->electrical_zero_q15;
    tick.identity.pole_pairs = m->motor_pole_pairs;
    tick.identity.table_crc = CoggingMap.crc32;
    return CoggingCompensation_Update(&CoggingCompensation, &CoggingMap, &tick);
}

/* ---------------------------------------------------------------- 台架保护 */

bool FocCogging_SetTorqueGuard(uint32_t lease_ticks, float speed_limit_rad_s)
{
    if (lease_ticks != 0U && (!isfinite(speed_limit_rad_s) || speed_limit_rad_s <= 0.0f))
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    torque_guard.lease_ticks = lease_ticks;
    torque_guard.speed_limit_rad_s = lease_ticks != 0U ? speed_limit_rad_s : 0.0f;
    torque_guard.enabled = lease_ticks != 0U;
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
    *snapshot = torque_guard;
    critical_hw_exit(mask);
    return true;
}

bool FocCogging_RequestTorqueSample(void)
{
    uint32_t mask = critical_hw_enter();
    bool accepted = torque_sample_state == 0U;
    if (accepted)
    {
        torque_sample_state = 1U;
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
    bool ready = torque_sample_state == 2U;
    if (ready)
    {
        *sample = torque_sample;
        torque_sample_state = 0U;
    }
    critical_hw_exit(mask);
    return ready;
}

bool FocCogging_TorqueGuard(MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    uint32_t trip = 0U;
    if (!torque_guard.enabled)
    {
        return true;
    }
    /* 租约：主机必须持续续租，进程失联即到期停机。 */
    if (torque_guard.lease_ticks == 0U)
    {
        trip = 1U;
    }
    else
    {
        --torque_guard.lease_ticks;
    }
    /* 测速上限：覆盖主机存活但脚本下发错误指令的情形。 */
    if (!isfinite(torque_guard.speed_limit_rad_s) || torque_guard.speed_limit_rad_s <= 0.0f ||
        !isfinite(e->vel_mech_continuous) ||
        fabsf(e->vel_mech_continuous) > torque_guard.speed_limit_rad_s)
    {
        trip = 2U;
    }
    if (trip == 0U)
    {
        return true;
    }
    torque_guard.trip = trip;
    torque_trip_pending = true;
    m->iqRef = m->idRef = 0.0f;
    CoggingCompensation_Disable(&CoggingCompensation);
    CoggingCompensation_ZeroBlend(&CoggingCompensation);
    return false;
}

bool FocCogging_TakeTorqueTrip(void)
{
    bool pending = torque_trip_pending;
    torque_trip_pending = false;
    return pending;
}

void FocCogging_TorqueObserve(const FOC_TypeDef *f,
                              const MotorControl_TypeDef *m,
                              const Encoder_TypeDef *e)
{
    ++torque_tick;
    if (torque_sample_state != 1U)
    {
        return;
    }
    torque_sample.tick = torque_tick;
    torque_sample.position_rad = Encoder_GetMecPos(e);
    torque_sample.velocity_rad_s = e->vel_mech_continuous;
    torque_sample.command_a = m->iqRef;
    torque_sample.compensation_a = CoggingCompensation.applied_a;
    torque_sample.total_a = CoggingCompensation.total_a;
    torque_sample.feedback_a = f->Iq;
    torque_sample.vbus_v = f->Vbus_filt;
    torque_sample.blend = CoggingCompensation.blend;
    torque_sample_state = 2U;
}

/* ---------------------------------------------------------------- 标定会话 */

bool FocCogging_CanStart(const MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    /* 整圈旋转绝不能在带行程约束的 roll/pitch 轴上尝试。 */
    return m != NULL && e != NULL && m->axis_profile_valid && m->axis_profile.magic == 0U &&
           m->ErrorNow == No_Error && Encoder_IsOnline(e) && e->velocity_ready &&
           (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL && isfinite(m->current_limit) &&
           m->current_limit > 0.0f && isfinite(m->calib_current) && m->calib_current > 0.0f &&
           isfinite(m->speed_limit) && m->speed_limit > 0.0f && isfinite(m->speed_Kp) &&
           m->speed_Kp > 0.0f && isfinite(m->speed_Ki) && m->speed_Ki > 0.0f &&
           isfinite(Encoder_GetMecVelContinuous(e)) &&
           fabsf(Encoder_GetMecVelContinuous(e)) < 0.05f && !finalize_pending && !save_pending;
}

/* 本地停止：只清理控制状态，不动功率级与模式（由运行状态机按结果协议执行）。 */
static void LocalStop(FOC_TypeDef *f, MotorControl_TypeDef *m, PI_Controller_TypeDef *pi)
{
    m->idRef = m->iqRef = m->speedRef = m->speedShadow = 0.0f;
    PI_Controller_Reset(pi);
    FOC_CurrentController_Reset(f);
    session_started = false;
}

void FocCogging_Abort(void)
{
    if (finalize_claimed)
    {
        finalize_cancelled = true;
        return;
    }
    finalize_pending = save_pending = false;
    CoggingCalibration_Abort(&CoggingCalib, COGGING_CANCELLED);
    session_started = false;
}

/* 每拍安全门：会话身份、输入有限性、母线/温度、电流与限流配置。 */
static bool SessionIsSafe(const FOC_TypeDef *f,
                          const MotorControl_TypeDef *m,
                          const Encoder_TypeDef *e,
                          float position_rad,
                          float velocity_rad_s)
{
    return Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           e->reverse == start_reverse && e->electrical_zero_q15 == start_zero &&
           m->motor_pole_pairs == start_pole_pairs && isfinite(Encoder_GetElePhase(e)) &&
           isfinite(Encoder_GetEleVel(e)) && isfinite(position_rad) && isfinite(velocity_rad_s) &&
           fabsf(velocity_rad_s) <= 1.5f && isfinite(f->Vbus) && isfinite(f->Vbus_filt) &&
           f->Vbus < BUS_VOLTAGE_HARD_OVERVOLTAGE_V && f->Vbus_filt <= BUS_VOLTAGE_ENABLE_MAX_V &&
           f->Vbus_filt >= BUS_VOLTAGE_ENABLE_MIN_V && isfinite(f->temp) && f->temp < 100.0f &&
           isfinite(f->Iq) && fabsf(f->Iq) <= calibration_current + 0.5f &&
           isfinite(m->current_limit) && m->current_limit >= calibration_current &&
           isfinite(m->speed_limit) && m->speed_limit > 0.0f && isfinite(m->speed_Kp) &&
           m->speed_Kp > 0.0f && isfinite(m->speed_Ki) && m->speed_Ki > 0.0f &&
           m->ErrorNow == No_Error;
}

/* 启动会话：安全门 + 状态初始化 + 接管功率许可；失败时已停机并置错。 */
static bool
StartSession(FOC_TypeDef *f, MotorControl_TypeDef *m, PI_Controller_TypeDef *pi, Encoder_TypeDef *e)
{
    if (!FocCogging_CanStart(m, e) || !isfinite(f->Vbus) || !isfinite(f->Vbus_filt) ||
        f->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V || f->Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V ||
        f->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V || !isfinite(f->temp) || f->temp >= 100.0f ||
        !isfinite(f->Iq) || fabsf(f->Iq) > 1.5f || !isfinite(Encoder_GetElePhase(e)) ||
        !isfinite(Encoder_GetEleVel(e)))
    {
        CoggingCalib.state = COGGING_FAILED;
        CoggingCalib.reason = COGGING_INVALID_INPUT;
        MotorFault_Report(m, CoggingCalibration_Error);
        LocalStop(f, m, pi);
        return false;
    }
    start_mech_rad = Encoder_GetMecPos(e);
    start_angle = (float)e->linearized_q15 * (_2PI / 65536.0f);
    hold_position = previous_position = start_angle;
    hold_velocity = 0.0f;
    reference_velocity = iq_block_sum = 0.0f;
    start_reverse = e->reverse;
    start_zero = e->electrical_zero_q15;
    start_pole_pairs = m->motor_pole_pairs;
    calibration_current = fminf(1.0f, fminf(m->calib_current, m->current_limit));
    (void)CoggingCalibration_Start(&CoggingCalib, start_angle, CURRENT_SENSE_PROFILE_FULL_SCALE_A);
    divider = 0U;
    PI_Controller_Reset(pi);
    FOC_CurrentController_Reset(f);
    m->idRef = m->iqRef = m->speedRef = m->speedShadow = 0.0f;
    session_started = true;
    /* 本模式自行接管功率许可；通用模式边沿不包含它。 */
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return true;
}

/* 20 kHz 采样与 2 kHz 服务：抽点、标定更新与位置保持。
 * 返回 false 表示会话已结束（已停机），调用方不得再驱动电流环。 */
static void
HoldCurrentPoint(MotorControl_TypeDef *m, PI_Controller_TypeDef *pi, float position_rad);

static bool ServiceTick2kHz(FOC_TypeDef *f,
                            MotorControl_TypeDef *m,
                            PI_Controller_TypeDef *pi,
                            float position_rad,
                            float velocity_rad_s)
{
    /* 先积分全部 20 kHz 反馈样本，再抽取到 2 kHz。 */
    iq_block_sum += f->Iq;
    if (++divider < SPEED_LOOP_DIVIDER)
    {
        return true;
    }
    const float iq_average = iq_block_sum / (float)SPEED_LOOP_DIVIDER;
    iq_block_sum = 0.0f;
    divider = 0U;
    /* 单个原始码宽的迟滞跟踪，抑制静止时的量化往返。 */
    const float deadband = _2PI / 32768.0f;
    float delta = position_rad - previous_position;
    delta = delta > deadband ? delta - deadband : (delta < -deadband ? delta + deadband : 0.0f);
    previous_position += delta;
    hold_velocity += 0.25f * (delta / Speed_Ts - hold_velocity);
    CoggingCalibration_Update(&CoggingCalib,
                              position_rad,
                              velocity_rad_s,
                              iq_average,
                              fabsf(m->iqRef) >= calibration_current * 0.98f);
    if (CoggingCalib.state == COGGING_FAILED)
    {
        MotorFault_Report(m, CoggingCalibration_Error);
        LocalStop(f, m, pi);
        return false;
    }
    if (CoggingCalib.state == COGGING_COMPLETE)
    {
        LocalStop(f, m, pi);
        finalize_pending = true;
        return false;
    }
    HoldCurrentPoint(m, pi, position_rad);
    return true;
}

/* 位置保持：受限斜坡 + 约 90 Hz 微分阻尼，PI 限幅随阻尼移动。 */
static void HoldCurrentPoint(MotorControl_TypeDef *m, PI_Controller_TypeDef *pi, float position_rad)
{
    const float distance = CoggingCalib.target_rad - hold_position;
    const float braking_speed = sqrtf(4.0f * fabsf(distance)); /* a = 2 rad/s^2 */
    const float speed_target =
        copysignf(fminf(braking_speed, fminf(0.1f, m->speed_limit)), distance);
    reference_velocity +=
        constrain(speed_target - reference_velocity, -2.0f * Speed_Ts, 2.0f * Speed_Ts);
    const float step = reference_velocity * Speed_Ts;
    if (distance * step >= 0.0f && fabsf(step) >= fabsf(distance))
    {
        hold_position = CoggingCalib.target_rad;
        reference_velocity = 0.0f;
    }
    else
    {
        hold_position += step;
    }
    const float damping = 0.12f * hold_velocity;
    PI_Controller_Configure(
        pi, 60.0f, 240.0f, Speed_Ts, -calibration_current + damping, calibration_current + damping);
    m->speedRef = m->speedShadow = reference_velocity;
    m->idRef = 0.0f;
    m->iqRef = constrain(PI_Controller_Run(pi, hold_position, position_rad) - damping,
                         -calibration_current,
                         calibration_current);
}

MotorTaskResult FocCogging_Task(FOC_TypeDef *f,
                                MotorControl_TypeDef *m,
                                PI_Controller_TypeDef *pi,
                                Encoder_TypeDef *e)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, No_Error};
    float position, velocity;

    if (!session_started)
    {
        if (StartSession(f, m, pi, e))
        {
            return (MotorTaskResult){MOTOR_TASK_PREPARED, No_Error};
        }
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = (m->ErrorNow != No_Error) ? m->ErrorNow : CoggingCalibration_Error;
        return outcome;
    }
    position = start_angle + (Encoder_GetMecPos(e) - start_mech_rad);
    velocity = Encoder_GetMecVelContinuous(e);
    if (!SessionIsSafe(f, m, e, position, velocity))
    {
        CoggingCalibration_Abort(&CoggingCalib, COGGING_SAFETY_FAULT);
        if (m->ErrorNow == No_Error)
        {
            MotorFault_Report(m, CoggingCalibration_Error);
        }
        LocalStop(f, m, pi);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = m->ErrorNow;
        return outcome;
    }
    if (!ServiceTick2kHz(f, m, pi, position, velocity))
    {
        /* 会话已结束：完成报告前台整理需求；
         * 失败时经故障结果停机；本函数不再触碰功率级与模式。 */
        if (CoggingCalib.state == COGGING_COMPLETE)
        {
            outcome.result = MOTOR_TASK_FINALIZE;
        }
        else
        {
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = (m->ErrorNow != No_Error) ? m->ErrorNow : CoggingCalibration_Error;
        }
        return outcome;
    }
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
    return outcome;
}

static uint32_t Signature(const MotorControl_TypeDef *MotorControl,
                          const Encoder_TypeDef *OnBoard_Encoder)
{
    return Cogging_EncoderSignature(Encoder_GetReverse(OnBoard_Encoder),
                                    OnBoard_Encoder->electrical_zero_q15,
                                    MotorControl->motor_pole_pairs,
                                    OnBoard_Encoder->linearization_lut_q15);
}

bool FocCogging_TableValid(const MotorControl_TypeDef *MotorControl,
                           const Encoder_TypeDef *OnBoard_Encoder)
{
    return (Encoder_GetCalibFlag(OnBoard_Encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           CoggingMap_Valid(&CoggingMap, Signature(MotorControl, OnBoard_Encoder));
}

void FocCogging_RequestCompensation(bool enabled)
{
    uint32_t mask = critical_hw_enter();
    compensation_request = enabled ? 1U : 2U;
    critical_hw_exit(mask);
}

void FocCogging_Service(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *OnBoard_Encoder)
{
    uint32_t mask = critical_hw_enter();
    if (compensation_request != 0U)
    {
        CoggingCompensation.request = compensation_request == 1U ? 1U : 0U;
        compensation_request = 0U;
    }
    critical_hw_exit(mask);
    if (MotorControl->control_mode != MOTOR_CONTROL_NONE &&
        MotorControl->control_mode != MOTOR_CONTROL_CURRENT)
    {
        /* 离开允许模式立即关断补偿；停机预开启由 request 位保持。 */
        CoggingCompensation_Disable(&CoggingCompensation);
        CoggingCompensation_ZeroBlend(&CoggingCompensation);
    }
    else
    {
        if (MotorControl->control_mode == MOTOR_CONTROL_NONE)
        {
            CoggingCompensation_ZeroBlend(&CoggingCompensation);
        }
        if (CoggingCompensation.request != CoggingCompensation.enabled)
        {
            (void)FocCogging_SetCompensation(
                CoggingCompensation.request == 1U, MotorControl, OnBoard_Encoder);
        }
    }
}

CoggingState FocCogging_GetState(void)
{
    return (finalize_pending || save_pending) ? COGGING_SAVING : CoggingCalib.state;
}

void FocCogging_SaveResult(bool success)
{
    if (!save_pending)
    {
        return;
    }
    save_pending = false;
    if (!success)
    {
        CoggingCalib.state = COGGING_FAILED;
        CoggingCalib.reason = COGGING_SAVE_FAILED;
    }
}

void FocCogging_ResetMap(void)
{
    memset(&CoggingMap, 0, sizeof(CoggingMap));
}
void FocCogging_ImportMap(const CoggingMapRecord *record)
{
    CoggingMap = *record;
}
void FocCogging_ExportMap(CoggingMapRecord *record,
                          const MotorControl_TypeDef *motor,
                          const Encoder_TypeDef *encoder)
{
    memset(record, 0, sizeof(*record));
    if (FocCogging_TableValid(motor, encoder))
    {
        *record = CoggingMap;
    }
}

bool FocCogging_NeedsFinalize(void)
{
    return finalize_pending;
}
bool FocCogging_IsReleased(void)
{
    return !session_started && !finalize_pending && !finalize_claimed;
}
bool FocCogging_BeginFinalize(void)
{
    if (!finalize_pending || finalize_claimed || session_started)
    {
        return false;
    }
    finalize_pending = false;
    finalize_claimed = true;
    finalize_cancelled = false;
    return true;
}
bool FocCogging_BuildCandidate(CoggingMapRecord *record,
                               const MotorControl_TypeDef *motor,
                               const Encoder_TypeDef *encoder)
{
    return finalize_claimed && !finalize_cancelled && CoggingCalib.state == COGGING_COMPLETE &&
           CoggingMap_Build(
               CoggingCalib.iq_q15, CoggingCalib.full_scale_a, Signature(motor, encoder), record);
}
bool FocCogging_PublishCandidate(const CoggingMapRecord *record)
{
    if (!finalize_claimed || finalize_cancelled)
    {
        return false;
    }
    CoggingMap = *record;
    save_pending = true;
    return true;
}
void FocCogging_ReleaseFinalize(bool published)
{
    if (!finalize_claimed)
    {
        return;
    }
    if (!published || finalize_cancelled)
    {
        save_pending = false;
        CoggingCalib.state = COGGING_FAILED;
        CoggingCalib.reason = finalize_cancelled ? COGGING_CANCELLED : COGGING_INVALID_INPUT;
    }
    finalize_claimed = false;
}
