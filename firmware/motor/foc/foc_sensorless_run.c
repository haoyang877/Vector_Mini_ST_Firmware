#include "foc_sensorless_run.h"

#include <math.h>
#include <stddef.h>

#include "foc_errhandle.h"
#include "foc_speed.h"
#include "foc_pid.h"
#include "control_config.h"
#include "utils.h"

/* 无感运行应用：启动序列（对准→开环→速度锁定→交接→闭环）与速度模式。
 * 观测器算法位于 foc_sensorless.{c,h}；本文件只编排控制流与安全门限。 */

void SensorlessStartup_Reset(SensorlessStartup_TypeDef *Startup)
{
    Startup->state = SENSORLESS_STARTUP_IDLE;
    Startup->open_loop_theta = 0.0f;
    Startup->open_loop_omega = 0.0f;
    Startup->handoff_phase_delta = 0.0f;
    Startup->handoff_id_reference = 0.0f;
    Startup->speed_feedback = 0.0f;
    Startup->lock_speed_feedback = 0.0f;
    Startup->direction = 1.0f;
    Startup->speed_pi_output_max = 1.0f;
    Startup->state_ticks = 0U;
    Startup->speed_loop_ticks = 0U;
    Startup->open_loop_ticks = 0U;
    Startup->lock_ticks = 0U;
    Startup->id_ramp_ticks = 0U;
    Startup->loss_ticks = 0U;
}

/* SENSORLESS_RUNTIME_BEGIN
 * 无感速度任务区：对准→开环→速度锁定→角度交接→闭环，与编码器标定模式共享。
 * 离线测试按本标记切取源码，标记文字不得修改。 */

/* 角度差归一化到 -PI..PI。 */
static float Sensorless_AngleDifference(float target, float source)
{
    float difference = target - source;

    if (difference > _PI)
    {
        difference -= _2PI;
    }
    else if (difference < -_PI)
    {
        difference += _2PI;
    }

    return difference;
}

const SensorlessStartupConfig_TypeDef SensorlessStartup_DefaultConfig = {
    SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S,
    SENSORLESS_ALIGN_HOLD_TIME_S,
    SENSORLESS_ALIGN_CURRENT_A,
    SENSORLESS_STARTUP_IQ_INITIAL_A,
    SENSORLESS_STARTUP_IQ_A,
    SENSORLESS_STARTUP_IQ_RAMP_TIME_S,
    SENSORLESS_STARTUP_ID_A,
    0.0f,
    SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S,
    SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S,
    SENSORLESS_STARTUP_RAMP_TIME_S,
    SENSORLESS_STARTUP_SPEED_LOCK_TIME_S,
    SENSORLESS_SPEED_LOCK_FILTER_ALPHA,
    SENSORLESS_OBSERVER_LOCK_RATIO,
    SENSORLESS_ANGLE_HANDOFF_TIME_S,
    SENSORLESS_STARTUP_LOCK_TIMEOUT_S,
    SENSORLESS_ID_RAMP_DOWN_TIME_S,
    SENSORLESS_OBSERVER_LOSS_TIME_S};

/* 观测器可用性：有限值且角速度在最大电角速度以内。 */
static bool Sensorless_ObserverIsUsable(const Fluxobserver_TypeDef *Fluxobserver)
{
    return Fluxobserver->theta_e == Fluxobserver->theta_e &&
           Fluxobserver->omega_e == Fluxobserver->omega_e &&
           fast_abs(Fluxobserver->omega_e) <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

/* 启动参数集准入：时间、电流与速度阈值必须自洽。 */
static bool Sensorless_StartupConfigIsValid(const SensorlessStartupConfig_TypeDef *Config)
{
    return Config != NULL && Config->align_current_ramp_time_s > 0.0f &&
           Config->align_hold_time_s >= 0.0f && Config->align_current_a > 0.0f &&
           Config->startup_iq_initial_a >= 0.0f &&
           Config->startup_iq_a >= Config->startup_iq_initial_a &&
           Config->startup_iq_ramp_time_s > 0.0f && Config->startup_id_a >= 0.0f &&
           Config->minimum_current_limit_a >= 0.0f &&
           Config->minimum_electrical_velocity_rad_s > 0.0f &&
           Config->target_electrical_velocity_rad_s >= Config->minimum_electrical_velocity_rad_s &&
           Config->startup_ramp_time_s > 0.0f && Config->speed_lock_time_s > 0.0f &&
           Config->speed_lock_filter_alpha > 0.0f && Config->speed_lock_filter_alpha <= 1.0f &&
           Config->observer_lock_ratio > 0.0f && Config->angle_handoff_time_s > 0.0f &&
           Config->lock_timeout_s >= Config->speed_lock_time_s &&
           Config->id_ramp_down_time_s > 0.0f && Config->observer_loss_time_s > 0.0f;
}

/* 启动电流准入：对准与开环电流的合成矢量不得超出电流限制。 */
static bool Sensorless_StartupCurrentsAreValid(const MotorControl_TypeDef *MotorControl,
                                               const SensorlessStartupConfig_TypeDef *Config)
{
    float current_limit_squared = MotorControl->current_limit * MotorControl->current_limit;
    float startup_current_squared =
        Config->startup_iq_a * Config->startup_iq_a + Config->startup_id_a * Config->startup_id_a;

    return MotorControl->current_limit >= Config->minimum_current_limit_a &&
           MotorControl->current_limit >= Config->align_current_a &&
           startup_current_squared <= current_limit_squared;
}

/* 速度 PI 在角度交接与正常跟踪期间保持同一实例运行；
 * 输出始终是观测器坐标系下的转矩电流参考。 */
static float Sensorless_UpdateSpeedLoop(MotorControl_TypeDef *MotorControl,
                                        PI_Controller_TypeDef *controller,
                                        SensorlessStartup_TypeDef *Startup,
                                        float observer_mech_vel)
{
    if (++Startup->speed_loop_ticks >= SPEED_LOOP_DIVIDER)
    {
        Startup->speed_feedback +=
            SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA * (observer_mech_vel - Startup->speed_feedback);
        MotorControl_UpdateSpeedRamp(MotorControl);
        PI_Controller_Configure(controller,
                                MotorControl->speed_Kp,
                                MotorControl->speed_Ki,
                                Speed_Ts,
                                -1.0f,
                                Startup->speed_pi_output_max);
        PI_Controller_Run(controller, MotorControl->speedShadow, Startup->speed_feedback);
        Startup->speed_loop_ticks = 0U;
    }
    return constrain(controller->Out, -1.0f, Startup->speed_pi_output_max) *
           MotorControl->current_limit;
}

/* 有界小角度 sin/cos：粗正弦表会把亚分度步长舍入为零，
 * 平方超过 0.01 时回退标准库实现。 */
static void Sensorless_SmallAngleSinCos(float angle, float *sin_out, float *cos_out)
{
    float squared = angle * angle;

    if (squared <= 0.01f)
    {
        *sin_out = angle * (1.0f - squared / 6.0f + squared * squared / 120.0f);
        *cos_out = 1.0f - squared * 0.5f + squared * squared / 24.0f;
    }
    else
    {
        *sin_out = sinf(angle);
        *cos_out = cosf(angle);
    }
}

/* 正向旋转 dq 参考到观测器坐标系（+delta）。 */
static void Sensorless_RotateToObserverFrame(
    float id, float iq, float phase_sin, float phase_cos, float *id_out, float *iq_out)
{
    *id_out = id * phase_cos - iq * phase_sin;
    *iq_out = id * phase_sin + iq * phase_cos;
}

/* 反向旋转观测器坐标系 dq 参考（-delta）；参数按值传入，原地旋转安全。 */
static void Sensorless_RotateFromObserverFrame(
    float id, float iq, float phase_sin, float phase_cos, float *id_out, float *iq_out)
{
    *id_out = id * phase_cos + iq * phase_sin;
    *iq_out = -id * phase_sin + iq * phase_cos;
}

/* 无感启动单 tick 上下文：绑定本 tick 的模块指针与派生量，避免逐状态重复传参。 */
typedef struct
{
    FOC_TypeDef *FOC;
    MotorControl_TypeDef *MotorControl;
    PI_Controller_TypeDef *controller;
    Fluxobserver_TypeDef *Fluxobserver;
    SensorlessStartup_TypeDef *Startup;
    const SensorlessStartupConfig_TypeDef *Config;
    float pole_pairs;
    float direction;
} SensorlessStartupRun_TypeDef;

/* 速度锁定完成：记录开环与观测器相位差，把开环系电流参考旋转换算到观测器系，
 * 并用现有输出预置速度 PI，避免交接瞬间产生转矩阶跃。 */
static void Sensorless_BeginHandoff(SensorlessStartupRun_TypeDef *Run)
{
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    float phase_sin;
    float phase_cos;
    float observer_iq;

    Startup->handoff_phase_delta = Sensorless_AngleDifference(
        Startup->open_loop_theta, Observer_GetElePhase(Run->Fluxobserver));
    phase_sin = sinf(Startup->handoff_phase_delta);
    phase_cos = cosf(Startup->handoff_phase_delta);
    /* 开环 dq 系可能带载角；若直接在观测器系复用其 Iq，对准电流会变成转矩。 */
    Sensorless_RotateToObserverFrame(MotorControl->idRef,
                                     MotorControl->iqRef,
                                     phase_sin,
                                     phase_cos,
                                     &Startup->handoff_id_reference,
                                     &observer_iq);
    Startup->speed_feedback = Startup->lock_speed_feedback / Run->pole_pairs;
    MotorControl->speedShadow = Startup->speed_feedback;
    Startup->speed_loop_ticks = 0U;
    PI_Controller_Reset(Run->controller);
    PI_Controller_Configure(Run->controller,
                            MotorControl->speed_Kp,
                            MotorControl->speed_Ki,
                            Speed_Ts,
                            -1.0f,
                            Startup->speed_pi_output_max);
    PI_Controller_TrackOutput(
        Run->controller,
        constrain(observer_iq / MotorControl->current_limit, -1.0f, Startup->speed_pi_output_max));
    Startup->state = SENSORLESS_STARTUP_HANDOFF;
    Startup->state_ticks = 0U;
}

/* 状态 1（ALIGN）：按斜坡施加 d 轴对准电流，保持时间结束后进入开环。 */
static void Sensorless_RunAlign(SensorlessStartupRun_TypeDef *Run)
{
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    const SensorlessStartupConfig_TypeDef *Config = Run->Config;

    MotorControl->idRef =
        Config->align_current_a * constrain(((float)Startup->state_ticks + 1.0f) * Current_Ts /
                                                Config->align_current_ramp_time_s,
                                            0.0f,
                                            1.0f);
    MotorControl->iqRef = 0.0f;
    FOC_Current(Run->FOC, MotorControl, 0.0f, 0.0f);
    if (++Startup->state_ticks >=
        (uint32_t)((Config->align_current_ramp_time_s + Config->align_hold_time_s) / Current_Ts))
    {
        Startup->state = SENSORLESS_STARTUP_OPEN_LOOP;
        Startup->open_loop_theta = 0.0f;
        Startup->open_loop_omega = 0.0f;
        Startup->direction = Run->direction;
        Startup->state_ticks = 0U;
        Startup->lock_ticks = 0U;
    }
}

/* 状态 2（OPEN_LOOP）：开环强拖。电角速度沿斜坡升到目标值，iq 同步爬升；
 * 方向请求变化时直接复位状态机。 */
static void Sensorless_RunOpenLoop(SensorlessStartupRun_TypeDef *Run)
{
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    const SensorlessStartupConfig_TypeDef *Config = Run->Config;
    float iq_ramp_ratio;

    if (Run->direction != Startup->direction)
    {
        SensorlessStartup_Reset(Startup);
        return;
    }

    if (fast_abs(Startup->open_loop_omega) < Config->target_electrical_velocity_rad_s)
    {
        Startup->open_loop_omega +=
            Startup->direction *
            (Config->target_electrical_velocity_rad_s / Config->startup_ramp_time_s) * Current_Ts;
        if (fast_abs(Startup->open_loop_omega) >= Config->target_electrical_velocity_rad_s)
        {
            Startup->open_loop_omega =
                Startup->direction * Config->target_electrical_velocity_rad_s;
        }
    }

    Startup->open_loop_ticks++;
    iq_ramp_ratio = constrain(
        (float)Startup->open_loop_ticks * Current_Ts / Config->startup_iq_ramp_time_s, 0.0f, 1.0f);

    Startup->open_loop_theta =
        normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
    MotorControl->idRef = Config->startup_id_a;
    MotorControl->iqRef = Startup->direction *
                          (Config->startup_iq_initial_a +
                           (Config->startup_iq_a - Config->startup_iq_initial_a) * iq_ramp_ratio);
    FOC_Current(Run->FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

    if (fast_abs(Startup->open_loop_omega) >= Config->target_electrical_velocity_rad_s)
    {
        Startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
        Startup->state_ticks = 0U;
        Startup->lock_ticks = 0U;
    }
}

/* 状态 3（SPEED_LOCK）：速度锁定。开环强拖期间比较滤波后的观测器速度，
 * 锁定足够长时间后进入交接；观测器不可用、方向变化或锁定超时按故障退出。 */
static void Sensorless_RunSpeedLock(SensorlessStartupRun_TypeDef *Run)
{
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    const SensorlessStartupConfig_TypeDef *Config = Run->Config;
    Fluxobserver_TypeDef *Fluxobserver = Run->Fluxobserver;
    float observer_velocity;
    float speed_error;
    bool is_observer_locked;

    if (Run->direction != Startup->direction || !Sensorless_ObserverIsUsable(Fluxobserver))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    Startup->open_loop_theta =
        normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
    Startup->state_ticks++;

    MotorControl->idRef = Config->startup_id_a;
    MotorControl->iqRef = Startup->direction * Config->startup_iq_a;
    FOC_Current(Run->FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

    observer_velocity = Observer_GetEleVel(Fluxobserver);
    if (Startup->state_ticks == 1U)
    {
        Startup->lock_speed_feedback = observer_velocity;
    }
    else
    {
        Startup->lock_speed_feedback +=
            Config->speed_lock_filter_alpha * (observer_velocity - Startup->lock_speed_feedback);
    }

    speed_error = fast_abs(Startup->lock_speed_feedback - Startup->open_loop_omega);
    is_observer_locked =
        Startup->lock_speed_feedback * Startup->open_loop_omega > 0.0f &&
        speed_error <= fast_abs(Startup->open_loop_omega) * Config->observer_lock_ratio;

    if (is_observer_locked)
    {
        Startup->lock_ticks++;
    }
    else
    {
        Startup->lock_ticks = 0U;
    }

    if (Startup->lock_ticks >= (uint32_t)(Config->speed_lock_time_s / Current_Ts))
    {
        Sensorless_BeginHandoff(Run);
    }

    if (Startup->state_ticks >= (uint32_t)(Config->lock_timeout_s / Current_Ts))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }
}

/* 状态 4（HANDOFF）：角度交接。开环与观测器参考按 blend 混合，电流参考随之旋转；
 * 只把刻意的坐标系修正量旋转进电流环积分器，避免存电压交接阶跃。 */
static void Sensorless_RunHandoff(SensorlessStartupRun_TypeDef *Run)
{
    FOC_TypeDef *FOC = Run->FOC;
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    const SensorlessStartupConfig_TypeDef *Config = Run->Config;
    Fluxobserver_TypeDef *Fluxobserver = Run->Fluxobserver;
    float blend;
    float phase;
    float phase_vel;
    float phase_offset;
    float frame_sin;
    float frame_cos;
    float observer_id;
    float observer_iq;
    float frame_step;
    float previous_blend;

    if (Run->direction != Startup->direction || !Sensorless_ObserverIsUsable(Fluxobserver))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    Startup->open_loop_theta =
        normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
    previous_blend = constrain(
        (float)Startup->state_ticks * Current_Ts / Config->angle_handoff_time_s, 0.0f, 1.0f);
    blend = constrain(
        (float)(++Startup->state_ticks) * Current_Ts / Config->angle_handoff_time_s, 0.0f, 1.0f);
    phase_offset = (1.0f - blend) * Startup->handoff_phase_delta;
    phase = normalizeAngle(Observer_GetElePhase(Fluxobserver) + phase_offset);
    frame_step = (previous_blend - blend) * Startup->handoff_phase_delta;
    phase_vel = Observer_GetEleVel(Fluxobserver) + frame_step / Current_Ts;

    observer_iq = Sensorless_UpdateSpeedLoop(
        MotorControl, Run->controller, Startup, Observer_GetEleVel(Fluxobserver) / Run->pole_pairs);
    observer_id = Startup->handoff_id_reference +
                  blend * (Config->startup_id_a - Startup->handoff_id_reference);
    Sensorless_RotateFromObserverFrame(observer_id,
                                       observer_iq,
                                       sinf(phase_offset),
                                       cosf(phase_offset),
                                       &MotorControl->idRef,
                                       &MotorControl->iqRef);

    /* 只旋转刻意的坐标系修正量，不旋转转子正常行进，使电流环存电压无交接阶跃。
     * 粗正弦表会把亚分度步长舍入为零，用有界小角度展开避免旋转误差累积。 */
    Sensorless_SmallAngleSinCos(frame_step, &frame_sin, &frame_cos);
    Sensorless_RotateFromObserverFrame(
        FOC->id_pi.Ui, FOC->iq_pi.Ui, frame_sin, frame_cos, &FOC->id_pi.Ui, &FOC->iq_pi.Ui);
    FOC_Current(Run->FOC, MotorControl, phase, phase_vel);

    if (blend >= 1.0f)
    {
        Startup->state = SENSORLESS_STARTUP_CLOSED_LOOP;
        Startup->state_ticks = 0U;
        Startup->id_ramp_ticks = 0U;
        Startup->loss_ticks = 0U;
        /* 保留速度 PI、滤波反馈与参考穿过本边界。 */
    }
}

/* 状态 5（CLOSED_LOOP）：观测器闭环。id 参考沿斜坡退出，速度环驱动 iq；
 * 低速反向复位等待下一轮启动，观测器丢失超时按故障退出。 */
static void Sensorless_RunClosedLoop(SensorlessStartupRun_TypeDef *Run)
{
    MotorControl_TypeDef *MotorControl = Run->MotorControl;
    SensorlessStartup_TypeDef *Startup = Run->Startup;
    const SensorlessStartupConfig_TypeDef *Config = Run->Config;
    Fluxobserver_TypeDef *Fluxobserver = Run->Fluxobserver;
    float observer_vel = Observer_GetEleVel(Fluxobserver);
    float observer_mech_vel = observer_vel / Run->pole_pairs;

    if (!Sensorless_ObserverIsUsable(Fluxobserver))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    if (Run->direction * observer_vel < 0.0f &&
        fast_abs(observer_vel) < Config->minimum_electrical_velocity_rad_s)
    {
        SensorlessStartup_Reset(Startup);
        FOC_CurrentController_Reset(Run->FOC);
        PI_Controller_Reset(Run->controller);
        return;
    }

    MotorControl->idRef =
        Config->startup_id_a *
        (1.0f - constrain((float)Startup->id_ramp_ticks * Current_Ts / Config->id_ramp_down_time_s,
                          0.0f,
                          1.0f));
    if (Startup->id_ramp_ticks < (uint32_t)(Config->id_ramp_down_time_s / Current_Ts))
    {
        Startup->id_ramp_ticks++;
    }

    MotorControl->iqRef =
        Sensorless_UpdateSpeedLoop(MotorControl, Run->controller, Startup, observer_mech_vel);

    if (fast_abs(observer_vel) < Config->minimum_electrical_velocity_rad_s * 0.5f)
    {
        Startup->loss_ticks++;
    }
    else
    {
        Startup->loss_ticks = 0U;
    }

    if (Startup->loss_ticks >= (uint32_t)(Config->observer_loss_time_s / Current_Ts))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    FOC_Current(Run->FOC, MotorControl, Observer_GetElePhase(Fluxobserver), observer_vel);
}

/**
 * @brief  无感速度控制任务：对准、开环启动与观测器交接。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  Fluxobserver 磁链观测器状态指针。
 * @param  Startup 无感启动状态机指针。
 * @param  Config 启动参数集，默认配置或编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；准入失败置故障并停止本 tick 输出。
 */
void SensorlessStartup_Run(FOC_TypeDef *FOC,
                           MotorControl_TypeDef *MotorControl,
                           PI_Controller_TypeDef *controller,
                           Fluxobserver_TypeDef *Fluxobserver,
                           SensorlessStartup_TypeDef *Startup,
                           const SensorlessStartupConfig_TypeDef *Config)
{
    float pole_pairs = (float)MotorControl->motor_pole_pairs;
    float min_mech_vel;
    SensorlessStartupRun_TypeDef run;

    if (!Sensorless_StartupConfigIsValid(Config) || pole_pairs <= 0.0f ||
        MotorControl->motor_phase_resistance <= 0.0f || MotorControl->motor_d_inductance <= 0.0f ||
        MotorControl->motor_q_inductance <= 0.0f || MotorControl->motor_flux <= 0.0f ||
        MotorControl->current_limit <= 0.0f)
    {
        Set_ErrorNow(MotorParam_Error);
        return;
    }

    if (fast_abs(MotorControl->speedRef) <= 1e-4f)
    {
        SensorlessStartup_Reset(Startup);
        PI_Controller_Reset(controller);
        MotorControl->speedShadow = 0.0f;
        MotorControl->idRef = 0.0f;
        MotorControl->iqRef = 0.0f;
        FOC_Current(FOC, MotorControl, 0.0f, 0.0f);
        return;
    }

    min_mech_vel = Config->minimum_electrical_velocity_rad_s / pole_pairs;
    if (fast_abs(MotorControl->speedRef) < min_mech_vel)
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    if (!Sensorless_StartupCurrentsAreValid(MotorControl, Config))
    {
        Set_ErrorNow(Sensorless_Error);
        return;
    }

    run.FOC = FOC;
    run.MotorControl = MotorControl;
    run.controller = controller;
    run.Fluxobserver = Fluxobserver;
    run.Startup = Startup;
    run.Config = Config;
    run.pole_pairs = pole_pairs;
    run.direction = MotorControl->speedRef >= 0.0f ? 1.0f : -1.0f;

    if (Startup->state == SENSORLESS_STARTUP_IDLE)
    {
        Fluxobserver_ParamInit(Fluxobserver);
        FOC_CurrentController_Reset(FOC);
        PI_Controller_Reset(controller);
        Startup->state = SENSORLESS_STARTUP_ALIGN;
        Startup->state_ticks = 0U;
        Startup->direction = run.direction;
    }

    switch (Startup->state)
    {
    case SENSORLESS_STARTUP_ALIGN:
        Sensorless_RunAlign(&run);
        break;

    case SENSORLESS_STARTUP_OPEN_LOOP:
        Sensorless_RunOpenLoop(&run);
        break;

    case SENSORLESS_STARTUP_SPEED_LOCK:
        Sensorless_RunSpeedLock(&run);
        break;

    case SENSORLESS_STARTUP_HANDOFF:
        Sensorless_RunHandoff(&run);
        break;

    case SENSORLESS_STARTUP_CLOSED_LOOP:
        Sensorless_RunClosedLoop(&run);
        break;

    default:
        SensorlessStartup_Reset(Startup);
        break;
    }
}

/* SENSORLESS_RUNTIME_END */
