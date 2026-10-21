#include "sensorless_startup.h"

#include <math.h>
#include <stddef.h>

#include "motor_fault.h"
#include "speed_control.h"
#include "foc_pid.h"
#include "control_config.h"
#include "motor_startup_profile.h"
#include "control_math.h"

/* 无感运行应用：启动序列（对准→开环→速度锁定→交接→闭环）与速度模式。
 * 观测器算法位于 foc_sensorless.{c,h}；本文件只编排控制流与安全门限。 */

void SensorlessStartup_Reset(SensorlessStartupState *startup)
{
    startup->state = SENSORLESS_STARTUP_IDLE;
    startup->open_loop_angle_rad = 0.0f;
    startup->open_loop_velocity_rad_s = 0.0f;
    startup->handoff_phase_delta_rad = 0.0f;
    startup->handoff_id_reference_a = 0.0f;
    startup->speed_feedback_rad_s = 0.0f;
    startup->lock_speed_feedback_rad_s = 0.0f;
    startup->direction = 1.0f;
    startup->speed_pi_output_max = 1.0f;
    startup->state_ticks = 0U;
    startup->speed_loop_ticks = 0U;
    startup->open_loop_ticks = 0U;
    startup->lock_ticks = 0U;
    startup->id_ramp_ticks = 0U;
    startup->loss_ticks = 0U;
}

/* SENSORLESS_RUNTIME_BEGIN
 * 无感速度任务区：对准→开环→速度锁定→角度交接→闭环，与编码器标定模式共享。
 * 离线测试按本标记切取源码，标记文字不得修改。 */

/* 角度差归一化到 -PI..PI。 */
static float sensorless_angle_difference(float target, float source)
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

/* 编码器标定（Mode 13）观测器启动参数集：更强的对准/启动电流与低频稳定目标，
 * 由 foc_encoder_calibration 模块使用；保留现有启动结构契约。 */
const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig = {
    SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
    SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_INITIAL_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_STARTUP_ID_A,
    SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A,
    SENSORLESS_ENCODER_CALIB_MIN_ELEC_VEL_RAD_S,
    SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S,
    SENSORLESS_ENCODER_CALIB_STARTUP_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_SPEED_LOCK_TIME_S,
    SENSORLESS_ENCODER_CALIB_SPEED_LOCK_FILTER_ALPHA,
    SENSORLESS_ENCODER_CALIB_OBSERVER_LOCK_RATIO,
    SENSORLESS_ENCODER_CALIB_ANGLE_HANDOFF_TIME_S,
    SENSORLESS_ENCODER_CALIB_LOCK_TIMEOUT_S,
    SENSORLESS_ENCODER_CALIB_ID_RAMP_DOWN_TIME_S,
    SENSORLESS_ENCODER_CALIB_OBSERVER_LOSS_TIME_S};

/* 观测器可用性：有限值且角速度在最大电角速度以内。 */
static bool sensorless_observer_is_usable(const FluxObserverState *flux_observer)
{
    return flux_observer->electrical_angle_rad == flux_observer->electrical_angle_rad &&
           flux_observer->electrical_velocity_rad_s == flux_observer->electrical_velocity_rad_s &&
           Math_Abs(flux_observer->electrical_velocity_rad_s) <=
               SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

/* 启动参数集准入：时间、电流与速度阈值必须自洽。 */
static bool sensorless_startup_config_is_valid(const SensorlessStartupConfig *Config)
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
static bool sensorless_startup_currents_are_valid(const MotorConfiguration *configuration,
                                                  const SensorlessStartupConfig *Config)
{
    float current_limit_squared =
        configuration->application.current_limit_a * configuration->application.current_limit_a;
    float startup_current_squared =
        Config->startup_iq_a * Config->startup_iq_a + Config->startup_id_a * Config->startup_id_a;

    return configuration->application.current_limit_a >= Config->minimum_current_limit_a &&
           configuration->application.current_limit_a >= Config->align_current_a &&
           startup_current_squared <= current_limit_squared;
}

/* 速度 PI 在角度交接与正常跟踪期间保持同一实例运行；
 * 输出始终是观测器坐标系下的转矩电流参考。 */
static float sensorless_update_speed_loop(const MotorConfiguration *configuration,
                                          MotorControlState *motor_state,
                                          PiController *controller,
                                          SensorlessStartupState *startup,
                                          float observer_mech_vel)
{
    if (++startup->speed_loop_ticks >= SPEED_LOOP_DIVIDER)
    {
        startup->speed_feedback_rad_s += SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA *
                                         (observer_mech_vel - startup->speed_feedback_rad_s);
        MotorAxis_UpdateSpeedRamp(configuration, motor_state);
        PiController_Configure(controller,
                               configuration->application.speed_kp,
                               configuration->application.speed_ki,
                               Speed_Ts,
                               -1.0f,
                               startup->speed_pi_output_max);
        PiController_Run(
            controller, motor_state->speed_ramped_rad_s, startup->speed_feedback_rad_s);
        startup->speed_loop_ticks = 0U;
    }
    return Math_Constrain(controller->output, -1.0f, startup->speed_pi_output_max) *
           configuration->application.current_limit_a;
}

/* 正向旋转 dq 参考到观测器坐标系（+delta）。 */
static void sensorless_rotate_to_observer_frame(
    float id, float iq, float phase_sin, float phase_cos, float *id_out, float *iq_out)
{
    *id_out = id * phase_cos - iq * phase_sin;
    *iq_out = id * phase_sin + iq * phase_cos;
}

/* 反向旋转观测器坐标系 dq 参考（-delta）；参数按值传入，原地旋转安全。 */
static void sensorless_rotate_from_observer_frame(
    float id, float iq, float phase_sin, float phase_cos, float *id_out, float *iq_out)
{
    *id_out = id * phase_cos + iq * phase_sin;
    *iq_out = -id * phase_sin + iq * phase_cos;
}

/* 无感启动单 tick 上下文：绑定本 tick 的模块指针与派生量，避免逐状态重复传参。 */
typedef struct
{
    const MotorConfiguration *configuration;
    FocState *foc;
    MotorControlState *motor_state;
    PiController *controller;
    FluxObserverState *flux_observer;
    SensorlessStartupState *startup;
    const SensorlessStartupConfig *Config;
    float pole_pairs;
    float direction;
} SensorlessStartupRunState;

/* 速度锁定完成：记录开环与观测器相位差，把开环系电流参考旋转换算到观测器系，
 * 并用现有输出预置速度 PI，避免交接瞬间产生转矩阶跃。 */
static void sensorless_begin_handoff(SensorlessStartupRunState *Run)
{
    const MotorConfiguration *configuration = Run->configuration;
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    float phase_sin;
    float phase_cos;
    float observer_iq;

    startup->handoff_phase_delta_rad = sensorless_angle_difference(
        startup->open_loop_angle_rad, Observer_GetElePhase(Run->flux_observer));
    Math_SinCos(startup->handoff_phase_delta_rad, &phase_sin, &phase_cos);
    /* 开环 dq 系可能带载角；若直接在观测器系复用其 iq_a，对准电流会变成转矩。 */
    sensorless_rotate_to_observer_frame(motor_state->id_target_a,
                                        motor_state->iq_target_a,
                                        phase_sin,
                                        phase_cos,
                                        &startup->handoff_id_reference_a,
                                        &observer_iq);
    startup->speed_feedback_rad_s = startup->lock_speed_feedback_rad_s / Run->pole_pairs;
    motor_state->speed_ramped_rad_s = startup->speed_feedback_rad_s;
    startup->speed_loop_ticks = 0U;
    PiController_Reset(Run->controller);
    PiController_Configure(Run->controller,
                           configuration->application.speed_kp,
                           configuration->application.speed_ki,
                           Speed_Ts,
                           -1.0f,
                           startup->speed_pi_output_max);
    PiController_TrackOutput(
        Run->controller,
        Math_Constrain(observer_iq / configuration->application.current_limit_a,
                       -1.0f,
                       startup->speed_pi_output_max));
    startup->state = SENSORLESS_STARTUP_HANDOFF;
    startup->state_ticks = 0U;
}

/* 状态 1（ALIGN）：按斜坡施加 d 轴对准电流，保持时间结束后进入开环。 */
static void sensorless_run_align(SensorlessStartupRunState *Run)
{
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    const SensorlessStartupConfig *Config = Run->Config;

    motor_state->id_target_a =
        Config->align_current_a * Math_Constrain(((float)startup->state_ticks + 1.0f) * Current_Ts /
                                                     Config->align_current_ramp_time_s,
                                                 0.0f,
                                                 1.0f);
    motor_state->iq_target_a = 0.0f;
    FOC_Current(Run->configuration, Run->foc, motor_state, 0.0f, 0.0f);
    if (++startup->state_ticks >=
        (uint32_t)((Config->align_current_ramp_time_s + Config->align_hold_time_s) / Current_Ts))
    {
        startup->state = SENSORLESS_STARTUP_OPEN_LOOP;
        startup->open_loop_angle_rad = 0.0f;
        startup->open_loop_velocity_rad_s = 0.0f;
        startup->direction = Run->direction;
        startup->state_ticks = 0U;
        startup->lock_ticks = 0U;
    }
}

/* 状态 2（OPEN_LOOP）：开环强拖。电角速度沿斜坡升到目标值，iq 同步爬升；
 * 方向请求变化时直接复位状态机。 */
static void sensorless_run_open_loop(SensorlessStartupRunState *Run)
{
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    const SensorlessStartupConfig *Config = Run->Config;
    float iq_ramp_ratio;

    if (Run->direction != startup->direction)
    {
        SensorlessStartup_Reset(startup);
        return;
    }

    if (Math_Abs(startup->open_loop_velocity_rad_s) < Config->target_electrical_velocity_rad_s)
    {
        startup->open_loop_velocity_rad_s +=
            startup->direction *
            (Config->target_electrical_velocity_rad_s / Config->startup_ramp_time_s) * Current_Ts;
        if (Math_Abs(startup->open_loop_velocity_rad_s) >= Config->target_electrical_velocity_rad_s)
        {
            startup->open_loop_velocity_rad_s =
                startup->direction * Config->target_electrical_velocity_rad_s;
        }
    }

    startup->open_loop_ticks++;
    iq_ramp_ratio = Math_Constrain(
        (float)startup->open_loop_ticks * Current_Ts / Config->startup_iq_ramp_time_s, 0.0f, 1.0f);

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * Current_Ts);
    motor_state->id_target_a = Config->startup_id_a;
    motor_state->iq_target_a =
        startup->direction *
        (Config->startup_iq_initial_a +
         (Config->startup_iq_a - Config->startup_iq_initial_a) * iq_ramp_ratio);
    FOC_Current(Run->configuration,
                Run->foc,
                motor_state,
                startup->open_loop_angle_rad,
                startup->open_loop_velocity_rad_s);

    if (Math_Abs(startup->open_loop_velocity_rad_s) >= Config->target_electrical_velocity_rad_s)
    {
        startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
        startup->state_ticks = 0U;
        startup->lock_ticks = 0U;
    }
}

/* 状态 3（SPEED_LOCK）：速度锁定。开环强拖期间比较滤波后的观测器速度，
 * 锁定足够长时间后进入交接；观测器不可用、方向变化或锁定超时按故障退出。 */
static void sensorless_run_speed_lock(SensorlessStartupRunState *Run)
{
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    const SensorlessStartupConfig *Config = Run->Config;
    FluxObserverState *flux_observer = Run->flux_observer;
    float observer_velocity;
    float speed_error;
    bool is_observer_locked;

    if (Run->direction != startup->direction || !sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * Current_Ts);
    startup->state_ticks++;

    motor_state->id_target_a = Config->startup_id_a;
    motor_state->iq_target_a = startup->direction * Config->startup_iq_a;
    FOC_Current(Run->configuration,
                Run->foc,
                motor_state,
                startup->open_loop_angle_rad,
                startup->open_loop_velocity_rad_s);

    observer_velocity = Observer_GetEleVel(flux_observer);
    if (startup->state_ticks == 1U)
    {
        startup->lock_speed_feedback_rad_s = observer_velocity;
    }
    else
    {
        startup->lock_speed_feedback_rad_s +=
            Config->speed_lock_filter_alpha *
            (observer_velocity - startup->lock_speed_feedback_rad_s);
    }

    speed_error = Math_Abs(startup->lock_speed_feedback_rad_s - startup->open_loop_velocity_rad_s);
    is_observer_locked =
        startup->lock_speed_feedback_rad_s * startup->open_loop_velocity_rad_s > 0.0f &&
        speed_error <= Math_Abs(startup->open_loop_velocity_rad_s) * Config->observer_lock_ratio;

    if (is_observer_locked)
    {
        startup->lock_ticks++;
    }
    else
    {
        startup->lock_ticks = 0U;
    }

    if (startup->lock_ticks >= (uint32_t)(Config->speed_lock_time_s / Current_Ts))
    {
        sensorless_begin_handoff(Run);
    }

    if (startup->state_ticks >= (uint32_t)(Config->lock_timeout_s / Current_Ts))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }
}

/* 状态 4（HANDOFF）：角度交接。开环与观测器参考按 blend 混合，电流参考随之旋转；
 * 只把刻意的坐标系修正量旋转进电流环积分器，避免存电压交接阶跃。 */
static void sensorless_run_handoff(SensorlessStartupRunState *Run)
{
    FocState *foc = Run->foc;
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    const SensorlessStartupConfig *Config = Run->Config;
    FluxObserverState *flux_observer = Run->flux_observer;
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

    if (Run->direction != startup->direction || !sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * Current_Ts);
    previous_blend = Math_Constrain(
        (float)startup->state_ticks * Current_Ts / Config->angle_handoff_time_s, 0.0f, 1.0f);
    blend = Math_Constrain(
        (float)(++startup->state_ticks) * Current_Ts / Config->angle_handoff_time_s, 0.0f, 1.0f);
    phase_offset = (1.0f - blend) * startup->handoff_phase_delta_rad;
    phase = Math_NormalizeAngle(Observer_GetElePhase(flux_observer) + phase_offset);
    frame_step = (previous_blend - blend) * startup->handoff_phase_delta_rad;
    phase_vel = Observer_GetEleVel(flux_observer) + frame_step / Current_Ts;

    observer_iq = sensorless_update_speed_loop(Run->configuration,
                                               motor_state,
                                               Run->controller,
                                               startup,
                                               Observer_GetEleVel(flux_observer) / Run->pole_pairs);
    observer_id = startup->handoff_id_reference_a +
                  blend * (Config->startup_id_a - startup->handoff_id_reference_a);
    Math_SinCos(phase_offset, &frame_sin, &frame_cos);
    sensorless_rotate_from_observer_frame(observer_id,
                                          observer_iq,
                                          frame_sin,
                                          frame_cos,
                                          &motor_state->id_target_a,
                                          &motor_state->iq_target_a);

    /* 只旋转刻意的坐标系修正量，不旋转转子正常行进，使电流环存电压无交接阶跃。
     * 统一插值查表保留亚分度步长，避免粗表量化造成旋转误差累积。 */
    Math_SinCos(frame_step, &frame_sin, &frame_cos);
    sensorless_rotate_from_observer_frame(foc->id_pi.integral_output,
                                          foc->iq_pi.integral_output,
                                          frame_sin,
                                          frame_cos,
                                          &foc->id_pi.integral_output,
                                          &foc->iq_pi.integral_output);
    FOC_Current(Run->configuration, Run->foc, motor_state, phase, phase_vel);

    if (blend >= 1.0f)
    {
        startup->state = SENSORLESS_STARTUP_CLOSED_LOOP;
        startup->state_ticks = 0U;
        startup->id_ramp_ticks = 0U;
        startup->loss_ticks = 0U;
        /* 保留速度 PI、滤波反馈与参考穿过本边界。 */
    }
}

/* 状态 5（CLOSED_LOOP）：观测器闭环。id 参考沿斜坡退出，速度环驱动 iq；
 * 低速反向复位等待下一轮启动，观测器丢失超时按故障退出。 */
static void sensorless_run_closed_loop(SensorlessStartupRunState *Run)
{
    MotorControlState *motor_state = Run->motor_state;
    SensorlessStartupState *startup = Run->startup;
    const SensorlessStartupConfig *Config = Run->Config;
    FluxObserverState *flux_observer = Run->flux_observer;
    float observer_vel = Observer_GetEleVel(flux_observer);
    float observer_mech_vel = observer_vel / Run->pole_pairs;

    if (!sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    if (Run->direction * observer_vel < 0.0f &&
        Math_Abs(observer_vel) < Config->minimum_electrical_velocity_rad_s)
    {
        SensorlessStartup_Reset(startup);
        FOC_CurrentController_Reset(Run->foc);
        PiController_Reset(Run->controller);
        return;
    }

    motor_state->id_target_a =
        Config->startup_id_a *
        (1.0f -
         Math_Constrain(
             (float)startup->id_ramp_ticks * Current_Ts / Config->id_ramp_down_time_s, 0.0f, 1.0f));
    if (startup->id_ramp_ticks < (uint32_t)(Config->id_ramp_down_time_s / Current_Ts))
    {
        startup->id_ramp_ticks++;
    }

    motor_state->iq_target_a = sensorless_update_speed_loop(
        Run->configuration, motor_state, Run->controller, startup, observer_mech_vel);

    if (Math_Abs(observer_vel) < Config->minimum_electrical_velocity_rad_s * 0.5f)
    {
        startup->loss_ticks++;
    }
    else
    {
        startup->loss_ticks = 0U;
    }

    if (startup->loss_ticks >= (uint32_t)(Config->observer_loss_time_s / Current_Ts))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    FOC_Current(Run->configuration,
                Run->foc,
                motor_state,
                Observer_GetElePhase(flux_observer),
                observer_vel);
}

/**
 * @brief  无感速度控制任务：对准、开环启动与观测器交接。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  flux_observer 磁链观测器状态指针。
 * @param  startup 无感启动状态机指针。
 * @param  Config 启动参数集，默认配置或编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；准入失败置故障并停止本 tick 输出。
 */
void SensorlessStartup_Run(const MotorConfiguration *configuration,
                           FocState *foc,
                           MotorControlState *motor_state,
                           PiController *controller,
                           FluxObserverState *flux_observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *Config)
{
    float pole_pairs = (float)configuration->model.motor_pole_pairs;
    float min_mech_vel;
    SensorlessStartupRunState run;

    if (!sensorless_startup_config_is_valid(Config) || pole_pairs <= 0.0f ||
        configuration->model.motor_phase_resistance_ohm <= 0.0f ||
        configuration->model.motor_d_inductance_h <= 0.0f ||
        configuration->model.motor_q_inductance_h <= 0.0f ||
        configuration->model.motor_flux_wb <= 0.0f ||
        configuration->application.current_limit_a <= 0.0f)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
        return;
    }

    if (Math_Abs(motor_state->speed_target_rad_s) <= 1e-4f)
    {
        SensorlessStartup_Reset(startup);
        PiController_Reset(controller);
        motor_state->speed_ramped_rad_s = 0.0f;
        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = 0.0f;
        FOC_Current(configuration, foc, motor_state, 0.0f, 0.0f);
        return;
    }

    min_mech_vel = Config->minimum_electrical_velocity_rad_s / pole_pairs;
    if (Math_Abs(motor_state->speed_target_rad_s) < min_mech_vel)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    if (!sensorless_startup_currents_are_valid(configuration, Config))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    run.configuration = configuration;
    run.foc = foc;
    run.motor_state = motor_state;
    run.controller = controller;
    run.flux_observer = flux_observer;
    run.startup = startup;
    run.Config = Config;
    run.pole_pairs = pole_pairs;
    run.direction = motor_state->speed_target_rad_s >= 0.0f ? 1.0f : -1.0f;

    if (startup->state == SENSORLESS_STARTUP_IDLE)
    {
        FluxObserver_ParamInit(flux_observer);
        FOC_CurrentController_Reset(foc);
        PiController_Reset(controller);
        startup->state = SENSORLESS_STARTUP_ALIGN;
        startup->state_ticks = 0U;
        startup->direction = run.direction;
    }

    switch (startup->state)
    {
    case SENSORLESS_STARTUP_ALIGN:
        sensorless_run_align(&run);
        break;

    case SENSORLESS_STARTUP_OPEN_LOOP:
        sensorless_run_open_loop(&run);
        break;

    case SENSORLESS_STARTUP_SPEED_LOCK:
        sensorless_run_speed_lock(&run);
        break;

    case SENSORLESS_STARTUP_HANDOFF:
        sensorless_run_handoff(&run);
        break;

    case SENSORLESS_STARTUP_CLOSED_LOOP:
        sensorless_run_closed_loop(&run);
        break;

    default:
        SensorlessStartup_Reset(startup);
        break;
    }
}

/* SENSORLESS_RUNTIME_END */
