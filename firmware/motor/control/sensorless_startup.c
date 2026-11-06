#include "sensorless_startup.h"

#include <math.h>
#include <stddef.h>

#include "motor_fault.h"
#include "speed_control.h"
#include "pi_controller.h"
#include "application_config.h"
#include "control_math.h"

/* 无感运行应用：启动序列（对准→开环→速度锁定→交接→闭环）与速度模式。
 * 观测器算法位于 flux_observer.{c,h}；本文件只编排控制流与安全门限。 */

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
static float sensorless_angle_difference(float target_rad, float source_rad)
{
    float difference_rad = target_rad - source_rad;

    if (difference_rad > _PI)
    {
        difference_rad -= _2PI;
    }
    else if (difference_rad < -_PI)
    {
        difference_rad += _2PI;
    }

    return difference_rad;
}

/* 编码器标定（Mode 13）观测器启动参数集：更强的对准/启动电流与低频稳定目标，
 * 由 encoder_calibration 模块使用；保留现有启动结构契约。 */
const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig =
    CALIB_STARTUP_CONFIG(SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_A,
                         SENSORLESS_ENCODER_CALIB_STARTUP_IQ_A,
                         SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S,
                         0.05f,
                         0.5f);

/* 观测器可用性：有限值且角速度在最大电角速度以内。 */
static bool sensorless_observer_is_usable(const FluxObserverState *flux_observer)
{
    return flux_observer->electrical_angle_rad == flux_observer->electrical_angle_rad &&
           flux_observer->electrical_velocity_rad_s == flux_observer->electrical_velocity_rad_s &&
           Math_Abs(flux_observer->electrical_velocity_rad_s) <=
               SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

/* 启动参数集准入：时间、电流与速度阈值必须自洽。 */
#define SENSORLESS_STARTUP_VALUES_VALID(align_current_ramp_time_s,                                 \
                                        align_hold_time_s,                                         \
                                        align_current_a,                                           \
                                        startup_iq_initial_a,                                      \
                                        startup_iq_a,                                              \
                                        startup_iq_ramp_time_s,                                    \
                                        startup_id_a,                                              \
                                        minimum_current_limit_a,                                   \
                                        minimum_electrical_velocity_rad_s,                         \
                                        target_electrical_velocity_rad_s,                          \
                                        startup_ramp_time_s,                                       \
                                        speed_lock_time_s,                                         \
                                        speed_lock_filter_alpha,                                   \
                                        observer_lock_ratio,                                       \
                                        angle_handoff_time_s,                                      \
                                        lock_timeout_s,                                            \
                                        id_ramp_down_time_s,                                       \
                                        observer_loss_time_s)                                      \
    (align_current_ramp_time_s > 0.0f && align_hold_time_s >= 0.0f && align_current_a > 0.0f &&    \
     startup_iq_initial_a >= 0.0f && startup_iq_a >= startup_iq_initial_a &&                       \
     startup_iq_ramp_time_s > 0.0f && startup_id_a >= 0.0f && minimum_current_limit_a >= 0.0f &&   \
     minimum_electrical_velocity_rad_s > 0.0f &&                                                   \
     target_electrical_velocity_rad_s >= minimum_electrical_velocity_rad_s &&                      \
     startup_ramp_time_s > 0.0f && speed_lock_time_s > 0.0f && speed_lock_filter_alpha > 0.0f &&   \
     speed_lock_filter_alpha <= 1.0f && observer_lock_ratio > 0.0f &&                              \
     angle_handoff_time_s > 0.0f && lock_timeout_s >= speed_lock_time_s &&                         \
     id_ramp_down_time_s > 0.0f && observer_loss_time_s > 0.0f)
static bool sensorless_startup_config_is_valid(const SensorlessStartupConfig *config)
{
    return config != NULL && isfinite(config->speed_kp) && config->speed_kp >= 0.0f &&
           isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
           SENSORLESS_STARTUP_VALUES_VALID(config->align_current_ramp_time_s,
                                           config->align_hold_time_s,
                                           config->align_current_a,
                                           config->startup_iq_initial_a,
                                           config->startup_iq_a,
                                           config->startup_iq_ramp_time_s,
                                           config->startup_id_a,
                                           config->minimum_current_limit_a,
                                           config->minimum_electrical_velocity_rad_s,
                                           config->target_electrical_velocity_rad_s,
                                           config->startup_ramp_time_s,
                                           config->speed_lock_time_s,
                                           config->speed_lock_filter_alpha,
                                           config->observer_lock_ratio,
                                           config->angle_handoff_time_s,
                                           config->lock_timeout_s,
                                           config->id_ramp_down_time_s,
                                           config->observer_loss_time_s);
}

/* 与运行时共用同一谓词；只读内置表在编译时求值，不增加可变许可缓存。 */
static const bool s_encoder_startup_config_valid =
    SENSORLESS_STARTUP_VALUES_VALID(SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
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
                                    SENSORLESS_ENCODER_CALIB_OBSERVER_LOSS_TIME_S);
#undef SENSORLESS_STARTUP_VALUES_VALID

/* 启动电流准入：对准与开环电流的合成矢量不得超出电流限制。 */
static float startup_current_limit(const MotorConfiguration *configuration)
{
    float current_limit_a = configuration->application.current_limit_a;
    return configuration->preset == NULL
               ? current_limit_a
               : fminf(current_limit_a, configuration->model.calibration_current_a);
}

static bool sensorless_startup_currents_are_valid(const MotorConfiguration *configuration,
                                                  const SensorlessStartupConfig *config)
{
    float current_limit_a = startup_current_limit(configuration);
    float startup_current_squared_a2 =
        config->startup_iq_a * config->startup_iq_a + config->startup_id_a * config->startup_id_a;

    return current_limit_a >= config->minimum_current_limit_a &&
           current_limit_a >= config->align_current_a &&
           startup_current_squared_a2 <= current_limit_a * current_limit_a;
}

/* 交接预置和闭环调速共用标定增益；配置限幅时保留积分状态。 */
static void configure_speed_pi(const SensorlessStartupConfig *config,
                               const SensorlessStartupState *startup,
                               PiController *controller)
{
    PiController_Configure(controller,
                           config->speed_kp,
                           config->speed_ki,
                           SPEED_LOOP_TS_S,
                           -1.0f,
                           startup->speed_pi_output_max);
}

/* 速度 PI 在角度交接与正常跟踪期间保持同一实例运行；
 * 输出始终是观测器坐标系下的转矩电流参考。 */
static float sensorless_update_speed_loop(const MotorConfiguration *configuration,
                                          const SensorlessStartupConfig *config,
                                          MotorControlState *motor_state,
                                          PiController *controller,
                                          SensorlessStartupState *startup,
                                          float observer_mech_vel_rad_s)
{
    if (++startup->speed_loop_ticks >= SPEED_LOOP_DIVIDER)
    {
        startup->speed_feedback_rad_s += SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA *
                                         (observer_mech_vel_rad_s - startup->speed_feedback_rad_s);
        SpeedControl_UpdateRamp(configuration, motor_state);
        configure_speed_pi(config, startup, controller);
        PiController_Run(
            controller, motor_state->speed_ramped_rad_s, startup->speed_feedback_rad_s);
        startup->speed_loop_ticks = 0U;
    }
    return Math_Constrain(controller->output, -1.0f, startup->speed_pi_output_max) *
           startup_current_limit(configuration);
}

/* 正向旋转 dq 分量到观测器坐标系（+delta）；输入输出单位一致。 */
static void sensorless_rotate_to_observer_frame(float d_component,
                                                float q_component,
                                                float phase_sin,
                                                float phase_cos,
                                                float *d_out,
                                                float *q_out)
{
    *d_out = d_component * phase_cos - q_component * phase_sin;
    *q_out = d_component * phase_sin + q_component * phase_cos;
}

/* 反向旋转观测器坐标系 dq 分量（-delta），用于电流参考及电压积分量。
 * 输入输出单位一致；参数按值传入，原地旋转安全。 */
static void sensorless_rotate_from_observer_frame(float d_component,
                                                  float q_component,
                                                  float phase_sin,
                                                  float phase_cos,
                                                  float *d_out,
                                                  float *q_out)
{
    *d_out = d_component * phase_cos + q_component * phase_sin;
    *q_out = -d_component * phase_sin + q_component * phase_cos;
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
    const SensorlessStartupConfig *config;
    float pole_pairs;
    float direction;
} SensorlessStartupRunState;

/* 速度锁定完成：记录开环与观测器相位差，把开环系电流参考旋转换算到观测器系，
 * 并用现有输出预置速度 PI，避免交接瞬间产生转矩阶跃。 */
static void sensorless_begin_handoff(SensorlessStartupRunState *run)
{
    const MotorConfiguration *configuration = run->configuration;
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    float phase_sin;
    float phase_cos;
    float observer_iq_a;

    startup->handoff_phase_delta_rad = sensorless_angle_difference(
        startup->open_loop_angle_rad, FluxObserver_GetElecPhase(run->flux_observer));
    Math_SinCos(startup->handoff_phase_delta_rad, &phase_sin, &phase_cos);
    /* 开环 dq 系可能带载角；若直接在观测器系复用其 q_component，对准电流会变成转矩。 */
    sensorless_rotate_to_observer_frame(motor_state->id_target_a,
                                        motor_state->iq_target_a,
                                        phase_sin,
                                        phase_cos,
                                        &startup->handoff_id_reference_a,
                                        &observer_iq_a);
    startup->speed_feedback_rad_s = startup->lock_speed_feedback_rad_s / run->pole_pairs;
    motor_state->speed_ramped_rad_s = startup->speed_feedback_rad_s;
    startup->speed_loop_ticks = 0U;
    PiController_Reset(run->controller);
    configure_speed_pi(run->config, startup, run->controller);
    PiController_TrackOutput(run->controller,
                             Math_Constrain(observer_iq_a / startup_current_limit(configuration),
                                            -1.0f,
                                            startup->speed_pi_output_max));
    startup->state = SENSORLESS_STARTUP_HANDOFF;
    startup->state_ticks = 0U;
}

/* 状态 1（ALIGN）：按斜坡施加 d 轴对准电流，保持时间结束后进入开环。 */
static void sensorless_run_align(SensorlessStartupRunState *run)
{
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    const SensorlessStartupConfig *config = run->config;

    motor_state->id_target_a =
        config->align_current_a * Math_Constrain(((float)startup->state_ticks + 1.0f) * FOC_TS_S /
                                                     config->align_current_ramp_time_s,
                                                 0.0f,
                                                 1.0f);
    motor_state->iq_target_a = 0.0f;
    Foc_RunCurrent(run->configuration, run->foc, motor_state, 0.0f, 0.0f);
    if (++startup->state_ticks >=
        (uint32_t)((config->align_current_ramp_time_s + config->align_hold_time_s) / FOC_TS_S))
    {
        startup->state = SENSORLESS_STARTUP_OPEN_LOOP;
        startup->open_loop_angle_rad = 0.0f;
        startup->open_loop_velocity_rad_s = 0.0f;
        startup->direction = run->direction;
        startup->state_ticks = 0U;
        startup->lock_ticks = 0U;
    }
}

/* 状态 2（OPEN_LOOP）：开环强拖。电角速度沿斜坡升到目标值，iq 同步爬升；
 * 方向请求变化时直接复位状态机。 */
static void sensorless_run_open_loop(SensorlessStartupRunState *run)
{
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    const SensorlessStartupConfig *config = run->config;
    float iq_ramp_ratio;

    if (run->direction != startup->direction)
    {
        SensorlessStartup_Reset(startup);
        return;
    }

    if (Math_Abs(startup->open_loop_velocity_rad_s) < config->target_electrical_velocity_rad_s)
    {
        startup->open_loop_velocity_rad_s +=
            startup->direction *
            (config->target_electrical_velocity_rad_s / config->startup_ramp_time_s) * FOC_TS_S;
        if (Math_Abs(startup->open_loop_velocity_rad_s) >= config->target_electrical_velocity_rad_s)
        {
            startup->open_loop_velocity_rad_s =
                startup->direction * config->target_electrical_velocity_rad_s;
        }
    }

    startup->open_loop_ticks++;
    iq_ramp_ratio = Math_Constrain(
        (float)startup->open_loop_ticks * FOC_TS_S / config->startup_iq_ramp_time_s, 0.0f, 1.0f);

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * FOC_TS_S);
    motor_state->id_target_a = config->startup_id_a;
    motor_state->iq_target_a =
        startup->direction *
        (config->startup_iq_initial_a +
         (config->startup_iq_a - config->startup_iq_initial_a) * iq_ramp_ratio);
    Foc_RunCurrent(run->configuration,
                   run->foc,
                   motor_state,
                   startup->open_loop_angle_rad,
                   startup->open_loop_velocity_rad_s);

    if (Math_Abs(startup->open_loop_velocity_rad_s) >= config->target_electrical_velocity_rad_s)
    {
        startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
        startup->state_ticks = 0U;
        startup->lock_ticks = 0U;
    }
}

/* 状态 3（SPEED_LOCK）：速度锁定。开环强拖期间比较滤波后的观测器速度，
 * 锁定足够长时间后进入交接；观测器不可用、方向变化或锁定超时按故障退出。 */
static void sensorless_run_speed_lock(SensorlessStartupRunState *run)
{
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    const SensorlessStartupConfig *config = run->config;
    FluxObserverState *flux_observer = run->flux_observer;
    float observer_elec_vel_rad_s;
    float speed_error_rad_s;
    bool is_observer_locked;

    if (run->direction != startup->direction || !sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * FOC_TS_S);
    startup->state_ticks++;

    motor_state->id_target_a = config->startup_id_a;
    motor_state->iq_target_a = startup->direction * config->startup_iq_a;
    Foc_RunCurrent(run->configuration,
                   run->foc,
                   motor_state,
                   startup->open_loop_angle_rad,
                   startup->open_loop_velocity_rad_s);

    observer_elec_vel_rad_s = FluxObserver_GetElecVel(flux_observer);
    if (startup->state_ticks == 1U)
    {
        startup->lock_speed_feedback_rad_s = observer_elec_vel_rad_s;
    }
    else
    {
        startup->lock_speed_feedback_rad_s +=
            config->speed_lock_filter_alpha *
            (observer_elec_vel_rad_s - startup->lock_speed_feedback_rad_s);
    }

    speed_error_rad_s =
        Math_Abs(startup->lock_speed_feedback_rad_s - startup->open_loop_velocity_rad_s);
    is_observer_locked =
        startup->lock_speed_feedback_rad_s * startup->open_loop_velocity_rad_s > 0.0f &&
        speed_error_rad_s <=
            Math_Abs(startup->open_loop_velocity_rad_s) * config->observer_lock_ratio;

    if (is_observer_locked)
    {
        startup->lock_ticks++;
    }
    else
    {
        startup->lock_ticks = 0U;
    }

    if (startup->lock_ticks >= (uint32_t)(config->speed_lock_time_s / FOC_TS_S))
    {
        sensorless_begin_handoff(run);
    }

    if (startup->state_ticks >= (uint32_t)(config->lock_timeout_s / FOC_TS_S))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }
}

/* 状态 4（HANDOFF）：角度交接。开环与观测器参考按 blend 混合，电流参考随之旋转；
 * 只把刻意的坐标系修正量旋转进电流环积分器，避免存电压交接阶跃。 */
static void sensorless_run_handoff(SensorlessStartupRunState *run)
{
    FocState *foc = run->foc;
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    const SensorlessStartupConfig *config = run->config;
    FluxObserverState *flux_observer = run->flux_observer;
    float blend;
    float phase_rad;
    float phase_vel_rad_s;
    float phase_offset_rad;
    float frame_sin;
    float frame_cos;
    float observer_id_a;
    float observer_iq_a;
    float frame_step_rad;
    float previous_blend;

    if (run->direction != startup->direction || !sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    startup->open_loop_angle_rad = Math_NormalizeAngle(
        startup->open_loop_angle_rad + startup->open_loop_velocity_rad_s * FOC_TS_S);
    previous_blend = Math_Constrain(
        (float)startup->state_ticks * FOC_TS_S / config->angle_handoff_time_s, 0.0f, 1.0f);
    blend = Math_Constrain(
        (float)(++startup->state_ticks) * FOC_TS_S / config->angle_handoff_time_s, 0.0f, 1.0f);
    phase_offset_rad = (1.0f - blend) * startup->handoff_phase_delta_rad;
    phase_rad = Math_NormalizeAngle(FluxObserver_GetElecPhase(flux_observer) + phase_offset_rad);
    frame_step_rad = (previous_blend - blend) * startup->handoff_phase_delta_rad;
    phase_vel_rad_s = FluxObserver_GetElecVel(flux_observer) + frame_step_rad / FOC_TS_S;

    observer_iq_a =
        sensorless_update_speed_loop(run->configuration,
                                     config,
                                     motor_state,
                                     run->controller,
                                     startup,
                                     FluxObserver_GetElecVel(flux_observer) / run->pole_pairs);
    observer_id_a = startup->handoff_id_reference_a +
                    blend * (config->startup_id_a - startup->handoff_id_reference_a);
    Math_SinCos(phase_offset_rad, &frame_sin, &frame_cos);
    sensorless_rotate_from_observer_frame(observer_id_a,
                                          observer_iq_a,
                                          frame_sin,
                                          frame_cos,
                                          &motor_state->id_target_a,
                                          &motor_state->iq_target_a);

    /* 只旋转刻意的坐标系修正量，不旋转转子正常行进，使电流环存电压无交接阶跃。
     * 统一插值查表保留亚分度步长，避免粗表量化造成旋转误差累积。 */
    Math_SinCos(frame_step_rad, &frame_sin, &frame_cos);
    sensorless_rotate_from_observer_frame(foc->id_pi.integral_output,
                                          foc->iq_pi.integral_output,
                                          frame_sin,
                                          frame_cos,
                                          &foc->id_pi.integral_output,
                                          &foc->iq_pi.integral_output);
    Foc_RunCurrent(run->configuration, run->foc, motor_state, phase_rad, phase_vel_rad_s);

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
static void sensorless_run_closed_loop(SensorlessStartupRunState *run)
{
    MotorControlState *motor_state = run->motor_state;
    SensorlessStartupState *startup = run->startup;
    const SensorlessStartupConfig *config = run->config;
    FluxObserverState *flux_observer = run->flux_observer;
    float observer_elec_vel_rad_s = FluxObserver_GetElecVel(flux_observer);
    float observer_mech_vel_rad_s = observer_elec_vel_rad_s / run->pole_pairs;

    if (!sensorless_observer_is_usable(flux_observer))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    if (run->direction * observer_elec_vel_rad_s < 0.0f &&
        Math_Abs(observer_elec_vel_rad_s) < config->minimum_electrical_velocity_rad_s)
    {
        SensorlessStartup_Reset(startup);
        Foc_ResetCurrent(run->foc);
        PiController_Reset(run->controller);
        return;
    }

    motor_state->id_target_a =
        config->startup_id_a *
        (1.0f -
         Math_Constrain(
             (float)startup->id_ramp_ticks * FOC_TS_S / config->id_ramp_down_time_s, 0.0f, 1.0f));
    if (startup->id_ramp_ticks < (uint32_t)(config->id_ramp_down_time_s / FOC_TS_S))
    {
        startup->id_ramp_ticks++;
    }

    motor_state->iq_target_a = sensorless_update_speed_loop(
        run->configuration, config, motor_state, run->controller, startup, observer_mech_vel_rad_s);

    if (Math_Abs(observer_elec_vel_rad_s) < config->minimum_electrical_velocity_rad_s * 0.5f)
    {
        startup->loss_ticks++;
    }
    else
    {
        startup->loss_ticks = 0U;
    }

    if (startup->loss_ticks >= (uint32_t)(config->observer_loss_time_s / FOC_TS_S))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    Foc_RunCurrent(run->configuration,
                   run->foc,
                   motor_state,
                   FluxObserver_GetElecPhase(flux_observer),
                   observer_elec_vel_rad_s);
}

/**
 * @brief  无感速度控制任务：对准、开环启动与观测器交接。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  flux_observer 磁链观测器状态指针。
 * @param  startup 无感启动状态机指针。
 * @param  config 启动参数集，默认配置或编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；准入失败置故障并停止本 tick 输出。
 */
void SensorlessStartup_Run(const MotorConfiguration *configuration,
                           FocState *foc,
                           MotorControlState *motor_state,
                           PiController *controller,
                           FluxObserverState *flux_observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *config)
{
    float pole_pairs = (float)configuration->model.motor_pole_pairs;
    float min_mech_vel_rad_s;
    SensorlessStartupRunState run;

    bool config_valid = s_encoder_startup_config_valid;
    if (config != &SensorlessStartup_EncoderCalibConfig)
    {
        /* 固件内只读型号表在会话首拍完整校验；其他调用者仍逐拍校验。 */
        bool bound = configuration->preset != NULL && config == configuration->preset->startup;
        config_valid = (bound && startup->state != SENSORLESS_STARTUP_IDLE) ||
                       sensorless_startup_config_is_valid(config);
    }
    if (!config_valid || pole_pairs <= 0.0f ||
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
        Foc_RunCurrent(configuration, foc, motor_state, 0.0f, 0.0f);
        return;
    }

    min_mech_vel_rad_s = config->minimum_electrical_velocity_rad_s / pole_pairs;
    if (Math_Abs(motor_state->speed_target_rad_s) < min_mech_vel_rad_s)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
        return;
    }

    if (!sensorless_startup_currents_are_valid(configuration, config))
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
    run.config = config;
    run.pole_pairs = pole_pairs;
    run.direction = motor_state->speed_target_rad_s >= 0.0f ? 1.0f : -1.0f;

    if (startup->state == SENSORLESS_STARTUP_IDLE)
    {
        FluxObserver_InitConfigured(flux_observer, configuration);
        Foc_ResetCurrent(foc);
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
