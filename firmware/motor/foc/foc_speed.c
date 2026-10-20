#include "foc_speed.h"

#include "control_config.h"

#include <math.h>

/* 运行速度外环拥有可选整形/前馈；顺序标定入口保留原斜坡与 PI。 */

/* 速度参考斜坡：把 speed_ramped_rad_s 按加减速限制推进到 speed_target_rad_s。
 * 速度模式与无感启动共用；两者对时基、单位与所有权的要求一致。 */
void MotorAxis_UpdateSpeedRamp(const MotorConfiguration *configuration,
                               MotorControlState *motor_state)
{
    motor_state->uses_speed_ramp = configuration->application.speed_acceleration_rad_s2 > 0.0f &&
                                   configuration->application.speed_deceleration_rad_s2 > 0.0f;

    if (motor_state->uses_speed_ramp)
    {
        if (motor_state->speed_target_rad_s > motor_state->speed_ramped_rad_s)
        {
            motor_state->speed_ramped_rad_s +=
                configuration->application.speed_acceleration_rad_s2 * Speed_Ts;
            if (motor_state->speed_ramped_rad_s > motor_state->speed_target_rad_s)
            {
                motor_state->speed_ramped_rad_s = motor_state->speed_target_rad_s;
            }
        }
        else if (motor_state->speed_target_rad_s < motor_state->speed_ramped_rad_s)
        {
            motor_state->speed_ramped_rad_s -=
                configuration->application.speed_deceleration_rad_s2 * Speed_Ts;
            if (motor_state->speed_ramped_rad_s < motor_state->speed_target_rad_s)
            {
                motor_state->speed_ramped_rad_s = motor_state->speed_target_rad_s;
            }
        }
    }
    else
    {
        motor_state->speed_ramped_rad_s = motor_state->speed_target_rad_s;
    }
}

/**
 * @brief  速度模式控制更新：斜坡推进速度参考并生成速度环输出。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  motor_state 电机控制状态指针，读写 speed_ramped_rad_s、id_target_a 与 iq_target_a。
 * @param  controller 速度 PI 控制器指针。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 供顺序标定任务使用；保持原斜坡与归一化 PI，不启用惯量前馈。
 */
void SpeedMode_UpdateControl(const MotorConfiguration *configuration,
                             MotorControlState *motor_state,
                             PiController *controller,
                             float mechanical_velocity_rad_s)
{
    MotorAxis_UpdateSpeedRamp(configuration, motor_state);

    PiController_Configure(controller,
                           configuration->application.speed_kp,
                           configuration->application.speed_ki,
                           Speed_Ts,
                           -1.0f,
                           1.0f);
    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a =
        PiController_Run(controller, motor_state->speed_ramped_rad_s, mechanical_velocity_rad_s) *
        configuration->application.current_limit_a;
}

bool SpeedMode_UpdateOuterControl(const MotorConfiguration *configuration,
                                  MotorControlState *motor_state,
                                  PiController *controller,
                                  SpeedFeedforwardState *state,
                                  float mechanical_velocity_rad_s,
                                  float current_limit_a)
{
    float configured_limit_a = configuration->application.current_limit_a;
    const SpeedFeedforwardConfig *config = &configuration->application.speed_feedforward;
    if (!isfinite(configured_limit_a) || configured_limit_a <= 0.0f || !isfinite(current_limit_a) ||
        current_limit_a < 0.0f || (config->enabled && !SpeedFeedforward_IsValid(config)))
    {
        return false;
    }
    current_limit_a = fminf(current_limit_a, configured_limit_a);
    if (current_limit_a == 0.0f)
    {
        /* 热保护归零时不推进规划；由热 owner 锁存其故障，不改报参数故障。 */
        SpeedFeedforward_Reset(state);
        PiController_Reset(controller);
        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = 0.0f;
        return true;
    }
    if (!config->enabled)
    {
        SpeedFeedforward_Reset(state);
        if (current_limit_a == configured_limit_a)
        {
            SpeedMode_UpdateControl(
                configuration, motor_state, controller, mechanical_velocity_rad_s);
        }
        else
        {
            float ratio = current_limit_a / configured_limit_a;
            MotorAxis_UpdateSpeedRamp(configuration, motor_state);
            PiController_Configure(controller,
                                   configuration->application.speed_kp,
                                   configuration->application.speed_ki,
                                   Speed_Ts,
                                   -ratio,
                                   ratio);
            motor_state->id_target_a = 0.0f;
            motor_state->iq_target_a = PiController_Run(controller,
                                                        motor_state->speed_ramped_rad_s,
                                                        mechanical_velocity_rad_s) *
                                       configured_limit_a;
        }
        return isfinite(motor_state->iq_target_a);
    }

    bool initialized = state->initialized;
    if (!SpeedFeedforward_Update(state,
                                 config,
                                 motor_state->speed_target_rad_s,
                                 mechanical_velocity_rad_s,
                                 Speed_Ts,
                                 current_limit_a))
    {
        return false;
    }
    if (!initialized)
    {
        PiController_Reset(controller);
    }
    motor_state->uses_speed_ramp = true;
    motor_state->speed_ramped_rad_s = state->speed_rad_s;
    /* 前馈占用同一电流预算；积分准入同步收窄，保留原归一化增益口径。 */
    PiController_Configure(controller,
                           configuration->application.speed_kp,
                           configuration->application.speed_ki,
                           Speed_Ts,
                           (-current_limit_a - state->current_a) / configured_limit_a,
                           (current_limit_a - state->current_a) / configured_limit_a);
    float normalized =
        PiController_Run(controller, motor_state->speed_ramped_rad_s, mechanical_velocity_rad_s);
    if (!isfinite(normalized))
    {
        return false;
    }
    float total_a =
        fminf(fmaxf(normalized * configured_limit_a + state->current_a, -current_limit_a),
              current_limit_a);
    PiController_TrackOutput(controller, (total_a - state->current_a) / configured_limit_a);
    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = total_a;
    return isfinite(total_a);
}

/**
 * @brief  顺序速度模式任务核心：分频执行速度环并保持电流闭环。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  electrical_angle_rad 电角度，单位 rad。
 * @param  electrical_velocity_rad_s 电角速度，单位 rad/s。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 */
void SpeedMode_Run(const MotorConfiguration *configuration,
                   FocState *foc,
                   MotorControlState *motor_state,
                   PiController *controller,
                   float electrical_angle_rad,
                   float electrical_velocity_rad_s,
                   float mechanical_velocity_rad_s)
{
    static unsigned speedloop_count;

    if (++speedloop_count >= SPEED_LOOP_DIVIDER)
    {
        SpeedMode_UpdateControl(configuration, motor_state, controller, mechanical_velocity_rad_s);
        speedloop_count = 0U;
    }
    FOC_Current(configuration, foc, motor_state, electrical_angle_rad, electrical_velocity_rad_s);
}
