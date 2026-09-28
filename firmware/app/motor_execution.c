#include "fast_loop_profile.h"
#include "motor_execution.h"
#include <math.h>
#include <string.h>

#include "control_config.h"
#include "foc_cogging_calibration.h"
#include "foc_errhandle.h"
#include "foc_sensorless.h"
#include "foc_speed.h"
#include "motor_axis_profile.h"
#include "motor_hardware_profile.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "utils.h"

/* 运行模式任务入口：电流/速度/位置/Vq 模式的快速环执行，
 * 以及延迟外环运行时的发布与保护接口。本文件只做任务编排与状态适配，
 * 具体控制算法位于 motor 层。 */

static void MotorOuterLoop_RequestReset(void);

/**
 * @brief  电流模式控制任务。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  Encoder 编码器状态指针。
 * @param  Fluxobserver 磁链观测器状态指针。
 * @note 在 20kHz 快速环上下文调用；齿槽转矩补偿与观测只作用于转矩闭环。
 */
void Task_Current_Mode(FOC_TypeDef *FOC,
                       MotorControl_TypeDef *MotorControl,
                       Encoder_TypeDef *Encoder,
                       Fluxobserver_TypeDef *Fluxobserver)
{
    float theta_elec;
    float vel_elec;

    if (!FocCogging_TorqueGuard(MotorControl, Encoder))
    {
        return;
    }

    if (MotorControl->isUseSensorless == true)
    {
        theta_elec = Observer_GetElePhase(Fluxobserver);
        vel_elec = Observer_GetEleVel(Fluxobserver);
    }
    else
    {
        theta_elec = Encoder_GetElePhase(Encoder);
        vel_elec = Encoder_GetEleVel(Encoder);
    }

    FOC_CurrentWithReference(
        FOC, MotorControl, theta_elec, vel_elec, FocCogging_Apply(MotorControl, Encoder));
    FocCogging_TorqueObserve(FOC, MotorControl, Encoder);
}

/**
 * @brief  速度模式控制任务（对外顺序入口）。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  Encoder 编码器状态指针。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 *       分频与速度环实现在 motor/foc/foc_speed.c；摩擦辨识与外部调用共用该核心。
 */
void Task_Speed_Mode(FOC_TypeDef *FOC,
                     MotorControl_TypeDef *MotorControl,
                     PI_Controller_TypeDef *controller,
                     Encoder_TypeDef *Encoder)
{
    SpeedMode_Run(FOC,
                  MotorControl,
                  controller,
                  Encoder_GetElePhase(Encoder),
                  Encoder_GetEleVel(Encoder),
                  Encoder_GetMecVel(Encoder));
}

/* 冷配置帧不进入调参不变的快速路径；此处仅为编译器提示，其他编译器行为相同。 */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define FOC_CONFIG_NOINLINE __attribute__((noinline))
#else
#define FOC_CONFIG_NOINLINE
#endif

/* 已辨识摩擦模型与板级阻尼环回退的固定参数。 */
static void PositionMode_ApplyFrictionConfiguration(MotorControl_TypeDef *MotorControl,
                                                    PositionCascadeConfig_TypeDef *config)
{
    config->friction_feedforward_enabled =
        MOTOR_DAMPING_FEEDFORWARD == MOTOR_DAMPING_FEEDFORWARD_ENABLED;
    if (MotorControl->friction_model_valid)
    {
        config->friction_coulomb_positive = MotorControl->friction_coulomb_pos_a;
        config->friction_coulomb_negative = MotorControl->friction_coulomb_neg_a;
        config->friction_viscous_positive = MotorControl->friction_viscous_pos_a_per_rad_s;
        config->friction_viscous_negative = MotorControl->friction_viscous_neg_a_per_rad_s;
    }
    else
    {
        /* 未辨识前使用板级阻尼环参数。 */
        config->friction_coulomb_positive = POSITION_SERVO_FALLBACK_FRICTION_POSITIVE_A;
        config->friction_coulomb_negative = POSITION_SERVO_FALLBACK_FRICTION_NEGATIVE_A;
        config->friction_viscous_positive = 0.0f;
        config->friction_viscous_negative = 0.0f;
    }
    config->friction_breakaway_ratio = POSITION_SERVO_FRICTION_BREAKAWAY_RATIO;
    config->friction_attack_slew_rate = POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S;
    config->friction_fast_release_slew_rate = POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S;
    config->friction_release_slew_rate = POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S;
    config->friction_reference_speed = POSITION_SERVO_FRICTION_REFERENCE_SPEED_RAD_S;
    config->friction_stop_speed = POSITION_SERVO_FRICTION_STOP_SPEED_RAD_S;
    config->friction_move_speed = POSITION_SERVO_FRICTION_MOVE_SPEED_RAD_S;
    config->friction_breakaway_distance = POSITION_SERVO_FRICTION_BREAKAWAY_DISTANCE_RAD;
    config->friction_stuck_time = POSITION_SERVO_FRICTION_STUCK_TIME_S;
}

/* 有效减速度：配置适配与热路径比较共用；NaN 原样传播，与旧实现一致。 */
static float PositionMode_EffectiveDeceleration(const MotorControl_TypeDef *MotorControl)
{
    float deceleration = MotorControl->posDec;

    if (deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
    {
        deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
    }
    return deceleration;
}

/* 有效最大速度：轴配置生效时取更严格的轴上限。 */
static float PositionMode_EffectiveMaximumSpeed(const MotorControl_TypeDef *MotorControl)
{
    float maximum_speed = MotorControl->pos_maxspeed;

    if (MotorControl->axis_profile.magic != 0U &&
        maximum_speed > MotorControl->axis_profile.maximum_speed_rad_s)
    {
        maximum_speed = MotorControl->axis_profile.maximum_speed_rad_s;
    }
    return maximum_speed;
}

/* 该适配器是唯一提供此控制器固定调参的调用方：每个快速 tick 全量比较实时输入，
 * 变化时重建并校验；不使用第二份 RAM 缓存或可能漏掉写入方的事件计数。 */
FOC_CONFIG_NOINLINE
static bool PositionMode_UpdateConfiguration(MotorControl_TypeDef *MotorControl,
                                             float theta_mech,
                                             float vel_mech,
                                             PositionCascadeControlOutput_TypeDef *output,
                                             uint16_t call_divider)
{
    PositionCascadeConfig_TypeDef config;

    config.update_period_s = Cascade_Position_Ts;
    config.call_divider = call_divider;
    config.target_position = MotorControl->posRef;
    config.position_error_window = MotorControl->pos_error_window;
    config.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
    config.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
    config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ *
                                (MotorControl->position_velocity_filter_half_cutoff ? 0.5f : 1.0f);
    config.hold_velocity_filter_hz =
        MotorControl->position_hold_filter_bypass
            ? 0.0f
            : POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ *
                  (MotorControl->position_hold_filter_half_cutoff ? 0.5f : 1.0f);
    config.following_error_limit = POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD;
    config.stiction_integral_rate = POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S;
    config.acceleration = MotorControl->posAcc;
    config.deceleration = PositionMode_EffectiveDeceleration(MotorControl);
    config.maximum_speed = PositionMode_EffectiveMaximumSpeed(MotorControl);
    config.speed_limit = MotorControl->speed_limit;
    config.jerk_limit =
        (config.acceleration > config.deceleration ? config.acceleration : config.deceleration) /
        POSITION_SERVO_JERK_RAMP_TIME_S;
    config.position_kp = MotorControl->cascade_pos_Kp;
    config.position_kd = MotorControl->cascade_pos_Kd;
    config.speed_kp = MotorControl->speed_Kp;
    config.speed_ki = MotorControl->speed_Ki;
    config.acceleration_feedforward_gain = POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2;
    config.current_limit = MotorControl->current_limit;
    PositionMode_ApplyFrictionConfiguration(MotorControl, &config);

    return PositionCascade_UpdateControl(&config, theta_mech, vel_mech, output);
}

/* 浮点位级比较：调参缓存判定必须把 NaN、+0/-0 视为不同值。 */
static bool PositionMode_SameTuningValue(float first, float second)
{
    uint32_t first_bits, second_bits;
    typedef char FloatMustBe32Bits[(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(FloatMustBe32Bits);
    memcpy(&first_bits, &first, sizeof(first_bits));
    memcpy(&second_bits, &second, sizeof(second_bits));
    return first_bits == second_bits;
}

/* 与上一帧配置逐字段比较，含摩擦模型切换与轴上限；任何实时输入变化都使缓存失效。 */
static bool PositionMode_ConfigurationMatches(const PositionCascadeConfig_TypeDef *config,
                                              const MotorControl_TypeDef *MotorControl)
{
    float deceleration;
    float maximum_speed;

    if (config == NULL)
    {
        return false;
    }
    deceleration = PositionMode_EffectiveDeceleration(MotorControl);
    maximum_speed = PositionMode_EffectiveMaximumSpeed(MotorControl);
    if (!PositionMode_SameTuningValue(
            config->velocity_filter_hz,
            POSITION_SERVO_VELOCITY_FILTER_HZ *
                (MotorControl->position_velocity_filter_half_cutoff ? 0.5f : 1.0f)) ||
        !PositionMode_SameTuningValue(
            config->hold_velocity_filter_hz,
            MotorControl->position_hold_filter_bypass
                ? 0.0f
                : POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ *
                      (MotorControl->position_hold_filter_half_cutoff ? 0.5f : 1.0f)) ||
        !PositionMode_SameTuningValue(config->position_error_window,
                                      MotorControl->pos_error_window) ||
        !PositionMode_SameTuningValue(config->acceleration, MotorControl->posAcc) ||
        !PositionMode_SameTuningValue(config->deceleration, deceleration) ||
        !PositionMode_SameTuningValue(config->maximum_speed, maximum_speed) ||
        !PositionMode_SameTuningValue(config->speed_limit, MotorControl->speed_limit) ||
        !PositionMode_SameTuningValue(config->position_kp, MotorControl->cascade_pos_Kp) ||
        !PositionMode_SameTuningValue(config->position_kd, MotorControl->cascade_pos_Kd) ||
        !PositionMode_SameTuningValue(config->speed_kp, MotorControl->speed_Kp) ||
        !PositionMode_SameTuningValue(config->speed_ki, MotorControl->speed_Ki) ||
        !PositionMode_SameTuningValue(config->current_limit, MotorControl->current_limit))
    {
        return false;
    }
    if (MotorControl->friction_model_valid)
    {
        return PositionMode_SameTuningValue(config->friction_coulomb_positive,
                                            MotorControl->friction_coulomb_pos_a) &&
               PositionMode_SameTuningValue(config->friction_coulomb_negative,
                                            MotorControl->friction_coulomb_neg_a) &&
               PositionMode_SameTuningValue(config->friction_viscous_positive,
                                            MotorControl->friction_viscous_pos_a_per_rad_s) &&
               PositionMode_SameTuningValue(config->friction_viscous_negative,
                                            MotorControl->friction_viscous_neg_a_per_rad_s);
    }
    return PositionMode_SameTuningValue(config->friction_coulomb_positive,
                                        POSITION_SERVO_FALLBACK_FRICTION_POSITIVE_A) &&
           PositionMode_SameTuningValue(config->friction_coulomb_negative,
                                        POSITION_SERVO_FALLBACK_FRICTION_NEGATIVE_A) &&
           PositionMode_SameTuningValue(config->friction_viscous_positive, 0.0f) &&
           PositionMode_SameTuningValue(config->friction_viscous_negative, 0.0f);
}

/* 位置请求被拒绝：清零电流参考并上报参数故障，等待上层复位。 */
static void PositionMode_RejectRequest(MotorControl_TypeDef *MotorControl)
{
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = 0.0f;
    MotorFault_Report(MotorControl, MotorParam_Error);
}

/**
 * @brief  模式 3：带加加速度限制的连续加速度位置伺服任务（顺序入口）。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  Encoder 编码器状态指针。
 * @note 在 20kHz 快速环上下文调用；配置未变化时复用缓存控制器，否则重建配置帧。
 *       生产运行由外环 worker 执行本适配器，本入口保留供外部/调试顺序调用。
 */
void Task_Position_Mode(FOC_TypeDef *FOC,
                        MotorControl_TypeDef *MotorControl,
                        Encoder_TypeDef *Encoder)
{
    bool updated;
    PositionCascadeControlOutput_TypeDef output;
    FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_ONLY);
    float theta_elec = Encoder_GetElePhase(Encoder);
    float theta_mech = Encoder_GetMecPos(Encoder);
    float vel_elec = Encoder_GetEleVel(Encoder);
    float vel_mech = Encoder_GetMecVelContinuous(Encoder);

    if (!MotorAxisProfile_AllowsPosition(&MotorControl->axis_profile,
                                         MotorControl->axis_profile_valid,
                                         theta_mech,
                                         MotorControl->posRef))
    {
        PositionMode_RejectRequest(MotorControl);
        FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
        return;
    }

    if (PositionMode_ConfigurationMatches(PositionCascade_GetConfiguration(), MotorControl))
    {
        updated = PositionCascade_UpdateTargetControl(
            MotorControl->posRef, theta_mech, vel_mech, &output);
    }
    else
    {
        updated = PositionMode_UpdateConfiguration(
            MotorControl, theta_mech, vel_mech, &output, CASCADE_POSITION_LOOP_DIVIDER);
    }

    if (!updated)
    {
        PositionMode_RejectRequest(MotorControl);
        FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
        return;
    }

    MotorControl->posShadow = output.position_reference;
    MotorControl->speedShadow = output.speed_reference;
    MotorControl->pos_vel_filtered = output.speed_feedback;
    MotorControl->isReachTargetPos = output.target_reached;
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = output.iq_reference;
    FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
    FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

/* 位置模式复位：请求外环在下一 2 kHz 慢拍重建，由唯一外环所有者消费复位请求。 */
void Task_Position_Mode_Reset(void)
{
    MotorOuterLoop_RequestReset();
}

/* OUTER_RUNTIME_BEGIN
 * 2 kHz 慢拍外环：位置级联/轨迹与速度 PI 在 TIM7 监督上下文直接执行，是位置/速度
 * 外环输出的唯一写者；20 kHz 快环只读取 MotorControl.iqRef。复位与模式变化由
 * 请求标志在本函数内消费，跨上下文只交换标志与单字状态。 */
static struct
{
    volatile bool reset_requested;
    bool ready, telemetry_valid;
    MotorControlMode mode;
    PositionCascadeTelemetry_TypeDef published_telemetry;
} outer;

static void MotorOuterLoop_RequestReset(void)
{
    outer.reset_requested = true;
    outer.ready = false;
    outer.telemetry_valid = false;
}

/* 输入有效性：与配置适配器同一组钳位与 NaN 拒绝规则，每个慢拍全量比较。 */
static bool MotorOuterLoop_InputsValid(const MotorControl_TypeDef *m, float position, float speed)
{
    float deceleration;
    float maximum_speed;

    if (!isfinite(speed))
    {
        return false;
    }
    if (m->control_mode == MOTOR_CONTROL_SPEED)
    {
        return isfinite(m->speedRef) && isfinite(m->speedAcc) && isfinite(m->speedDec) &&
               isfinite(m->current_limit) && m->current_limit > 0.0f && isfinite(m->speed_Kp) &&
               m->speed_Kp >= 0.0f && isfinite(m->speed_Ki) && m->speed_Ki >= 0.0f;
    }
    if (!MotorAxisProfile_AllowsPosition(
            &m->axis_profile, m->axis_profile_valid, position, m->posRef))
    {
        return false;
    }
    if (!isfinite(m->current_limit) || m->current_limit <= 0.0f || !isfinite(m->speed_Kp) ||
        m->speed_Kp < 0.0f || !isfinite(m->speed_Ki) || m->speed_Ki < 0.0f)
    {
        return false;
    }
    deceleration = PositionMode_EffectiveDeceleration(m);
    maximum_speed = PositionMode_EffectiveMaximumSpeed(m);
    if (!isfinite(m->pos_error_window) || m->pos_error_window <= 0.0f ||
        m->pos_error_window > POSITION_SERVO_HOLD_ENTER_POSITION_RAD || !isfinite(m->posAcc) ||
        m->posAcc <= 0.0f || !isfinite(deceleration) || deceleration <= 0.0f ||
        !isfinite(maximum_speed) || maximum_speed <= 0.0f || !isfinite(m->speed_limit) ||
        m->speed_limit < maximum_speed ||
        !isfinite((m->posAcc > deceleration ? m->posAcc : deceleration) /
                  POSITION_SERVO_JERK_RAMP_TIME_S) ||
        !isfinite(m->cascade_pos_Kp) || m->cascade_pos_Kp < 0.0f ||
        m->cascade_pos_Kp > CASCADE_POSITION_KP_MAX_PER_S || !isfinite(m->cascade_pos_Kd) ||
        m->cascade_pos_Kd < 0.0f || m->cascade_pos_Kd > CASCADE_POSITION_KD_MAX)
    {
        return false;
    }
    return !m->friction_model_valid ||
           (isfinite(m->friction_coulomb_pos_a) && m->friction_coulomb_pos_a >= 0.0f &&
            isfinite(m->friction_coulomb_neg_a) && m->friction_coulomb_neg_a >= 0.0f &&
            isfinite(m->friction_viscous_pos_a_per_rad_s) &&
            m->friction_viscous_pos_a_per_rad_s >= 0.0f &&
            isfinite(m->friction_viscous_neg_a_per_rad_s) &&
            m->friction_viscous_neg_a_per_rad_s >= 0.0f);
}

bool MotorOuterLoop_IsReady(void)
{
    return outer.ready;
}

bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry)
{
    /* 慢拍写、快速上下文读；只在有效期返回已发布快照。 */
    if (!outer.telemetry_valid || telemetry == NULL)
    {
        return false;
    }
    *telemetry = outer.published_telemetry;
    return true;
}

/**
 * @brief  外环 2 kHz 慢拍：位置级联/轨迹与速度 PI 直接执行并发布电流参考。
 * @param  m 电机控制状态指针。
 * @param  pi 速度环控制器指针（速度模式使用）。
 * @param  encoder 编码器状态指针。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是外环输出的唯一写者，快环只读取
 *       MotorControl.iqRef；模式变化、故障与复位请求在当前慢拍内消费。
 */
void MotorOuterLoop_SlowTick(MotorControl_TypeDef *m,
                             PI_Controller_TypeDef *pi,
                             Encoder_TypeDef *encoder)
{
    PositionCascadeControlOutput_TypeDef output;
    bool active =
        m->control_mode == MOTOR_CONTROL_POSITION || m->control_mode == MOTOR_CONTROL_SPEED;
    float position = Encoder_GetMecPos(encoder);
    float speed = m->control_mode == MOTOR_CONTROL_SPEED ? Encoder_GetMecVel(encoder)
                                                         : Encoder_GetMecVelContinuous(encoder);

    if (m->control_mode != outer.mode)
    {
        MotorOuterLoop_RequestReset();
        outer.mode = m->control_mode;
        if (active)
        {
            m->idRef = 0.0f;
            m->iqRef = 0.0f;
        }
    }
    if (outer.reset_requested)
    {
        PositionCascade_Reset();
        outer.reset_requested = false;
    }
    if (!active || m->ErrorNow != No_Error)
    {
        return;
    }
    if (!MotorOuterLoop_InputsValid(m, position, speed))
    {
        MotorFault_Report(m, MotorParam_Error);
        m->idRef = 0.0f;
        m->iqRef = 0.0f;
        outer.ready = false;
        outer.telemetry_valid = false;
        PositionCascade_Reset();
        return;
    }

    if (m->control_mode == MOTOR_CONTROL_POSITION)
    {
        bool valid;
        if (PositionMode_ConfigurationMatches(PositionCascade_GetConfiguration(), m))
        {
            valid = PositionCascade_UpdateTargetControl(m->posRef, position, speed, &output);
        }
        else
        {
            valid = PositionMode_UpdateConfiguration(m, position, speed, &output, 1U);
        }
        valid = valid && isfinite(output.iq_reference);
        if (!valid)
        {
            MotorFault_Report(m, MotorParam_Error);
            m->idRef = 0.0f;
            m->iqRef = 0.0f;
            outer.ready = false;
            outer.telemetry_valid = false;
            return;
        }
        m->idRef = 0.0f;
        /* 实时电流限制下调立即生效。 */
        m->iqRef = fminf(fmaxf(output.iq_reference, -m->current_limit), m->current_limit);
        m->posShadow = output.position_reference;
        m->speedShadow = output.speed_reference;
        m->pos_vel_filtered = output.speed_feedback;
        m->isReachTargetPos = output.target_reached;
        if (PositionCascade_GetTelemetry(&outer.published_telemetry))
        {
            m->pos_trajectory_speed_rad_s = outer.published_telemetry.trajectory_speed_reference;
            outer.telemetry_valid = true;
        }
        else
        {
            /* 模式 3 规划量发布：无效窗口写 NaN。 */
            m->pos_trajectory_speed_rad_s = NAN;
            outer.telemetry_valid = false;
        }
        outer.ready = true;
    }
    else
    {
        SpeedMode_UpdateControl(m, pi, speed);
        outer.ready = true;
    }
}
/* OUTER_RUNTIME_END */

/**
 * @brief  Q 轴电压模式：d 轴电流闭环到零，q 轴直接给定电压。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  Encoder 编码器状态指针。
 * @note 在 20kHz 快速环上下文调用；编码器离线或未标定时置故障并返回。
 */
void Task_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
    if (!Encoder_IsOnline(Encoder))
    {
        MotorFault_Report(MotorControl, Encoder_Error);
        return;
    }
    if ((Encoder_GetCalibFlag(Encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        MotorFault_Report(MotorControl, Encoder_NotCalibrated);
        return;
    }

    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = 0.0f;

    FOC_Vq_Mode(FOC, MotorControl, Encoder_GetElePhase(Encoder), Encoder_GetEleVel(Encoder));
}

/* 调用方已完成故障与生命周期准入；控制功能不反查状态机或选择维护任务。 */
MotorTaskResult MotorExecution_ControlStep(FOC_TypeDef *foc,
                                           MotorControl_TypeDef *motor,
                                           Encoder_TypeDef *encoder,
                                           Fluxobserver_TypeDef *observer)
{
    switch (motor->control_mode)
    {
    case MOTOR_CONTROL_CURRENT:
        Task_Current_Mode(foc, motor, encoder, observer);
        if (FocCogging_TakeTorqueTrip())
        {
            return (MotorTaskResult){MOTOR_TASK_STOP_REQUESTED, No_Error};
        }
        break;
    case MOTOR_CONTROL_SPEED:
        FOC_Current(foc, motor, Encoder_GetElePhase(encoder), Encoder_GetEleVel(encoder));
        break;
    case MOTOR_CONTROL_POSITION:
        FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_WITH_CURRENT);
        FOC_Current(foc, motor, Encoder_GetElePhase(encoder), Encoder_GetEleVel(encoder));
        FAST_PROFILE_END(FAST_PROFILE_POSITION_WITH_CURRENT);
        break;
    case MOTOR_CONTROL_VQ:
        Task_Vq_Mode(foc, motor, encoder);
        break;
    default:
        break;
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, No_Error};
}
