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

/* 运行模式任务入口：电流/速度/位置/vq_v 模式的快速环执行，
 * 以及延迟外环运行时的发布与保护接口。本文件只做任务编排与状态适配，
 * 具体控制算法位于 motor 层。 */

static void motor_outer_loop_request_reset(void);

/**
 * @brief  电流模式控制任务。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  encoder 编码器状态指针。
 * @param  flux_observer 磁链观测器状态指针。
 * @note 在 20kHz 快速环上下文调用；齿槽转矩补偿与观测只作用于转矩闭环。
 */
void Task_Current_Mode(FocState *foc,
                       MotorControlState *motor_state,
                       EncoderState *encoder,
                       FluxObserverState *flux_observer)
{
    float electrical_angle_rad;
    float electrical_velocity_rad_s;

    if (!FocCogging_TorqueGuard(motor_state, encoder))
    {
        return;
    }

    if (motor_state->uses_sensorless_feedback == true)
    {
        electrical_angle_rad = Observer_GetElePhase(flux_observer);
        electrical_velocity_rad_s = Observer_GetEleVel(flux_observer);
    }
    else
    {
        electrical_angle_rad = Encoder_GetElePhase(encoder);
        electrical_velocity_rad_s = Encoder_GetEleVel(encoder);
    }

    FOC_CurrentWithReference(foc,
                             motor_state,
                             electrical_angle_rad,
                             electrical_velocity_rad_s,
                             FocCogging_Apply(motor_state, encoder));
    FocCogging_TorqueObserve(foc, motor_state, encoder);
}

/* 冷配置帧不进入调参不变的快速路径；此处仅为编译器提示，其他编译器行为相同。 */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define FOC_CONFIG_NOINLINE __attribute__((noinline))
#else
#define FOC_CONFIG_NOINLINE
#endif

/* 已辨识摩擦模型与板级阻尼环回退的固定参数。 */
static void position_mode_apply_friction_configuration(MotorControlState *motor_state,
                                                       PositionCascadeConfig *config)
{
    config->friction_feedforward_enabled =
        MOTOR_DAMPING_FEEDFORWARD == MOTOR_DAMPING_FEEDFORWARD_ENABLED;
    if (motor_state->friction_model_valid)
    {
        config->friction_coulomb_positive = motor_state->friction_coulomb_pos_a;
        config->friction_coulomb_negative = motor_state->friction_coulomb_neg_a;
        config->friction_viscous_positive = motor_state->friction_viscous_pos_a_per_rad_s;
        config->friction_viscous_negative = motor_state->friction_viscous_neg_a_per_rad_s;
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
static float position_mode_effective_deceleration(const MotorControlState *motor_state)
{
    float deceleration = motor_state->position_deceleration_rad_s2;

    if (deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
    {
        deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
    }
    return deceleration;
}

/* 有效最大速度：轴配置生效时取更严格的轴上限。 */
static float position_mode_effective_maximum_speed(const MotorControlState *motor_state)
{
    float maximum_speed = motor_state->position_maximum_speed_rad_s;

    if (motor_state->axis_profile.magic != 0U &&
        maximum_speed > motor_state->axis_profile.maximum_speed_rad_s)
    {
        maximum_speed = motor_state->axis_profile.maximum_speed_rad_s;
    }
    return maximum_speed;
}

/* 该适配器是唯一提供此控制器固定调参的调用方：每个快速 tick 全量比较实时输入，
 * 变化时重建并校验；不使用第二份 RAM 缓存或可能漏掉写入方的事件计数。 */
FOC_CONFIG_NOINLINE
static bool position_mode_update_configuration(MotorControlState *motor_state,
                                               float mechanical_position_rad,
                                               float mechanical_velocity_rad_s,
                                               PositionCascadeControlOutput *output,
                                               uint16_t call_divider)
{
    PositionCascadeConfig config;

    config.update_period_s = Cascade_Position_Ts;
    config.call_divider = call_divider;
    config.target_position = motor_state->position_target_rad;
    config.position_error_window = motor_state->position_error_window_rad;
    config.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
    config.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
    config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;
    config.hold_velocity_filter_hz = POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ;
    config.following_error_limit = POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD;
    config.stiction_integral_rate = POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S;
    config.acceleration = motor_state->position_acceleration_rad_s2;
    config.deceleration = position_mode_effective_deceleration(motor_state);
    config.maximum_speed = position_mode_effective_maximum_speed(motor_state);
    config.speed_limit_rad_s = motor_state->speed_limit_rad_s;
    config.jerk_limit =
        (config.acceleration > config.deceleration ? config.acceleration : config.deceleration) /
        POSITION_SERVO_JERK_RAMP_TIME_S;
    config.position_kp = motor_state->cascade_position_kp;
    config.position_kd = motor_state->cascade_position_kd;
    config.speed_kp = motor_state->speed_kp;
    config.speed_ki = motor_state->speed_ki;
    config.acceleration_feedforward_gain = POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2;
    config.current_limit_a = motor_state->current_limit_a;
    position_mode_apply_friction_configuration(motor_state, &config);

    return PositionCascade_UpdateControl(
        &config, mechanical_position_rad, mechanical_velocity_rad_s, output);
}

/* 浮点位级比较：调参缓存判定必须把 NaN、+0/-0 视为不同值。 */
static bool position_mode_same_tuning_value(float first, float second)
{
    uint32_t first_bits, second_bits;
    typedef char FloatMustBe32Bits[(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(FloatMustBe32Bits);
    memcpy(&first_bits, &first, sizeof(first_bits));
    memcpy(&second_bits, &second, sizeof(second_bits));
    return first_bits == second_bits;
}

/* 与上一帧配置逐字段比较，含摩擦模型切换与轴上限；任何实时输入变化都使缓存失效。 */
static bool position_mode_configuration_matches(const PositionCascadeConfig *config,
                                                const MotorControlState *motor_state)
{
    float deceleration;
    float maximum_speed;

    if (config == NULL)
    {
        return false;
    }
    deceleration = position_mode_effective_deceleration(motor_state);
    maximum_speed = position_mode_effective_maximum_speed(motor_state);
    if (!position_mode_same_tuning_value(config->velocity_filter_hz,
                                         POSITION_SERVO_VELOCITY_FILTER_HZ) ||
        !position_mode_same_tuning_value(config->hold_velocity_filter_hz,
                                         POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ) ||
        !position_mode_same_tuning_value(config->position_error_window,
                                         motor_state->position_error_window_rad) ||
        !position_mode_same_tuning_value(config->acceleration,
                                         motor_state->position_acceleration_rad_s2) ||
        !position_mode_same_tuning_value(config->deceleration, deceleration) ||
        !position_mode_same_tuning_value(config->maximum_speed, maximum_speed) ||
        !position_mode_same_tuning_value(config->speed_limit_rad_s,
                                         motor_state->speed_limit_rad_s) ||
        !position_mode_same_tuning_value(config->position_kp, motor_state->cascade_position_kp) ||
        !position_mode_same_tuning_value(config->position_kd, motor_state->cascade_position_kd) ||
        !position_mode_same_tuning_value(config->speed_kp, motor_state->speed_kp) ||
        !position_mode_same_tuning_value(config->speed_ki, motor_state->speed_ki) ||
        !position_mode_same_tuning_value(config->current_limit_a, motor_state->current_limit_a))
    {
        return false;
    }
    if (motor_state->friction_model_valid)
    {
        return position_mode_same_tuning_value(config->friction_coulomb_positive,
                                               motor_state->friction_coulomb_pos_a) &&
               position_mode_same_tuning_value(config->friction_coulomb_negative,
                                               motor_state->friction_coulomb_neg_a) &&
               position_mode_same_tuning_value(config->friction_viscous_positive,
                                               motor_state->friction_viscous_pos_a_per_rad_s) &&
               position_mode_same_tuning_value(config->friction_viscous_negative,
                                               motor_state->friction_viscous_neg_a_per_rad_s);
    }
    return position_mode_same_tuning_value(config->friction_coulomb_positive,
                                           POSITION_SERVO_FALLBACK_FRICTION_POSITIVE_A) &&
           position_mode_same_tuning_value(config->friction_coulomb_negative,
                                           POSITION_SERVO_FALLBACK_FRICTION_NEGATIVE_A) &&
           position_mode_same_tuning_value(config->friction_viscous_positive, 0.0f) &&
           position_mode_same_tuning_value(config->friction_viscous_negative, 0.0f);
}

/* 位置模式复位：请求外环在下一 2 kHz 慢拍重建，由唯一外环所有者消费复位请求。 */
void MotorOuterLoop_RequestReset(void)
{
    motor_outer_loop_request_reset();
}

/* OUTER_RUNTIME_BEGIN
 * 2 kHz 慢拍外环：位置级联/轨迹与速度 PI 在 TIM7 监督上下文直接执行，是位置/速度
 * 外环输出的唯一写者；20 kHz 快环只读取 motor_state.iq_target_a。复位与模式变化由
 * 请求标志在本函数内消费，跨上下文只交换标志与单字状态。 */
static struct
{
    volatile bool reset_requested;
    bool ready, telemetry_valid;
    MotorControlMode mode;
    PositionCascadeTelemetry published_telemetry;
} s_outer;

static void motor_outer_loop_request_reset(void)
{
    s_outer.reset_requested = true;
    s_outer.ready = false;
    s_outer.telemetry_valid = false;
}

/* 输入有效性：与配置适配器同一组钳位与 NaN 拒绝规则，每个慢拍全量比较。 */
static bool motor_outer_loop_inputs_valid(const MotorControlState *m, float position, float speed)
{
    float deceleration;
    float maximum_speed;

    if (!isfinite(speed))
    {
        return false;
    }
    if (m->control_mode == MOTOR_CONTROL_SPEED)
    {
        return isfinite(m->speed_target_rad_s) && isfinite(m->speed_acceleration_rad_s2) &&
               isfinite(m->speed_deceleration_rad_s2) && isfinite(m->current_limit_a) &&
               m->current_limit_a > 0.0f && isfinite(m->speed_kp) && m->speed_kp >= 0.0f &&
               isfinite(m->speed_ki) && m->speed_ki >= 0.0f;
    }
    if (!MotorAxisProfile_AllowsPosition(
            &m->axis_profile, m->axis_profile_valid, position, m->position_target_rad))
    {
        return false;
    }
    if (!isfinite(m->current_limit_a) || m->current_limit_a <= 0.0f || !isfinite(m->speed_kp) ||
        m->speed_kp < 0.0f || !isfinite(m->speed_ki) || m->speed_ki < 0.0f)
    {
        return false;
    }
    deceleration = position_mode_effective_deceleration(m);
    maximum_speed = position_mode_effective_maximum_speed(m);
    if (!isfinite(m->position_error_window_rad) || m->position_error_window_rad <= 0.0f ||
        m->position_error_window_rad > POSITION_SERVO_HOLD_ENTER_POSITION_RAD ||
        !isfinite(m->position_acceleration_rad_s2) || m->position_acceleration_rad_s2 <= 0.0f ||
        !isfinite(deceleration) || deceleration <= 0.0f || !isfinite(maximum_speed) ||
        maximum_speed <= 0.0f || !isfinite(m->speed_limit_rad_s) ||
        m->speed_limit_rad_s < maximum_speed ||
        !isfinite((m->position_acceleration_rad_s2 > deceleration ? m->position_acceleration_rad_s2
                                                                  : deceleration) /
                  POSITION_SERVO_JERK_RAMP_TIME_S) ||
        !isfinite(m->cascade_position_kp) || m->cascade_position_kp < 0.0f ||
        m->cascade_position_kp > CASCADE_POSITION_KP_MAX_PER_S ||
        !isfinite(m->cascade_position_kd) || m->cascade_position_kd < 0.0f ||
        m->cascade_position_kd > CASCADE_POSITION_KD_MAX)
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
    return s_outer.ready;
}

bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry *telemetry)
{
    /* 慢拍写、快速上下文读；只在有效期返回已发布快照。 */
    if (!s_outer.telemetry_valid || telemetry == NULL)
    {
        return false;
    }
    *telemetry = s_outer.published_telemetry;
    return true;
}

/**
 * @brief  外环 2 kHz 慢拍：位置级联/轨迹与速度 PI 直接执行并发布电流参考。
 * @param  m 电机控制状态指针。
 * @param  pi 速度环控制器指针（速度模式使用）。
 * @param  encoder 编码器状态指针。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是外环输出的唯一写者，快环只读取
 *       motor_state.iq_target_a；模式变化、故障与复位请求在当前慢拍内消费。
 */
void MotorOuterLoop_SlowTick(MotorControlState *m, PiController *pi, EncoderState *encoder)
{
    PositionCascadeControlOutput output;
    bool active =
        m->control_mode == MOTOR_CONTROL_POSITION || m->control_mode == MOTOR_CONTROL_SPEED;
    float position = Encoder_GetMecPos(encoder);
    float speed = m->control_mode == MOTOR_CONTROL_SPEED ? Encoder_GetMecVel(encoder)
                                                         : Encoder_GetMecVelContinuous(encoder);

    if (m->control_mode != s_outer.mode)
    {
        motor_outer_loop_request_reset();
        s_outer.mode = m->control_mode;
        if (active)
        {
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
        }
    }
    if (s_outer.reset_requested)
    {
        PositionCascade_Reset();
        s_outer.reset_requested = false;
    }
    if (!active || m->fault_code != MOTOR_FAULT_NONE)
    {
        return;
    }
    if (!motor_outer_loop_inputs_valid(m, position, speed))
    {
        MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
        m->id_target_a = 0.0f;
        m->iq_target_a = 0.0f;
        s_outer.ready = false;
        s_outer.telemetry_valid = false;
        PositionCascade_Reset();
        return;
    }

    if (m->control_mode == MOTOR_CONTROL_POSITION)
    {
        bool valid;
        if (position_mode_configuration_matches(PositionCascade_GetConfiguration(), m))
        {
            valid = PositionCascade_UpdateTargetControl(
                m->position_target_rad, position, speed, &output);
        }
        else
        {
            valid = position_mode_update_configuration(m, position, speed, &output, 1U);
        }
        valid = valid && isfinite(output.iq_reference);
        if (!valid)
        {
            MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
            s_outer.ready = false;
            s_outer.telemetry_valid = false;
            return;
        }
        m->id_target_a = 0.0f;
        /* 实时电流限制下调立即生效。 */
        m->iq_target_a = fminf(fmaxf(output.iq_reference, -m->current_limit_a), m->current_limit_a);
        m->position_reference_rad = output.position_reference;
        m->speed_ramped_rad_s = output.speed_reference;
        m->position_velocity_filtered_rad_s = output.speed_feedback_rad_s;
        m->position_target_reached = output.target_reached;
        if (PositionCascade_GetTelemetry(&s_outer.published_telemetry))
        {
            m->position_trajectory_speed_rad_s =
                s_outer.published_telemetry.trajectory_speed_reference;
            s_outer.telemetry_valid = true;
        }
        else
        {
            /* 模式 3 规划量发布：无效窗口写 NaN。 */
            m->position_trajectory_speed_rad_s = NAN;
            s_outer.telemetry_valid = false;
        }
        s_outer.ready = true;
    }
    else
    {
        SpeedMode_UpdateControl(m, pi, speed);
        s_outer.ready = true;
    }
}
/* OUTER_RUNTIME_END */

/**
 * @brief  Q 轴电压模式：d 轴电流闭环到零，q 轴直接给定电压。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  encoder 编码器状态指针。
 * @note 在 20kHz 快速环上下文调用；编码器离线或未标定时置故障并返回。
 */
void Task_Vq_Mode(FocState *foc, MotorControlState *motor_state, EncoderState *encoder)
{
    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        return;
    }
    if ((Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
        return;
    }

    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = 0.0f;

    FOC_Vq_Mode(foc, motor_state, Encoder_GetElePhase(encoder), Encoder_GetEleVel(encoder));
}

/* 调用方已完成故障与生命周期准入；控制功能不反查状态机或选择维护任务。 */
MotorTaskResult MotorExecution_ControlStep(FocState *foc,
                                           MotorControlState *motor,
                                           EncoderState *encoder,
                                           FluxObserverState *observer)
{
    switch (motor->control_mode)
    {
    case MOTOR_CONTROL_CURRENT:
        Task_Current_Mode(foc, motor, encoder, observer);
        if (FocCogging_TakeTorqueTrip())
        {
            return (MotorTaskResult){MOTOR_TASK_STOP_REQUESTED, MOTOR_FAULT_NONE};
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
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
