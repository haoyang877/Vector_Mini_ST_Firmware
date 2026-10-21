#include "motor_control_execution.h"
#include <math.h>

#include "control_config.h"
#include "foc_errhandle.h"
#include "foc_sensorless.h"
#include "foc_speed.h"
#include "axis_config.h"
#include "motor_thermal_control.h"
#include "position_cascade.h"
#include "position_cascade_config.h"

/* 控制执行核心独占外环状态；调用方传入配置和运行对象，不反查APP生命周期。
 * 调用顺序、数学与故障发布保持原核心合同，不接管功率使能或参数事务。 */

/**
 * @brief  电流模式控制任务。
 * @param configuration 只读活动配置；由调用方保持同拍一致。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  encoder 编码器状态指针。
 * @param  flux_observer 磁链观测器状态指针。
 * @note 在 20kHz 快速环上下文调用，显式选择编码器或必要的无感反馈。
 */
void Task_Current_Mode(const MotorConfiguration *configuration,
                       FocState *foc,
                       MotorControlState *motor_state,
                       EncoderState *encoder,
                       FluxObserverState *flux_observer)
{
    float electrical_angle_rad;
    float electrical_velocity_rad_s;

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

    FOC_Current(configuration, foc, motor_state, electrical_angle_rad, electrical_velocity_rad_s);
}

/* 轴记录生效时沿用更严格的机械速度上限。 */
static float position_mode_maximum_speed(const MotorConfiguration *configuration)
{
    float maximum_speed = configuration->application.position_maximum_speed_rad_s;
    if (configuration->application.axis_profile.magic != 0U &&
        maximum_speed > configuration->application.axis_profile.maximum_speed_rad_s)
    {
        maximum_speed = configuration->application.axis_profile.maximum_speed_rad_s;
    }
    return maximum_speed;
}

/* 只装配位置串级核心输入，扩展规划与摩擦字段不再组装。 */
static bool position_mode_update_control(const MotorConfiguration *configuration,
                                         const MotorControlState *motor,
                                         float position_rad,
                                         float speed_rad_s,
                                         PositionCascadeControlOutput *output)
{
    PositionCascadeConfig config = {0};
    config.update_period_s = Cascade_Position_Ts;
    config.call_divider = 1U;
    config.target_position = motor->position_target_rad;
    config.position_error_window = configuration->application.position_error_window_rad;
    config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;
    config.maximum_speed = position_mode_maximum_speed(configuration);
    config.speed_limit_rad_s = configuration->application.speed_limit_rad_s;
    config.position_kp = configuration->application.cascade_position_kp;
    config.position_kd = configuration->application.cascade_position_kd;
    config.speed_kp = configuration->application.speed_kp;
    config.speed_ki = configuration->application.speed_ki;
    config.current_limit_a = configuration->application.current_limit_a;
    return PositionCascade_UpdateControlLimited(
        &config,
        position_rad,
        speed_rad_s,
        MotorThermalControl_CurrentLimit(configuration->application.current_limit_a),
        output);
}

/* OUTER_RUNTIME_BEGIN
 * 2 kHz 慢拍外环：位置级联与速度 PI 在 TIM7 监督上下文直接执行，是位置/速度
 * 外环输出的唯一写者；20 kHz 快环只读取 motor_state.iq_target_a。复位与模式变化由
 * 请求标志在本函数内消费，跨上下文只交换标志与单字状态。 */
static struct
{
    volatile bool reset_requested;
    bool ready;
    MotorControlMode mode;
    SpeedFeedforwardState speed_feedforward;
} s_outer;

void MotorOuterLoop_RequestReset(void)
{
    s_outer.reset_requested = true;
    s_outer.ready = false;
}

/* 有限反馈、轴边界和原核心增益/电流/速度限值每个慢拍都复核。 */
static bool motor_outer_loop_inputs_valid(const MotorConfiguration *configuration,
                                          const MotorControlState *motor,
                                          float position_rad,
                                          float speed_rad_s)
{
    if (!isfinite(speed_rad_s) || !isfinite(configuration->application.current_limit_a) ||
        configuration->application.current_limit_a <= 0.0f ||
        !isfinite(configuration->application.speed_kp) ||
        configuration->application.speed_kp < 0.0f ||
        !isfinite(configuration->application.speed_ki) ||
        configuration->application.speed_ki < 0.0f)
    {
        return false;
    }
    if (motor->control_mode == MOTOR_CONTROL_SPEED)
    {
        return isfinite(motor->speed_target_rad_s) &&
               isfinite(configuration->application.speed_acceleration_rad_s2) &&
               isfinite(configuration->application.speed_deceleration_rad_s2);
    }
    float maximum_speed_rad_s = position_mode_maximum_speed(configuration);
    return MotorAxisProfile_AllowsPosition(&configuration->application.axis_profile,
                                           configuration->application.axis_profile_valid,
                                           position_rad,
                                           motor->position_target_rad) &&
           isfinite(configuration->application.position_error_window_rad) &&
           configuration->application.position_error_window_rad > 0.0f &&
           configuration->application.position_error_window_rad <=
               POSITION_CASCADE_POSITION_ERROR_WINDOW_MAX_RAD &&
           isfinite(maximum_speed_rad_s) && maximum_speed_rad_s > 0.0f &&
           isfinite(configuration->application.speed_limit_rad_s) &&
           configuration->application.speed_limit_rad_s >= maximum_speed_rad_s &&
           isfinite(configuration->application.cascade_position_kp) &&
           configuration->application.cascade_position_kp >= 0.0f &&
           configuration->application.cascade_position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
           isfinite(configuration->application.cascade_position_kd) &&
           configuration->application.cascade_position_kd >= 0.0f &&
           configuration->application.cascade_position_kd <= CASCADE_POSITION_KD_MAX;
}

bool MotorOuterLoop_IsReady(void)
{
    return s_outer.ready;
}

/**
 * @brief  外环 2 kHz 慢拍：位置级联与速度 PI 直接执行并发布电流参考。
 * @param configuration 只读活动配置；由调用方保持同拍一致。
 * @param  m 电机控制状态指针。
 * @param  pi 速度环控制器指针（速度模式使用）。
 * @param  encoder 编码器状态指针。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是外环输出的唯一写者，快环只读取
 *       motor_state.iq_target_a；模式变化、故障与复位请求在当前慢拍内消费。
 */
void MotorOuterLoop_SlowTick(const MotorConfiguration *configuration,
                             MotorControlState *m,
                             PiController *pi,
                             EncoderState *encoder)
{
    PositionCascadeControlOutput output;
    bool active =
        m->control_mode == MOTOR_CONTROL_POSITION || m->control_mode == MOTOR_CONTROL_SPEED;
    float position = Encoder_GetMecPos(encoder);
    float speed = m->control_mode == MOTOR_CONTROL_SPEED ? Encoder_GetMecVel(encoder)
                                                         : Encoder_GetMecVelContinuous(encoder);

    if (m->control_mode != s_outer.mode)
    {
        MotorOuterLoop_RequestReset();
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
        SpeedFeedforward_Reset(&s_outer.speed_feedforward);
        s_outer.reset_requested = false;
    }
    if (!active || m->fault_code != MOTOR_FAULT_NONE)
    {
        SpeedFeedforward_Reset(&s_outer.speed_feedforward);
        s_outer.ready = false;
        return;
    }
    if (!motor_outer_loop_inputs_valid(configuration, m, position, speed))
    {
        MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
        m->id_target_a = 0.0f;
        m->iq_target_a = 0.0f;
        s_outer.ready = false;
        PositionCascade_Reset();
        SpeedFeedforward_Reset(&s_outer.speed_feedforward);
        return;
    }

    if (m->control_mode == MOTOR_CONTROL_POSITION)
    {
        bool valid = position_mode_update_control(configuration, m, position, speed, &output);
        valid = valid && isfinite(output.iq_reference);
        if (!valid)
        {
            MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
            s_outer.ready = false;
            return;
        }
        m->id_target_a = 0.0f;
        /* 实时电流限制下调立即生效。 */
        m->iq_target_a =
            fminf(fmaxf(output.iq_reference, -configuration->application.current_limit_a),
                  configuration->application.current_limit_a);
        m->position_reference_rad = output.position_reference;
        m->speed_ramped_rad_s = output.speed_reference;
        m->position_velocity_filtered_rad_s = output.speed_feedback_rad_s;
        m->position_target_reached = output.target_reached;
        m->position_trajectory_speed_rad_s = NAN;
        s_outer.ready = true;
    }
    else
    {
        s_outer.ready = SpeedMode_UpdateOuterControl(
            configuration,
            m,
            pi,
            &s_outer.speed_feedforward,
            speed,
            MotorThermalControl_CurrentLimit(configuration->application.current_limit_a));
        if (!s_outer.ready)
        {
            MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
            SpeedFeedforward_Reset(&s_outer.speed_feedforward);
        }
    }
}
/* OUTER_RUNTIME_END */

/**
 * @brief  Q 轴电压模式：d 轴电流闭环到零，q 轴直接给定电压。
 * @param configuration 只读活动配置；由调用方保持同拍一致。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  encoder 编码器状态指针。
 * @note 在 20kHz 快速环上下文调用；编码器离线或未标定时置故障并返回。
 */
void Task_Vq_Mode(const MotorConfiguration *configuration,
                  FocState *foc,
                  MotorControlState *motor_state,
                  EncoderState *encoder)
{
    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        return;
    }
    if ((Encoder_GetCalibFlag(&configuration->device.encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
        return;
    }

    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = 0.0f;

    FOC_Vq_Mode(
        configuration, foc, motor_state, Encoder_GetElePhase(encoder), Encoder_GetEleVel(encoder));
}

/* 调用方已完成故障与生命周期准入；控制功能不反查状态机或选择维护任务。 */
MotorTaskResult MotorExecution_ControlStep(const MotorConfiguration *configuration,
                                           FocState *foc,
                                           MotorControlState *motor,
                                           EncoderState *encoder,
                                           FluxObserverState *observer)
{
    switch (motor->control_mode)
    {
    case MOTOR_CONTROL_CURRENT:
        Task_Current_Mode(configuration, foc, motor, encoder, observer);
        break;
    case MOTOR_CONTROL_SPEED:
    case MOTOR_CONTROL_POSITION:
        FOC_Current(
            configuration, foc, motor, Encoder_GetElePhase(encoder), Encoder_GetEleVel(encoder));
        break;
    case MOTOR_CONTROL_VQ:
        Task_Vq_Mode(configuration, foc, motor, encoder);
        break;
    default:
        break;
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
