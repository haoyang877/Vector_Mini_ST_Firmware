#include "motor_control.h"
#include <math.h>

#include "application_config.h"
#include "motor_fault.h"
#include "flux_observer.h"
#include "speed_control.h"
#include "axis_config.h"
#include "control_math.h"
#include "position_control.h"
#include "position_otg.h"

/* 控制执行核心独占外环状态；调用方传入配置和运行对象，不反查APP生命周期。
 * 调用顺序、数学与故障发布保持原核心合同，不接管功率使能或参数事务。 */

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

/* 预留 4 ms 前台建表时间。旧轨迹照常推进；相同绝对时刻的 p/v/a 作为新段表起点。 */
#define POSITION_PLAN_LEAD_TICKS (CASCADE_POSITION_LOOP_FREQ / 250U)

bool MotorOuterLoop_TakePositionPlan(MotorOuterLoop *outer, MotorPositionPlan *work)
{
    if (outer->plan.state != PLAN_QUEUED)
    {
        return false;
    }
    *work = outer->plan.work;
    outer->plan.state = PLAN_BUILDING;
    return true;
}

bool MotorOuterLoop_BuildPositionPlan(MotorPositionPlan *work)
{
    float lead_s = POSITION_PLAN_LEAD_TICKS * CASCADE_POSITION_LOOP_TS_S;
    if (work->trajectory.has_plan && !PositionOtg_Sample(&work->trajectory, lead_s))
    {
        return false;
    }
    return PositionOtg_SetTarget(&work->trajectory, work->target_rad, work->limits);
}

void MotorOuterLoop_PublishPositionPlan(MotorOuterLoop *outer,
                                        const MotorPositionPlan *work,
                                        bool valid)
{
    if (outer->plan.state == PLAN_BUILDING && work->serial == outer->plan.work.serial)
    {
        if (valid)
        {
            outer->plan.work.trajectory = work->trajectory;
        }
        outer->plan.state = valid ? PLAN_READY : PLAN_INVALID;
    }
}

static bool position_plan_sample(MotorOuterLoop *outer,
                                 PositionOtg *otg,
                                 PositionOtgLimits limits,
                                 float target_rad)
{
    if (outer->plan.state != PLAN_IDLE && outer->plan.tick == outer->plan.apply_tick)
    {
        if (outer->plan.state != PLAN_READY)
        {
            outer->plan.missed_deadline = outer->plan.state != PLAN_INVALID;
            return false;
        }
        *otg = outer->plan.work.trajectory;
        outer->plan.state = PLAN_IDLE;
    }
    if (outer->plan.state == PLAN_IDLE &&
        (!otg->has_plan || target_rad != otg->target_rad ||
         limits.speed_rad_s != otg->limits.speed_rad_s ||
         limits.acceleration_rad_s2 != otg->limits.acceleration_rad_s2 ||
         limits.jerk_rad_s3 != otg->limits.jerk_rad_s3))
    {
        outer->plan.apply_tick = outer->plan.tick + POSITION_PLAN_LEAD_TICKS;
        MotorPositionPlan *work = &outer->plan.work;
        work->trajectory = *otg;
        if (!otg->has_plan)
        {
            /* 从反馈起步到首个交接点，保持初始速度与零加速度。 */
            work->trajectory.reference.position_rad =
                outer->plan.initial_position_rad +
                otg->reference.speed_rad_s * (outer->plan.apply_tick * CASCADE_POSITION_LOOP_TS_S);
        }
        work->limits = limits;
        work->target_rad = target_rad;
        work->serial = ++outer->plan_serial;
        outer->plan.state = PLAN_QUEUED;
    }
    ++outer->plan.tick;
    if (!otg->has_plan)
    {
        otg->reference.position_rad =
            outer->plan.initial_position_rad +
            otg->reference.speed_rad_s * (outer->plan.tick * CASCADE_POSITION_LOOP_TS_S);
        return isfinite(otg->reference.position_rad) &&
               fabsf(otg->reference.speed_rad_s) <= limits.speed_rad_s;
    }
    float last_acceleration = otg->reference.acceleration_rad_s2;
    if (!PositionOtg_Sample(otg, CASCADE_POSITION_LOOP_TS_S))
    {
        return false;
    }
    /* 新限值在交接前也生效；已接受的同限值段表在建表时已校验。 */
    if (limits.speed_rad_s < otg->limits.speed_rad_s ||
        limits.acceleration_rad_s2 < otg->limits.acceleration_rad_s2 ||
        limits.jerk_rad_s3 < otg->limits.jerk_rad_s3)
    {
        return fabsf(otg->reference.speed_rad_s) <= limits.speed_rad_s &&
               fabsf(otg->reference.acceleration_rad_s2) <= limits.acceleration_rad_s2 &&
               fabsf(otg->reference.acceleration_rad_s2 - last_acceleration) <=
                   limits.jerk_rad_s3 * CASCADE_POSITION_LOOP_TS_S;
    }
    return true;
}

/* 规划器与外环同实例复位；控制器跟踪本拍位置和速度参考。 */
static bool position_mode_update_control(MotorOuterLoop *outer,
                                         const MotorConfiguration *configuration,
                                         const MotorControlState *motor,
                                         PositionOtg *otg,
                                         float velocity_filter_hz,
                                         float position_rad,
                                         float speed_rad_s,
                                         PositionCascadeControlOutput *output)
{
    PositionCascadeConfig config = {0};
    /* 位置 PI 与 OTG 同拍推进，周期由位置外环节拍给出。 */
    config.update_period_s = CASCADE_POSITION_LOOP_TS_S;
    config.call_divider = 1U;
    float acceleration = configuration->application.position_acceleration_rad_s2;
    float deceleration = configuration->application.position_deceleration_rad_s2;
    if (!isfinite(acceleration) || !isfinite(deceleration))
    {
        return false;
    }
    float limit = fminf(acceleration, deceleration);
    PositionOtgLimits limits = {
        position_mode_maximum_speed(configuration), limit, limit / POSITION_OTG_ACCEL_RISE_TIME_S};
    if (!otg->initialized)
    {
        if (!PositionOtg_Reset(otg, (PositionOtgReference){position_rad, speed_rad_s, 0.0f}))
        {
            return false;
        }
        outer->plan.initial_position_rad = position_rad;
    }
    if (!(limit > 0.0f) || !isfinite(limits.jerk_rad_s3) ||
        !position_plan_sample(outer, otg, limits, motor->position_target_rad) ||
        /* 本拍已检查轴身份、实测位置和最终目标；生成的有限参考还须满足行程边界。 */
        (!motor->host_position_unlimited && configuration->application.axis_profile.magic != 0U &&
         !MotorAxisProfile_TargetWithinBounds(&configuration->application.axis_profile,
                                              otg->reference.position_rad)))
    {
        return false;
    }
    config.target_position = otg->reference.position_rad;
    config.reference_speed_rad_s = otg->reference.speed_rad_s;
    config.position_error_window = configuration->application.position_error_window_rad;
    config.velocity_filter_hz = velocity_filter_hz;
    config.maximum_speed = limits.speed_rad_s;
    config.speed_limit_rad_s = limits.speed_rad_s;
    config.position_kp = configuration->application.cascade_position_kp;
    config.position_kd = configuration->application.cascade_position_kd;
    config.speed_kp = configuration->application.speed_kp;
    config.speed_ki = configuration->application.speed_ki;
    config.current_limit_a = configuration->application.current_limit_a;
    return PositionCascade_UpdateControlLimited(&outer->cascade,
                                                &config,
                                                position_rad,
                                                speed_rad_s,
                                                configuration->application.current_limit_a,
                                                output);
}

/* OUTER_RUNTIME_BEGIN
 * 慢拍外环：位置级联与速度 PI 在外环执行上下文运行，是位置/速度
 * 外环输出的唯一写者；20 kHz 快环只读取 motor_state.iq_target_a。复位与模式变化由
 * 请求标志在本函数内消费，跨上下文只交换标志与单字状态。 */

void MotorOuterLoop_RequestReset(MotorOuterLoop *outer)
{
    outer->reset_requested = true;
    outer->ready = false;
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
    bool position_allowed =
        motor->host_position_unlimited
            ? configuration->application.axis_profile_valid && isfinite(position_rad) &&
                  isfinite(motor->position_target_rad)
            : MotorAxisProfile_AllowsPosition(&configuration->application.axis_profile,
                                              configuration->application.axis_profile_valid,
                                              position_rad,
                                              motor->position_target_rad);
    return position_allowed && isfinite(configuration->application.position_error_window_rad) &&
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

bool MotorOuterLoop_IsReady(const MotorOuterLoop *outer)
{
    return outer->ready;
}

/**
 * @brief  外环 慢拍：位置级联与速度 PI 直接执行并发布电流参考。
 * @param configuration 只读活动配置；由调用方保持同拍一致。
 * @param  m 电机控制状态指针。
 * @param  pi 速度环控制器指针（速度模式使用）。
 * @param  encoder 编码器状态指针。
 * @note 仅由外环 tick 调用；本函数是外环输出的唯一写者，快环只读取
 *       motor_state.iq_target_a；模式变化、故障与复位请求在当前慢拍内消费。
 */
void MotorOuterLoop_SlowTick(MotorOuterLoop *outer,
                             const MotorConfiguration *configuration,
                             MotorControlState *m,
                             PiController *pi,
                             EncoderState *encoder)
{
    PositionCascadeControlOutput output;
    bool active =
        m->control_mode == MOTOR_CONTROL_POSITION || m->control_mode == MOTOR_CONTROL_SPEED;
    float position = Encoder_GetMechPos(encoder);
    /* 产品轴的位置串级与速度模式共用电机端独立测速 PLL。 */
    bool position_speed_pll =
        m->control_mode == MOTOR_CONTROL_POSITION && encoder->speed_pll_hz > 0.0f &&
        (configuration->preset != NULL || configuration->application.axis_profile.magic == 0U);
    float speed = (m->control_mode == MOTOR_CONTROL_SPEED || position_speed_pll)
                      ? Encoder_GetMechVel(encoder)
                      : Encoder_GetMechVelContinuous(encoder);

    if (m->control_mode != outer->mode)
    {
        MotorOuterLoop_RequestReset(outer);
        outer->mode = m->control_mode;
        if (active)
        {
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
        }
    }
    if (outer->reset_requested)
    {
        outer->position_otg = (PositionOtg){0};
        outer->plan.state = PLAN_IDLE;
        outer->plan.tick = 0U;
        outer->plan.missed_deadline = false;
        PositionCascade_Reset(&outer->cascade);
        SpeedFeedforward_Reset(&outer->speed_feedforward);
        outer->reset_requested = false;
    }
    if (!active || m->fault_code != MOTOR_FAULT_NONE)
    {
        SpeedFeedforward_Reset(&outer->speed_feedforward);
        outer->ready = false;
        return;
    }
    if (!motor_outer_loop_inputs_valid(configuration, m, position, speed))
    {
        MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
        m->id_target_a = 0.0f;
        m->iq_target_a = 0.0f;
        outer->ready = false;
        PositionCascade_Reset(&outer->cascade);
        SpeedFeedforward_Reset(&outer->speed_feedforward);
        return;
    }

    if (m->control_mode == MOTOR_CONTROL_POSITION)
    {
        bool valid = position_mode_update_control(
            outer,
            configuration,
            m,
            &outer->position_otg,
            position_speed_pll ? 0.0f : POSITION_SERVO_VELOCITY_FILTER_HZ,
            position,
            speed,
            &output);
        valid = valid && isfinite(output.iq_reference);
        if (!valid)
        {
            MotorFault_Report(m,
                              outer->plan.missed_deadline ? MOTOR_FAULT_CONTROL_OVERRUN
                                                          : MOTOR_FAULT_INVALID_PARAMETERS);
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
            outer->ready = false;
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
        m->position_target_reached = outer->position_otg.finished &&
                                     outer->position_otg.target_rad == m->position_target_rad &&
                                     output.target_reached;
        m->position_trajectory_speed_rad_s = outer->position_otg.reference.speed_rad_s;
        m->position_trajectory_acceleration_rad_s2 =
            outer->position_otg.reference.acceleration_rad_s2;
        outer->ready = true;
    }
    else
    {
        outer->ready = SpeedControl_UpdateOuter(configuration,
                                                m,
                                                pi,
                                                &outer->speed_feedforward,
                                                speed,
                                                configuration->application.current_limit_a);
        if (!outer->ready)
        {
            MotorFault_Report(m, MOTOR_FAULT_INVALID_PARAMETERS);
            m->id_target_a = 0.0f;
            m->iq_target_a = 0.0f;
            SpeedFeedforward_Reset(&outer->speed_feedforward);
        }
    }
}
/* OUTER_RUNTIME_END */

/* 模式只选择角度来源、控制律与参考，共用Foc_Step负责所有调制和PWM。 */
MotorTaskResult MotorControl_Step(const MotorConfiguration *configuration,
                                  FocState *foc,
                                  MotorControlState *motor,
                                  EncoderState *encoder,
                                  FluxObserverState *observer)
{
    float phase, velocity;
    float d_reference = motor->id_target_a;
    float q_reference = motor->iq_target_a;
    FocControlLaw law = FOC_CONTROL_CURRENT;
    switch (motor->control_mode)
    {
    case MOTOR_CONTROL_CURRENT:
        if (motor->uses_sensorless_feedback)
        {
            phase = FluxObserver_GetElecPhase(observer);
            velocity = FluxObserver_GetElecVel(observer);
        }
        else
        {
            phase = Encoder_GetElecPhase(encoder);
            velocity = Encoder_GetElecVel(encoder);
        }
        break;
    case MOTOR_CONTROL_SPEED:
    case MOTOR_CONTROL_POSITION:
        phase = Encoder_GetElecPhase(encoder);
        velocity = Encoder_GetElecVel(encoder);
        break;
    case MOTOR_CONTROL_VQ:
        if (!Encoder_IsOnline(encoder))
        {
            MotorFault_Report(motor, MOTOR_FAULT_ENCODER);
            return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        }
        if ((Encoder_GetCalibrationFlag(&configuration->device.encoder) & ENC_CALIB_ALL) !=
            ENC_CALIB_ALL)
        {
            MotorFault_Report(motor, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
            return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        }
        motor->id_target_a = motor->iq_target_a = 0.0f;
        phase = Encoder_GetElecPhase(encoder);
        velocity = Encoder_GetElecVel(encoder);
        d_reference = 0.0f;
        q_reference = motor->vq_target_v;
        law = FOC_CONTROL_ID_VQ;
        break;
    case MOTOR_CONTROL_OPEN_VOLTAGE:
    case MOTOR_CONTROL_OPEN_CURRENT:
        if (!isfinite(motor->open_loop_frequency_hz) ||
            fabsf(motor->open_loop_frequency_hz) >
                MotorConfiguration_OpenLoopLimitHz(configuration,
                                                   MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ) ||
            !isfinite(motor->open_loop_angle_rad) || motor->open_loop_angle_rad < 0.0f ||
            motor->open_loop_angle_rad >= _2PI)
        {
            MotorFault_Report(motor, MOTOR_FAULT_INVALID_PARAMETERS);
            return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        }
        phase = motor->open_loop_angle_rad;
        velocity = _2PI * motor->open_loop_frequency_hz;
        if (motor->control_mode == MOTOR_CONTROL_OPEN_VOLTAGE)
        {
            law = FOC_CONTROL_VOLTAGE;
            d_reference = motor->vd_target_v;
            q_reference = motor->vq_target_v;
        }
        break;
    default:
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }
    Foc_Step(configuration, foc, motor, law, phase, velocity, d_reference, q_reference);
    if (MotorControlMode_IsOpenLoop(motor->control_mode) && motor->fault_code == MOTOR_FAULT_NONE)
    {
        /* 200Hz上限保证每拍小于一圈；保持正/负频率连续，不做无界归一化。 */
        float next = phase + velocity * FOC_TS_S;
        if (next >= _2PI)
        {
            next -= _2PI;
        }
        if (next < 0.0f)
        {
            next += _2PI;
        }
        motor->open_loop_angle_rad = next;
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
