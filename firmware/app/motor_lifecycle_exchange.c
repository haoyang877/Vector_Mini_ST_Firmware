#include "motor_lifecycle.h"

#include <math.h>
#include "application_config.h"
#include "axis_config.h"
#include "bsp_system.h"

/* 先检查该模式的全部字段和限值，拒绝时保留原目标。 */
static bool dq_target_within_limit(float d, float q, float limit)
{
    return isfinite(limit) && limit > 0.0f && d * d + q * q <= limit * limit;
}

static MotorControlResult validate_and_set_mode_target(MotorLifecycle *life,
                                                       const MotorTarget *target)
{
    const MotorLifecycleBindings *drive = &life->drive;
    const MotorConfiguration *configuration = drive->configuration;
    MotorControlState *motor = drive->motor;

    switch ((MotorControlMode)target->mode)
    {
    case MOTOR_CONTROL_CURRENT:
    {
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->vd_v != 0.0f ||
            target->vq_v != 0.0f || target->frequency_hz != 0.0f || target->id_a != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_a = configuration->application.current_limit_a;
        if (motor->thermal_limit_active)
        {
            limit_a = fminf(limit_a, motor->thermal_current_limit_a);
        }
        if (!dq_target_within_limit(target->id_a, target->iq_a, limit_a))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->id_target_a = target->id_a;
        motor->iq_target_a = target->iq_a;
        break;
    }
    case MOTOR_CONTROL_SPEED:
    {
        if (target->position_rad != 0.0f || target->id_a != 0.0f || target->iq_a != 0.0f ||
            target->vd_v != 0.0f || target->vq_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_rad_s = configuration->application.speed_limit_rad_s;
        if (!isfinite(limit_rad_s) || fabsf(target->speed_rad_s) > limit_rad_s)
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->speed_target_rad_s = target->speed_rad_s;
        return MOTOR_CONTROL_ACCEPTED;
    }
    case MOTOR_CONTROL_POSITION:
        if (target->speed_rad_s != 0.0f || target->id_a != 0.0f || target->iq_a != 0.0f ||
            target->vd_v != 0.0f || target->vq_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!MotorAxisProfile_AllowsPosition(&configuration->application.axis_profile,
                                             configuration->application.axis_profile_valid,
                                             Encoder_GetMechPos(drive->rotor_encoder),
                                             target->position_rad))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->position_target_rad = target->position_rad;
        return MOTOR_CONTROL_ACCEPTED;
    case MOTOR_CONTROL_VQ:
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->id_a != 0.0f ||
            target->iq_a != 0.0f || target->vd_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!dq_target_within_limit(
                target->vd_v, target->vq_v, Foc_MaxVoltage(drive->foc->bus_voltage_filtered_v)))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->vd_target_v = target->vd_v;
        motor->vq_target_v = target->vq_v;
        break;
    case MOTOR_CONTROL_OPEN_VOLTAGE:
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->id_a != 0.0f ||
            target->iq_a != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!(fabsf(target->frequency_hz) <=
              MotorConfiguration_OpenLoopLimitHz(configuration,
                                                 MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ)) ||
            !dq_target_within_limit(
                target->vd_v, target->vq_v, Foc_MaxVoltage(drive->foc->bus_voltage_filtered_v)))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->vd_target_v = target->vd_v;
        motor->vq_target_v = target->vq_v;
        motor->open_loop_frequency_hz = target->frequency_hz;
        break;
    case MOTOR_CONTROL_OPEN_CURRENT:
    {
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->vd_v != 0.0f ||
            target->vq_v != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_a = configuration->application.current_limit_a;
        if (motor->thermal_limit_active)
        {
            limit_a = fminf(limit_a, motor->thermal_current_limit_a);
        }
        if (!(fabsf(target->frequency_hz) <=
              MotorConfiguration_OpenLoopLimitHz(configuration,
                                                 MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ)) ||
            !dq_target_within_limit(target->id_a, target->iq_a, limit_a))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->id_target_a = target->id_a;
        motor->iq_target_a = target->iq_a;
        motor->open_loop_frequency_hz = target->frequency_hz;
        break;
    }
    default:
        return MOTOR_CONTROL_INVALID;
    }

    return MOTOR_CONTROL_ACCEPTED;
}

MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *life, const MotorTarget *target)
{
    if (target == NULL || !MotorControlMode_IsValid((MotorControlMode)target->mode) ||
        !isfinite(target->position_rad) || !isfinite(target->speed_rad_s) ||
        !isfinite(target->iq_a) || !isfinite(target->vq_v) || !isfinite(target->frequency_hz) ||
        !isfinite(target->id_a) || !isfinite(target->vd_v))
    {
        return MOTOR_CONTROL_INVALID;
    }

    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (life->state == MOTOR_STATE_RUNNING && !life->stop_pending && !life->starts_locked &&
             target->mode == life->drive.motor->control_mode)
    {
        result = validate_and_set_mode_target(life, target);
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

void MotorLifecycle_BeginOuter(MotorLifecycle *life, MotorOuterWork *work)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    work->owner = life;
    work->allowed = false;
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t fast = life->fast_sample_ticks;
        BspSystem_MemoryBarrier();
        work->epoch = life->epoch;
        work->motor = *life->drive.motor;
        work->speed_pi = *life->drive.speed_pi;
        BspSystem_MemoryBarrier();
        if (fast == life->fast_sample_ticks)
        {
            work->allowed = (life->state == MOTOR_STATE_RUNNING || life->start_pending) &&
                            work->motor.control_mode != MOTOR_CONTROL_NONE &&
                            work->motor.fault_code == MOTOR_FAULT_NONE && !life->stop_pending &&
                            !life->starts_locked;
            break;
        }
    }
    BspSystem_ExitControlCritical(mask);
}

static MotorStatus build_motor_feedback_snapshot(const MotorLifecycle *life)
{
    const MotorControlState *motor = life->drive.motor;
    const FocState *foc = life->drive.foc;
    const EncoderState *encoder = life->drive.rotor_encoder;
    bool active = life->state == MOTOR_STATE_RUNNING && motor->control_mode != MOTOR_CONTROL_NONE;
    bool current = active && motor->control_mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
                   motor->control_mode != MOTOR_CONTROL_VQ;
    bool position = active && motor->control_mode == MOTOR_CONTROL_POSITION;
    bool speed = active && motor->control_mode == MOTOR_CONTROL_SPEED;
    return (MotorStatus){
        .state = life->state,
        .mode = (uint16_t)motor->control_mode,
        .fault = (uint16_t)motor->fault_code,
        .measurements_valid = life->fast_samples_fresh,
        .position_reference_rad = position ? motor->position_reference_rad : NAN,
        .position_rad = encoder->has_valid_sample ? encoder->mechanical_position_rad : NAN,
        .speed_reference_rad_s = speed      ? motor->speed_target_rad_s
                                 : position ? motor->speed_ramped_rad_s
                                            : NAN,
        .speed_rad_s =
            encoder->has_valid_sample ? encoder->mechanical_velocity_continuous_rad_s : NAN,
        .bus_voltage_v = foc->bus_voltage_filtered_v,
        .bus_current_a = foc->bus_current_filtered_a,
        .iq_reference_a = current ? motor->iq_target_a : NAN,
        .iq_a = active ? foc->current_dq_a.q : NAN,
        .temperature_c = McuTemperature_IsFresh(life->drive.temperature) ? foc->temperature_c : NAN,
        .vq_v = active ? foc->voltage_dq_v.q : NAN,
        .vd_v = active ? foc->voltage_dq_v.d : NAN,
    };
}

bool MotorLifecycle_ReadStatus(const MotorLifecycle *life, MotorStatus *status)
{
    /* 每个计数只有对应 ISR 写。前台被抢占后重读一次，连续争用则报告本帧不可用。 */
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t fast = life->fast_sample_ticks;
        uint32_t outer = life->outer_ticks;
        uint32_t supervisor = life->supervisor_ticks;
        BspSystem_MemoryBarrier();
        MotorStatus sample = build_motor_feedback_snapshot(life);
        BspSystem_MemoryBarrier();
        if (fast == life->fast_sample_ticks && outer == life->outer_ticks &&
            supervisor == life->supervisor_ticks)
        {
            *status = sample;
            return true;
        }
    }
    return false;
}
