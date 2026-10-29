#include "motor_command.h"
#include <math.h>

#include "axis_config.h"
#include "control_math.h"

/* APP命令入口只校验目标格式和数值范围；控制计算归motor层。 */

/* 只验证 SI 目标格式与控制范围，不读取生命周期、不修改对象或输出。 */
MotorControlResult MotorCommand_ValidateTarget(const MotorConfiguration *configuration,
                                               const MotorControlCommand *command,
                                               const FocState *foc,
                                               const EncoderState *encoder)
{
    if (command == NULL || !isfinite(command->position_rad) || !isfinite(command->speed_rad_s) ||
        !isfinite(command->iq_a) || !isfinite(command->vq_v) || !isfinite(command->id_a) ||
        !isfinite(command->vd_v) || !isfinite(command->electrical_frequency_hz))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (!MotorControlMode_IsValid(command->mode))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if ((command->mode != MOTOR_CONTROL_POSITION && command->position_rad != 0.0f) ||
        (command->mode != MOTOR_CONTROL_SPEED && command->speed_rad_s != 0.0f) ||
        (command->mode != MOTOR_CONTROL_CURRENT && command->mode != MOTOR_CONTROL_OPEN_CURRENT &&
         command->iq_a != 0.0f) ||
        (command->mode != MOTOR_CONTROL_VQ && command->mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
         command->vq_v != 0.0f) ||
        (command->mode != MOTOR_CONTROL_OPEN_CURRENT && command->id_a != 0.0f) ||
        (command->mode != MOTOR_CONTROL_OPEN_VOLTAGE && command->vd_v != 0.0f) ||
        (!MotorControlMode_IsOpenLoop(command->mode) && command->electrical_frequency_hz != 0.0f))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (configuration == NULL || foc == NULL || encoder == NULL)
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (MotorControlMode_IsOpenLoop(command->mode))
    {
        float electrical_hz = fabsf(command->electrical_frequency_hz);
        float speed_limit = configuration->application.speed_limit_rad_s;
        if (!isfinite(speed_limit) || speed_limit <= 0.0f)
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        if (configuration->application.axis_profile.magic != 0U)
        {
            float axis_limit = configuration->application.axis_profile.maximum_speed_rad_s;
            if (!isfinite(axis_limit) || axis_limit <= 0.0f)
            {
                return MOTOR_CONTROL_OUT_OF_RANGE;
            }
            speed_limit = fminf(speed_limit, axis_limit);
        }
        if (configuration->model.motor_pole_pairs <= 0 ||
            electrical_hz > MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ ||
            electrical_hz * _2PI > speed_limit * configuration->model.motor_pole_pairs)
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        float squared = command->id_a * command->id_a + command->iq_a * command->iq_a;
        float limit = configuration->application.current_limit_a;
        if (command->mode == MOTOR_CONTROL_OPEN_VOLTAGE)
        {
            squared = command->vd_v * command->vd_v + command->vq_v * command->vq_v;
            limit = FOC_MaximumVoltage(foc->bus_voltage_filtered_v);
        }
        if (!isfinite(limit) || limit <= 0.0f || !isfinite(squared) || squared > limit * limit)
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        return MOTOR_CONTROL_ACCEPTED;
    }
    switch (command->mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return isfinite(configuration->application.current_limit_a) &&
                       configuration->application.current_limit_a > 0.0f &&
                       fabsf(command->iq_a) <= configuration->application.current_limit_a
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_SPEED:
        return isfinite(configuration->application.speed_limit_rad_s) &&
                       configuration->application.speed_limit_rad_s > 0.0f &&
                       fabsf(command->speed_rad_s) <=
                           configuration->application.speed_limit_rad_s &&
                       (configuration->application.axis_profile.magic == 0U ||
                        (isfinite(configuration->application.axis_profile.maximum_speed_rad_s) &&
                         configuration->application.axis_profile.maximum_speed_rad_s > 0.0f &&
                         fabsf(command->speed_rad_s) <=
                             configuration->application.axis_profile.maximum_speed_rad_s))
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_POSITION:
        return MotorAxisProfile_AllowsPosition(&configuration->application.axis_profile,
                                               configuration->application.axis_profile_valid,
                                               Encoder_GetMecPos(encoder),
                                               command->position_rad)
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    case MOTOR_CONTROL_VQ:
        return fabsf(command->vq_v) <= FOC_MaximumVoltage(foc->bus_voltage_filtered_v)
                   ? MOTOR_CONTROL_ACCEPTED
                   : MOTOR_CONTROL_OUT_OF_RANGE;
    default:
        return MOTOR_CONTROL_INVALID;
    }
}
