#include "motor_execution.h"
#include <math.h>

#include "axis_config.h"

/* APP命令入口只校验目标格式和数值范围；控制计算归motor层。 */

/* 只验证 SI 目标格式与控制范围，不读取生命周期、不修改对象或输出。 */
MotorControlResult MotorExecution_ValidateTarget(const MotorConfiguration *configuration,
                                                 const MotorControlCommand *command,
                                                 const FocState *foc,
                                                 const EncoderState *encoder)
{
    if (command == NULL || !isfinite(command->position_rad) || !isfinite(command->speed_rad_s) ||
        !isfinite(command->iq_a) || !isfinite(command->vq_v))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if ((unsigned)command->mode > MOTOR_CONTROL_VQ)
    {
        return MOTOR_CONTROL_INVALID;
    }
    if ((command->mode != MOTOR_CONTROL_POSITION && command->position_rad != 0.0f) ||
        (command->mode != MOTOR_CONTROL_SPEED && command->speed_rad_s != 0.0f) ||
        (command->mode != MOTOR_CONTROL_CURRENT && command->iq_a != 0.0f) ||
        (command->mode != MOTOR_CONTROL_VQ && command->vq_v != 0.0f))
    {
        return MOTOR_CONTROL_INVALID;
    }
    if (configuration == NULL || foc == NULL || encoder == NULL)
    {
        return MOTOR_CONTROL_INVALID;
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
