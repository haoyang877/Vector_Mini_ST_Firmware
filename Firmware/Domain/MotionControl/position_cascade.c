#include "position_cascade.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float PositionCascade_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static float PositionCascade_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static bool PositionCascade_ConfigIsValid(
    const PositionCascadeConfig *config)
{
    return config != NULL &&
        isfinite(config->target_position) &&
        isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
        isfinite(config->acceleration) && config->acceleration > 0.0f &&
        isfinite(config->deceleration) && config->deceleration > 0.0f &&
        isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
        isfinite(config->position_kp) && config->position_kp >= 0.0f &&
        isfinite(config->position_kp_limit) && config->position_kp_limit > 0.0f &&
        config->position_kp <= config->position_kp_limit &&
        isfinite(config->position_kd) && config->position_kd >= 0.0f &&
        isfinite(config->position_kd_limit) && config->position_kd_limit >= 0.0f &&
        config->position_kd <= config->position_kd_limit &&
        isfinite(config->speed_kp) && config->speed_kp >= 0.0f &&
        isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
        isfinite(config->current_limit_a) && config->current_limit_a > 0.0f &&
        isfinite(config->position_sample_period_s) &&
        config->position_sample_period_s > 0.0f &&
        isfinite(config->speed_sample_period_s) &&
        config->speed_sample_period_s > 0.0f &&
        config->position_loop_divider > 0U && config->speed_loop_divider > 0U;
}

static void PositionCascade_CopyOutput(const PositionCascadeContext *context,
    PositionCascadeOutput *output, float measured_speed)
{
    output->position_reference = context->position_reference;
    output->speed_reference = context->speed_reference;
    output->speed_feedback = measured_speed;
    output->iq_reference = context->iq_reference;
    output->target_reached = context->target_reached;
}

void PositionCascade_Reset(PositionCascadeContext *context)
{
    if (context == NULL)
        return;
    memset(context, 0, sizeof(*context));
    PI_Controller_Reset(&context->speed_controller);
    TrapezoidalTrajectory_Reset(&context->trajectory);
}

bool PositionCascade_Update(PositionCascadeContext *context,
    const PositionCascadeConfig *config,
    float measured_position, float measured_speed,
    PositionCascadeOutput *output)
{
    bool trajectory_needs_update;

    if (context == NULL || output == NULL || !isfinite(measured_position) ||
        !isfinite(measured_speed) || !PositionCascade_ConfigIsValid(config))
        return false;

    trajectory_needs_update = !context->initialized ||
        context->last_target != config->target_position ||
        context->last_acceleration != config->acceleration ||
        context->last_deceleration != config->deceleration ||
        context->last_maximum_speed != config->maximum_speed;
    if (trajectory_needs_update)
    {
        if (!context->initialized)
        {
            context->position_reference = measured_position;
            context->speed_reference = 0.0f;
            context->initialized = true;
        }
        if (context->last_target != config->target_position)
            context->target_reached = false;
        if (!TrapezoidalTrajectory_Plan(&context->trajectory,
            config->target_position, context->position_reference,
            context->speed_reference, config->maximum_speed,
            config->acceleration, config->deceleration))
            return false;
        context->last_target = config->target_position;
        context->last_acceleration = config->acceleration;
        context->last_deceleration = config->deceleration;
        context->last_maximum_speed = config->maximum_speed;
    }

    if (++context->position_loop_count >= config->position_loop_divider)
    {
        float position_error;
        float derivative_error;
        float speed_correction;

        context->position_loop_count = 0U;
        TrapezoidalTrajectory_Evaluate(&context->trajectory,
            config->position_sample_period_s);
        context->position_reference = context->trajectory.Y;
        context->speed_reference = context->trajectory.Yd;
        position_error = context->position_reference - measured_position;
        derivative_error = (position_error - context->last_position_error) /
            config->position_sample_period_s;
        speed_correction = config->position_kp * position_error;
        if (context->target_reached)
            speed_correction += config->position_kd * derivative_error;
        context->speed_controller.Ref = PositionCascade_Clamp(
            context->speed_reference + speed_correction,
            -config->maximum_speed, config->maximum_speed);
        if (PositionCascade_Abs(measured_position - config->target_position) <=
            config->position_error_window)
            context->target_reached = true;
        context->last_position_error = position_error;
    }

    if (++context->speed_loop_count >= config->speed_loop_divider)
    {
        context->speed_loop_count = 0U;
        PI_Controller_Configure(&context->speed_controller, config->speed_kp,
            config->speed_ki, config->speed_sample_period_s, -1.0f, 1.0f);
        context->iq_reference = PI_Controller_Run(&context->speed_controller,
            context->speed_controller.Ref, measured_speed) * config->current_limit_a;
    }
    if (!isfinite(context->iq_reference))
        return false;
    context->iq_reference = PositionCascade_Clamp(context->iq_reference,
        -config->current_limit_a, config->current_limit_a);
    PositionCascade_CopyOutput(context, output, measured_speed);
    return true;
}
