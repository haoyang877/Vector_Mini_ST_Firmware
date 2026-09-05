#include "position_impedance.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define POSITION_IMPEDANCE_TWO_PI                              6.2831853072f
#define POSITION_IMPEDANCE_VELOCITY_FILTER_HZ                  20.0f
#define POSITION_IMPEDANCE_INTEGRAL_ZONE_RAD                   0.08726646f
#define POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S                0.03f
#define POSITION_IMPEDANCE_INTEGRAL_DECAY_DISTANCE_RAD         0.08726646f
#define POSITION_IMPEDANCE_INTEGRAL_DECAY_MIN_SPEED_RAD_S      0.02f
#define POSITION_IMPEDANCE_INTEGRAL_OPPOSING_DECAY_RATIO       0.01f
#define POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_RATIO            0.01f
#define POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A           0.005f
#define POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A           0.0005f
#define POSITION_IMPEDANCE_HOLD_ENTER_SPEED_RAD_S              0.03f
#define POSITION_IMPEDANCE_HOLD_EXIT_SPEED_RAD_S               0.08f
#define POSITION_IMPEDANCE_HOLD_TIME_S                          0.05f

typedef enum
{
	POSITION_IMPEDANCE_FRICTION_TRACK = 0,
	POSITION_IMPEDANCE_FRICTION_LANDING,
	POSITION_IMPEDANCE_FRICTION_HOLD,
	POSITION_IMPEDANCE_FRICTION_RECOVERY
} PositionImpedanceFrictionState;

static float PositionImpedance_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static float PositionImpedance_Min(float left, float right)
{
    return left < right ? left : right;
}

static float PositionImpedance_Max(float left, float right)
{
	return left > right ? left : right;
}

static float PositionImpedance_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static bool PositionImpedance_ConfigIsValid(
    const PositionImpedanceConfig *config)
{
    return config != NULL &&
        isfinite(config->target_position) &&
        isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
        isfinite(config->acceleration) && config->acceleration > 0.0f &&
        isfinite(config->deceleration) && config->deceleration > 0.0f &&
        isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
        isfinite(config->maximum_speed_limit_rad_s) && config->maximum_speed_limit_rad_s > 0.0f &&
        config->maximum_speed <= config->maximum_speed_limit_rad_s &&
        isfinite(config->speed_limit_rad_s) && config->speed_limit_rad_s > 0.0f &&
        config->maximum_speed <= config->speed_limit_rad_s &&
        isfinite(config->kp) && config->kp >= 0.0f &&
        isfinite(config->kp_limit) && config->kp_limit > 0.0f &&
        config->kp <= config->kp_limit &&
        isfinite(config->kd) && config->kd >= 0.0f &&
        isfinite(config->kd_limit) && config->kd_limit >= 0.0f &&
        config->kd <= config->kd_limit &&
        isfinite(config->ki) && config->ki >= 0.0f &&
        isfinite(config->ki_limit) && config->ki_limit >= 0.0f &&
        config->ki <= config->ki_limit &&
        isfinite(config->integral_limit) && config->integral_limit >= 0.0f &&
        isfinite(config->maximum_current_limit_a) && config->maximum_current_limit_a > 0.0f &&
        config->integral_limit <= config->maximum_current_limit_a &&
        isfinite(config->output_limit) && config->output_limit > 0.0f &&
        config->output_limit <= config->maximum_current_limit_a &&
		isfinite(config->friction_positive_current) &&
		config->friction_positive_current >= 0.0f &&
		config->friction_positive_current <= config->maximum_current_limit_a &&
		isfinite(config->friction_negative_current) &&
		config->friction_negative_current >= 0.0f &&
		config->friction_negative_current <= config->maximum_current_limit_a &&
		isfinite(config->breakaway_positive_current) &&
		config->breakaway_positive_current >= config->friction_positive_current &&
		config->breakaway_positive_current <= config->maximum_current_limit_a &&
		isfinite(config->breakaway_negative_current) &&
		config->breakaway_negative_current >= config->friction_negative_current &&
		config->breakaway_negative_current <= config->maximum_current_limit_a &&
		isfinite(config->friction_current_slew_rate) &&
		config->friction_current_slew_rate > 0.0f &&
		isfinite(config->friction_position_enter) &&
		config->friction_position_enter > 0.0f &&
		isfinite(config->friction_position_exit) &&
		config->friction_position_exit > config->friction_position_enter &&
		isfinite(config->friction_reference_speed) &&
		config->friction_reference_speed > 0.0f &&
		isfinite(config->friction_stop_speed) &&
		config->friction_stop_speed >= 0.0f &&
		isfinite(config->friction_move_speed) &&
		config->friction_move_speed > config->friction_stop_speed &&
		isfinite(config->friction_stuck_time) && config->friction_stuck_time >= 0.0f &&
		isfinite(config->friction_landing_position) &&
		config->friction_landing_position > config->friction_position_exit &&
		isfinite(config->friction_landing_speed) &&
		config->friction_landing_speed > config->friction_reference_speed &&
		isfinite(config->friction_recovery_delay) &&
		config->friction_recovery_delay >= 0.0f &&
		isfinite(config->friction_recovery_pulse_time) &&
		config->friction_recovery_pulse_time > 0.0f &&
		isfinite(config->friction_recovery_cooldown) &&
		config->friction_recovery_cooldown >= 0.0f &&
        isfinite(config->sample_period_s) && config->sample_period_s > 0.0f &&
        config->loop_divider > 0U;
}

static void PositionImpedance_CopyOutput(
    const PositionImpedanceContext *context,
    PositionImpedanceOutput *output)
{
    output->position_reference = context->position_reference;
    output->speed_reference = context->speed_reference;
    output->velocity_feedback = context->velocity_filtered;
    output->iq_reference = context->iq_reference;
	output->friction_current = context->friction_current;
	output->target_reached = context->target_reached;
}

static float PositionImpedance_GetFrictionMagnitude(
	const PositionImpedanceConfig *config, int8_t direction, bool breakaway)
{
	if (direction > 0)
		return breakaway ? config->breakaway_positive_current :
			config->friction_positive_current;
	if (direction < 0)
		return breakaway ? config->breakaway_negative_current :
			config->friction_negative_current;
	return 0.0f;
}

static uint32_t PositionImpedance_TimeToTicks(float time_s,
	float sample_period_s)
{
	float ticks;
	if (time_s <= 0.0f)
		return 0U;
	ticks = time_s / sample_period_s + 0.5f;
	if (ticks >= 4294967295.0f)
		return UINT32_MAX;
	return ticks < 1.0f ? 1U : (uint32_t)ticks;
}

static void PositionImpedance_UpdateFriction(PositionImpedanceContext *context,
	const PositionImpedanceConfig *config, float target_error)
{
	float friction_target = 0.0f;
	float friction_step;
	float recovery_magnitude;
	uint32_t stuck_ticks;
	uint32_t recovery_delay_ticks;
	uint32_t recovery_pulse_ticks;
	uint32_t recovery_cooldown_ticks;
	int8_t desired_direction;
	bool reference_is_moving = PositionImpedance_Abs(context->speed_reference) >
		config->friction_reference_speed;
	bool feedback_is_stopped = PositionImpedance_Abs(context->velocity_filtered) <=
		config->friction_stop_speed;
	bool feedback_is_moving = PositionImpedance_Abs(context->velocity_filtered) >=
		config->friction_move_speed;
	bool trajectory_is_finished = PositionImpedance_Abs(config->target_position -
		context->position_reference) <= config->position_error_window;

	if (!config->friction_feedforward_enabled)
	{
		context->friction_state = POSITION_IMPEDANCE_FRICTION_TRACK;
		context->friction_direction = 0;
		context->friction_stuck_counter = 0U;
		context->friction_recovery_delay_counter = 0U;
		context->friction_recovery_pulse_counter = 0U;
		context->friction_recovery_cooldown_counter = 0U;
		context->friction_current = 0.0f;
		return;
	}

	friction_step = config->friction_current_slew_rate * config->sample_period_s;
	stuck_ticks = PositionImpedance_TimeToTicks(config->friction_stuck_time,
		config->sample_period_s);
	recovery_delay_ticks = PositionImpedance_TimeToTicks(
		config->friction_recovery_delay, config->sample_period_s);
	recovery_pulse_ticks = PositionImpedance_TimeToTicks(
		config->friction_recovery_pulse_time, config->sample_period_s);
	recovery_cooldown_ticks = PositionImpedance_TimeToTicks(
		config->friction_recovery_cooldown, config->sample_period_s);
	if (context->friction_recovery_cooldown_counter > 0U)
		context->friction_recovery_cooldown_counter--;

	switch ((PositionImpedanceFrictionState)context->friction_state)
	{
		case POSITION_IMPEDANCE_FRICTION_TRACK:
			if (PositionImpedance_Abs(target_error) <=
					config->friction_landing_position &&
				PositionImpedance_Abs(context->speed_reference) <=
					config->friction_landing_speed)
			{
				context->friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
				context->friction_stuck_counter = 0U;
				context->friction_recovery_delay_counter = 0U;
				break;
			}
			desired_direction = context->friction_direction;
			if (reference_is_moving)
				desired_direction = context->speed_reference > 0.0f ? 1 : -1;
			else if (desired_direction == 0 &&
				PositionImpedance_Abs(target_error) >= config->friction_position_exit)
				desired_direction = target_error > 0.0f ? 1 : -1;
			context->friction_direction = desired_direction;
			if (desired_direction == 0)
			{
				context->friction_stuck_counter = 0U;
				break;
			}
			if (feedback_is_moving)
				context->friction_stuck_counter = 0U;
			else if (feedback_is_stopped &&
				context->friction_stuck_counter < UINT32_MAX)
				context->friction_stuck_counter++;
			else
				context->friction_stuck_counter = 0U;
			friction_target = PositionImpedance_GetFrictionMagnitude(config,
				desired_direction, stuck_ticks == 0U ||
				context->friction_stuck_counter >= stuck_ticks);
			if (desired_direction < 0)
				friction_target = -friction_target;
			break;

		case POSITION_IMPEDANCE_FRICTION_LANDING:
			context->friction_stuck_counter = 0U;
			if (context->target_reached)
			{
				context->friction_state = POSITION_IMPEDANCE_FRICTION_HOLD;
				context->friction_recovery_delay_counter = 0U;
			}
			else if (trajectory_is_finished && !feedback_is_moving &&
				PositionImpedance_Abs(target_error) > config->friction_position_exit &&
				context->friction_recovery_cooldown_counter == 0U)
			{
				if (context->friction_recovery_delay_counter < UINT32_MAX)
					context->friction_recovery_delay_counter++;
				if (recovery_delay_ticks == 0U ||
					context->friction_recovery_delay_counter >= recovery_delay_ticks)
				{
					context->friction_state = POSITION_IMPEDANCE_FRICTION_RECOVERY;
					context->friction_direction = target_error > 0.0f ? 1 : -1;
					context->friction_recovery_delay_counter = 0U;
					context->friction_recovery_pulse_counter = 0U;
					friction_target = PositionImpedance_GetFrictionMagnitude(config,
						context->friction_direction, true);
					if (context->friction_direction < 0)
						friction_target = -friction_target;
				}
			}
			else
				context->friction_recovery_delay_counter = 0U;
			break;

		case POSITION_IMPEDANCE_FRICTION_HOLD:
			context->friction_stuck_counter = 0U;
			context->friction_recovery_delay_counter = 0U;
			if (!context->target_reached)
				context->friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
			break;

		case POSITION_IMPEDANCE_FRICTION_RECOVERY:
		default:
			recovery_magnitude = PositionImpedance_GetFrictionMagnitude(config,
				context->friction_direction, true);
			friction_target = context->friction_direction > 0 ?
				recovery_magnitude : -recovery_magnitude;
			if (context->friction_direction == 0 ||
				context->friction_direction * target_error <=
					config->friction_position_enter || feedback_is_moving)
			{
				context->friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
				context->friction_recovery_pulse_counter = 0U;
				context->friction_recovery_cooldown_counter = recovery_cooldown_ticks;
				friction_target = 0.0f;
			}
			else if (PositionImpedance_Abs(context->friction_current) >=
				PositionImpedance_Min(recovery_magnitude, config->output_limit) -
					friction_step)
			{
				if (context->friction_recovery_pulse_counter < UINT32_MAX)
					context->friction_recovery_pulse_counter++;
				if (recovery_pulse_ticks == 0U ||
					context->friction_recovery_pulse_counter >= recovery_pulse_ticks)
				{
					context->friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
					context->friction_recovery_pulse_counter = 0U;
					context->friction_recovery_cooldown_counter = recovery_cooldown_ticks;
					friction_target = 0.0f;
				}
			}
			break;
	}

	context->friction_current += PositionImpedance_Clamp(
		friction_target - context->friction_current, -friction_step, friction_step);
	context->friction_current = PositionImpedance_Clamp(context->friction_current,
		-config->output_limit, config->output_limit);
	if (!isfinite(context->friction_current))
		context->friction_current = 0.0f;
}

void PositionImpedance_Reset(PositionImpedanceContext *context)
{
    if (context == NULL)
        return;
    memset(context, 0, sizeof(*context));
    TrapezoidalTrajectory_Reset(&context->trajectory);
}

bool PositionImpedance_Update(PositionImpedanceContext *context,
    const PositionImpedanceConfig *config,
    float measured_position, PositionImpedanceOutput *output)
{
    float raw_velocity;
    float velocity_filter_ratio;
    float velocity_filter_alpha;
    float target_error;
    float position_error;
    float velocity_error;
    float proportional_current;
    float damping_current;
    float integral_candidate;
    float current_candidate;
    float integral_limit;
    float desired_motion;
    float integral_transport_distance;
    float integral_decay_ratio;
    float integral_decay_step;
	float hold_exit_position;
    uint32_t hold_ticks;
    bool trajectory_is_moving;
    bool target_is_unsettled;
    bool integral_opposes_motion;
    bool feedback_follows_motion;
    bool trajectory_needs_update;

    if (context == NULL || output == NULL || !isfinite(measured_position) ||
        !PositionImpedance_ConfigIsValid(config))
        return false;

    if (!context->initialized)
    {
        PositionImpedance_Reset(context);
        context->initialized = true;
        context->last_position = measured_position;
        context->position_reference = measured_position;
        context->last_target = config->target_position;
        context->last_acceleration = config->acceleration;
        context->last_deceleration = config->deceleration;
        context->last_maximum_speed = config->maximum_speed;
        trajectory_needs_update = true;
    }
    else
    {
        trajectory_needs_update =
            context->last_target != config->target_position ||
            context->last_acceleration != config->acceleration ||
            context->last_deceleration != config->deceleration ||
            context->last_maximum_speed != config->maximum_speed;
        if (context->last_target != config->target_position)
        {
            context->target_reached = false;
            context->hold_counter = 0U;
            context->integral_transport_active = true;
			context->friction_state = POSITION_IMPEDANCE_FRICTION_TRACK;
			context->friction_direction = 0;
			context->friction_stuck_counter = 0U;
			context->friction_recovery_delay_counter = 0U;
			context->friction_recovery_pulse_counter = 0U;
			context->friction_recovery_cooldown_counter = 0U;
        }
    }

    if (trajectory_needs_update)
    {
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

    if (++context->loop_counter < config->loop_divider)
    {
        PositionImpedance_CopyOutput(context, output);
        return true;
    }
    context->loop_counter = 0U;

    TrapezoidalTrajectory_Evaluate(&context->trajectory,
        config->sample_period_s);
    context->position_reference = context->trajectory.Y;
    context->speed_reference = context->trajectory.Yd;

    raw_velocity = (measured_position - context->last_position) /
        config->sample_period_s;
    context->last_position = measured_position;
    if (!isfinite(raw_velocity))
        raw_velocity = 0.0f;
    raw_velocity = PositionImpedance_Clamp(raw_velocity,
        -config->speed_limit_rad_s, config->speed_limit_rad_s);
    velocity_filter_ratio = POSITION_IMPEDANCE_TWO_PI *
        POSITION_IMPEDANCE_VELOCITY_FILTER_HZ * config->sample_period_s;
    velocity_filter_alpha = velocity_filter_ratio / (1.0f + velocity_filter_ratio);
    context->velocity_filtered += velocity_filter_alpha *
        (raw_velocity - context->velocity_filtered);
    if (!isfinite(context->velocity_filtered))
        context->velocity_filtered = 0.0f;

    target_error = config->target_position - measured_position;
	hold_exit_position = 2.0f * config->position_error_window;
	if (config->friction_feedforward_enabled)
		hold_exit_position = PositionImpedance_Max(hold_exit_position,
			config->friction_position_exit);
    hold_ticks = (uint32_t)(POSITION_IMPEDANCE_HOLD_TIME_S /
        config->sample_period_s + 0.5f);
    if (hold_ticks < 1U)
        hold_ticks = 1U;
    if (context->target_reached)
    {
        if (PositionImpedance_Abs(target_error) > hold_exit_position ||
            PositionImpedance_Abs(context->velocity_filtered) >
                POSITION_IMPEDANCE_HOLD_EXIT_SPEED_RAD_S)
        {
            context->target_reached = false;
            context->hold_counter = 0U;
        }
    }
    else if (PositionImpedance_Abs(config->target_position -
                 context->position_reference) <= config->position_error_window &&
             PositionImpedance_Abs(context->speed_reference) <=
                 POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
             PositionImpedance_Abs(target_error) <= config->position_error_window &&
             PositionImpedance_Abs(context->velocity_filtered) <=
                 POSITION_IMPEDANCE_HOLD_ENTER_SPEED_RAD_S)
    {
        if (context->hold_counter < hold_ticks)
            context->hold_counter++;
        if (context->hold_counter >= hold_ticks)
            context->target_reached = true;
    }
    else
    {
        context->hold_counter = 0U;
    }

    position_error = context->position_reference - measured_position;
    velocity_error = context->speed_reference - context->velocity_filtered;
    proportional_current = config->kp * position_error;
    damping_current = config->kd * velocity_error;
	PositionImpedance_UpdateFriction(context, config, target_error);
    integral_limit = PositionImpedance_Min(config->integral_limit,
        config->output_limit);
    if (!isfinite(context->integral))
        context->integral = 0.0f;
    context->integral = PositionImpedance_Clamp(context->integral,
        -integral_limit, integral_limit);
    if (context->integral_transport_active &&
        (context->target_reached ||
         PositionImpedance_Abs(context->integral) <=
             POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A))
        context->integral_transport_active = false;

    if (integral_limit <= 0.0f)
    {
        context->integral = 0.0f;
        context->integral_transport_active = false;
    }
	else if (config->friction_feedforward_enabled &&
		context->friction_state != POSITION_IMPEDANCE_FRICTION_TRACK)
	{
		context->integral_transport_active = false;
	}
    else if (config->ki <= 0.0f)
    {
        integral_decay_step = PositionImpedance_Clamp(context->integral *
            POSITION_IMPEDANCE_INTEGRAL_OPPOSING_DECAY_RATIO,
            -POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A,
             POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A);
        context->integral -= integral_decay_step;
        if (PositionImpedance_Abs(context->integral) <=
            POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A)
        {
            context->integral = 0.0f;
            context->integral_transport_active = false;
        }
    }
    else
    {
        trajectory_is_moving = PositionImpedance_Abs(context->speed_reference) >
            POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S;
        target_is_unsettled = PositionImpedance_Abs(target_error) >
            config->position_error_window;
        desired_motion = trajectory_is_moving ? context->speed_reference : target_error;
        integral_opposes_motion = target_is_unsettled &&
            context->integral * desired_motion < 0.0f;
        feedback_follows_motion = target_is_unsettled &&
            PositionImpedance_Abs(context->velocity_filtered) >
                POSITION_IMPEDANCE_INTEGRAL_DECAY_MIN_SPEED_RAD_S &&
            desired_motion * context->velocity_filtered > 0.0f &&
            desired_motion * raw_velocity > 0.0f;

        if (context->integral_transport_active &&
            (trajectory_is_moving || target_is_unsettled) &&
            (integral_opposes_motion || feedback_follows_motion))
        {
            if (integral_opposes_motion)
                integral_decay_ratio =
                    POSITION_IMPEDANCE_INTEGRAL_OPPOSING_DECAY_RATIO;
            else
            {
                integral_transport_distance =
                    PositionImpedance_Abs(raw_velocity) * config->sample_period_s;
                integral_decay_ratio = PositionImpedance_Clamp(
                    integral_transport_distance /
                        POSITION_IMPEDANCE_INTEGRAL_DECAY_DISTANCE_RAD,
                    0.0f, POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_RATIO);
            }
            integral_decay_step = PositionImpedance_Clamp(
                context->integral * integral_decay_ratio,
                -POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A,
                 POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A);
            context->integral -= integral_decay_step;
            if (PositionImpedance_Abs(context->integral) <=
                POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A)
            {
                context->integral = 0.0f;
                context->integral_transport_active = false;
            }
        }

		if (!context->target_reached &&
			PositionImpedance_Abs(context->speed_reference) <=
                POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
            PositionImpedance_Abs(context->velocity_filtered) <=
                POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
            PositionImpedance_Abs(position_error) > config->position_error_window &&
            PositionImpedance_Abs(position_error) <=
                POSITION_IMPEDANCE_INTEGRAL_ZONE_RAD)
        {
            integral_candidate = PositionImpedance_Clamp(context->integral +
                config->ki * position_error * config->sample_period_s,
                -integral_limit, integral_limit);
            current_candidate = proportional_current + damping_current +
				integral_candidate + context->friction_current;
            if ((current_candidate >= -config->output_limit &&
                 current_candidate <= config->output_limit) ||
                (current_candidate > config->output_limit && position_error < 0.0f) ||
                (current_candidate < -config->output_limit && position_error > 0.0f))
                context->integral = integral_candidate;
        }
    }

    current_candidate = proportional_current + damping_current + context->integral +
		context->friction_current;
    if (!isfinite(current_candidate))
        return false;
    context->iq_reference = PositionImpedance_Clamp(current_candidate,
        -config->output_limit, config->output_limit);
    PositionImpedance_CopyOutput(context, output);
    return true;
}
