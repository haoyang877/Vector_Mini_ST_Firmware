#include "product_config.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define PRODUCT_CONFIG_PI                  3.14159265358979323846f
#define PRODUCT_CONFIG_TWO_PI              6.28318530717958647692f
#define PRODUCT_CONFIG_MAX_RESIDUAL_Q15     32768U

enum
{
	PRODUCT_FEATURE_DETAIL_SPEED = 1U,
	PRODUCT_FEATURE_DETAIL_POSITION,
	PRODUCT_FEATURE_DETAIL_OUTPUT_POSITION,
	PRODUCT_FEATURE_DETAIL_SENSORLESS,
	PRODUCT_FEATURE_DETAIL_ANGLE_REDUNDANCY,
	PRODUCT_FEATURE_DETAIL_TEMPERATURE_MONITORING,
	PRODUCT_FEATURE_DETAIL_TEMPERATURE_PROTECTION
};

static bool ProductConfig_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static bool ProductConfig_IsPositiveFinite(float value)
{
	return ProductConfig_IsFinite(value) && value > 0.0f;
}

#if defined(PRODUCT_CATALOG_INCLUDE_ALL)
static bool ProductConfig_IsNonnegativeFinite(float value)
{
	return ProductConfig_IsFinite(value) && value >= 0.0f;
}

static bool ProductConfig_IsPositiveUnitRatio(float value)
{
	return ProductConfig_IsPositiveFinite(value) && value <= 1.0f;
}

static bool ProductConfig_LoopFrequencyIsValid(uint32_t board_frequency_hz,
	uint32_t loop_frequency_hz)
{
	return board_frequency_hz > 0U && loop_frequency_hz > 0U &&
		loop_frequency_hz <= board_frequency_hz &&
		board_frequency_hz % loop_frequency_hz == 0U &&
		board_frequency_hz / loop_frequency_hz <= UINT16_MAX;
}

static bool ProductConfig_SecondsFitTicks(float seconds,
	uint32_t control_frequency_hz, bool allow_zero)
{
	if (!ProductConfig_IsFinite(seconds) ||
		(allow_zero ? seconds < 0.0f : seconds <= 0.0f) ||
		control_frequency_hz == 0U)
		return false;
	return seconds <= (float)UINT32_MAX / (float)control_frequency_hz;
}

static bool ProductConfig_MillisecondsFitTicks(uint32_t milliseconds,
	uint32_t control_frequency_hz)
{
	uint64_t numerator;

	if (milliseconds == 0U || control_frequency_hz == 0U)
		return false;
	numerator = (uint64_t)milliseconds * (uint64_t)control_frequency_hz;
	return numerator <= UINT32_MAX && numerator >= UINT64_C(1000);
}

static float ProductConfig_CommandCurrentLimit(const ProductConfig *config)
{
	float limit = FLT_MAX;

	if (config->motor != NULL &&
		ProductConfig_IsPositiveFinite(config->motor->current_limit_a) &&
		config->motor->current_limit_a < limit)
		limit = config->motor->current_limit_a;
	if (config->board != NULL && ProductConfig_IsPositiveFinite(
			config->board->command_phase_current_limit_a) &&
		config->board->command_phase_current_limit_a < limit)
		limit = config->board->command_phase_current_limit_a;
	return limit;
}

static float ProductConfig_CalibrationCurrentLimit(const ProductConfig *config)
{
	float limit = FLT_MAX;

	if (config->motor != NULL && ProductConfig_IsPositiveFinite(
			config->motor->calibration_current_a) &&
		config->motor->calibration_current_a < limit)
		limit = config->motor->calibration_current_a;
	if (config->board != NULL && ProductConfig_IsPositiveFinite(
			config->board->calibration_phase_current_limit_a) &&
		config->board->calibration_phase_current_limit_a < limit)
		limit = config->board->calibration_phase_current_limit_a;
	return limit;
}

static bool ProductConfig_MechanicalSpeedFitsMotor(
	const ProductConfig *config, float speed_rad_s)
{
	return config->motor == NULL ||
		!ProductConfig_IsPositiveFinite(config->motor->speed_limit_rad_s) ||
		speed_rad_s <= config->motor->speed_limit_rad_s;
}

static bool ProductConfig_MechanicalSpeedFitsLoad(
	const ProductConfig *config, float motor_speed_rad_s)
{
	return config->load == NULL ||
		!ProductConfig_IsPositiveFinite(config->load->transmission_ratio) ||
		!ProductConfig_IsPositiveFinite(
			config->load->maximum_output_speed_rad_s) ||
		motor_speed_rad_s / config->load->transmission_ratio <=
			config->load->maximum_output_speed_rad_s;
}

static bool ProductConfig_ElectricalSpeedFitsMotor(
	const ProductConfig *config, float electrical_speed_rad_s)
{
	return config->motor == NULL || config->motor->pole_pairs == 0U ||
		!ProductConfig_IsPositiveFinite(config->motor->speed_limit_rad_s) ||
		electrical_speed_rad_s / (float)config->motor->pole_pairs <=
			config->motor->speed_limit_rad_s;
}

static bool ProductConfig_StartupCurrentsFit(float current_limit_a,
	const ProductSensorlessStartupConfig *startup)
{
	float iq_ratio;
	float id_ratio;

	if (current_limit_a == FLT_MAX)
		return true;
	if (!ProductConfig_IsPositiveFinite(current_limit_a) ||
		startup->align_current_a > current_limit_a ||
		startup->startup_iq_initial_a > current_limit_a ||
		startup->startup_iq_a > current_limit_a ||
		startup->startup_id_a > current_limit_a ||
		startup->minimum_current_limit_a > current_limit_a)
		return false;
	iq_ratio = startup->startup_iq_a / current_limit_a;
	id_ratio = startup->startup_id_a / current_limit_a;
	return iq_ratio * iq_ratio + id_ratio * id_ratio <= 1.0f;
}

static bool ProductConfig_StartupIsValid(
	const ProductSensorlessStartupConfig *startup,
	uint32_t control_frequency_hz)
{
	return ProductConfig_SecondsFitTicks(startup->align_current_ramp_time_s,
			control_frequency_hz, false) &&
		ProductConfig_SecondsFitTicks(startup->align_hold_time_s,
			control_frequency_hz, true) &&
		ProductConfig_IsPositiveFinite(startup->align_current_a) &&
		ProductConfig_IsNonnegativeFinite(startup->startup_iq_initial_a) &&
		ProductConfig_IsFinite(startup->startup_iq_a) &&
		startup->startup_iq_a >= startup->startup_iq_initial_a &&
		ProductConfig_SecondsFitTicks(startup->startup_iq_ramp_time_s,
			control_frequency_hz, false) &&
		ProductConfig_IsNonnegativeFinite(startup->startup_id_a) &&
		ProductConfig_IsNonnegativeFinite(startup->minimum_current_limit_a) &&
		ProductConfig_IsPositiveFinite(
			startup->minimum_electrical_velocity_rad_s) &&
		ProductConfig_IsFinite(startup->target_electrical_velocity_rad_s) &&
		startup->target_electrical_velocity_rad_s >=
			startup->minimum_electrical_velocity_rad_s &&
		ProductConfig_SecondsFitTicks(startup->startup_ramp_time_s,
			control_frequency_hz, false) &&
		ProductConfig_SecondsFitTicks(startup->speed_lock_time_s,
			control_frequency_hz, false) &&
		ProductConfig_IsPositiveUnitRatio(startup->speed_lock_filter_alpha) &&
		ProductConfig_IsPositiveUnitRatio(startup->observer_lock_ratio) &&
		ProductConfig_SecondsFitTicks(startup->angle_handoff_time_s,
			control_frequency_hz, false) &&
		ProductConfig_SecondsFitTicks(startup->lock_timeout_s,
			control_frequency_hz, false) &&
		startup->lock_timeout_s >= startup->speed_lock_time_s &&
		ProductConfig_SecondsFitTicks(startup->id_ramp_down_time_s,
			control_frequency_hz, false) &&
		ProductConfig_SecondsFitTicks(startup->observer_loss_time_s,
			control_frequency_hz, false);
}

static bool ProductConfig_PositionFrictionIsValid(
	const ProductPositionFrictionControlConfig *friction,
	float current_limit_a)
{
	bool currents_fit = current_limit_a == FLT_MAX ||
		(friction->friction_positive_current_a <= current_limit_a &&
		 friction->friction_negative_current_a <= current_limit_a &&
		 friction->breakaway_positive_current_a <= current_limit_a &&
		 friction->breakaway_negative_current_a <= current_limit_a);

	return ProductConfig_IsNonnegativeFinite(
			friction->friction_positive_current_a) &&
		ProductConfig_IsNonnegativeFinite(
			friction->friction_negative_current_a) &&
		ProductConfig_IsNonnegativeFinite(
			friction->breakaway_positive_current_a) &&
		ProductConfig_IsNonnegativeFinite(
			friction->breakaway_negative_current_a) &&
		friction->breakaway_positive_current_a >=
			friction->friction_positive_current_a &&
		friction->breakaway_negative_current_a >=
			friction->friction_negative_current_a && currents_fit &&
		ProductConfig_IsPositiveFinite(friction->current_slew_rate_a_per_s) &&
		ProductConfig_IsPositiveFinite(friction->position_enter_rad) &&
		ProductConfig_IsFinite(friction->position_exit_rad) &&
		friction->position_exit_rad > friction->position_enter_rad &&
		ProductConfig_IsPositiveFinite(friction->reference_speed_rad_s) &&
		ProductConfig_IsPositiveFinite(friction->stop_speed_rad_s) &&
		ProductConfig_IsFinite(friction->move_speed_rad_s) &&
		friction->reference_speed_rad_s >= friction->stop_speed_rad_s &&
		friction->move_speed_rad_s >= friction->reference_speed_rad_s &&
		ProductConfig_IsPositiveFinite(friction->stuck_time_s) &&
		ProductConfig_IsFinite(friction->landing_position_rad) &&
		friction->landing_position_rad >= friction->position_exit_rad &&
		ProductConfig_IsFinite(friction->landing_speed_rad_s) &&
		friction->landing_speed_rad_s >= friction->move_speed_rad_s &&
		ProductConfig_IsPositiveFinite(friction->recovery_delay_s) &&
		ProductConfig_IsPositiveFinite(friction->recovery_pulse_time_s) &&
		ProductConfig_IsPositiveFinite(friction->recovery_cooldown_s);
}
#endif

static uint8_t ProductConfig_BoundedAngleSensorCount(
	const ProductConfig *config)
{
	return config->angle_sensor_count <= PRODUCT_CONFIG_MAX_ANGLE_SENSORS ?
		config->angle_sensor_count : PRODUCT_CONFIG_MAX_ANGLE_SENSORS;
}

static uint8_t ProductConfig_BoundedTemperatureSensorCount(
	const ProductConfig *config)
{
	return config->temperature_sensor_count <=
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS ?
		config->temperature_sensor_count :
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS;
}

static bool ProductConfig_IsMotorRotorRole(ProductAngleSensorRole role)
{
	return role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY ||
		role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT;
}

static uint8_t ProductConfig_CurrentSenseChannelCount(
	ProductCurrentSenseTopology topology)
{
	switch (topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			return 3U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			return 2U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			return 1U;
		default:
			return 0U;
	}
}

static bool ProductConfig_CurrentSenseUsesShunt(
	ProductCurrentSenseTopology topology)
{
	return topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT ||
		topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT ||
		topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT;
}

static bool ProductConfig_CurrentRoleIsPhase(ProductCurrentChannelRole role)
{
	return role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A ||
		role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B ||
		role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C;
}

static bool ProductConfig_CurrentRoleMatchesTopology(
	ProductCurrentSenseTopology topology, ProductCurrentChannelRole role)
{
	switch (topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			return ProductConfig_CurrentRoleIsPhase(role);
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			return role == PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK;
		default:
			return false;
	}
}

static bool ProductConfig_CurrentPolarityIsValid(
	ProductCurrentChannelPolarity polarity)
{
	return polarity == PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL ||
		polarity == PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED;
}

static bool ProductConfig_CurrentUnusedChannelIsZero(
	const ProductCurrentSenseConfig *current_sense, uint8_t channel)
{
	return current_sense->channel_roles[channel] ==
			PRODUCT_CURRENT_CHANNEL_ROLE_INVALID &&
		current_sense->channel_polarities[channel] ==
			PRODUCT_CURRENT_CHANNEL_POLARITY_INVALID &&
		current_sense->channel_endpoints[channel] ==
			PRODUCT_CONFIG_ENDPOINT_NONE &&
		current_sense->current_a_per_count[channel] == 0.0f &&
		current_sense->default_offset_count[channel] == 0U &&
		current_sense->minimum_valid_offset_count[channel] == 0U &&
		current_sense->maximum_valid_offset_count[channel] == 0U;
}

static bool ProductConfig_CurrentSenseIsStructurallyReady(
	const ProductCurrentSenseConfig *current_sense)
{
	uint8_t channel;
	uint8_t other;
	uint8_t required_channel_count;
	uint8_t role_mask = 0U;

	if (current_sense == NULL)
		return false;
	required_channel_count = ProductConfig_CurrentSenseChannelCount(
		current_sense->topology);
	if (required_channel_count == 0U)
		return false;
	if (current_sense->physical_channel_count != required_channel_count ||
		!current_sense->pwm_synchronized ||
		current_sense->samples_per_pwm_period == 0U)
		return false;

	for (channel = 0U; channel < required_channel_count; channel++)
	{
		if (current_sense->channel_endpoints[channel] ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			!ProductConfig_CurrentRoleMatchesTopology(current_sense->topology,
				current_sense->channel_roles[channel]) ||
			!ProductConfig_CurrentPolarityIsValid(
				current_sense->channel_polarities[channel]) ||
			!ProductConfig_IsPositiveFinite(
				current_sense->current_a_per_count[channel]))
			return false;
		role_mask |= (uint8_t)(1U <<
			(uint8_t)current_sense->channel_roles[channel]);
		for (other = (uint8_t)(channel + 1U);
			other < required_channel_count; other++)
		{
			if (current_sense->channel_endpoints[channel] ==
					current_sense->channel_endpoints[other] ||
				current_sense->channel_roles[channel] ==
					current_sense->channel_roles[other])
				return false;
		}
	}
	for (channel = required_channel_count;
		channel < PRODUCT_CONFIG_MAX_CURRENT_CHANNELS; channel++)
	{
		if (!ProductConfig_CurrentUnusedChannelIsZero(current_sense, channel))
			return false;
	}
	if ((current_sense->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT ||
		 current_sense->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT) &&
		role_mask != (uint8_t)((1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A) |
			(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B) |
			(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C)))
		return false;

	if (current_sense->topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		return current_sense->samples_per_pwm_period >= 2U &&
			current_sense->captures_pwm_sector &&
			current_sense->supports_sample_window_compensation;
	}
	return true;
}

static bool ProductConfig_MotorSupportsObserver(
	const ProductMotorDesign *motor)
{
	return motor != NULL && motor->pole_pairs > 0U &&
		ProductConfig_IsPositiveFinite(motor->phase_resistance_ohm) &&
		ProductConfig_IsPositiveFinite(motor->d_axis_inductance_h) &&
		ProductConfig_IsPositiveFinite(motor->q_axis_inductance_h) &&
		ProductConfig_IsPositiveFinite(motor->flux_weber);
}

static bool ProductConfig_SensorlessIsStructurallyReady(
	const ProductConfig *config)
{
	return config != NULL && config->board != NULL &&
		config->board->bus_voltage_measurement_available &&
		ProductConfig_IsPositiveFinite(config->board->bus_voltage_v_per_count) &&
		ProductConfig_CurrentSenseIsStructurallyReady(
			&config->board->current_sense) &&
		ProductConfig_MotorSupportsObserver(config->motor);
}

static void ProductConfig_AddIssue(ProductConfigValidationResult *result,
	ProductConfigErrorCode code, ProductConfigValidationSubject subject,
	uint8_t instance_index, uint32_t detail)
{
	ProductConfigValidationIssue *issue;

	if (result->stored_error_count < PRODUCT_CONFIG_MAX_VALIDATION_ERRORS)
	{
		issue = &result->issues[result->stored_error_count++];
		issue->code = code;
		issue->subject = subject;
		issue->instance_index = instance_index;
		issue->detail = detail;
	}
	else
	{
		result->truncated = true;
	}
	if (result->total_error_count < UINT16_MAX)
		result->total_error_count++;
}

#if defined(PRODUCT_CATALOG_INCLUDE_ALL)
static void ProductConfig_ValidateMotorAcceptance(const ProductConfig *config,
	ProductConfigValidationResult *result)
{
	const ProductMotorAcceptanceConfig *acceptance =
		&config->motor_acceptance;
	bool ranges_valid =
		ProductConfig_IsPositiveFinite(
			acceptance->phase_resistance_min_ohm) &&
		ProductConfig_IsFinite(acceptance->phase_resistance_max_ohm) &&
		acceptance->phase_resistance_max_ohm >
			acceptance->phase_resistance_min_ohm &&
		ProductConfig_IsPositiveFinite(acceptance->inductance_min_h) &&
		ProductConfig_IsFinite(acceptance->inductance_max_h) &&
		acceptance->inductance_max_h > acceptance->inductance_min_h &&
		ProductConfig_IsPositiveFinite(acceptance->flux_min_weber) &&
		ProductConfig_IsFinite(acceptance->flux_max_weber) &&
		acceptance->flux_max_weber > acceptance->flux_min_weber;

	if (!ranges_valid)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_MOTOR_ACCEPTANCE_RANGE_INVALID,
			PRODUCT_CONFIG_SUBJECT_MOTOR_ACCEPTANCE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		return;
	}
	if (config->motor != NULL &&
		(config->motor->phase_resistance_ohm <
			acceptance->phase_resistance_min_ohm ||
		 config->motor->phase_resistance_ohm >
			acceptance->phase_resistance_max_ohm ||
		 config->motor->d_axis_inductance_h < acceptance->inductance_min_h ||
		 config->motor->d_axis_inductance_h > acceptance->inductance_max_h ||
		 config->motor->q_axis_inductance_h < acceptance->inductance_min_h ||
		 config->motor->q_axis_inductance_h > acceptance->inductance_max_h ||
		 config->motor->flux_weber < acceptance->flux_min_weber ||
		 config->motor->flux_weber > acceptance->flux_max_weber))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_MOTOR_OUTSIDE_ACCEPTANCE,
			PRODUCT_CONFIG_SUBJECT_MOTOR_ACCEPTANCE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
}

static void ProductConfig_ValidateControl(const ProductConfig *config,
	ProductConfigValidationResult *result)
{
	const ProductControlConfig *control = &config->control;
	const ProductControlParameterLimits *limits = &control->parameter_limits;
	const ProductSensorlessControlConfig *sensorless = &control->sensorless;
	uint32_t board_frequency_hz = config->board != NULL ?
		config->board->control_frequency_hz : 0U;
	float current_limit_a = ProductConfig_CommandCurrentLimit(config);
	bool frequency_valid = ProductConfig_LoopFrequencyIsValid(
		board_frequency_hz, control->speed_loop_frequency_hz) &&
		ProductConfig_LoopFrequencyIsValid(board_frequency_hz,
			control->position_loop_frequency_hz) &&
		ProductConfig_LoopFrequencyIsValid(board_frequency_hz,
			control->cascade_position_loop_frequency_hz);
	bool parameters_valid =
		ProductConfig_IsPositiveFinite(control->current_loop_bandwidth_rad_s) &&
		ProductConfig_IsPositiveFinite(control->open_loop_voltage_v) &&
		ProductConfig_IsPositiveFinite(
			control->open_loop_electrical_velocity_rad_s) &&
		ProductConfig_IsFinite(control->open_loop_initial_theta_rad) &&
		ProductConfig_IsPositiveFinite(control->default_speed_limit_rad_s) &&
		ProductConfig_IsPositiveFinite(control->speed_acceleration_rad_s2) &&
		ProductConfig_IsPositiveFinite(control->speed_deceleration_rad_s2) &&
		ProductConfig_IsPositiveFinite(control->speed_kp) &&
		ProductConfig_IsNonnegativeFinite(control->speed_ki) &&
		ProductConfig_IsPositiveFinite(control->position_acceleration_rad_s2) &&
		ProductConfig_IsPositiveFinite(control->position_deceleration_rad_s2) &&
		ProductConfig_IsPositiveFinite(
			control->default_position_max_speed_rad_s) &&
		ProductConfig_IsPositiveFinite(control->position_kp_a_per_rad) &&
		ProductConfig_IsNonnegativeFinite(control->position_kd_a_per_rad_s) &&
		ProductConfig_IsNonnegativeFinite(control->position_ki_a_per_rad_s) &&
		ProductConfig_IsNonnegativeFinite(control->position_integral_limit_a) &&
		ProductConfig_IsNonnegativeFinite(control->position_error_window_rad) &&
		ProductConfig_IsPositiveFinite(control->cascade_position_kp_per_s) &&
		ProductConfig_IsNonnegativeFinite(control->cascade_position_kd);
	bool limits_valid =
		ProductConfig_IsPositiveFinite(limits->speed_limit_max_rad_s) &&
		ProductConfig_IsPositiveFinite(limits->speed_ramp_max_rad_s2) &&
		ProductConfig_IsPositiveFinite(limits->position_ramp_max_rad_s2) &&
		ProductConfig_IsPositiveFinite(limits->position_speed_limit_rad_s) &&
		ProductConfig_IsPositiveFinite(limits->position_kp_limit_a_per_rad) &&
		ProductConfig_IsPositiveFinite(limits->position_kd_limit_a_per_rad_s) &&
		ProductConfig_IsPositiveFinite(limits->position_ki_limit_a_per_rad_s) &&
		ProductConfig_IsPositiveFinite(
			limits->cascade_position_kp_limit_per_s) &&
		ProductConfig_IsPositiveFinite(limits->cascade_position_kd_limit);
	bool defaults_fit = parameters_valid && limits_valid &&
		control->default_speed_limit_rad_s <= limits->speed_limit_max_rad_s &&
		control->speed_acceleration_rad_s2 <= limits->speed_ramp_max_rad_s2 &&
		control->speed_deceleration_rad_s2 <= limits->speed_ramp_max_rad_s2 &&
		control->position_acceleration_rad_s2 <=
			limits->position_ramp_max_rad_s2 &&
		control->position_deceleration_rad_s2 <=
			limits->position_ramp_max_rad_s2 &&
		control->default_position_max_speed_rad_s <=
			limits->position_speed_limit_rad_s &&
		control->position_kp_a_per_rad <=
			limits->position_kp_limit_a_per_rad &&
		control->position_kd_a_per_rad_s <=
			limits->position_kd_limit_a_per_rad_s &&
		control->position_ki_a_per_rad_s <=
			limits->position_ki_limit_a_per_rad_s &&
		control->cascade_position_kp_per_s <=
			limits->cascade_position_kp_limit_per_s &&
		control->cascade_position_kd <= limits->cascade_position_kd_limit &&
		(current_limit_a == FLT_MAX ||
		 control->position_integral_limit_a <= current_limit_a) &&
		ProductConfig_MechanicalSpeedFitsMotor(config,
			control->default_speed_limit_rad_s) &&
		ProductConfig_MechanicalSpeedFitsMotor(config,
			control->default_position_max_speed_rad_s) &&
		ProductConfig_MechanicalSpeedFitsLoad(config,
			control->default_speed_limit_rad_s) &&
		ProductConfig_MechanicalSpeedFitsLoad(config,
			control->default_position_max_speed_rad_s) &&
		ProductConfig_ElectricalSpeedFitsMotor(config,
			control->open_loop_electrical_velocity_rad_s) &&
		(!ProductConfig_IsPositiveFinite(config->safety.overvoltage_trip_v) ||
		 control->open_loop_voltage_v <= config->safety.overvoltage_trip_v);
	bool sensorless_valid = ProductConfig_StartupIsValid(&sensorless->startup,
			board_frequency_hz) &&
		ProductConfig_StartupCurrentsFit(current_limit_a,
			&sensorless->startup) &&
		ProductConfig_ElectricalSpeedFitsMotor(config,
			sensorless->startup.target_electrical_velocity_rad_s) &&
		ProductConfig_IsPositiveFinite(
			sensorless->observer_max_electrical_velocity_rad_s) &&
		sensorless->observer_max_electrical_velocity_rad_s >=
			sensorless->startup.target_electrical_velocity_rad_s &&
		ProductConfig_IsPositiveUnitRatio(sensorless->speed_feedback_lpf_alpha) &&
		ProductConfig_IsPositiveFinite(sensorless->flux_observer_gamma) &&
		ProductConfig_IsPositiveFinite(
			sensorless->flux_observer_resistance_scale) &&
		ProductConfig_IsPositiveFinite(
			sensorless->flux_observer_max_correction_step_rad) &&
		sensorless->flux_observer_max_correction_step_rad <= PRODUCT_CONFIG_PI &&
		ProductConfig_IsPositiveFinite(
			sensorless->flux_observer_minimum_flux_weber) &&
		(config->motor == NULL ||
		 !ProductConfig_IsPositiveFinite(config->motor->flux_weber) ||
		 sensorless->flux_observer_minimum_flux_weber <
			config->motor->flux_weber) &&
		ProductConfig_IsPositiveUnitRatio(
			sensorless->flux_observer_velocity_lpf_alpha) &&
		ProductConfig_IsFinite(
			sensorless->flux_observer_angle_wrap_threshold_rad) &&
		sensorless->flux_observer_angle_wrap_threshold_rad >=
			PRODUCT_CONFIG_PI &&
		sensorless->flux_observer_angle_wrap_threshold_rad <=
			PRODUCT_CONFIG_TWO_PI;

	if (!frequency_valid)
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			control->speed_loop_frequency_hz);
	if (!parameters_valid)
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CONTROL_PARAMETER_INVALID,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!limits_valid)
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CONTROL_LIMIT_INVALID,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!defaults_fit)
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CONTROL_DEFAULT_EXCEEDS_LIMIT,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!sensorless_valid)
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_SENSORLESS_CONTROL_INVALID,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!ProductConfig_PositionFrictionIsValid(&control->position_friction,
		current_limit_a))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_POSITION_FRICTION_CONTROL_INVALID,
			PRODUCT_CONFIG_SUBJECT_CONTROL,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
}

static bool ProductConfig_PhaseResistanceTuningIsValid(
	const ProductConfig *config)
{
	const ProductPhaseResistanceCommissioningConfig *tuning =
		&config->commissioning_tuning.phase_resistance;
	uint32_t frequency_hz = config->board != NULL ?
		config->board->control_frequency_hz : 0U;

	return ProductConfig_IsPositiveFinite(tuning->test_current_min_a) &&
		ProductConfig_IsFinite(tuning->test_current_low_a) &&
		tuning->test_current_low_a >= tuning->test_current_min_a &&
		ProductConfig_IsFinite(tuning->test_current_high_a) &&
		tuning->test_current_high_a > tuning->test_current_low_a &&
		ProductConfig_IsFinite(tuning->test_current_max_a) &&
		tuning->test_current_max_a >= tuning->test_current_high_a &&
		ProductConfig_IsNonnegativeFinite(tuning->current_tolerance_a) &&
		tuning->current_tolerance_a <= tuning->test_current_low_a &&
		ProductConfig_IsNonnegativeFinite(tuning->q_current_tolerance_a) &&
		tuning->q_current_tolerance_a <= tuning->test_current_low_a &&
		ProductConfig_IsNonnegativeFinite(tuning->voltage_tolerance_v) &&
		ProductConfig_IsNonnegativeFinite(tuning->voltage_min_delta_v) &&
		tuning->voltage_min_delta_v <= tuning->voltage_tolerance_v &&
		ProductConfig_IsPositiveUnitRatio(tuning->voltage_filter_alpha) &&
		ProductConfig_MillisecondsFitTicks(tuning->ramp_time_ms,
			frequency_hz) &&
		ProductConfig_MillisecondsFitTicks(tuning->settle_time_ms,
			frequency_hz) &&
		ProductConfig_MillisecondsFitTicks(tuning->sample_time_ms,
			frequency_hz) &&
		ProductConfig_MillisecondsFitTicks(tuning->pause_time_ms,
			frequency_hz) &&
		ProductConfig_MillisecondsFitTicks(tuning->timeout_ms,
			frequency_hz) &&
		tuning->timeout_ms >= tuning->settle_time_ms &&
		ProductConfig_IsNonnegativeFinite(tuning->balance_warning_pct) &&
		ProductConfig_IsFinite(tuning->balance_fault_pct) &&
		tuning->balance_fault_pct >= tuning->balance_warning_pct &&
		tuning->balance_fault_pct <= 100.0f &&
		ProductConfig_IsNonnegativeFinite(tuning->design_tolerance_pct) &&
		tuning->design_tolerance_pct <= 100.0f;
}

static bool ProductConfig_AngleTuningIsValid(const ProductConfig *config)
{
	const ProductAngleCommissioningConfig *angle =
		&config->commissioning_tuning.angle;
	uint32_t frequency_hz = config->board != NULL ?
		config->board->control_frequency_hz : 0U;
	float required_startup_time_s;
	float required_sample_time_s;
	float required_verify_time_s;

	if (!ProductConfig_StartupIsValid(&angle->startup, frequency_hz) ||
		!ProductConfig_ElectricalSpeedFitsMotor(config,
			angle->startup.target_electrical_velocity_rad_s) ||
		!ProductConfig_SecondsFitTicks(angle->linearization_align_time_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(angle->linearization_ramp_time_s,
			frequency_hz, false) ||
		!ProductConfig_IsPositiveFinite(
			angle->linearization_speed_electrical_rad_s) ||
		!ProductConfig_ElectricalSpeedFitsMotor(config,
			angle->linearization_speed_electrical_rad_s) ||
		!ProductConfig_IsFinite(angle->linearization_timeout_factor) ||
		angle->linearization_timeout_factor < 1.0f ||
		!ProductConfig_SecondsFitTicks(angle->linearization_unlock_timeout_s,
			frequency_hz, false) ||
		!ProductConfig_IsPositiveFinite(
			angle->calibration_speed_mechanical_rad_s) ||
		!ProductConfig_MechanicalSpeedFitsMotor(config,
			angle->calibration_speed_mechanical_rad_s) ||
		!ProductConfig_IsPositiveUnitRatio(
			angle->calibration_speed_error_ratio) ||
		!ProductConfig_SecondsFitTicks(
			angle->calibration_speed_stable_time_s, frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(
			angle->calibration_speed_stable_timeout_s, frequency_hz, false) ||
		angle->calibration_speed_stable_timeout_s <
			angle->calibration_speed_stable_time_s ||
		!ProductConfig_SecondsFitTicks(angle->calibration_align_sample_time_s,
			frequency_hz, false) ||
		angle->calibration_mechanical_turns == 0U ||
		angle->calibration_verify_mechanical_turns == 0U ||
		angle->calibration_verify_mechanical_turns >
			angle->calibration_mechanical_turns ||
		angle->calibration_min_samples_per_bin == 0U ||
		angle->calibration_lut_build_bins_per_cycle == 0U ||
		!ProductConfig_SecondsFitTicks(angle->calibration_find_origin_timeout_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(angle->calibration_sample_timeout_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(angle->calibration_verify_timeout_s,
			frequency_hz, false) ||
		angle->calibration_max_rms_residual_q15 == 0U ||
		angle->calibration_max_peak_residual_q15 <
			angle->calibration_max_rms_residual_q15 ||
		angle->calibration_max_peak_residual_q15 >
			PRODUCT_CONFIG_MAX_RESIDUAL_Q15 ||
		!ProductConfig_SecondsFitTicks(angle->calibration_startup_timeout_s,
			frequency_hz, false) ||
		!ProductConfig_IsFinite(angle->calibration_stop_speed_margin) ||
		angle->calibration_stop_speed_margin < 1.0f ||
		!ProductConfig_SecondsFitTicks(
			angle->calibration_stop_deceleration_time_s, frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(
			angle->calibration_stop_deceleration_timeout_s,
			frequency_hz, false) ||
		angle->calibration_stop_deceleration_timeout_s <
			angle->calibration_stop_deceleration_time_s ||
		!ProductConfig_IsPositiveUnitRatio(
			angle->calibration_stop_speed_tolerance_ratio) ||
		!ProductConfig_SecondsFitTicks(
			angle->calibration_stop_current_ramp_time_s, frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(
			angle->electrical_zero_current_ramp_time_s, frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(angle->electrical_zero_hold_time_s,
			frequency_hz, false) ||
		!ProductConfig_IsNonnegativeFinite(
			angle->electrical_zero_min_align_current_a) ||
		!ProductConfig_SecondsFitTicks(angle->direction_align_time_s,
			frequency_hz, false) ||
		!ProductConfig_IsPositiveFinite(
			angle->direction_speed_electrical_rad_s) ||
		!ProductConfig_ElectricalSpeedFitsMotor(config,
			angle->direction_speed_electrical_rad_s))
		return false;

	required_startup_time_s = angle->startup.align_current_ramp_time_s +
		angle->startup.align_hold_time_s + angle->startup.startup_ramp_time_s +
		angle->startup.lock_timeout_s + angle->startup.angle_handoff_time_s;
	required_sample_time_s =
		(float)angle->calibration_mechanical_turns * PRODUCT_CONFIG_TWO_PI /
		angle->calibration_speed_mechanical_rad_s;
	required_verify_time_s =
		(float)angle->calibration_verify_mechanical_turns *
		PRODUCT_CONFIG_TWO_PI / angle->calibration_speed_mechanical_rad_s;
	return ProductConfig_IsFinite(required_startup_time_s) &&
		angle->calibration_startup_timeout_s >= required_startup_time_s &&
		ProductConfig_IsFinite(required_sample_time_s) &&
		angle->calibration_sample_timeout_s >= required_sample_time_s &&
		ProductConfig_IsFinite(required_verify_time_s) &&
		angle->calibration_verify_timeout_s >= required_verify_time_s;
}

static bool ProductConfig_FrictionTuningIsValid(const ProductConfig *config)
{
	const ProductFrictionIdentificationConfig *friction =
		&config->commissioning_tuning.friction;
	uint32_t frequency_hz = config->board != NULL ?
		config->board->control_frequency_hz : 0U;
	uint32_t index;

	if (friction->speed_point_count < 2U ||
		friction->speed_point_count >
			PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT ||
		!ProductConfig_SecondsFitTicks(friction->stable_time_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(friction->track_timeout_s,
			frequency_hz, false) ||
		friction->track_timeout_s <= friction->stable_time_s ||
		!ProductConfig_SecondsFitTicks(friction->sample_timeout_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(friction->stop_hold_time_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(friction->stop_timeout_s,
			frequency_hz, false) ||
		friction->stop_timeout_s <= friction->stop_hold_time_s ||
		!ProductConfig_IsPositiveUnitRatio(friction->speed_tolerance_ratio) ||
		!ProductConfig_IsPositiveFinite(
			friction->minimum_speed_tolerance_rad_s) ||
		!ProductConfig_IsPositiveFinite(friction->stop_speed_rad_s) ||
		!ProductConfig_IsPositiveFinite(friction->sample_turns) ||
		!ProductConfig_SecondsFitTicks(friction->minimum_sample_time_s,
			frequency_hz, false) ||
		friction->sample_timeout_s < friction->minimum_sample_time_s ||
		!ProductConfig_IsPositiveUnitRatio(friction->current_ratio_max) ||
		!ProductConfig_SecondsFitTicks(friction->saturation_time_s,
			frequency_hz, false) ||
		!ProductConfig_IsNonnegativeFinite(friction->rmse_floor_a) ||
		!ProductConfig_IsPositiveUnitRatio(friction->rmse_ratio_max))
		return false;

	for (index = 0U; index < friction->speed_point_count; index++)
	{
		if (!ProductConfig_IsPositiveFinite(
				friction->speed_points_rad_s[index]) ||
			(index > 0U && friction->speed_points_rad_s[index] <=
				friction->speed_points_rad_s[index - 1U]) ||
			!ProductConfig_MechanicalSpeedFitsMotor(config,
				friction->speed_points_rad_s[index]) ||
			!ProductConfig_MechanicalSpeedFitsLoad(config,
				friction->speed_points_rad_s[index]))
			return false;
	}
	return friction->stop_speed_rad_s < friction->speed_points_rad_s[0];
}

static bool ProductConfig_CoggingTuningIsValid(const ProductConfig *config)
{
	const ProductCoggingIdentificationConfig *cogging =
		&config->commissioning_tuning.cogging;
	uint32_t frequency_hz = config->board != NULL ?
		config->board->control_frequency_hz : 0U;
	float required_travel_time_s;

	if (!ProductConfig_IsPositiveFinite(cogging->speed_rad_s) ||
		!ProductConfig_MechanicalSpeedFitsMotor(config,
			cogging->speed_rad_s) ||
		!ProductConfig_MechanicalSpeedFitsLoad(config,
			cogging->speed_rad_s) || cogging->turns == 0U ||
		!ProductConfig_SecondsFitTicks(cogging->stable_time_s,
			frequency_hz, false) ||
		!ProductConfig_SecondsFitTicks(cogging->stage_timeout_s,
			frequency_hz, false) ||
		cogging->stage_timeout_s <= cogging->stable_time_s ||
		!ProductConfig_IsPositiveUnitRatio(cogging->speed_tolerance_ratio) ||
		cogging->minimum_samples_per_bin == 0U ||
		!ProductConfig_IsPositiveFinite(cogging->maximum_current_a))
		return false;
	required_travel_time_s = (float)cogging->turns * PRODUCT_CONFIG_TWO_PI /
		cogging->speed_rad_s;
	return ProductConfig_IsFinite(required_travel_time_s) &&
		cogging->stage_timeout_s >= required_travel_time_s;
}

static bool ProductConfig_CommissioningCurrentsFit(
	const ProductConfig *config)
{
	const ProductCommissioningTuningConfig *tuning =
		&config->commissioning_tuning;
	float command_limit_a = ProductConfig_CommandCurrentLimit(config);
	float calibration_limit_a = ProductConfig_CalibrationCurrentLimit(config);

	return (calibration_limit_a == FLT_MAX ||
		(tuning->phase_resistance.test_current_max_a <= calibration_limit_a &&
		 tuning->cogging.maximum_current_a <= calibration_limit_a)) &&
		ProductConfig_StartupCurrentsFit(command_limit_a,
			&tuning->angle.startup) &&
		(command_limit_a == FLT_MAX ||
		 tuning->angle.electrical_zero_min_align_current_a <= command_limit_a);
}

static void ProductConfig_ValidateCommissioningTuning(
	const ProductConfig *config, ProductConfigValidationResult *result)
{
	if (!ProductConfig_PhaseResistanceTuningIsValid(config))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_PHASE_RESISTANCE_TUNING_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!ProductConfig_AngleTuningIsValid(config))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!ProductConfig_FrictionTuningIsValid(config))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FRICTION_IDENTIFICATION_TUNING_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!ProductConfig_CoggingTuningIsValid(config))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COGGING_IDENTIFICATION_TUNING_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	if (!ProductConfig_CommissioningCurrentsFit(config))
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMISSIONING_EXCEEDS_LIMIT,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
}
#endif

static bool ProductConfig_FeatureRequirementIsValid(
	ProductFeatureRequirement requirement)
{
	return requirement <= PRODUCT_FEATURE_REQUIRED;
}

static bool ProductConfig_CommissioningRequirementIsValid(
	ProductCommissioningRequirement requirement)
{
	return requirement <= PRODUCT_COMMISSIONING_REQUIRED;
}

static void ProductConfig_ValidateFeaturePolicyValue(
	ProductConfigValidationResult *result,
	ProductFeatureRequirement requirement, uint32_t feature_detail)
{
	if (!ProductConfig_FeatureRequirementIsValid(requirement))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEATURE_POLICY_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, feature_detail);
	}
}

static void ProductConfig_ValidateCommissioningPolicyValue(
	ProductConfigValidationResult *result,
	ProductCommissioningRequirement requirement,
	ProductCommissioningStepMask step)
{
	if (!ProductConfig_CommissioningRequirementIsValid(requirement))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMISSIONING_POLICY_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, step);
	}
}

static void ProductConfig_RequireCommissioningCapability(
	ProductConfigValidationResult *result,
	ProductCommissioningRequirement requirement,
	ProductCapabilityMask capabilities,
	ProductCapabilityMask required_capability,
	ProductCommissioningStepMask step)
{
	if (requirement == PRODUCT_COMMISSIONING_REQUIRED &&
		(capabilities & required_capability) != required_capability)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMISSIONING_STEP_UNSUPPORTED,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, step);
	}
}

static void ProductConfig_ValidateFeedbackRoute(const ProductConfig *config,
	ProductConfigValidationResult *result, ProductFeedbackSourceRef source,
	ProductFeedbackSignal signal)
{
	const ProductAngleSensorInstanceConfig *sensor;
	uint32_t required_capability = 0U;
	bool role_matches = true;

	if (source.kind == PRODUCT_FEEDBACK_SOURCE_NONE)
		return;
	if (source.kind == PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER)
	{
		if (signal != PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE &&
			signal != PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY &&
			signal != PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		}
		else if (!ProductConfig_SensorlessIsStructurallyReady(config))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		}
		return;
	}
	if (source.kind != PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_SOURCE_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		return;
	}
	if (source.angle_sensor_index >=
		ProductConfig_BoundedAngleSensorCount(config))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_SENSOR_INDEX_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
		return;
	}
	sensor = &config->angle_sensors[source.angle_sensor_index];
	if (sensor->design == NULL)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
		return;
	}
	switch (signal)
	{
		case PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE:
			required_capability = PRODUCT_ANGLE_CAP_POSITION |
				PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY:
			required_capability = PRODUCT_ANGLE_CAP_VELOCITY;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = sensor->role ==
				PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
			break;
		case PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		default:
			role_matches = false;
			break;
	}
	if (!role_matches)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_ROLE_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
	}
	if (required_capability == 0U ||
		(sensor->design->capabilities & required_capability) !=
			required_capability)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
	}
}

static void ProductConfig_ValidateCurrentSense(
	const ProductCurrentSenseConfig *current_sense,
	ProductConfigValidationResult *result)
{
	uint8_t required_channel_count = 0U;
	uint8_t channel;
	uint8_t other;
	uint8_t role_mask = 0U;

	required_channel_count = ProductConfig_CurrentSenseChannelCount(
		current_sense->topology);
	if (required_channel_count == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_TOPOLOGY_INVALID,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			(uint32_t)current_sense->topology);
	}
	if (required_channel_count != 0U &&
		current_sense->physical_channel_count != required_channel_count)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_COUNT_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			(uint32_t)current_sense->physical_channel_count);
	}
	if (ProductConfig_CurrentSenseUsesShunt(current_sense->topology) &&
		current_sense->nominal_shunt_milliohm == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_SHUNT_INVALID,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	for (channel = 0U; channel < required_channel_count; channel++)
	{
		ProductCurrentChannelRole role =
			current_sense->channel_roles[channel];

		if (role <= PRODUCT_CURRENT_CHANNEL_ROLE_INVALID ||
			role > PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
				(uint32_t)role);
		}
		else
		{
			role_mask |= (uint8_t)(1U << (uint8_t)role);
			if (!ProductConfig_CurrentRoleMatchesTopology(
				current_sense->topology, role))
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_TOPOLOGY_MISMATCH,
					PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
					(uint32_t)role);
			}
		}
		if (!ProductConfig_CurrentPolarityIsValid(
			current_sense->channel_polarities[channel]))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_POLARITY_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
				(uint32_t)current_sense->channel_polarities[channel]);
		}
		if (current_sense->channel_endpoints[channel] ==
			PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
		if (current_sense->minimum_valid_offset_count[channel] >
			current_sense->maximum_valid_offset_count[channel])
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_RANGE_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
		else if (current_sense->default_offset_count[channel] <
				current_sense->minimum_valid_offset_count[channel] ||
			current_sense->default_offset_count[channel] >
				current_sense->maximum_valid_offset_count[channel])
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_DEFAULT_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
				current_sense->default_offset_count[channel]);
		}
		if (!ProductConfig_IsPositiveFinite(
			current_sense->current_a_per_count[channel]))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_SCALE_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
		for (other = (uint8_t)(channel + 1U);
			other < required_channel_count; other++)
		{
			if (current_sense->channel_endpoints[channel] !=
					PRODUCT_CONFIG_ENDPOINT_NONE &&
				current_sense->channel_endpoints[channel] ==
					current_sense->channel_endpoints[other])
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
					(uint32_t)other);
			}
			if (role != PRODUCT_CURRENT_CHANNEL_ROLE_INVALID &&
				role == current_sense->channel_roles[other])
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
					(uint32_t)other);
			}
		}
	}
	if ((current_sense->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT ||
		 current_sense->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT) &&
		role_mask != (uint8_t)((1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A) |
			(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B) |
			(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C)))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_TOPOLOGY_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)role_mask);
	}
	for (channel = required_channel_count;
		required_channel_count != 0U &&
		channel < PRODUCT_CONFIG_MAX_CURRENT_CHANNELS; channel++)
	{
		if (!ProductConfig_CurrentUnusedChannelIsZero(current_sense, channel))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_UNUSED_CHANNEL_CONFIGURED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
	}
	if (current_sense->offset_calibration_supported &&
		current_sense->offset_calibration_sample_count == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_CALIBRATION_INVALID,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (!current_sense->pwm_synchronized ||
		current_sense->samples_per_pwm_period == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_PWM_SYNC_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (current_sense->topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		if (current_sense->samples_per_pwm_period < 2U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_TWO_SAMPLES_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)current_sense->samples_per_pwm_period);
		}
		if (!current_sense->captures_pwm_sector)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_PWM_SECTOR_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (!current_sense->supports_sample_window_compensation)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_WINDOW_COMPENSATION_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
}

static void ProductConfig_ValidateSafety(const ProductConfig *config,
	ProductConfigValidationResult *result)
{
	const ProductSafetyConfig *safety = &config->safety;
	bool electrical_limits_are_valid =
		ProductConfig_IsPositiveFinite(safety->software_overcurrent_trip_a) &&
		ProductConfig_IsFinite(safety->undervoltage_trip_v) &&
		safety->undervoltage_trip_v >= 0.0f &&
		ProductConfig_IsPositiveFinite(safety->overvoltage_trip_v) &&
		safety->overvoltage_trip_v > safety->undervoltage_trip_v &&
		ProductConfig_IsPositiveFinite(safety->bus_voltage_filter_alpha) &&
		safety->bus_voltage_filter_alpha <= 1.0f;

	if (config->board != NULL &&
		safety->software_overcurrent_trip_a >
			config->board->reliable_phase_current_limit_a)
		electrical_limits_are_valid = false;
	if (!electrical_limits_are_valid)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_SAFETY_LIMIT_INVALID,
			PRODUCT_CONFIG_SUBJECT_SAFETY_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (safety->overcurrent_confirm_cycles == 0U ||
		safety->voltage_confirm_cycles == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_SAFETY_CONFIRMATION_INVALID,
			PRODUCT_CONFIG_SUBJECT_SAFETY_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
}

bool ProductConfig_Validate(const ProductConfig *config,
	ProductConfigValidationResult *result)
{
	uint8_t index;
	uint8_t other;
	uint8_t angle_count;
	uint8_t temperature_count;
	uint8_t motor_rotor_lut_count = 0U;
	ProductTemperatureZoneMask zone_mask;

	if (result == NULL)
		return false;
	memset(result, 0, sizeof(*result));
	if (config == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_NULL_CONFIG,
			PRODUCT_CONFIG_SUBJECT_CONFIG,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		return false;
	}
	(void)ProductConfig_Derive(config, &result->derived);

	if (config->identity.product_id == 0U ||
		config->identity.variant_id == 0U ||
		config->identity.platform_id == 0U ||
		config->identity.configuration_fingerprint == 0U ||
		config->identity.configuration_schema_version !=
			PRODUCT_CONFIG_SCHEMA_VERSION)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_INVALID_IDENTITY,
			PRODUCT_CONFIG_SUBJECT_IDENTITY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->board == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_BOARD_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_BOARD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else
	{
		if (config->board->design_id == 0U ||
			config->board->platform_id == 0U ||
			config->board->motor_drive_endpoint == PRODUCT_CONFIG_ENDPOINT_NONE ||
			config->board->control_frequency_hz == 0U ||
			!ProductConfig_IsPositiveFinite(
				config->board->reliable_phase_current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->board->command_phase_current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->board->calibration_phase_current_limit_a) ||
			config->board->command_phase_current_limit_a >
				config->board->reliable_phase_current_limit_a ||
			config->board->calibration_phase_current_limit_a >
				config->board->reliable_phase_current_limit_a ||
			(config->board->bus_voltage_measurement_available &&
			 !ProductConfig_IsPositiveFinite(
				 config->board->bus_voltage_v_per_count)))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_INVALID_BOARD_LIMIT,
				PRODUCT_CONFIG_SUBJECT_BOARD,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->identity.platform_id != config->board->platform_id)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_PLATFORM_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_IDENTITY,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				config->board->platform_id);
		}
		if (!ProductConfig_IsFinite(
				config->board->phase_resistance_path_compensation_ohm) ||
			config->board->phase_resistance_path_compensation_ohm < 0.0f)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_BOARD_PATH_COMPENSATION_INVALID,
				PRODUCT_CONFIG_SUBJECT_BOARD,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		ProductConfig_ValidateCurrentSense(&config->board->current_sense,
			result);
	}
	ProductConfig_ValidateSafety(config, result);

	if (config->motor == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_MOTOR_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_MOTOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else
	{
		if (config->motor->design_id == 0U || config->motor->pole_pairs == 0U ||
			!ProductConfig_IsPositiveFinite(
				config->motor->phase_resistance_ohm) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->d_axis_inductance_h) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->q_axis_inductance_h) ||
			!ProductConfig_IsPositiveFinite(config->motor->flux_weber) ||
			!ProductConfig_IsPositiveFinite(config->motor->current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->calibration_current_a) ||
			!ProductConfig_IsPositiveFinite(config->motor->speed_limit_rad_s) ||
			config->motor->calibration_current_a >
				config->motor->current_limit_a)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_INVALID_MOTOR_DESIGN,
				PRODUCT_CONFIG_SUBJECT_MOTOR,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board != NULL &&
			(config->motor->current_limit_a >
				config->board->command_phase_current_limit_a ||
			 config->motor->calibration_current_a >
				config->board->calibration_phase_current_limit_a))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_MOTOR_EXCEEDS_BOARD_LIMIT,
				PRODUCT_CONFIG_SUBJECT_MOTOR,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
	if (config->load == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_LOAD_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_LOAD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else if (config->load->design_id == 0U ||
		!ProductConfig_IsPositiveFinite(config->load->transmission_ratio) ||
		!ProductConfig_IsPositiveFinite(
			config->load->maximum_output_speed_rad_s))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_INVALID_LOAD_DESIGN,
			PRODUCT_CONFIG_SUBJECT_LOAD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
#if defined(PRODUCT_CATALOG_INCLUDE_ALL)
	ProductConfig_ValidateMotorAcceptance(config, result);
	ProductConfig_ValidateControl(config, result);
	ProductConfig_ValidateCommissioningTuning(config, result);
#endif

	if (config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_COUNT_EXCEEDED,
			PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, config->angle_sensor_count);
	}
	angle_count = ProductConfig_BoundedAngleSensorCount(config);
	for (index = 0U; index < angle_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[index];
		if (sensor->design == NULL)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_DESIGN_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		if (sensor->instance_id == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		if (sensor->source == PRODUCT_ANGLE_SENSOR_SOURCE_INVALID ||
			(sensor->design != NULL && sensor->source != sensor->design->source))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_SOURCE_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index,
				(uint32_t)sensor->source);
		}
		if (sensor->role < PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY ||
			sensor->role > PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index,
				(uint32_t)sensor->role);
		}
		if (sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		if (sensor->design != NULL &&
			ProductConfig_IsMotorRotorRole(sensor->role) &&
			(sensor->design->capabilities &
			 PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION) != 0U)
		{
			motor_rotor_lut_count++;
		}
		for (other = (uint8_t)(index + 1U); other < angle_count; other++)
		{
			if (sensor->instance_id != 0U && sensor->instance_id ==
				config->angle_sensors[other].instance_id)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, other);
			}
			if (sensor->role != PRODUCT_ANGLE_SENSOR_ROLE_INVALID &&
				sensor->role == config->angle_sensors[other].role)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, other);
			}
		}
	}
	if (motor_rotor_lut_count > 1U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_SECONDARY_ROTOR_LUT_UNSUPPORTED,
			PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, motor_rotor_lut_count);
	}

	if (config->temperature_sensor_count >
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_COUNT_EXCEEDED,
			PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			config->temperature_sensor_count);
	}
	temperature_count = ProductConfig_BoundedTemperatureSensorCount(config);
	for (index = 0U; index < temperature_count; index++)
	{
		const ProductTemperatureSensorInstanceConfig *sensor =
			&config->temperature_sensors[index];
		if (sensor->design == NULL)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_DESIGN_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->instance_id == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->source == PRODUCT_TEMPERATURE_SENSOR_SOURCE_INVALID ||
			(sensor->design != NULL && sensor->source != sensor->design->source))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_SOURCE_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index,
				(uint32_t)sensor->source);
		}
		if (sensor->zone < PRODUCT_TEMPERATURE_ZONE_MCU ||
			sensor->zone > PRODUCT_TEMPERATURE_ZONE_AMBIENT)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ZONE_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index,
				(uint32_t)sensor->zone);
		}
		if (sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->sample_period_ms == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SAMPLE_PERIOD_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->pending_timeout_ms == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_PENDING_TIMEOUT_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->protection_enabled &&
			(sensor->design == NULL ||
			 !ProductConfig_IsFinite(sensor->protection_limit_c) ||
			 sensor->protection_limit_c <
				sensor->design->minimum_temperature_c ||
			 sensor->protection_limit_c >
				sensor->design->maximum_temperature_c))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_LIMIT_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		for (other = (uint8_t)(index + 1U); other < temperature_count;
			other++)
		{
			if (sensor->instance_id != 0U && sensor->instance_id ==
				config->temperature_sensors[other].instance_id)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, other);
			}
		}
	}

	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.electrical_angle,
		PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.motor_velocity,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.motor_position,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.output_position,
		PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.calibration_reference,
		PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.fallback_electrical_angle,
		PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE);

	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.speed_control, PRODUCT_FEATURE_DETAIL_SPEED);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.position_control, PRODUCT_FEATURE_DETAIL_POSITION);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.output_position_control,
		PRODUCT_FEATURE_DETAIL_OUTPUT_POSITION);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.sensorless_control,
		PRODUCT_FEATURE_DETAIL_SENSORLESS);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.angle_redundancy_monitor,
		PRODUCT_FEATURE_DETAIL_ANGLE_REDUNDANCY);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.temperature_monitoring,
		PRODUCT_FEATURE_DETAIL_TEMPERATURE_MONITORING);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.temperature_protection,
		PRODUCT_FEATURE_DETAIL_TEMPERATURE_PROTECTION);

	if (config->features.speed_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_SPEED_FEEDBACK_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.position_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 (PRODUCT_CAP_MOTOR_POSITION_FEEDBACK |
		  PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK)) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_POSITION_FEEDBACK_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.output_position_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_OUTPUT_POSITION_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.sensorless_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities & PRODUCT_CAP_SENSORLESS_OBSERVER) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_SENSORLESS_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.angle_redundancy_monitor ==
			PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities & PRODUCT_CAP_ANGLE_REDUNDANCY) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_REDUNDANCY_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.temperature_monitoring == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_TEMPERATURE_MONITORING) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.temperature_protection == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_TEMPERATURE_PROTECTION) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	for (index = PRODUCT_TEMPERATURE_ZONE_MCU;
		index <= PRODUCT_TEMPERATURE_ZONE_AMBIENT; index++)
	{
		zone_mask = PRODUCT_TEMPERATURE_ZONE_MASK(index);
		if ((config->features.required_monitored_temperature_zones &
			 zone_mask) != 0U &&
			(result->derived.monitored_temperature_zones & zone_mask) == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE,
				PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY, index, zone_mask);
		}
		if ((config->features.required_protected_temperature_zones &
			 zone_mask) != 0U &&
			(result->derived.protected_temperature_zones & zone_mask) == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE,
				PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY, index, zone_mask);
		}
	}

#define VALIDATE_COMMISSIONING_POLICY(field, step) \
	ProductConfig_ValidateCommissioningPolicyValue(result, \
		config->commissioning.field, step)
	VALIDATE_COMMISSIONING_POLICY(current_offset,
		PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET);
	VALIDATE_COMMISSIONING_POLICY(phase_resistance,
		PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE);
	VALIDATE_COMMISSIONING_POLICY(angle_direction,
		PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION);
	VALIDATE_COMMISSIONING_POLICY(angle_linearization,
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION);
	VALIDATE_COMMISSIONING_POLICY(electrical_zero,
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO);
	VALIDATE_COMMISSIONING_POLICY(mechanical_zero,
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO);
	VALIDATE_COMMISSIONING_POLICY(dual_angle_alignment,
		PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT);
	VALIDATE_COMMISSIONING_POLICY(friction_identification,
		PRODUCT_COMMISSIONING_STEP_FRICTION);
	VALIDATE_COMMISSIONING_POLICY(cogging_identification,
		PRODUCT_COMMISSIONING_STEP_COGGING);
	VALIDATE_COMMISSIONING_POLICY(sensorless_validation,
		PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION);
	VALIDATE_COMMISSIONING_POLICY(save_results,
		PRODUCT_COMMISSIONING_STEP_SAVE);
#undef VALIDATE_COMMISSIONING_POLICY

#define REQUIRE_COMMISSIONING(field, capability, step) \
	ProductConfig_RequireCommissioningCapability(result, \
		config->commissioning.field, result->derived.capabilities, \
		capability, step)
	REQUIRE_COMMISSIONING(current_offset,
		PRODUCT_CAP_CURRENT_OFFSET_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET);
	REQUIRE_COMMISSIONING(phase_resistance,
		PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE);
	REQUIRE_COMMISSIONING(angle_direction,
		PRODUCT_CAP_ANGLE_DIRECTION_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION);
	REQUIRE_COMMISSIONING(angle_linearization,
		PRODUCT_CAP_ANGLE_LINEARIZATION_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION);
	REQUIRE_COMMISSIONING(electrical_zero,
		PRODUCT_CAP_ELECTRICAL_ZERO_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO);
	REQUIRE_COMMISSIONING(mechanical_zero,
		PRODUCT_CAP_MECHANICAL_ZERO_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO);
	REQUIRE_COMMISSIONING(dual_angle_alignment,
		PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT,
		PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT);
	REQUIRE_COMMISSIONING(friction_identification,
		PRODUCT_CAP_FRICTION_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_FRICTION);
	REQUIRE_COMMISSIONING(cogging_identification,
		PRODUCT_CAP_COGGING_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_COGGING);
	REQUIRE_COMMISSIONING(sensorless_validation,
		PRODUCT_CAP_SENSORLESS_OBSERVER,
		PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION);
#undef REQUIRE_COMMISSIONING

	if (config->can.mode > PRODUCT_CAN_MODE_FD)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CAN_MODE_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)config->can.mode);
	}
	else if (config->can.mode == PRODUCT_CAN_MODE_CLASSIC)
	{
		if (config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board == NULL || !config->board->classic_can_supported)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)config->can.mode);
		}
		if (config->can.nominal_bitrate_kbps == 0U ||
			config->can.nominal_bitrate_kbps > UINT32_MAX / UINT32_C(1000) ||
			config->can.data_bitrate_kbps != 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->can.bit_rate_switching)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BRS_REQUIRES_FD,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->can.maximum_payload_bytes == 0U ||
			config->can.maximum_payload_bytes > 8U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_PAYLOAD_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				config->can.maximum_payload_bytes);
		}
	}
	else if (config->can.mode == PRODUCT_CAN_MODE_FD)
	{
		if (config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board == NULL || !config->board->can_fd_supported ||
			(config->can.bit_rate_switching &&
			 !config->board->can_brs_supported))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)config->can.mode);
		}
		if (config->can.nominal_bitrate_kbps == 0U ||
			config->can.nominal_bitrate_kbps > UINT32_MAX / UINT32_C(1000) ||
			config->can.data_bitrate_kbps == 0U ||
			config->can.data_bitrate_kbps > UINT32_MAX / UINT32_C(1000) ||
			(!config->can.bit_rate_switching &&
			 config->can.data_bitrate_kbps !=
				config->can.nominal_bitrate_kbps))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->can.maximum_payload_bytes == 0U ||
			config->can.maximum_payload_bytes > 64U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_PAYLOAD_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				config->can.maximum_payload_bytes);
		}
	}
	if (config->can.default_node_id > 7U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CAN_NODE_ID_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			config->can.default_node_id);
	}
	if (config->can.minimum_heartbeat_ms >
			config->can.maximum_heartbeat_ms ||
		(config->can.heartbeat_ms != 0U &&
		 (config->can.heartbeat_ms < config->can.minimum_heartbeat_ms ||
		  config->can.heartbeat_ms > config->can.maximum_heartbeat_ms)))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, config->can.heartbeat_ms);
	}
	if (config->service_stream.enabled &&
		config->service_stream.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 1U);
	}

	return result->total_error_count == 0U;
}
