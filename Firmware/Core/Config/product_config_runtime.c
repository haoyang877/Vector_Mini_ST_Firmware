#include "product_config.h"

#include <float.h>
#include <stddef.h>

static bool ProductConfigRuntime_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static bool ProductConfigRuntime_IsPositiveFinite(float value)
{
	return ProductConfigRuntime_IsFinite(value) && value > 0.0f;
}

static bool ProductConfigRuntime_IsNonnegativeFinite(float value)
{
	return ProductConfigRuntime_IsFinite(value) && value >= 0.0f;
}

static bool ProductConfigRuntime_IsUnitRatio(float value)
{
	return ProductConfigRuntime_IsPositiveFinite(value) && value <= 1.0f;
}

static bool ProductConfigRuntime_LoopFrequencyGate(uint32_t fast_hz,
	uint32_t loop_hz)
{
	return fast_hz > 0U && loop_hz > 0U && loop_hz <= fast_hz &&
		fast_hz % loop_hz == 0U && fast_hz / loop_hz <= UINT16_MAX;
}

static bool ProductConfigRuntime_StartupGate(
	const ProductSensorlessStartupConfig *startup, float current_limit_a,
	const ProductMotorDesign *motor)
{
	float iq_ratio;
	float id_ratio;

	if (!ProductConfigRuntime_IsPositiveFinite(
			startup->align_current_ramp_time_s) ||
		!ProductConfigRuntime_IsNonnegativeFinite(startup->align_hold_time_s) ||
		!ProductConfigRuntime_IsPositiveFinite(startup->align_current_a) ||
		!ProductConfigRuntime_IsNonnegativeFinite(startup->startup_iq_initial_a) ||
		!ProductConfigRuntime_IsFinite(startup->startup_iq_a) ||
		startup->startup_iq_a < startup->startup_iq_initial_a ||
		!ProductConfigRuntime_IsPositiveFinite(startup->startup_iq_ramp_time_s) ||
		!ProductConfigRuntime_IsNonnegativeFinite(startup->startup_id_a) ||
		!ProductConfigRuntime_IsNonnegativeFinite(
			startup->minimum_current_limit_a) ||
		!ProductConfigRuntime_IsPositiveFinite(
			startup->minimum_electrical_velocity_rad_s) ||
		!ProductConfigRuntime_IsFinite(
			startup->target_electrical_velocity_rad_s) ||
		startup->target_electrical_velocity_rad_s <
			startup->minimum_electrical_velocity_rad_s ||
		startup->target_electrical_velocity_rad_s /
			(float)motor->pole_pairs > motor->speed_limit_rad_s ||
		!ProductConfigRuntime_IsPositiveFinite(startup->startup_ramp_time_s) ||
		!ProductConfigRuntime_IsPositiveFinite(startup->speed_lock_time_s) ||
		!ProductConfigRuntime_IsUnitRatio(startup->speed_lock_filter_alpha) ||
		!ProductConfigRuntime_IsUnitRatio(startup->observer_lock_ratio) ||
		!ProductConfigRuntime_IsPositiveFinite(startup->angle_handoff_time_s) ||
		!ProductConfigRuntime_IsPositiveFinite(startup->lock_timeout_s) ||
		startup->lock_timeout_s < startup->speed_lock_time_s ||
		!ProductConfigRuntime_IsPositiveFinite(startup->id_ramp_down_time_s) ||
		!ProductConfigRuntime_IsPositiveFinite(startup->observer_loss_time_s) ||
		startup->align_current_a > current_limit_a ||
		startup->startup_iq_a > current_limit_a ||
		startup->startup_id_a > current_limit_a ||
		startup->minimum_current_limit_a > current_limit_a)
		return false;
	iq_ratio = startup->startup_iq_a / current_limit_a;
	id_ratio = startup->startup_id_a / current_limit_a;
	return iq_ratio * iq_ratio + id_ratio * id_ratio <= 1.0f;
}

static bool ProductConfigRuntime_MotorAcceptanceGate(
	const ProductConfig *config)
{
	const ProductMotorAcceptanceConfig *range = &config->motor_acceptance;

	return ProductConfigRuntime_IsPositiveFinite(
			range->phase_resistance_min_ohm) &&
		ProductConfigRuntime_IsFinite(range->phase_resistance_max_ohm) &&
		range->phase_resistance_max_ohm > range->phase_resistance_min_ohm &&
		ProductConfigRuntime_IsPositiveFinite(range->inductance_min_h) &&
		ProductConfigRuntime_IsFinite(range->inductance_max_h) &&
		range->inductance_max_h > range->inductance_min_h &&
		ProductConfigRuntime_IsPositiveFinite(range->flux_min_weber) &&
		ProductConfigRuntime_IsFinite(range->flux_max_weber) &&
		range->flux_max_weber > range->flux_min_weber &&
		config->motor->phase_resistance_ohm >= range->phase_resistance_min_ohm &&
		config->motor->phase_resistance_ohm <= range->phase_resistance_max_ohm &&
		config->motor->d_axis_inductance_h >= range->inductance_min_h &&
		config->motor->d_axis_inductance_h <= range->inductance_max_h &&
		config->motor->q_axis_inductance_h >= range->inductance_min_h &&
		config->motor->q_axis_inductance_h <= range->inductance_max_h &&
		config->motor->flux_weber >= range->flux_min_weber &&
		config->motor->flux_weber <= range->flux_max_weber;
}

static bool ProductConfigRuntime_ControlGate(const ProductConfig *config)
{
	const ProductControlConfig *control = &config->control;
	const ProductControlParameterLimits *limits = &control->parameter_limits;
	const ProductSensorlessControlConfig *sensorless = &control->sensorless;
	const ProductPositionFrictionControlConfig *friction =
		&control->position_friction;
	uint32_t fast_hz = config->board->control_frequency_hz;
	float current_limit_a = config->motor->current_limit_a <
		config->board->command_phase_current_limit_a ?
		config->motor->current_limit_a :
		config->board->command_phase_current_limit_a;

	if (!ProductConfigRuntime_LoopFrequencyGate(fast_hz,
			control->speed_loop_frequency_hz) ||
		!ProductConfigRuntime_LoopFrequencyGate(fast_hz,
			control->position_loop_frequency_hz) ||
		!ProductConfigRuntime_LoopFrequencyGate(fast_hz,
			control->cascade_position_loop_frequency_hz) ||
		!ProductConfigRuntime_IsPositiveFinite(control->speed_kp) ||
		!ProductConfigRuntime_IsNonnegativeFinite(control->speed_ki) ||
		!ProductConfigRuntime_IsPositiveFinite(limits->speed_limit_max_rad_s) ||
		!ProductConfigRuntime_IsPositiveFinite(
			limits->position_kp_limit_a_per_rad) ||
		!ProductConfigRuntime_IsPositiveFinite(control->default_speed_limit_rad_s) ||
		control->default_speed_limit_rad_s > limits->speed_limit_max_rad_s ||
		control->default_speed_limit_rad_s > config->motor->speed_limit_rad_s ||
		control->default_speed_limit_rad_s / config->load->transmission_ratio >
			config->load->maximum_output_speed_rad_s ||
		!ProductConfigRuntime_IsPositiveFinite(
			control->current_loop_bandwidth_rad_s) ||
		!ProductConfigRuntime_StartupGate(&sensorless->startup,
			current_limit_a, config->motor) ||
		!ProductConfigRuntime_IsUnitRatio(
			sensorless->speed_feedback_lpf_alpha) ||
		!ProductConfigRuntime_IsPositiveFinite(
			sensorless->flux_observer_resistance_scale))
		return false;
	if (!friction->enabled)
		return true;
	return ProductConfigRuntime_IsNonnegativeFinite(
			friction->friction_positive_current_a) &&
		ProductConfigRuntime_IsNonnegativeFinite(
			friction->friction_negative_current_a) &&
		ProductConfigRuntime_IsNonnegativeFinite(
			friction->breakaway_positive_current_a) &&
		ProductConfigRuntime_IsNonnegativeFinite(
			friction->breakaway_negative_current_a) &&
		friction->breakaway_positive_current_a <= current_limit_a &&
		friction->breakaway_negative_current_a <= current_limit_a;
}

static bool ProductConfigRuntime_CommissioningGate(
	const ProductConfig *config)
{
	const ProductCommissioningTuningConfig *tuning =
		&config->commissioning_tuning;
	const ProductPhaseResistanceCommissioningConfig *phase =
		&tuning->phase_resistance;
	const ProductAngleCommissioningConfig *angle = &tuning->angle;
	const ProductFrictionIdentificationConfig *friction = &tuning->friction;
	const ProductCoggingIdentificationConfig *cogging = &tuning->cogging;
	uint32_t index;
	float command_limit_a = config->motor->current_limit_a <
		config->board->command_phase_current_limit_a ?
		config->motor->current_limit_a :
		config->board->command_phase_current_limit_a;
	float calibration_limit_a = config->motor->calibration_current_a <
		config->board->calibration_phase_current_limit_a ?
		config->motor->calibration_current_a :
		config->board->calibration_phase_current_limit_a;

	if (!ProductConfigRuntime_IsPositiveFinite(phase->test_current_min_a) ||
		!ProductConfigRuntime_IsFinite(phase->test_current_low_a) ||
		phase->test_current_low_a < phase->test_current_min_a ||
		!ProductConfigRuntime_IsFinite(phase->test_current_high_a) ||
		phase->test_current_high_a <= phase->test_current_low_a ||
		!ProductConfigRuntime_IsFinite(phase->test_current_max_a) ||
		phase->test_current_max_a < phase->test_current_high_a ||
		phase->test_current_max_a > calibration_limit_a ||
		!ProductConfigRuntime_IsFinite(phase->voltage_tolerance_v) ||
		!ProductConfigRuntime_IsUnitRatio(phase->voltage_filter_alpha) ||
		phase->ramp_time_ms == 0U || phase->settle_time_ms == 0U ||
		phase->sample_time_ms == 0U || phase->pause_time_ms == 0U ||
		phase->timeout_ms < phase->settle_time_ms ||
		!ProductConfigRuntime_StartupGate(&angle->startup,
			command_limit_a, config->motor) ||
		angle->calibration_mechanical_turns == 0U ||
		angle->calibration_verify_mechanical_turns == 0U ||
		angle->calibration_min_samples_per_bin == 0U ||
		angle->calibration_lut_build_bins_per_cycle == 0U ||
		!ProductConfigRuntime_IsUnitRatio(
			angle->calibration_speed_error_ratio) ||
		!ProductConfigRuntime_IsPositiveFinite(
			angle->calibration_sample_timeout_s) ||
		angle->electrical_zero_min_align_current_a > command_limit_a ||
		friction->speed_point_count < 2U ||
		friction->speed_point_count >
			PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT ||
		!ProductConfigRuntime_IsUnitRatio(friction->current_ratio_max) ||
		cogging->turns == 0U || cogging->minimum_samples_per_bin == 0U ||
		!ProductConfigRuntime_IsPositiveFinite(cogging->maximum_current_a) ||
		cogging->maximum_current_a > calibration_limit_a)
		return false;
	for (index = 0U; index < friction->speed_point_count; index++)
	{
		if (!ProductConfigRuntime_IsPositiveFinite(
				friction->speed_points_rad_s[index]) ||
			(index > 0U && friction->speed_points_rad_s[index] <=
				friction->speed_points_rad_s[index - 1U]))
			return false;
	}
	return true;
}

static bool ProductConfigRuntime_Fail(ProductConfigRuntimeError *error,
	ProductConfigRuntimeError value)
{
	if (error != NULL)
		*error = value;
	return false;
}

static uint8_t ProductConfigRuntime_CurrentChannelCount(
	ProductCurrentSenseTopology topology)
{
	switch (topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT: return 3U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT: return 2U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT: return 1U;
		default: return 0U;
	}
}

static bool ProductConfigRuntime_CurrentSenseUsesShunt(
	ProductCurrentSenseTopology topology)
{
	return topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT ||
		topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT ||
		topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT;
}

static bool ProductConfigRuntime_CurrentRoleIsPhase(
	ProductCurrentChannelRole role)
{
	return role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A ||
		role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B ||
		role == PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C;
}

static bool ProductConfigRuntime_CurrentRoleMatchesTopology(
	ProductCurrentSenseTopology topology, ProductCurrentChannelRole role)
{
	switch (topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			return ProductConfigRuntime_CurrentRoleIsPhase(role);
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			return role == PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK;
		default:
			return false;
	}
}

static bool ProductConfigRuntime_CurrentPolarityIsValid(
	ProductCurrentChannelPolarity polarity)
{
	return polarity == PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL ||
		polarity == PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED;
}

static bool ProductConfigRuntime_CurrentUnusedChannelIsZero(
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

static bool ProductConfigRuntime_SafetyIsValid(const ProductConfig *config)
{
	const ProductSafetyConfig *safety = &config->safety;

	return ProductConfigRuntime_IsPositiveFinite(
			safety->software_overcurrent_trip_a) &&
		safety->software_overcurrent_trip_a <=
			config->board->reliable_phase_current_limit_a &&
		ProductConfigRuntime_IsFinite(safety->undervoltage_trip_v) &&
		safety->undervoltage_trip_v >= 0.0f &&
		ProductConfigRuntime_IsPositiveFinite(safety->overvoltage_trip_v) &&
		safety->overvoltage_trip_v > safety->undervoltage_trip_v &&
		ProductConfigRuntime_IsPositiveFinite(
			safety->bus_voltage_filter_alpha) &&
		safety->bus_voltage_filter_alpha <= 1.0f &&
		safety->overcurrent_confirm_cycles > 0U &&
		safety->voltage_confirm_cycles > 0U;
}

bool ProductConfig_ValidateRuntime(const ProductConfig *config,
	ProductConfigRuntimeError *error)
{
	uint8_t index;
	uint8_t other;
	uint8_t current_channel_count;
	uint8_t current_role_mask = 0U;

	if (error != NULL)
		*error = PRODUCT_CONFIG_RUNTIME_OK;
	if (config == NULL || config->board == NULL || config->motor == NULL ||
		config->load == NULL)
		return ProductConfigRuntime_Fail(error, PRODUCT_CONFIG_RUNTIME_NULL);
	if (config->identity.product_id == 0U || config->identity.variant_id == 0U ||
		config->identity.configuration_fingerprint == 0U ||
		config->identity.configuration_schema_version !=
			PRODUCT_CONFIG_SCHEMA_VERSION ||
		config->identity.platform_id != config->board->platform_id)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_IDENTITY);
	if (config->board->design_id == 0U || config->motor->design_id == 0U ||
		config->load->design_id == 0U || config->motor->pole_pairs == 0U ||
		config->board->bsp_binding_fingerprint == 0U ||
		config->board->motor_drive_endpoint == PRODUCT_CONFIG_ENDPOINT_NONE ||
		config->board->control_frequency_hz == 0U ||
		!ProductConfigRuntime_IsPositiveFinite(
			config->board->reliable_phase_current_limit_a) ||
		!ProductConfigRuntime_IsPositiveFinite(
			config->board->command_phase_current_limit_a) ||
		!ProductConfigRuntime_IsPositiveFinite(
			config->board->calibration_phase_current_limit_a) ||
		config->board->command_phase_current_limit_a >
			config->board->reliable_phase_current_limit_a ||
		config->board->calibration_phase_current_limit_a >
			config->board->reliable_phase_current_limit_a ||
		!ProductConfigRuntime_IsFinite(
			config->board->phase_resistance_path_compensation_ohm) ||
		config->board->phase_resistance_path_compensation_ohm < 0.0f ||
		(config->board->bus_voltage_measurement_available &&
		 !ProductConfigRuntime_IsPositiveFinite(
			 config->board->bus_voltage_v_per_count)))
		return ProductConfigRuntime_Fail(error, PRODUCT_CONFIG_RUNTIME_DESIGN);
	if (!ProductConfigRuntime_MotorAcceptanceGate(config))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_MOTOR_ACCEPTANCE);

	current_channel_count = ProductConfigRuntime_CurrentChannelCount(
		config->board->current_sense.topology);
	if (current_channel_count == 0U ||
		config->board->current_sense.physical_channel_count !=
			current_channel_count ||
		!config->board->current_sense.pwm_synchronized ||
		config->board->current_sense.samples_per_pwm_period == 0U)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	for (index = 0U; index < current_channel_count; index++)
	{
		const ProductCurrentSenseConfig *current =
			&config->board->current_sense;

		if (!ProductConfigRuntime_CurrentRoleMatchesTopology(current->topology,
				current->channel_roles[index]) ||
			!ProductConfigRuntime_CurrentPolarityIsValid(
				current->channel_polarities[index]) ||
			current->channel_endpoints[index] ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			!ProductConfigRuntime_IsPositiveFinite(
				current->current_a_per_count[index]) ||
			current->minimum_valid_offset_count[index] >
				current->maximum_valid_offset_count[index] ||
			current->default_offset_count[index] <
				current->minimum_valid_offset_count[index] ||
			current->default_offset_count[index] >
				current->maximum_valid_offset_count[index])
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
		current_role_mask |= (uint8_t)(1U <<
			(uint8_t)current->channel_roles[index]);
		for (other = (uint8_t)(index + 1U);
			other < current_channel_count; other++)
		{
			if (current->channel_endpoints[index] ==
					current->channel_endpoints[other] ||
				current->channel_roles[index] == current->channel_roles[other])
				return ProductConfigRuntime_Fail(error,
					PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
		}
	}
	for (index = current_channel_count;
		index < PRODUCT_CONFIG_MAX_CURRENT_CHANNELS; index++)
	{
		if (!ProductConfigRuntime_CurrentUnusedChannelIsZero(
			&config->board->current_sense, index))
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	}
	if ((config->board->current_sense.topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT ||
		 config->board->current_sense.topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT) &&
		current_role_mask !=
			(uint8_t)((1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A) |
				(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B) |
				(1U << PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C)))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	if ((ProductConfigRuntime_CurrentSenseUsesShunt(
			config->board->current_sense.topology) &&
		 config->board->current_sense.nominal_shunt_milliohm == 0U) ||
		(config->board->current_sense.offset_calibration_supported &&
		 config->board->current_sense.offset_calibration_sample_count == 0U))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	if (config->board->current_sense.topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT &&
		(config->board->current_sense.samples_per_pwm_period < 2U ||
		 !config->board->current_sense.captures_pwm_sector ||
		 !config->board->current_sense.supports_sample_window_compensation))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	if (!ProductConfigRuntime_SafetyIsValid(config))
		return ProductConfigRuntime_Fail(error, PRODUCT_CONFIG_RUNTIME_SAFETY);
	if (!ProductConfigRuntime_ControlGate(config))
		return ProductConfigRuntime_Fail(error, PRODUCT_CONFIG_RUNTIME_CONTROL);
	if (!ProductConfigRuntime_CommissioningGate(config))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING);

	if (config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_ANGLE_SENSOR);
	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		if (config->angle_sensors[index].design == NULL ||
			config->angle_sensors[index].instance_id == 0U ||
			config->angle_sensors[index].endpoint ==
				PRODUCT_CONFIG_ENDPOINT_NONE)
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_ANGLE_SENSOR);
	}
	if ((config->features.speed_control == PRODUCT_FEATURE_REQUIRED &&
		 config->feedback.motor_velocity.kind == PRODUCT_FEEDBACK_SOURCE_NONE) ||
		(config->features.position_control == PRODUCT_FEATURE_REQUIRED &&
		 config->feedback.motor_position.kind == PRODUCT_FEEDBACK_SOURCE_NONE &&
		 config->feedback.output_position.kind == PRODUCT_FEEDBACK_SOURCE_NONE))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_FEEDBACK);

	if (config->temperature_sensor_count >
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	if ((config->features.required_monitored_temperature_zones != 0U ||
		 config->features.required_protected_temperature_zones != 0U) &&
		config->temperature_sensor_count == 0U)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	for (index = 0U; index < config->temperature_sensor_count; index++)
	{
		if (config->temperature_sensors[index].design == NULL ||
			config->temperature_sensors[index].instance_id == 0U ||
			config->temperature_sensors[index].source ==
				PRODUCT_TEMPERATURE_SENSOR_SOURCE_INVALID ||
			(config->temperature_sensors[index].design != NULL &&
			 config->temperature_sensors[index].source !=
				 config->temperature_sensors[index].design->source) ||
			config->temperature_sensors[index].zone <
				PRODUCT_TEMPERATURE_ZONE_MCU ||
			config->temperature_sensors[index].zone >
				PRODUCT_TEMPERATURE_ZONE_AMBIENT ||
			config->temperature_sensors[index].endpoint ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			config->temperature_sensors[index].sample_period_ms == 0U ||
			config->temperature_sensors[index].pending_timeout_ms == 0U ||
			(config->temperature_sensors[index].protection_enabled &&
			 (!ProductConfigRuntime_IsFinite(
				 config->temperature_sensors[index].protection_limit_c) ||
			  config->temperature_sensors[index].protection_limit_c <
				 config->temperature_sensors[index].design->minimum_temperature_c ||
			  config->temperature_sensors[index].protection_limit_c >
				 config->temperature_sensors[index].design->maximum_temperature_c)))
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
		for (other = (uint8_t)(index + 1U);
			other < config->temperature_sensor_count; other++)
		{
			if (config->temperature_sensors[index].instance_id ==
					config->temperature_sensors[other].instance_id ||
				config->temperature_sensors[index].endpoint ==
					config->temperature_sensors[other].endpoint)
				return ProductConfigRuntime_Fail(error,
					PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
		}
	}

	if (config->can.mode > PRODUCT_CAN_MODE_FD ||
		(config->can.mode != PRODUCT_CAN_MODE_DISABLED &&
		 config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE) ||
		(config->can.mode == PRODUCT_CAN_MODE_CLASSIC &&
		 (config->board == NULL ||
		  !config->board->classic_can_supported ||
		  config->can.bit_rate_switching ||
		  config->can.nominal_bitrate_kbps == 0U ||
		  config->can.data_bitrate_kbps != 0U ||
		  config->can.maximum_payload_bytes == 0U ||
		  config->can.maximum_payload_bytes > 8U)) ||
		(config->can.mode == PRODUCT_CAN_MODE_FD &&
		 (config->board == NULL || !config->board->can_fd_supported ||
		  (config->can.bit_rate_switching &&
		   !config->board->can_brs_supported) ||
		  config->can.nominal_bitrate_kbps == 0U ||
		  config->can.data_bitrate_kbps == 0U ||
		  (!config->can.bit_rate_switching &&
		   config->can.data_bitrate_kbps !=
			config->can.nominal_bitrate_kbps) ||
		  config->can.maximum_payload_bytes == 0U ||
		  config->can.maximum_payload_bytes > 64U)) ||
		config->can.default_node_id > 7U ||
		config->can.minimum_heartbeat_ms >
			config->can.maximum_heartbeat_ms ||
		(config->can.heartbeat_ms != 0U &&
		 (config->can.heartbeat_ms < config->can.minimum_heartbeat_ms ||
		  config->can.heartbeat_ms > config->can.maximum_heartbeat_ms)) ||
		(config->service_stream.enabled &&
		 config->service_stream.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_COMMUNICATION);
	return true;
}
