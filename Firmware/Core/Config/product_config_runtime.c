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
	uint8_t current_channel_count;

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
		if (config->board->current_sense.channel_endpoints[index] ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			!ProductConfigRuntime_IsPositiveFinite(
				config->board->current_sense.current_a_per_count[index]) ||
			config->board->current_sense.minimum_valid_offset_count[index] >
				config->board->current_sense.maximum_valid_offset_count[index] ||
			config->board->current_sense.default_offset_count[index] <
				config->board->current_sense.minimum_valid_offset_count[index] ||
			config->board->current_sense.default_offset_count[index] >
				config->board->current_sense.maximum_valid_offset_count[index])
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	}
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
	if (config->features.required_temperature_zones != 0U &&
		config->temperature_sensor_count == 0U)
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	for (index = 0U; index < config->temperature_sensor_count; index++)
	{
		if (config->temperature_sensors[index].design == NULL ||
			config->temperature_sensors[index].instance_id == 0U ||
			config->temperature_sensors[index].endpoint ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			config->temperature_sensors[index].sample_divider == 0U ||
			(config->temperature_sensors[index].protection_enabled &&
			 (!ProductConfigRuntime_IsFinite(
				 config->temperature_sensors[index].protection_limit_c) ||
			  config->temperature_sensors[index].protection_limit_c <
				 config->temperature_sensors[index].design->minimum_temperature_c ||
			  config->temperature_sensors[index].protection_limit_c >
				 config->temperature_sensors[index].design->maximum_temperature_c)))
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
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
