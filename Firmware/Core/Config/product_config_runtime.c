#include "product_config.h"

#include <stddef.h>

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
		config->board->motor_drive_endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
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
			PRODUCT_CONFIG_ENDPOINT_NONE)
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	}
	if (config->board->current_sense.topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT &&
		(config->board->current_sense.samples_per_pwm_period < 2U ||
		 !config->board->current_sense.captures_pwm_sector ||
		 !config->board->current_sense.supports_sample_window_compensation))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);

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
				PRODUCT_CONFIG_ENDPOINT_NONE)
			return ProductConfigRuntime_Fail(error,
				PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	}

	if (config->can.mode > PRODUCT_CAN_MODE_FD ||
		(config->can.mode != PRODUCT_CAN_MODE_DISABLED &&
		 config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE) ||
		(config->can.mode == PRODUCT_CAN_MODE_CLASSIC &&
		 config->can.bit_rate_switching) ||
		(config->service_stream.enabled &&
		 config->service_stream.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE))
		return ProductConfigRuntime_Fail(error,
			PRODUCT_CONFIG_RUNTIME_COMMUNICATION);
	return true;
}
