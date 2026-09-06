#include "bsp_product_binding.h"

#include <string.h>

static bool BspProductBinding_KilobitsToBits(uint32_t kilobits_per_second,
	uint32_t *bits_per_second)
{
	if (bits_per_second == 0 ||
		kilobits_per_second > UINT32_MAX / UINT32_C(1000))
		return false;
	*bits_per_second = kilobits_per_second * UINT32_C(1000);
	return true;
}

static bool BspProductBinding_MapCurrentTopology(
	ProductCurrentSenseTopology source, BspCurrentSenseTopology *target)
{
	if (target == 0)
		return false;
	switch (source)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
			*target = BSP_CURRENT_SENSE_PHASE_INLINE_THREE_SENSOR;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			*target = BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			*target = BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			*target = BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT;
			return true;
		default:
			return false;
	}
}

static BspAngleEndpointKind BspProductBinding_MapAngleKind(
	ProductAngleSensorSource source)
{
	switch (source)
	{
		case PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL:
			return BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL;
		case PRODUCT_ANGLE_SENSOR_SOURCE_INCREMENTAL_QUADRATURE:
			return BSP_ANGLE_ENDPOINT_INCREMENTAL;
		case PRODUCT_ANGLE_SENSOR_SOURCE_HALL:
			return BSP_ANGLE_ENDPOINT_PULSE;
		case PRODUCT_ANGLE_SENSOR_SOURCE_RESOLVER:
		case PRODUCT_ANGLE_SENSOR_SOURCE_ANALOG:
			return BSP_ANGLE_ENDPOINT_ANALOG;
		default:
			return BSP_ANGLE_ENDPOINT_UNSPECIFIED;
	}
}

static BspTemperatureSourceKind BspProductBinding_MapTemperatureSource(
	ProductTemperatureSensorSource source)
{
	switch (source)
	{
		case PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL:
			return BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE;
		case PRODUCT_TEMPERATURE_SENSOR_SOURCE_ANALOG:
			return BSP_TEMPERATURE_SOURCE_ANALOG_RESISTIVE;
		case PRODUCT_TEMPERATURE_SENSOR_SOURCE_DIGITAL:
			return BSP_TEMPERATURE_SOURCE_DIGITAL;
		default:
			return BSP_TEMPERATURE_SOURCE_UNSPECIFIED;
	}
}

static BspTemperatureLocation BspProductBinding_MapTemperatureLocation(
	ProductTemperatureZone zone)
{
	switch (zone)
	{
		case PRODUCT_TEMPERATURE_ZONE_MCU:
			return BSP_TEMPERATURE_LOCATION_PROCESSOR;
		case PRODUCT_TEMPERATURE_ZONE_PCB:
		case PRODUCT_TEMPERATURE_ZONE_AMBIENT:
			return BSP_TEMPERATURE_LOCATION_BOARD_AMBIENT;
		case PRODUCT_TEMPERATURE_ZONE_POWER_STAGE:
			return BSP_TEMPERATURE_LOCATION_POWER_STAGE;
		case PRODUCT_TEMPERATURE_ZONE_MOTOR_WINDING:
			return BSP_TEMPERATURE_LOCATION_MOTOR;
		default:
			return BSP_TEMPERATURE_LOCATION_UNSPECIFIED;
	}
}

static bool BspProductBinding_BuildRequest(const ProductConfig *config,
	BspBoardBindingRequest *request)
{
	uint8_t index;
	uint8_t channel_count;

	if (config == 0 || config->board == 0 || request == 0)
		return false;
	(void)memset(request, 0, sizeof(*request));
	request->motor_drive_endpoint = config->board->motor_drive_endpoint;
	if (!BspProductBinding_MapCurrentTopology(
		config->board->current_sense.topology,
		&request->current_sense_topology))
		return false;
	channel_count = config->board->current_sense.physical_channel_count;
	if (channel_count > BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT)
		return false;
	request->current_sensor_binding_count = channel_count;
	for (index = 0U; index < channel_count; index++)
	{
		request->current_sensor_endpoints[index] =
			config->board->current_sense.channel_endpoints[index];
	}
	request->require_synchronized_current_sampling =
		config->board->current_sense.pwm_synchronized;
	request->require_hardware_shutdown =
		config->board->require_hardware_shutdown;

	if (config->angle_sensor_count > BSP_BOARD_MAX_ANGLE_BINDING_COUNT ||
		config->temperature_sensor_count >
			BSP_BOARD_MAX_TEMPERATURE_BINDING_COUNT)
		return false;
	request->angle_binding_count = config->angle_sensor_count;
	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		request->angle_bindings[index].endpoint_id =
			config->angle_sensors[index].endpoint;
		request->angle_bindings[index].required_endpoint_kind =
			BspProductBinding_MapAngleKind(config->angle_sensors[index].source);
	}
	request->temperature_binding_count = config->temperature_sensor_count;
	for (index = 0U; index < config->temperature_sensor_count; index++)
	{
		request->temperature_bindings[index].endpoint_id =
			config->temperature_sensors[index].endpoint;
		request->temperature_bindings[index].required_source_kind =
			BspProductBinding_MapTemperatureSource(
				config->temperature_sensors[index].source);
		request->temperature_bindings[index].required_location =
			BspProductBinding_MapTemperatureLocation(
				config->temperature_sensors[index].zone);
	}

	if (config->can.mode != PRODUCT_CAN_MODE_DISABLED)
	{
		request->communication_bindings[0].endpoint_id = config->can.endpoint;
		request->communication_bindings[0].required_kind = BSP_COMMUNICATION_CAN;
		request->communication_bindings[0].required_features =
			config->can.mode == PRODUCT_CAN_MODE_FD ?
			BSP_COMMUNICATION_FEATURE_CAN_FD :
			BSP_COMMUNICATION_FEATURE_CAN_CLASSIC;
		if (config->can.bit_rate_switching)
			request->communication_bindings[0].required_features |=
				BSP_COMMUNICATION_FEATURE_CAN_BRS;
		request->communication_bindings[0].required_payload_bytes =
			config->can.maximum_payload_bytes;
		if (!BspProductBinding_KilobitsToBits(
			config->can.nominal_bitrate_kbps,
			&request->communication_bindings[0].required_nominal_bit_rate))
			return false;
		if (config->can.data_bitrate_kbps != 0U &&
			!BspProductBinding_KilobitsToBits(
				config->can.data_bitrate_kbps,
				&request->communication_bindings[0].required_data_bit_rate))
			return false;
		request->communication_binding_count = 1U;
	}
	if (config->service_stream.enabled)
	{
		uint8_t communication_index = request->communication_binding_count;
		if (communication_index >= BSP_BOARD_MAX_COMMUNICATION_BINDING_COUNT)
			return false;
		request->communication_bindings[communication_index].endpoint_id =
			config->service_stream.endpoint;
		request->communication_bindings[communication_index].required_kind =
			BSP_COMMUNICATION_BYTE_STREAM;
		request->communication_bindings[communication_index].required_features =
			BSP_COMMUNICATION_FEATURE_BYTE_STREAM;
		request->communication_bindings[communication_index].required_payload_bytes =
			1U;
		request->communication_binding_count++;
	}
	request->required_system_features =
		BSP_SYSTEM_FEATURE_MONOTONIC_CLOCK |
		BSP_SYSTEM_FEATURE_EXECUTION_TIMER |
		BSP_SYSTEM_FEATURE_CRITICAL_SECTION |
		BSP_SYSTEM_FEATURE_UNIQUE_ID |
		BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE |
		BSP_SYSTEM_FEATURE_SOFTWARE_RESET |
		BSP_SYSTEM_FEATURE_RESET_REASON |
		BSP_SYSTEM_FEATURE_DIAGNOSTIC_SINK |
		BSP_SYSTEM_FEATURE_STATUS_INDICATOR;
	return true;
}

bool BspProductBinding_Validate(const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities,
	ProductConfigValidationResult *product_result,
	BspBoardValidationResult *board_result)
{
	BspBoardBindingRequest request;

	if (product_result == 0 || board_result == 0)
		return false;
	if (!ProductConfig_Validate(config, product_result))
		return false;
	*board_result = BspBoard_ValidateCapabilities(board_capabilities);
	if (board_result->code != BSP_BOARD_VALIDATION_OK)
		return false;
	if (!BspProductBinding_BuildRequest(config, &request))
		return false;
	*board_result = BspBoard_ValidateBindingRequest(board_capabilities,
		&request);
	return board_result->code == BSP_BOARD_VALIDATION_OK;
}
