#include "bsp_board.h"

static BspBoardValidationResult BspBoard_Result(BspBoardValidationCode code,
	BspBoardResourceKind resource, size_t binding_index,
	BspEndpointId endpoint_id)
{
	BspBoardValidationResult result;

	result.code = code;
	result.resource = resource;
	result.binding_index = binding_index;
	result.endpoint_id = endpoint_id;
	return result;
}

static bool BspBoard_AvailabilityIsValid(BspEndpointAvailability availability)
{
	return availability == BSP_ENDPOINT_UNAVAILABLE ||
		availability == BSP_ENDPOINT_AVAILABLE ||
		availability == BSP_ENDPOINT_PROVISIONED_UNPOPULATED;
}

static bool BspBoard_CurrentSenseTopologyIsValid(
	BspCurrentSenseTopology topology)
{
	return (uint32_t)topology < (uint32_t)BSP_CURRENT_SENSE_TOPOLOGY_COUNT;
}

static uint8_t BspBoard_CurrentSensorCountRequired(
	BspCurrentSenseTopology topology)
{
	switch (topology)
	{
		case BSP_CURRENT_SENSE_NONE:
			return 0U;
		case BSP_CURRENT_SENSE_PHASE_INLINE_THREE_SENSOR:
		case BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT:
			return 3U;
		case BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT:
			return 2U;
		case BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT:
			return 1U;
		default:
			return UINT8_MAX;
	}
}

static bool BspBoard_AngleKindIsValid(BspAngleEndpointKind kind)
{
	return kind >= BSP_ANGLE_ENDPOINT_UNSPECIFIED &&
		kind <= BSP_ANGLE_ENDPOINT_PULSE;
}

static bool BspBoard_TemperatureSourceIsValid(
	BspTemperatureSourceKind source_kind)
{
	return source_kind >= BSP_TEMPERATURE_SOURCE_UNSPECIFIED &&
		source_kind <= BSP_TEMPERATURE_SOURCE_DIGITAL;
}

static bool BspBoard_TemperatureLocationIsValid(
	BspTemperatureLocation location)
{
	return location >= BSP_TEMPERATURE_LOCATION_UNSPECIFIED &&
		location <= BSP_TEMPERATURE_LOCATION_BOARD_AMBIENT;
}

static bool BspBoard_CommunicationKindIsValid(BspCommunicationKind kind)
{
	return kind >= BSP_COMMUNICATION_UNSPECIFIED &&
		kind <= BSP_COMMUNICATION_BYTE_STREAM;
}

static BspBoardValidationResult BspBoard_ValidateMotorDescriptors(
	const BspBoardCapabilities *capabilities)
{
	const BspCurrentSenseTopologySet known_topologies =
		(UINT32_C(1) << (uint32_t)BSP_CURRENT_SENSE_TOPOLOGY_COUNT) -
		UINT32_C(1);
	size_t index;
	size_t previous;
	size_t sensor;
	size_t previous_sensor;

	for (index = 0U; index < capabilities->motor_drive_endpoint_count; ++index)
	{
		const BspMotorDriveEndpointCapabilities *endpoint =
			&capabilities->motor_drive_endpoints[index];
		BspCurrentSenseTopology topology;

		if (endpoint->endpoint_id == BSP_ENDPOINT_ID_NONE ||
			!BspBoard_AvailabilityIsValid(endpoint->availability) ||
			(endpoint->supported_current_sense_topologies &
			 ~known_topologies) != 0U ||
			endpoint->current_sensor_capacity >
				BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT ||
			(endpoint->availability == BSP_ENDPOINT_AVAILABLE &&
			 endpoint->supported_current_sense_topologies == 0U))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
				BSP_BOARD_RESOURCE_MOTOR_DRIVE, index, endpoint->endpoint_id);
		}
		for (sensor = 0U; sensor < BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT;
			++sensor)
		{
			if ((sensor < endpoint->current_sensor_capacity &&
				 endpoint->current_sensor_endpoints[sensor] ==
					BSP_ENDPOINT_ID_NONE) ||
				(sensor >= endpoint->current_sensor_capacity &&
				 endpoint->current_sensor_endpoints[sensor] !=
					BSP_ENDPOINT_ID_NONE) ||
				endpoint->current_sensor_endpoints[sensor] ==
					endpoint->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
					BSP_BOARD_RESOURCE_MOTOR_DRIVE, index,
					endpoint->current_sensor_endpoints[sensor]);
			}
			for (previous_sensor = 0U; previous_sensor < sensor;
				++previous_sensor)
			{
				if (endpoint->current_sensor_endpoints[sensor] !=
						BSP_ENDPOINT_ID_NONE &&
					endpoint->current_sensor_endpoints[sensor] ==
						endpoint->current_sensor_endpoints[previous_sensor])
				{
					return BspBoard_Result(
						BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
						BSP_BOARD_RESOURCE_MOTOR_DRIVE, index,
						endpoint->current_sensor_endpoints[sensor]);
				}
			}
		}

		for (topology = BSP_CURRENT_SENSE_NONE;
			topology < BSP_CURRENT_SENSE_TOPOLOGY_COUNT;
			topology = (BspCurrentSenseTopology)((uint32_t)topology + 1U))
		{
			if ((endpoint->supported_current_sense_topologies &
				BSP_CURRENT_SENSE_TOPOLOGY_BIT(topology)) != 0U &&
				BspBoard_CurrentSensorCountRequired(topology) >
				endpoint->current_sensor_capacity)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
					BSP_BOARD_RESOURCE_MOTOR_DRIVE, index,
					endpoint->endpoint_id);
			}
		}

		for (previous = 0U; previous < index; ++previous)
		{
			if (capabilities->motor_drive_endpoints[previous].endpoint_id ==
				endpoint->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
					BSP_BOARD_RESOURCE_MOTOR_DRIVE, index,
					endpoint->endpoint_id);
			}
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static BspBoardValidationResult BspBoard_ValidateAngleDescriptors(
	const BspBoardCapabilities *capabilities)
{
	size_t index;
	size_t previous;

	for (index = 0U; index < capabilities->angle_sensor_endpoint_count; ++index)
	{
		const BspAngleSensorEndpointCapabilities *endpoint =
			&capabilities->angle_sensor_endpoints[index];

		if (endpoint->endpoint_id == BSP_ENDPOINT_ID_NONE ||
			!BspBoard_AvailabilityIsValid(endpoint->availability) ||
			!BspBoard_AngleKindIsValid(endpoint->kind) ||
			(endpoint->availability != BSP_ENDPOINT_UNAVAILABLE &&
			 endpoint->kind == BSP_ANGLE_ENDPOINT_UNSPECIFIED))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
				BSP_BOARD_RESOURCE_ANGLE_SENSOR, index, endpoint->endpoint_id);
		}
		for (previous = 0U; previous < index; ++previous)
		{
			if (capabilities->angle_sensor_endpoints[previous].endpoint_id ==
				endpoint->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
					BSP_BOARD_RESOURCE_ANGLE_SENSOR, index,
					endpoint->endpoint_id);
			}
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static BspBoardValidationResult BspBoard_ValidateTemperatureDescriptors(
	const BspBoardCapabilities *capabilities)
{
	size_t index;
	size_t previous;

	for (index = 0U; index < capabilities->temperature_endpoint_count; ++index)
	{
		const BspTemperatureEndpointCapabilities *endpoint =
			&capabilities->temperature_endpoints[index];

		if (endpoint->endpoint_id == BSP_ENDPOINT_ID_NONE ||
			!BspBoard_AvailabilityIsValid(endpoint->availability) ||
			!BspBoard_TemperatureSourceIsValid(endpoint->source_kind) ||
			!BspBoard_TemperatureLocationIsValid(endpoint->location) ||
			(endpoint->availability != BSP_ENDPOINT_UNAVAILABLE &&
			 (endpoint->source_kind == BSP_TEMPERATURE_SOURCE_UNSPECIFIED ||
			  endpoint->location == BSP_TEMPERATURE_LOCATION_UNSPECIFIED)))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
				BSP_BOARD_RESOURCE_TEMPERATURE, index, endpoint->endpoint_id);
		}
		for (previous = 0U; previous < index; ++previous)
		{
			if (capabilities->temperature_endpoints[previous].endpoint_id ==
				endpoint->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
					BSP_BOARD_RESOURCE_TEMPERATURE, index,
					endpoint->endpoint_id);
			}
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static bool BspBoard_CommunicationDescriptorIsValid(
	const BspCommunicationEndpointCapabilities *endpoint)
{
	const BspCommunicationFeatureSet known_features =
		BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
		BSP_COMMUNICATION_FEATURE_CAN_FD |
		BSP_COMMUNICATION_FEATURE_CAN_BRS |
		BSP_COMMUNICATION_FEATURE_EXTENDED_ID |
		BSP_COMMUNICATION_FEATURE_BYTE_STREAM |
		BSP_COMMUNICATION_FEATURE_FULL_DUPLEX;

	if (endpoint->endpoint_id == BSP_ENDPOINT_ID_NONE ||
		!BspBoard_AvailabilityIsValid(endpoint->availability) ||
		endpoint->kind < BSP_COMMUNICATION_CAN ||
		endpoint->kind > BSP_COMMUNICATION_BYTE_STREAM ||
		(endpoint->features & ~known_features) != 0U)
		return false;

	if (endpoint->kind == BSP_COMMUNICATION_CAN)
	{
		return (endpoint->features & (BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
			BSP_COMMUNICATION_FEATURE_CAN_FD)) != 0U &&
			(endpoint->features & BSP_COMMUNICATION_FEATURE_BYTE_STREAM) == 0U &&
			((endpoint->features & BSP_COMMUNICATION_FEATURE_CAN_BRS) == 0U ||
			 (endpoint->features & BSP_COMMUNICATION_FEATURE_CAN_FD) != 0U);
	}
	return (endpoint->features & BSP_COMMUNICATION_FEATURE_BYTE_STREAM) != 0U &&
		(endpoint->features & (BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
		 BSP_COMMUNICATION_FEATURE_CAN_FD |
		 BSP_COMMUNICATION_FEATURE_CAN_BRS |
		 BSP_COMMUNICATION_FEATURE_EXTENDED_ID)) == 0U;
}

static BspBoardValidationResult BspBoard_ValidateCommunicationDescriptors(
	const BspBoardCapabilities *capabilities)
{
	size_t index;
	size_t previous;

	for (index = 0U; index < capabilities->communication_endpoint_count; ++index)
	{
		const BspCommunicationEndpointCapabilities *endpoint =
			&capabilities->communication_endpoints[index];

		if (!BspBoard_CommunicationDescriptorIsValid(endpoint))
			return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
				BSP_BOARD_RESOURCE_COMMUNICATION, index, endpoint->endpoint_id);
		for (previous = 0U; previous < index; ++previous)
		{
			if (capabilities->communication_endpoints[previous].endpoint_id ==
				endpoint->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
					BSP_BOARD_RESOURCE_COMMUNICATION, index,
					endpoint->endpoint_id);
			}
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

BspBoardValidationResult BspBoard_ValidateCapabilities(
	const BspBoardCapabilities *capabilities)
{
	BspBoardValidationResult result;
	const BspSystemCapabilities *system;
	const BspSystemFeatureSet known_system_features =
		BSP_SYSTEM_FEATURE_MONOTONIC_CLOCK |
		BSP_SYSTEM_FEATURE_EXECUTION_TIMER |
		BSP_SYSTEM_FEATURE_CRITICAL_SECTION |
		BSP_SYSTEM_FEATURE_UNIQUE_ID |
		BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE |
		BSP_SYSTEM_FEATURE_SOFTWARE_RESET;

	if (capabilities == NULL)
		return BspBoard_Result(BSP_BOARD_VALIDATION_NULL_ARGUMENT,
			BSP_BOARD_RESOURCE_NONE, BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			BSP_ENDPOINT_ID_NONE);
	if (capabilities->board_name == NULL || capabilities->board_id == 0U ||
		capabilities->binding_fingerprint == 0U ||
		(capabilities->motor_drive_endpoint_count > 0U &&
		 capabilities->motor_drive_endpoints == NULL) ||
		(capabilities->angle_sensor_endpoint_count > 0U &&
		 capabilities->angle_sensor_endpoints == NULL) ||
		(capabilities->temperature_endpoint_count > 0U &&
		 capabilities->temperature_endpoints == NULL) ||
		(capabilities->communication_endpoint_count > 0U &&
		 capabilities->communication_endpoints == NULL))
	{
		return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
			BSP_BOARD_RESOURCE_NONE, BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			BSP_ENDPOINT_ID_NONE);
	}

	result = BspBoard_ValidateMotorDescriptors(capabilities);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateAngleDescriptors(capabilities);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateTemperatureDescriptors(capabilities);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateCommunicationDescriptors(capabilities);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;

	system = &capabilities->system;
	if ((system->features & ~known_system_features) != 0U ||
		((system->features & BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE) != 0U &&
		 (system->nonvolatile_capacity_bytes == 0U ||
		  system->nonvolatile_erase_size_bytes == 0U ||
		  system->nonvolatile_program_alignment_bytes == 0U ||
		  system->nonvolatile_capacity_bytes %
			system->nonvolatile_erase_size_bytes != 0U ||
		  system->nonvolatile_erase_size_bytes %
			system->nonvolatile_program_alignment_bytes != 0U)))
	{
		return BspBoard_Result(BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
			BSP_BOARD_RESOURCE_SYSTEM, BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			BSP_ENDPOINT_ID_NONE);
	}

	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static const BspMotorDriveEndpointCapabilities *BspBoard_FindMotorEndpoint(
	const BspBoardCapabilities *capabilities, BspEndpointId endpoint_id)
{
	size_t index;

	for (index = 0U; index < capabilities->motor_drive_endpoint_count; ++index)
	{
		if (capabilities->motor_drive_endpoints[index].endpoint_id == endpoint_id)
			return &capabilities->motor_drive_endpoints[index];
	}
	return NULL;
}

static const BspAngleSensorEndpointCapabilities *BspBoard_FindAngleEndpoint(
	const BspBoardCapabilities *capabilities, BspEndpointId endpoint_id)
{
	size_t index;

	for (index = 0U; index < capabilities->angle_sensor_endpoint_count; ++index)
	{
		if (capabilities->angle_sensor_endpoints[index].endpoint_id == endpoint_id)
			return &capabilities->angle_sensor_endpoints[index];
	}
	return NULL;
}

static const BspTemperatureEndpointCapabilities *BspBoard_FindTemperatureEndpoint(
	const BspBoardCapabilities *capabilities, BspEndpointId endpoint_id)
{
	size_t index;

	for (index = 0U; index < capabilities->temperature_endpoint_count; ++index)
	{
		if (capabilities->temperature_endpoints[index].endpoint_id == endpoint_id)
			return &capabilities->temperature_endpoints[index];
	}
	return NULL;
}

static const BspCommunicationEndpointCapabilities *
	BspBoard_FindCommunicationEndpoint(const BspBoardCapabilities *capabilities,
		BspEndpointId endpoint_id)
{
	size_t index;

	for (index = 0U; index < capabilities->communication_endpoint_count; ++index)
	{
		if (capabilities->communication_endpoints[index].endpoint_id == endpoint_id)
			return &capabilities->communication_endpoints[index];
	}
	return NULL;
}

static BspBoardValidationResult BspBoard_ValidateMotorBinding(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request)
{
	const BspMotorDriveEndpointCapabilities *endpoint;
	uint8_t required_sensor_count;
	size_t sensor;
	size_t available_sensor;

	if (!BspBoard_CurrentSenseTopologyIsValid(request->current_sense_topology))
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_INVALID,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);

	endpoint = BspBoard_FindMotorEndpoint(capabilities,
		request->motor_drive_endpoint);
	if (endpoint == NULL)
		return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	if (endpoint->availability != BSP_ENDPOINT_AVAILABLE)
		return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	if ((endpoint->supported_current_sense_topologies &
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(request->current_sense_topology)) == 0U)
	{
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_UNSUPPORTED,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	}
	required_sensor_count = BspBoard_CurrentSensorCountRequired(
		request->current_sense_topology);
	if (required_sensor_count > endpoint->current_sensor_capacity)
	{
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_CURRENT_SENSOR_CAPACITY_INSUFFICIENT,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	}
	if (request->current_sensor_binding_count != required_sensor_count)
	{
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_CURRENT_SENSOR_BINDING_MISMATCH,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	}
	for (sensor = 0U; sensor < request->current_sensor_binding_count; ++sensor)
	{
		bool found = false;
		size_t previous_sensor;

		for (previous_sensor = 0U; previous_sensor < sensor; ++previous_sensor)
		{
			if (request->current_sensor_endpoints[previous_sensor] ==
				request->current_sensor_endpoints[sensor])
			{
				return BspBoard_Result(
					BSP_BOARD_VALIDATION_DUPLICATE_BINDING,
					BSP_BOARD_RESOURCE_MOTOR_DRIVE, sensor,
					request->current_sensor_endpoints[sensor]);
			}
		}
		for (available_sensor = 0U;
			available_sensor < endpoint->current_sensor_capacity;
			++available_sensor)
		{
			if (endpoint->current_sensor_endpoints[available_sensor] ==
				request->current_sensor_endpoints[sensor])
			{
				found = true;
				break;
			}
		}
		if (!found)
			return BspBoard_Result(
				BSP_BOARD_VALIDATION_CURRENT_SENSOR_BINDING_MISMATCH,
				BSP_BOARD_RESOURCE_MOTOR_DRIVE, sensor,
				request->current_sensor_endpoints[sensor]);
	}
	if (request->require_synchronized_current_sampling &&
		!endpoint->supports_synchronized_sampling)
	{
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_SYNCHRONIZED_SAMPLING_UNSUPPORTED,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	}
	if (request->require_hardware_shutdown &&
		!endpoint->supports_hardware_shutdown)
	{
		return BspBoard_Result(
			BSP_BOARD_VALIDATION_HARDWARE_SHUTDOWN_UNSUPPORTED,
			BSP_BOARD_RESOURCE_MOTOR_DRIVE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			request->motor_drive_endpoint);
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static BspBoardValidationResult BspBoard_ValidateAngleBindings(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request)
{
	size_t index;
	size_t previous;

	if (request->angle_binding_count > BSP_BOARD_MAX_ANGLE_BINDING_COUNT)
		return BspBoard_Result(BSP_BOARD_VALIDATION_BINDING_COUNT_EXCEEDED,
			BSP_BOARD_RESOURCE_ANGLE_SENSOR,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);

	for (index = 0U; index < request->angle_binding_count; ++index)
	{
		const BspAngleEndpointBindingRequest *binding =
			&request->angle_bindings[index];
		const BspAngleSensorEndpointCapabilities *endpoint;

		for (previous = 0U; previous < index; ++previous)
		{
			if (request->angle_bindings[previous].endpoint_id ==
				binding->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_BINDING,
					BSP_BOARD_RESOURCE_ANGLE_SENSOR, index,
					binding->endpoint_id);
			}
		}
		endpoint = BspBoard_FindAngleEndpoint(capabilities, binding->endpoint_id);
		if (endpoint == NULL)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND,
				BSP_BOARD_RESOURCE_ANGLE_SENSOR, index, binding->endpoint_id);
		if (endpoint->availability != BSP_ENDPOINT_AVAILABLE)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE,
				BSP_BOARD_RESOURCE_ANGLE_SENSOR, index, binding->endpoint_id);
		if (!BspBoard_AngleKindIsValid(binding->required_endpoint_kind) ||
			(binding->required_endpoint_kind != BSP_ANGLE_ENDPOINT_UNSPECIFIED &&
			 binding->required_endpoint_kind != endpoint->kind))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_KIND_MISMATCH,
				BSP_BOARD_RESOURCE_ANGLE_SENSOR, index, binding->endpoint_id);
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static BspBoardValidationResult BspBoard_ValidateTemperatureBindings(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request)
{
	size_t index;
	size_t previous;

	if (request->temperature_binding_count >
		BSP_BOARD_MAX_TEMPERATURE_BINDING_COUNT)
	{
		return BspBoard_Result(BSP_BOARD_VALIDATION_BINDING_COUNT_EXCEEDED,
			BSP_BOARD_RESOURCE_TEMPERATURE,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
	}

	for (index = 0U; index < request->temperature_binding_count; ++index)
	{
		const BspTemperatureEndpointBindingRequest *binding =
			&request->temperature_bindings[index];
		const BspTemperatureEndpointCapabilities *endpoint;

		for (previous = 0U; previous < index; ++previous)
		{
			if (request->temperature_bindings[previous].endpoint_id ==
				binding->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_BINDING,
					BSP_BOARD_RESOURCE_TEMPERATURE, index,
					binding->endpoint_id);
			}
		}
		endpoint = BspBoard_FindTemperatureEndpoint(capabilities,
			binding->endpoint_id);
		if (endpoint == NULL)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND,
				BSP_BOARD_RESOURCE_TEMPERATURE, index, binding->endpoint_id);
		if (endpoint->availability != BSP_ENDPOINT_AVAILABLE)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE,
				BSP_BOARD_RESOURCE_TEMPERATURE, index, binding->endpoint_id);
		if (!BspBoard_TemperatureSourceIsValid(binding->required_source_kind) ||
			(binding->required_source_kind != BSP_TEMPERATURE_SOURCE_UNSPECIFIED &&
			 binding->required_source_kind != endpoint->source_kind) ||
			!BspBoard_TemperatureLocationIsValid(binding->required_location) ||
			(binding->required_location != BSP_TEMPERATURE_LOCATION_UNSPECIFIED &&
			 binding->required_location != endpoint->location))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_KIND_MISMATCH,
				BSP_BOARD_RESOURCE_TEMPERATURE, index, binding->endpoint_id);
		}
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

static BspBoardValidationResult BspBoard_ValidateCommunicationBindings(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request)
{
	size_t index;
	size_t previous;

	if (request->communication_binding_count >
		BSP_BOARD_MAX_COMMUNICATION_BINDING_COUNT)
	{
		return BspBoard_Result(BSP_BOARD_VALIDATION_BINDING_COUNT_EXCEEDED,
			BSP_BOARD_RESOURCE_COMMUNICATION,
			BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
	}

	for (index = 0U; index < request->communication_binding_count; ++index)
	{
		const BspCommunicationEndpointBindingRequest *binding =
			&request->communication_bindings[index];
		const BspCommunicationEndpointCapabilities *endpoint;

		for (previous = 0U; previous < index; ++previous)
		{
			if (request->communication_bindings[previous].endpoint_id ==
				binding->endpoint_id)
			{
				return BspBoard_Result(BSP_BOARD_VALIDATION_DUPLICATE_BINDING,
					BSP_BOARD_RESOURCE_COMMUNICATION, index,
					binding->endpoint_id);
			}
		}
		endpoint = BspBoard_FindCommunicationEndpoint(capabilities,
			binding->endpoint_id);
		if (endpoint == NULL)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND,
				BSP_BOARD_RESOURCE_COMMUNICATION, index, binding->endpoint_id);
		if (endpoint->availability != BSP_ENDPOINT_AVAILABLE)
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE,
				BSP_BOARD_RESOURCE_COMMUNICATION, index, binding->endpoint_id);
		if (!BspBoard_CommunicationKindIsValid(binding->required_kind) ||
			(binding->required_kind != BSP_COMMUNICATION_UNSPECIFIED &&
			 binding->required_kind != endpoint->kind))
		{
			return BspBoard_Result(BSP_BOARD_VALIDATION_ENDPOINT_KIND_MISMATCH,
				BSP_BOARD_RESOURCE_COMMUNICATION, index, binding->endpoint_id);
		}
		if ((binding->required_features & ~endpoint->features) != 0U)
			return BspBoard_Result(
				BSP_BOARD_VALIDATION_ENDPOINT_FEATURE_UNSUPPORTED,
				BSP_BOARD_RESOURCE_COMMUNICATION, index, binding->endpoint_id);
	}
	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}

BspBoardValidationResult BspBoard_ValidateBindingRequest(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request)
{
	BspBoardValidationResult result;

	if (capabilities == NULL || request == NULL)
		return BspBoard_Result(BSP_BOARD_VALIDATION_NULL_ARGUMENT,
			BSP_BOARD_RESOURCE_NONE, BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			BSP_ENDPOINT_ID_NONE);

	result = BspBoard_ValidateCapabilities(capabilities);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateMotorBinding(capabilities, request);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateAngleBindings(capabilities, request);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateTemperatureBindings(capabilities, request);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	result = BspBoard_ValidateCommunicationBindings(capabilities, request);
	if (result.code != BSP_BOARD_VALIDATION_OK)
		return result;
	if ((request->required_system_features & ~capabilities->system.features) !=
		0U)
	{
		return BspBoard_Result(BSP_BOARD_VALIDATION_SYSTEM_FEATURE_UNSUPPORTED,
			BSP_BOARD_RESOURCE_SYSTEM, BSP_BOARD_VALIDATION_NO_BINDING_INDEX,
			BSP_ENDPOINT_ID_NONE);
	}

	return BspBoard_Result(BSP_BOARD_VALIDATION_OK, BSP_BOARD_RESOURCE_NONE,
		BSP_BOARD_VALIDATION_NO_BINDING_INDEX, BSP_ENDPOINT_ID_NONE);
}
