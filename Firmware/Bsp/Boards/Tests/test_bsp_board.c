#include <string.h>

#include "../VectorMiniSt/vector_mini_st_bsp.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

static BspBoardBindingRequest ValidRequest(void)
{
	BspBoardBindingRequest request;

	(void)memset(&request, 0, sizeof(request));
	request.motor_drive_endpoint =
		BSP_VECTOR_MINI_ST_MOTOR_DRIVE_ENDPOINT_MAIN;
	request.current_sense_topology =
		BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
	request.current_sensor_binding_count = 3U;
	request.current_sensor_endpoints[0] =
		BSP_VECTOR_MINI_ST_CURRENT_SENSOR_ENDPOINT_PHASE_A;
	request.current_sensor_endpoints[1] =
		BSP_VECTOR_MINI_ST_CURRENT_SENSOR_ENDPOINT_PHASE_B;
	request.current_sensor_endpoints[2] =
		BSP_VECTOR_MINI_ST_CURRENT_SENSOR_ENDPOINT_PHASE_C;
	request.require_synchronized_current_sampling = true;
	/* This board revision has no routed timer-break input. */
	request.require_hardware_shutdown = false;
	request.angle_binding_count = 2U;
	request.angle_bindings[0].endpoint_id =
		BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_ONBOARD;
	request.angle_bindings[0].required_endpoint_kind =
		BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL;
	request.angle_bindings[1].endpoint_id =
		BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_EXTERNAL;
	request.angle_bindings[1].required_endpoint_kind =
		BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL;
	request.temperature_binding_count = 1U;
	request.temperature_bindings[0].endpoint_id =
		BSP_VECTOR_MINI_ST_TEMPERATURE_ENDPOINT_PROCESSOR;
	request.temperature_bindings[0].required_source_kind =
		BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE;
	request.temperature_bindings[0].required_location =
		BSP_TEMPERATURE_LOCATION_PROCESSOR;
	request.communication_binding_count = 2U;
	request.communication_bindings[0].endpoint_id =
		BSP_VECTOR_MINI_ST_COMMUNICATION_ENDPOINT_FIELD_BUS;
	request.communication_bindings[0].required_kind = BSP_COMMUNICATION_CAN;
	request.communication_bindings[0].required_features =
		BSP_COMMUNICATION_FEATURE_CAN_CLASSIC;
	request.communication_bindings[0].required_payload_bytes =
		BSP_CAN_CLASSIC_MAX_DATA_LENGTH;
	request.communication_bindings[0].required_nominal_bit_rate = 1000000U;
	request.communication_bindings[0].required_data_bit_rate = 0U;
	request.communication_bindings[1].endpoint_id =
		BSP_VECTOR_MINI_ST_COMMUNICATION_ENDPOINT_SERVICE_STREAM;
	request.communication_bindings[1].required_kind =
		BSP_COMMUNICATION_BYTE_STREAM;
	request.communication_bindings[1].required_features =
		BSP_COMMUNICATION_FEATURE_BYTE_STREAM;
	request.communication_bindings[1].required_payload_bytes = 1U;
	request.required_system_features =
		BSP_SYSTEM_FEATURE_MONOTONIC_CLOCK |
		BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE;
	return request;
}

int BspBoard_RunHostTests(void)
{
	BspBoardBindingRequest request = ValidRequest();
	BspBoardValidationResult result;
	BspBoardCapabilities board;
	BspMotorDriveEndpointCapabilities motor_endpoint;
	BspTemperatureEndpointCapabilities temperature_endpoints[2];
	BspCommunicationEndpointCapabilities communication_endpoints[2];
	const BspCommunicationEndpointCapabilities *communication_endpoint;

	result = BspBoard_ValidateCapabilities(&BspVectorMiniSt_Capabilities);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);
	communication_endpoint = BspBoard_FindCommunicationEndpoint(
		&BspVectorMiniSt_Capabilities,
		BSP_VECTOR_MINI_ST_COMMUNICATION_ENDPOINT_FIELD_BUS);
	TEST_CHECK(communication_endpoint != NULL);
	TEST_CHECK(communication_endpoint->kind == BSP_COMMUNICATION_CAN);
	TEST_CHECK(BspBoard_FindCommunicationEndpoint(
		&BspVectorMiniSt_Capabilities, 0x7FFEU) == NULL);
	TEST_CHECK(BspBoard_FindCommunicationEndpoint(NULL,
		BSP_VECTOR_MINI_ST_COMMUNICATION_ENDPOINT_FIELD_BUS) == NULL);
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);

	/* The runtime supports FD+BRS framing with the protocol's 8-byte payload. */
	request.communication_bindings[0].required_features =
		BSP_COMMUNICATION_FEATURE_CAN_FD |
		BSP_COMMUNICATION_FEATURE_CAN_BRS;
	request.communication_bindings[0].required_data_bit_rate = 5000000U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);

	request.communication_bindings[0].required_payload_bytes = 9U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_PAYLOAD_UNSUPPORTED);

	request = ValidRequest();
	request.communication_bindings[0].required_nominal_bit_rate = 1000001U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_BIT_RATE_UNSUPPORTED);
	request = ValidRequest();
	request.communication_bindings[0].required_features =
		BSP_COMMUNICATION_FEATURE_CAN_FD |
		BSP_COMMUNICATION_FEATURE_CAN_BRS;
	request.communication_bindings[0].required_data_bit_rate = 5000001U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_BIT_RATE_UNSUPPORTED);

	request = ValidRequest();
	request.communication_bindings[0].required_payload_bytes = 9U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_PAYLOAD_UNSUPPORTED);

	request = ValidRequest();
	request.communication_bindings[0].required_features =
		BSP_COMMUNICATION_FEATURE_CAN_BRS;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_ENDPOINT_FEATURE_UNSUPPORTED);

	/* Descriptor metadata must describe one coherent communication kind. */
	board = BspVectorMiniSt_Capabilities;
	communication_endpoints[0] = board.communication_endpoints[0];
	communication_endpoints[1] = board.communication_endpoints[1];
	board.communication_endpoints = communication_endpoints;
	communication_endpoints[0].features |=
		BSP_COMMUNICATION_FEATURE_FULL_DUPLEX;
	result = BspBoard_ValidateCapabilities(&board);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR);

	communication_endpoints[0] =
		BspVectorMiniSt_Capabilities.communication_endpoints[0];
	communication_endpoints[0].features =
		BSP_COMMUNICATION_FEATURE_CAN_CLASSIC;
	communication_endpoints[0].maximum_payload_bytes = 9U;
	communication_endpoints[0].maximum_data_bit_rate = 0U;
	result = BspBoard_ValidateCapabilities(&board);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR);

	communication_endpoints[0].maximum_payload_bytes = 8U;
	communication_endpoints[0].maximum_data_bit_rate = 1000000U;
	result = BspBoard_ValidateCapabilities(&board);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR);

	request = ValidRequest();
	request.require_hardware_shutdown = true;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_HARDWARE_SHUTDOWN_UNSUPPORTED);
	board = BspVectorMiniSt_Capabilities;
	motor_endpoint = board.motor_drive_endpoints[0];
	motor_endpoint.supports_hardware_shutdown = true;
	board.motor_drive_endpoints = &motor_endpoint;
	result = BspBoard_ValidateBindingRequest(&board, &request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);

	request = ValidRequest();
	request.current_sense_topology = BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_UNSUPPORTED);

	request = ValidRequest();
	request.temperature_bindings[0].endpoint_id =
		BSP_VECTOR_MINI_ST_TEMPERATURE_ENDPOINT_POWER_STAGE_OPTION;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE);
	TEST_CHECK(result.resource == BSP_BOARD_RESOURCE_TEMPERATURE);

	/* Temperature is optional at the BSP binding level. */
	request = ValidRequest();
	request.temperature_binding_count = 0U;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);
	request = ValidRequest();
	request.temperature_bindings[0].endpoint_id = 0x7FFEU;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND);
	TEST_CHECK(result.resource == BSP_BOARD_RESOURCE_TEMPERATURE);

	request = ValidRequest();
	request.angle_bindings[1].endpoint_id =
		request.angle_bindings[0].endpoint_id;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_DUPLICATE_BINDING);
	TEST_CHECK(result.resource == BSP_BOARD_RESOURCE_ANGLE_SENSOR);

	/* Endpoint identifiers form one global namespace across all resources. */
	board = BspVectorMiniSt_Capabilities;
	temperature_endpoints[0] = board.temperature_endpoints[0];
	temperature_endpoints[1] = board.temperature_endpoints[1];
	temperature_endpoints[0].endpoint_id =
		board.angle_sensor_endpoints[0].endpoint_id;
	board.temperature_endpoints = temperature_endpoints;
	result = BspBoard_ValidateCapabilities(&board);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID);

	return 0;
}
