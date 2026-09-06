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
	request.communication_bindings[1].endpoint_id =
		BSP_VECTOR_MINI_ST_COMMUNICATION_ENDPOINT_SERVICE_STREAM;
	request.communication_bindings[1].required_kind =
		BSP_COMMUNICATION_BYTE_STREAM;
	request.communication_bindings[1].required_features =
		BSP_COMMUNICATION_FEATURE_BYTE_STREAM;
	request.required_system_features =
		BSP_SYSTEM_FEATURE_MONOTONIC_CLOCK |
		BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE;
	return request;
}

int BspBoard_RunHostTests(void)
{
	BspBoardBindingRequest request = ValidRequest();
	BspBoardValidationResult result;

	result = BspBoard_ValidateCapabilities(&BspVectorMiniSt_Capabilities);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_OK);

	request.require_hardware_shutdown = true;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code ==
		BSP_BOARD_VALIDATION_HARDWARE_SHUTDOWN_UNSUPPORTED);

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

	request = ValidRequest();
	request.angle_bindings[1].endpoint_id =
		request.angle_bindings[0].endpoint_id;
	result = BspVectorMiniSt_ValidateBindingRequest(&request);
	TEST_CHECK(result.code == BSP_BOARD_VALIDATION_DUPLICATE_BINDING);
	TEST_CHECK(result.resource == BSP_BOARD_RESOURCE_ANGLE_SENSOR);

	return 0;
}
