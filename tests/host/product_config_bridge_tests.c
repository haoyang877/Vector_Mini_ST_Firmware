#include "bsp_product_binding.h"
#include "product_catalog.h"
#include "product_config_bridge.h"
#include "vector_mini_st_bsp.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

int ProductConfigBridge_RunHostTests(void)
{
	BspBoardCapabilities board;
	BspMotorDriveEndpointCapabilities motor_endpoint;
	ProductBoardDesign product_board;
	ProductConfig candidate;
	ProductConfigValidationResult product_result;
	BspBoardValidationResult board_result;
	ProductConfigBridgeStatus bridge_status;
	BspBoardRuntimeIdentity identity = BspVectorMiniSt_RuntimeIdentity;
	const ProductConfig *config = ProductCatalog_GetCurrent();

	TEST_CHECK(ProductConfigBridge_ValidateRuntime(config,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);
	TEST_CHECK(config->identity.configuration_fingerprint == 0x9C501111UL);
	TEST_CHECK(config->board->current_sense.topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT);
	TEST_CHECK(config->angle_sensor_count == 1U);
	TEST_CHECK(config->temperature_sensor_count == 1U);
	TEST_CHECK(config->temperature_sensors[0].source ==
		PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL);

	TEST_CHECK(BspProductBinding_Validate(config,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(product_result.total_error_count == 0U);
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_OK);

	/* The generic model and BSP can express CAN FD. The deployed runtime accepts
	 * FD only with its explicit nominal/data-rate relationship. */
	candidate = *config;
	candidate.can.mode = PRODUCT_CAN_MODE_FD;
	candidate.can.data_bitrate_kbps = 5000U;
	candidate.can.bit_rate_switching = true;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	candidate.can.data_bitrate_kbps = 5001U;
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(product_result.total_error_count == 0U);
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_BIT_RATE_UNSUPPORTED);

	candidate = *config;
	candidate.can.mode = PRODUCT_CAN_MODE_FD;
	candidate.can.data_bitrate_kbps = 5000U;
	candidate.can.bit_rate_switching = false;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate.can.data_bitrate_kbps = candidate.can.nominal_bitrate_kbps;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));

	/* A topology can be structurally valid in ProductConfig while still lacking
	 * a production acquisition strategy. The bridge rejects it fail-closed. */
	candidate = *config;
	product_board = *config->board;
	product_board.current_sense.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT;
	candidate.board = &product_board;
	TEST_CHECK(ProductConfig_ValidateRuntime(&candidate, 0));
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	candidate = *config;
	candidate.angle_sensor_count = 2U;
	candidate.angle_sensors[1].instance_id = 2U;
	candidate.angle_sensors[1].design =
		&ProductCatalog_AbsoluteSerial16BitAngleSensor;
	candidate.angle_sensors[1].source =
		PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL;
	candidate.angle_sensors[1].role = PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
	candidate.angle_sensors[1].endpoint = 0x0202U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	candidate = *config;
	candidate.temperature_sensors[0].source =
		PRODUCT_TEMPERATURE_SENSOR_SOURCE_ANALOG;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	/* Board identity and ProductConfig identity are checked directly; there is
	 * no compact shadow selection that can drift from the catalog. */
	identity.binding_fingerprint++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(config, &identity,
		&bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH);
	candidate = *config;
	candidate.identity.configuration_fingerprint = 0U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);
	candidate.identity.configuration_fingerprint =
		PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT + 1U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	/* Hardware shutdown remains an explicit product requirement projected into
	 * BSP validation, not an implicit board default. */
	candidate = *config;
	product_board = *config->board;
	product_board.require_hardware_shutdown = true;
	candidate.board = &product_board;
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_HARDWARE_SHUTDOWN_UNSUPPORTED);
	board = BspVectorMiniSt_Capabilities;
	motor_endpoint = board.motor_drive_endpoints[0];
	motor_endpoint.supports_hardware_shutdown = true;
	board.motor_drive_endpoints = &motor_endpoint;
	TEST_CHECK(BspProductBinding_Validate(&candidate,
		&board, &product_result, &board_result));

	candidate = *config;
	candidate.angle_sensors[0].endpoint = 0x7FFEU;
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND);

	return 0;
}
