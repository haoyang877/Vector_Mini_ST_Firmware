#include "bsp_product_binding.h"
#include "product_catalog.h"
#include "product_config_bridge.h"
#include "product_variant.h"
#include "vector_mini_st_bsp.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

static bool ProductConfigBridgeTests_NearlyEqual(float lhs, float rhs,
	float tolerance)
{
	float difference = lhs - rhs;

	if (difference < 0.0f)
		difference = -difference;
	return difference <= tolerance;
}

int ProductConfigBridge_RunHostTests(void)
{
	BspBoardCapabilities board;
	BspMotorDriveEndpointCapabilities motor_endpoint;
	ProductConfig invalid_config;
	ProductRuntimeSelection selection;
	ProductConfigValidationResult product_result;
	BspBoardValidationResult board_result;
	ProductConfigBridgeStatus bridge_status;
	ProductVariant runtime_product;
	const ProductConfig *config = ProductCatalog_GetCurrent();
	const float two_pi = 6.28318530717958647692f;

	TEST_CHECK(ProductVariant_GetActive(&runtime_product));
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(
		&ProductCatalog_CurrentRuntimeSelection,
		&BspVectorMiniSt_RuntimeIdentity,
		runtime_product.configuration_fingerprint, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);

	/* The legacy runtime remains active during P1. Keep every overlapping
	 * product field equal until its consumer migrates to ProductConfig. */
	TEST_CHECK(config->identity.product_id == runtime_product.identity->product_id);
	TEST_CHECK(config->identity.platform_id == runtime_product.identity->mcu_id);
	TEST_CHECK(config->identity.hardware_revision ==
		runtime_product.identity->hardware_revision);
	TEST_CHECK(config->identity.configuration_fingerprint ==
		runtime_product.configuration_fingerprint);
	TEST_CHECK(ProductCatalog_CurrentRuntimeSelection.product_id ==
		config->identity.product_id);
	TEST_CHECK(ProductCatalog_CurrentRuntimeSelection.platform_id ==
		config->identity.platform_id);
	TEST_CHECK(ProductCatalog_CurrentRuntimeSelection.board_design_id ==
		config->board->design_id);
	TEST_CHECK(ProductCatalog_CurrentRuntimeSelection.configuration_fingerprint ==
		config->identity.configuration_fingerprint);
	TEST_CHECK(ProductCatalog_CurrentRuntimeSelection.bsp_binding_fingerprint ==
		config->board->bsp_binding_fingerprint);
	TEST_CHECK(config->board->control_frequency_hz ==
		runtime_product.board->control_frequency_hz);
	TEST_CHECK(config->board->current_sense.topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT);
	TEST_CHECK(config->board->current_sense.physical_channel_count == 3U);
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->board->reliable_phase_current_limit_a,
		runtime_product.board->current_sense_reliable_limit_a, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->board->command_phase_current_limit_a,
		runtime_product.board->current_command_limit_a, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->board->calibration_phase_current_limit_a,
		runtime_product.board->calibration_current_limit_a, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->board->bus_voltage_v_per_count,
		runtime_product.board->bus_voltage_v_per_adc_count, 1.0e-8f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->board->current_sense.current_a_per_count[0],
		runtime_product.board->current_a_per_adc_count, 1.0e-8f));
	TEST_CHECK(config->board->current_sense.default_offset_count[0] ==
		runtime_product.board->default_phase_a_current_offset_adc);
	TEST_CHECK(config->board->current_sense.default_offset_count[1] ==
		runtime_product.board->default_phase_b_current_offset_adc);
	TEST_CHECK(config->board->current_sense.default_offset_count[2] ==
		runtime_product.board->default_phase_c_current_offset_adc);
	TEST_CHECK(config->motor->pole_pairs == runtime_product.motor->pole_pairs);
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->motor->phase_resistance_ohm,
		runtime_product.motor->phase_resistance_ohm, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->motor->d_axis_inductance_h,
		runtime_product.motor->d_axis_inductance_h, 1.0e-9f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->motor->q_axis_inductance_h,
		runtime_product.motor->q_axis_inductance_h, 1.0e-9f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(config->motor->flux_weber,
		runtime_product.motor->flux_weber, 1.0e-8f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(config->motor->current_limit_a,
		runtime_product.motor->current_limit_a, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->motor->calibration_current_a,
		runtime_product.motor->calibration_current_a, 1.0e-6f));
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(config->motor->speed_limit_rad_s,
		runtime_product.motor->speed_limit_rps * two_pi, 1.0e-4f));
	TEST_CHECK(config->angle_sensor_count == 1U);
	TEST_CHECK(config->angle_sensors[0].design->counts_per_turn ==
		runtime_product.encoder->counts_per_revolution);
	TEST_CHECK(ProductConfigBridgeTests_NearlyEqual(
		config->load->maximum_output_speed_rad_s,
		runtime_product.mechanical_load->default_speed_limit_rps * two_pi,
		1.0e-5f));
	TEST_CHECK(config->can.default_node_id ==
		runtime_product.board->default_can_node_id);
	TEST_CHECK(config->can.nominal_bitrate_kbps ==
		runtime_product.board->default_can_bitrate_kbps);
	TEST_CHECK(config->can.heartbeat_ms ==
		runtime_product.board->default_can_heartbeat_ms);
	TEST_CHECK((config->can.mode == PRODUCT_CAN_MODE_FD) ==
		runtime_product.board->can_fd_enabled);
	TEST_CHECK(config->can.bit_rate_switching ==
		runtime_product.board->can_brs_enabled);

	TEST_CHECK(BspProductBinding_Validate(config,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(product_result.total_error_count == 0U);
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_OK);

	invalid_config = *config;
	invalid_config.angle_sensors[0].endpoint = 0x7FFEU;
	TEST_CHECK(!BspProductBinding_Validate(&invalid_config,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND);

	board = BspVectorMiniSt_Capabilities;
	motor_endpoint = board.motor_drive_endpoints[0];
	board.motor_drive_endpoints = &motor_endpoint;
	motor_endpoint.supported_current_sense_topologies =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT);
	TEST_CHECK(!BspProductBinding_Validate(config,
		&board, &product_result, &board_result));
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_UNSUPPORTED);

	{
		BspBoardRuntimeIdentity identity = BspVectorMiniSt_RuntimeIdentity;
		identity.binding_fingerprint++;
		TEST_CHECK(!ProductConfigBridge_ValidateRuntime(
			&ProductCatalog_CurrentRuntimeSelection, &identity,
			runtime_product.configuration_fingerprint,
			&bridge_status));
		TEST_CHECK(bridge_status ==
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH);
	}

	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(
		&ProductCatalog_CurrentRuntimeSelection,
		&BspVectorMiniSt_RuntimeIdentity,
		runtime_product.configuration_fingerprint + 1U, &bridge_status));
	TEST_CHECK(bridge_status ==
		PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_MISMATCH);
	selection = ProductCatalog_CurrentRuntimeSelection;
	selection.configuration_fingerprint++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&selection,
		&BspVectorMiniSt_RuntimeIdentity,
		runtime_product.configuration_fingerprint, &bridge_status));
	TEST_CHECK(bridge_status ==
		PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_MISMATCH);
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(
		&ProductCatalog_CurrentRuntimeSelection,
		&BspVectorMiniSt_RuntimeIdentity, 0U, &bridge_status));
	TEST_CHECK(bridge_status ==
		PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_INVALID);

	return 0;
}
