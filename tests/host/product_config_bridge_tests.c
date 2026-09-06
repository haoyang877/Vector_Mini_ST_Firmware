#include "bsp_product_binding.h"
#include "product_catalog.h"
#include "Bsp/Boards/VectorMiniSt/Bootstrap/product_config_bridge.h"
#include "vector_mini_st_bsp.h"

#include <string.h>

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

static bool ProductConfigBridgeTest_ValidateRuntime(
	const ProductCatalogEntry *entry,
	const BspBoardRuntimeIdentity *board_identity,
	ProductConfigBridgeStatus *bridge_status)
{
	return ProductConfigBridge_ValidateRuntime(entry, board_identity,
		&BspVectorMiniSt_Capabilities, bridge_status);
}

#define ProductConfigBridge_ValidateRuntime \
	ProductConfigBridgeTest_ValidateRuntime

static void ProductConfigBridgeTest_ReorderCurrentChannels(
	ProductCurrentSenseConfig *current, const uint8_t order[3])
{
	ProductCurrentSenseConfig original = *current;
	uint8_t channel;

	for (channel = 0U; channel < 3U; ++channel)
	{
		uint8_t source = order[channel];

		current->channel_roles[channel] = original.channel_roles[source];
		current->channel_polarities[channel] =
			original.channel_polarities[source];
		current->channel_endpoints[channel] =
			original.channel_endpoints[source];
		current->current_a_per_count[channel] =
			original.current_a_per_count[source];
		current->default_offset_count[channel] =
			original.default_offset_count[source];
		current->minimum_valid_offset_count[channel] =
			original.minimum_valid_offset_count[source];
		current->maximum_valid_offset_count[channel] =
			original.maximum_valid_offset_count[source];
	}
}

static MotorCommissioningStageMask
	ProductConfigBridgeTest_MapDerivedCommissioningSteps(
		ProductCommissioningStepMask steps)
{
	MotorCommissioningStageMask result = 0U;

	if ((steps & PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_CURRENT_OFFSET);
	if ((steps & PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK);
	if ((steps & PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_ENCODER_DIRECTION);
	if ((steps & PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_ENCODER_LUT);
	if ((steps & (PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO |
			PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO)) != 0U)
	{
		result |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO);
	}
	if ((steps & PRODUCT_COMMISSIONING_STEP_FRICTION) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_FRICTION);
	if ((steps & PRODUCT_COMMISSIONING_STEP_COGGING) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_COGGING);
	if ((steps & PRODUCT_COMMISSIONING_STEP_SAVE) != 0U)
		result |= MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_SAVE);
	return result;
}

int ProductConfigBridge_RunHostTests(void)
{
	BspBoardCapabilities board;
	BspMotorDriveEndpointCapabilities motor_endpoint;
	BspTemperatureEndpointCapabilities temperature_endpoint;
	ProductBoardDesign product_board;
	ProductMotorDesign motor_design;
	ProductConfig candidate;
	ProductAngleSensorDesign angle_design;
	ProductTemperatureSensorDesign temperature_design;
	ProductCatalogEntry candidate_entry;
	ProductConfigValidationResult product_result;
	BspBoardValidationResult board_result;
	ProductConfigBridgeStatus bridge_status;
	const BspMotorDriveEndpointCapabilities *motor_capabilities;
	uint8_t acquisition_indices[3];
	const uint8_t reordered_channels[3] = {2U, 0U, 1U};
	BspBoardRuntimeIdentity identity = BspVectorMiniSt_RuntimeIdentity;
	ProductCurrentSenseProjection current_projection;
	RotorFeedbackRuntimeConfig feedback_projection;
	MotorControlModeMask control_mode_projection;
	ProductTemperatureRuntimeProjection temperature_projection;
	MotorCommissioningStageMask commissioning_projection;
	ProductConfigDerived derived;
	ProductCurrentSenseConfig projected_current;
	const ProductCatalogEntry *entry = ProductCatalog_GetCurrent();
	const ProductCatalogEntry *no_damper_entry = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_NO_DAMPER);
	const ProductCatalogEntry *damped_entry = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_DAMPED);
	const ProductConfig *config = entry != 0 ? entry->config : 0;

	TEST_CHECK(entry != 0 && config != 0 && no_damper_entry != 0 &&
		no_damper_entry->config != 0 && damped_entry != 0 &&
		damped_entry->config != 0);
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(0,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);
	candidate_entry = *entry;
	candidate_entry.config = 0;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);
	candidate_entry = *entry;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);
	TEST_CHECK(config->identity.configuration_fingerprint ==
		entry->manifest.configuration_fingerprint);
	TEST_CHECK(config->board->current_sense.topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT);
	TEST_CHECK(config->angle_sensor_count == 1U);
	TEST_CHECK(config->temperature_sensor_count == 1U);
	TEST_CHECK(config->temperature_sensors[0].source ==
		PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL);
	TEST_CHECK(config->temperature_sensors[0].sample_period_ms == 1U);
	TEST_CHECK(config->temperature_sensors[0].pending_timeout_ms == 2U);
	TEST_CHECK(!config->temperature_sensors[0].protection_enabled);
	TEST_CHECK(config->features.temperature_monitoring ==
		PRODUCT_FEATURE_REQUIRED);
	TEST_CHECK(config->features.temperature_protection == PRODUCT_FEATURE_OFF);
	TEST_CHECK(config->feedback.fallback_electrical_angle.kind ==
		PRODUCT_FEEDBACK_SOURCE_NONE);
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(config,
		&feedback_projection));
	TEST_CHECK(feedback_projection.primary_encoder_index == 0U);
	TEST_CHECK(feedback_projection.routing.angle_sensor_count == 1U);
	TEST_CHECK(!feedback_projection.
		use_output_position_for_position_control);
	TEST_CHECK(feedback_projection.routing.fallback_electrical_angle.kind ==
		FEEDBACK_ROUTER_SOURCE_NONE);
	TEST_CHECK(feedback_projection.routing.sensorless_observer_available);
	TEST_CHECK(ProductConfigBridge_ProjectControlModes(config,
		&feedback_projection, &control_mode_projection));
	TEST_CHECK(control_mode_projection ==
		MOTOR_CONTROL_SUPPORTED_MODE_MASK);
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(config,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	TEST_CHECK(temperature_projection.sensor_index == 0U);
	TEST_CHECK(temperature_projection.supervision_enabled);
	TEST_CHECK(!temperature_projection.protection_enabled);

	/* Product feature policy is projected into an exact command allow-list.
	 * Open-loop voltage is the only unconditional control mode. */
	candidate = *config;
	candidate.features.speed_control = PRODUCT_FEATURE_OFF;
	candidate.features.sensorless_control = PRODUCT_FEATURE_OFF;
	candidate.features.position_control = PRODUCT_FEATURE_OFF;
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(ProductConfigBridge_ProjectControlModes(&candidate,
		&feedback_projection, &control_mode_projection));
	TEST_CHECK(control_mode_projection ==
		(MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_CURRENT) |
		 MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP) |
		 MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_VQ)));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_NONE));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		(MotorControlMode)8));

	candidate = *config;
	motor_design = *config->motor;
	motor_design.flux_weber = 0.0f;
	candidate.motor = &motor_design;
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(ProductConfigBridge_ProjectControlModes(&candidate,
		&feedback_projection, &control_mode_projection));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_SENSORLESS_SPEED));
	candidate.features.sensorless_control = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(!ProductConfigBridge_ProjectControlModes(&candidate,
		&feedback_projection, &control_mode_projection));

	/* Monitoring and protection independently activate the one supported
	 * channel. Protection OFF always suppresses a trip projection. */
	candidate = *config;
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_OFF;
	candidate.features.required_monitored_temperature_zones = 0U;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	TEST_CHECK(!temperature_projection.supervision_enabled);
	candidate.features.temperature_protection = PRODUCT_FEATURE_OPTIONAL;
	candidate.temperature_sensors[0].protection_enabled = true;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	TEST_CHECK(temperature_projection.supervision_enabled);
	TEST_CHECK(temperature_projection.protection_enabled);
	candidate.features.temperature_protection = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	candidate.temperature_sensors[0].protection_enabled = false;
	TEST_CHECK(!ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	candidate.temperature_sensors[0].protection_enabled = true;
	candidate.features.required_protected_temperature_zones =
		PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU);
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	candidate.features.temperature_protection = PRODUCT_FEATURE_OFF;
	TEST_CHECK(!ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	candidate.features.temperature_protection = PRODUCT_FEATURE_OFF;
	candidate.features.required_protected_temperature_zones = 0U;
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_OPTIONAL;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	TEST_CHECK(temperature_projection.supervision_enabled);
	TEST_CHECK(!temperature_projection.protection_enabled);
	candidate.temperature_sensor_count = 0U;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	TEST_CHECK(!temperature_projection.supervision_enabled);
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(!ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));

	candidate = *config;
	board = BspVectorMiniSt_Capabilities;
	temperature_endpoint = board.temperature_endpoints[0];
	temperature_endpoint.availability = BSP_ENDPOINT_PROVISIONED_UNPOPULATED;
	board.temperature_endpoints = &temperature_endpoint;
	board.temperature_endpoint_count = 1U;
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_OPTIONAL;
	candidate.features.required_monitored_temperature_zones = 0U;
	TEST_CHECK(ProductConfigBridge_ProjectTemperature(&candidate, &board,
		&temperature_projection));
	TEST_CHECK(!temperature_projection.supervision_enabled);
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(!ProductConfigBridge_ProjectTemperature(&candidate, &board,
		&temperature_projection));

	candidate = *config;
	temperature_design = *candidate.temperature_sensors[0].design;
	temperature_design.design_id++;
	candidate.temperature_sensors[0].design = &temperature_design;
	TEST_CHECK(!ProductConfigBridge_ProjectTemperature(&candidate,
		&BspVectorMiniSt_Capabilities, &temperature_projection));
	candidate = *config;
	candidate.features.output_position_control = PRODUCT_FEATURE_OPTIONAL;
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(!feedback_projection.
		use_output_position_for_position_control);
	candidate.features.output_position_control = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(!ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(config,
		&commissioning_projection));
	TEST_CHECK(commissioning_projection ==
		MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK);
	TEST_CHECK(ProductConfig_Derive(config, &derived));
	TEST_CHECK(commissioning_projection ==
		ProductConfigBridgeTest_MapDerivedCommissioningSteps(
			derived.commissioning_steps));
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(
		no_damper_entry->config,
		&commissioning_projection));
	TEST_CHECK(commissioning_projection ==
		MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK);
	TEST_CHECK(ProductConfig_Derive(
		no_damper_entry->config, &derived));
	TEST_CHECK(commissioning_projection ==
		ProductConfigBridgeTest_MapDerivedCommissioningSteps(
			derived.commissioning_steps));
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(
		damped_entry->config,
		&commissioning_projection));
	TEST_CHECK(commissioning_projection ==
		MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK);
	TEST_CHECK(ProductConfig_Derive(
		damped_entry->config, &derived));
	TEST_CHECK(commissioning_projection ==
		ProductConfigBridgeTest_MapDerivedCommissioningSteps(
			derived.commissioning_steps));

	/* A policy may only crop the fixed workflow; disabled stages must not be
	 * reintroduced by either projection or the boot offset request. */
	candidate = *config;
	candidate.commissioning.current_offset = PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.phase_resistance = PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.angle_direction = PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.electrical_zero = PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.mechanical_zero = PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.friction_identification =
		PRODUCT_COMMISSIONING_DISABLED;
	candidate.commissioning.cogging_identification =
		PRODUCT_COMMISSIONING_DISABLED;
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	TEST_CHECK(commissioning_projection ==
		(MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_ENCODER_LUT) |
		 MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_SAVE)));
	TEST_CHECK((commissioning_projection & MOTOR_COMMISSIONING_STAGE_MASK(
		COMMISSIONING_STAGE_CURRENT_OFFSET)) == 0U);
	candidate = *config;
	memset(&candidate.commissioning, 0, sizeof(candidate.commissioning));
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	TEST_CHECK(commissioning_projection == 0U);

	/* AUTO tracks capability presence instead of becoming an unconditional
	 * production request. */
	candidate = *config;
	angle_design = *candidate.angle_sensors[0].design;
	angle_design.capabilities &=
		~PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION;
	candidate.angle_sensors[0].design = &angle_design;
	candidate.commissioning.angle_linearization = PRODUCT_COMMISSIONING_AUTO;
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	TEST_CHECK((commissioning_projection & MOTOR_COMMISSIONING_STAGE_MASK(
		COMMISSIONING_STAGE_ENCODER_LUT)) == 0U);

	candidate = *config;
	candidate.commissioning.mechanical_zero = PRODUCT_COMMISSIONING_DISABLED;
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	candidate = *config;
	candidate.commissioning.electrical_zero = PRODUCT_COMMISSIONING_DISABLED;
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	candidate = *config;
	candidate.commissioning.dual_angle_alignment = PRODUCT_COMMISSIONING_AUTO;
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	candidate = *config;
	candidate.commissioning.sensorless_validation =
		PRODUCT_COMMISSIONING_REQUIRED;
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	/* Sensorless mode availability is a feature-policy choice, even when all
	 * regular feedback routes are served by the encoder. */
	candidate = *config;
	candidate.feedback.calibration_reference = (ProductFeedbackSourceRef){
		PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U};
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(feedback_projection.routing.sensorless_observer_available);

	/* The same projection has explicit, bounded shapes for zero sensors and for
	 * a primary motor encoder plus one independent output encoder. */
	candidate = *config;
	memset(candidate.angle_sensors, 0, sizeof(candidate.angle_sensors));
	candidate.angle_sensor_count = 0U;
	candidate.feedback.electrical_angle = (ProductFeedbackSourceRef){
		PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER,
		PRODUCT_CONFIG_SENSOR_INDEX_NONE};
	candidate.feedback.motor_velocity = candidate.feedback.electrical_angle;
	candidate.feedback.motor_position = (ProductFeedbackSourceRef){
		PRODUCT_FEEDBACK_SOURCE_NONE, PRODUCT_CONFIG_SENSOR_INDEX_NONE};
	candidate.feedback.output_position = candidate.feedback.motor_position;
	candidate.feedback.calibration_reference =
		candidate.feedback.electrical_angle;
	candidate.feedback.fallback_electrical_angle =
		candidate.feedback.motor_position;
	candidate.features.position_control = PRODUCT_FEATURE_OFF;
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(feedback_projection.primary_encoder_index ==
		ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE);
	TEST_CHECK(feedback_projection.routing.angle_sensor_count == 0U);
	TEST_CHECK(ProductConfigBridge_ProjectControlModes(&candidate,
		&feedback_projection, &control_mode_projection));
	TEST_CHECK(control_mode_projection ==
		(MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_SENSORLESS_SPEED) |
		 MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP)));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_CURRENT));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_VQ));
	TEST_CHECK(!MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_SPEED));

	candidate = *config;
	candidate.angle_sensor_count = 2U;
	candidate.angle_sensors[1].instance_id = 2U;
	candidate.angle_sensors[1].design =
		&ProductCatalog_Tle5012bAngleSensor;
	candidate.angle_sensors[1].source =
		PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL;
	candidate.angle_sensors[1].role = PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
	candidate.angle_sensors[1].endpoint =
		BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_EXTERNAL;
	candidate.feedback.output_position = (ProductFeedbackSourceRef){
		PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 1U};
	candidate.features.output_position_control = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(feedback_projection.routing.angle_sensor_count == 2U);
	TEST_CHECK(feedback_projection.
		use_output_position_for_position_control);
	TEST_CHECK(feedback_projection.routing.output_position.index == 1U);
	TEST_CHECK(ProductConfigBridge_ProjectControlModes(&candidate,
		&feedback_projection, &control_mode_projection));
	TEST_CHECK(MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_POSITION_CASCADE));
	TEST_CHECK(MotorControlModeMask_Allows(control_mode_projection,
		MOTOR_CONTROL_MODE_POSITION_IMPEDANCE));
	/* The single persisted EncoderContext cannot calibrate the output sensor's
	 * LUT or mechanical zero on behalf of that sensor. */
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	candidate.commissioning.angle_linearization =
		PRODUCT_COMMISSIONING_DISABLED;
	TEST_CHECK(!ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	/* A raw/factory-aligned output model does not advertise calibration work
	 * that the one-Encoder runtime cannot execute. */
	angle_design = *candidate.angle_sensors[1].design;
	angle_design.capabilities &=
		~(PRODUCT_ANGLE_CAP_DIRECTION_CALIBRATION |
		  PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION |
		  PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION |
		  PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION);
	candidate.angle_sensors[1].design = &angle_design;
	candidate.commissioning = config->commissioning;
	TEST_CHECK(ProductConfigBridge_ProjectCommissioning(&candidate,
		&commissioning_projection));
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);

	candidate = *config;
	candidate.feedback.fallback_electrical_angle =
		(ProductFeedbackSourceRef){
			PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE};
	TEST_CHECK(!ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	candidate = *config;
	candidate.features.angle_redundancy_monitor = PRODUCT_FEATURE_OPTIONAL;
	TEST_CHECK(!ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));

	TEST_CHECK(BspProductBinding_Validate(config,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(product_result.total_error_count == 0U);
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_OK);

	/* Product declaration order is not the ADC/JDR acquisition order. The
	 * physical endpoint identity is the only valid mapping key. */
	candidate = *config;
	product_board = *config->board;
	ProductConfigBridgeTest_ReorderCurrentChannels(
		&product_board.current_sense, reordered_channels);
	candidate.board = &product_board;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	motor_capabilities = BspBoard_FindMotorDriveEndpoint(
		&BspVectorMiniSt_Capabilities,
		candidate.board->motor_drive_endpoint);
	TEST_CHECK(BspBoard_ResolveCurrentAcquisitionIndices(motor_capabilities,
		candidate.board->current_sense.channel_endpoints, 3U,
		acquisition_indices));
	TEST_CHECK(acquisition_indices[0] == 2U &&
		acquisition_indices[1] == 0U && acquisition_indices[2] == 1U);
	TEST_CHECK(ProductConfigBridge_ProjectCurrentSense(
		&candidate.board->current_sense, motor_capabilities,
		&current_projection));
	TEST_CHECK(current_projection.measurement.topology ==
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT);
	TEST_CHECK(current_projection.measurement.channels[0].acquisition_index ==
		2U);
	TEST_CHECK(current_projection.measurement.channels[1].acquisition_index ==
		0U);
	TEST_CHECK(current_projection.measurement.channels[2].acquisition_index ==
		1U);
	TEST_CHECK(current_projection.measurement.channels[0].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C);
	TEST_CHECK(current_projection.measurement.channels[1].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A);
	TEST_CHECK(current_projection.measurement.channels[2].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B);
	motor_endpoint = *motor_capabilities;
	motor_endpoint.availability = BSP_ENDPOINT_PROVISIONED_UNPOPULATED;
	TEST_CHECK(!ProductConfigBridge_ProjectCurrentSense(
		&candidate.board->current_sense, &motor_endpoint,
		&current_projection));
	motor_endpoint = *motor_capabilities;
	motor_endpoint.supports_synchronized_sampling = false;
	TEST_CHECK(!ProductConfigBridge_ProjectCurrentSense(
		&candidate.board->current_sense, &motor_endpoint,
		&current_projection));
	motor_endpoint = *motor_capabilities;
	motor_endpoint.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_DYNAMIC);
	TEST_CHECK(!ProductConfigBridge_ProjectCurrentSense(
		&candidate.board->current_sense, &motor_endpoint,
		&current_projection));

	/* Two-shunt endpoints may occupy non-compact BSP raw slots. Flash slots
	 * remain canonical A/B/C and unused phase B is the verifiable zero tuple. */
	memset(&projected_current, 0, sizeof(projected_current));
	projected_current.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT;
	projected_current.physical_channel_count = 2U;
	projected_current.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C;
	projected_current.channel_roles[1] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A;
	projected_current.channel_polarities[0] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL;
	projected_current.channel_polarities[1] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED;
	projected_current.channel_endpoints[0] =
		motor_capabilities->current_sensor_endpoints[1];
	projected_current.channel_endpoints[1] =
		motor_capabilities->current_sensor_endpoints[2];
	projected_current.current_a_per_count[0] = 0.01f;
	projected_current.current_a_per_count[1] = 0.02f;
	projected_current.default_offset_count[0] = 2001U;
	projected_current.default_offset_count[1] = 2002U;
	projected_current.minimum_valid_offset_count[0] = 1901U;
	projected_current.minimum_valid_offset_count[1] = 1902U;
	projected_current.maximum_valid_offset_count[0] = 2101U;
	projected_current.maximum_valid_offset_count[1] = 2102U;
	projected_current.pwm_synchronized = true;
	projected_current.samples_per_pwm_period = 1U;
	TEST_CHECK(!ProductConfigBridge_ProjectCurrentSense(&projected_current,
		motor_capabilities, &current_projection));
	motor_endpoint = *motor_capabilities;
	motor_endpoint.supported_current_sense_topologies =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT);
	motor_endpoint.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_FIXED);
	TEST_CHECK(ProductConfigBridge_ProjectCurrentSense(&projected_current,
		&motor_endpoint, &current_projection));
	TEST_CHECK(current_projection.measurement.topology ==
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT);
	TEST_CHECK(current_projection.measurement.channels[0].acquisition_index ==
		1U);
	TEST_CHECK(current_projection.measurement.channels[1].acquisition_index ==
		2U);
	TEST_CHECK(current_projection.measurement.channels[0].current_a_per_count ==
		0.01f);
	TEST_CHECK(current_projection.measurement.channels[1].current_a_per_count ==
		-0.02f);
	TEST_CHECK(current_projection.default_offset_adc[0] == 2002U);
	TEST_CHECK(current_projection.minimum_offset_adc[0] == 1902U);
	TEST_CHECK(current_projection.maximum_offset_adc[0] == 2102U);
	TEST_CHECK(current_projection.default_offset_adc[1] == 0U);
	TEST_CHECK(current_projection.minimum_offset_adc[1] == 0U);
	TEST_CHECK(current_projection.maximum_offset_adc[1] == 0U);
	TEST_CHECK(current_projection.default_offset_adc[2] == 2001U);

	/* One physical DC-link endpoint still resolves to its real BSP slot, while
	 * its two control observations are ordered PWM windows. */
	memset(&projected_current, 0, sizeof(projected_current));
	projected_current.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT;
	projected_current.physical_channel_count = 1U;
	projected_current.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK;
	projected_current.channel_polarities[0] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL;
	projected_current.channel_endpoints[0] =
		motor_capabilities->current_sensor_endpoints[2];
	projected_current.current_a_per_count[0] = 0.01f;
	projected_current.default_offset_count[0] = 2048U;
	projected_current.minimum_valid_offset_count[0] = 1900U;
	projected_current.maximum_valid_offset_count[0] = 2200U;
	projected_current.pwm_synchronized = true;
	projected_current.samples_per_pwm_period = 2U;
	projected_current.captures_pwm_sector = true;
	projected_current.supports_sample_window_compensation = true;
	TEST_CHECK(!ProductConfigBridge_ProjectCurrentSense(&projected_current,
		motor_capabilities, &current_projection));
	motor_endpoint = *motor_capabilities;
	motor_endpoint.supported_current_sense_topologies =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT);
	motor_endpoint.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_DYNAMIC);
	TEST_CHECK(ProductConfigBridge_ProjectCurrentSense(&projected_current,
		&motor_endpoint, &current_projection));
	TEST_CHECK(current_projection.measurement.topology ==
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT);
	TEST_CHECK(current_projection.measurement.channels[0].acquisition_index ==
		2U);
	TEST_CHECK(current_projection.measurement.channels[0].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK);
	TEST_CHECK(current_projection.default_offset_adc[0] == 2048U);
	TEST_CHECK(current_projection.default_offset_adc[1] == 0U);
	TEST_CHECK(current_projection.default_offset_adc[2] == 0U);

	/* Target startup validates exactly the transport contract that the
	 * composition root is about to instantiate. */
	candidate = *config;
	candidate.can.mode = PRODUCT_CAN_MODE_FD;
	candidate.can.data_bitrate_kbps = 5000U;
	candidate.can.bit_rate_switching = true;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	candidate.can.data_bitrate_kbps = 5001U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(product_result.total_error_count == 0U);
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_COMMUNICATION_BIT_RATE_UNSUPPORTED);

	candidate = *config;
	candidate.can.mode = PRODUCT_CAN_MODE_FD;
	candidate.can.data_bitrate_kbps = 5000U;
	candidate.can.bit_rate_switching = false;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate.can.data_bitrate_kbps = candidate.can.nominal_bitrate_kbps;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));

	candidate = *config;
	candidate.can.mode = PRODUCT_CAN_MODE_DISABLED;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate = *config;
	candidate.service_stream.enabled = false;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate = *config;
	candidate.can.maximum_payload_bytes = 9U;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate = *config;
	candidate.can.bit_rate_switching = true;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	candidate = *config;
	candidate.service_stream.endpoint = 0x7FFBU;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	/* Generic Product validation remains MCU-independent, while this target
	 * bridge rejects topologies its concrete TIM1/ADC2 adapter cannot run. */
	candidate = *config;
	product_board = *config->board;
	product_board.current_sense.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT;
	candidate.board = &product_board;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfig_ValidateRuntime(&candidate, 0));
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_UNSUPPORTED);

	candidate = *config;
	candidate.angle_sensor_count = 2U;
	candidate.angle_sensors[1].instance_id = 2U;
	candidate.angle_sensors[1].design =
		&ProductCatalog_Tle5012bAngleSensor;
	candidate.angle_sensors[1].source =
		PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL;
	candidate.angle_sensors[1].role = PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
	candidate.angle_sensors[1].endpoint = 0x0202U;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);
	TEST_CHECK(BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));

	candidate.angle_sensors[1].role =
		PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT;
	TEST_CHECK(!ProductConfigBridge_ProjectFeedback(&candidate,
		&feedback_projection));
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	candidate = *config;
	angle_design = *candidate.angle_sensors[0].design;
	angle_design.design_id++;
	candidate.angle_sensors[0].design = &angle_design;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	candidate = *config;
	candidate.temperature_sensors[0].source =
		PRODUCT_TEMPERATURE_SENSOR_SOURCE_ANALOG;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	TEST_CHECK(!ProductConfig_ValidateRuntime(&candidate, 0));

	candidate = *config;
	candidate.features.temperature_monitoring = PRODUCT_FEATURE_OFF;
	candidate.features.required_monitored_temperature_zones = 0U;
	candidate.features.temperature_protection = PRODUCT_FEATURE_OPTIONAL;
	candidate.temperature_sensors[0].protection_enabled = true;
	candidate_entry.config = &candidate;
	TEST_CHECK(ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_OK);
	candidate.features.temperature_protection =
		(ProductFeatureRequirement)(PRODUCT_FEATURE_REQUIRED + 1U);
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	/* Board identity and ProductConfig identity are checked directly; there is
	 * no compact shadow selection that can drift from the catalog. */
	identity.binding_fingerprint++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(entry, &identity,
		&bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH);
	candidate = *config;
	candidate.identity.configuration_fingerprint = 0U;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);
	candidate.identity.configuration_fingerprint =
		config->identity.configuration_fingerprint + 1U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate = *config;
	candidate.identity.configuration_schema_version++;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate = *config;
	candidate.identity.product_id++;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate = *config;
	candidate.identity.variant_id =
		candidate.identity.variant_id == PRODUCT_CATALOG_VARIANT_DAMPED ?
			PRODUCT_CATALOG_VARIANT_NO_DAMPER :
			PRODUCT_CATALOG_VARIANT_DAMPED;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate = *config;
	candidate.identity.platform_id++;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate_entry = *entry;
	candidate_entry.manifest.configuration_fingerprint++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	candidate_entry = *entry;
	candidate_entry.persistence.hardware_compatibility_id++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID);

	identity = BspVectorMiniSt_RuntimeIdentity;
	identity.board_id++;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(entry, &identity,
		&bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH);
	identity = BspVectorMiniSt_RuntimeIdentity;
	identity.board_id = 0U;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(entry, &identity,
		&bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_BOARD_INVALID);

	/* Hardware shutdown remains an explicit product requirement projected into
	 * BSP validation, not an implicit board default. */
	candidate = *config;
	product_board = *config->board;
	product_board.require_hardware_shutdown = true;
	candidate.board = &product_board;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
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
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code == BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND);

	candidate = *config;
	product_board = *config->board;
	product_board.current_sense.channel_endpoints[0] = 0x7FFDU;
	candidate.board = &product_board;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);
	TEST_CHECK(!BspProductBinding_Validate(&candidate,
		&BspVectorMiniSt_Capabilities, &product_result, &board_result));
	TEST_CHECK(board_result.code ==
		BSP_BOARD_VALIDATION_CURRENT_SENSOR_BINDING_MISMATCH);
	TEST_CHECK(!BspBoard_ResolveCurrentAcquisitionIndices(motor_capabilities,
		candidate.board->current_sense.channel_endpoints, 3U,
		acquisition_indices));

	candidate = *config;
	candidate.can.endpoint = 0x7FFCU;
	candidate_entry = *entry;
	candidate_entry.config = &candidate;
	TEST_CHECK(!ProductConfigBridge_ValidateRuntime(&candidate_entry,
		&BspVectorMiniSt_RuntimeIdentity, &bridge_status));
	TEST_CHECK(bridge_status == PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED);

	return 0;
}
