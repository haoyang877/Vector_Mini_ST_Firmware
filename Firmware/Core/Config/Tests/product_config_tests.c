#include "product_catalog.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static bool ProductConfigTests_HasError(
	const ProductConfigValidationResult *result, ProductConfigErrorCode code)
{
	uint16_t index;

	for (index = 0U; index < result->stored_error_count; index++)
	{
		if (result->issues[index].code == code)
			return true;
	}
	return false;
}

static ProductFeedbackSourceRef ProductConfigTests_NoFeedback(void)
{
	ProductFeedbackSourceRef source;

	source.kind = PRODUCT_FEEDBACK_SOURCE_NONE;
	source.angle_sensor_index = PRODUCT_CONFIG_SENSOR_INDEX_NONE;
	return source;
}

static ProductFeedbackSourceRef ProductConfigTests_AngleFeedback(uint8_t index)
{
	ProductFeedbackSourceRef source;

	source.kind = PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR;
	source.angle_sensor_index = index;
	return source;
}

static ProductFeedbackSourceRef ProductConfigTests_SensorlessFeedback(void)
{
	ProductFeedbackSourceRef source;

	source.kind = PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER;
	source.angle_sensor_index = PRODUCT_CONFIG_SENSOR_INDEX_NONE;
	return source;
}

static void ProductConfigTests_DisablePhysicalAngleSensors(
	ProductConfig *config)
{
	memset(config->angle_sensors, 0, sizeof(config->angle_sensors));
	config->angle_sensor_count = 0U;
	config->feedback.electrical_angle = ProductConfigTests_NoFeedback();
	config->feedback.motor_velocity = ProductConfigTests_NoFeedback();
	config->feedback.motor_position = ProductConfigTests_NoFeedback();
	config->feedback.output_position = ProductConfigTests_NoFeedback();
	config->feedback.calibration_reference = ProductConfigTests_NoFeedback();
	config->feedback.fallback_electrical_angle =
		ProductConfigTests_NoFeedback();
}

static void ProductConfigTests_ConfigureSensorlessOnly(ProductConfig *config)
{
	ProductConfigTests_DisablePhysicalAngleSensors(config);
	config->feedback.electrical_angle =
		ProductConfigTests_SensorlessFeedback();
	config->feedback.motor_velocity =
		ProductConfigTests_SensorlessFeedback();
	config->feedback.calibration_reference =
		ProductConfigTests_SensorlessFeedback();
	config->features.speed_control = PRODUCT_FEATURE_REQUIRED;
	config->features.position_control = PRODUCT_FEATURE_OFF;
	config->features.output_position_control = PRODUCT_FEATURE_OFF;
	config->features.sensorless_control = PRODUCT_FEATURE_REQUIRED;
	config->features.angle_redundancy_monitor = PRODUCT_FEATURE_OFF;
	config->commissioning.angle_direction = PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.angle_linearization = PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.electrical_zero = PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.mechanical_zero = PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.dual_angle_alignment =
		PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.friction_identification =
		PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.cogging_identification =
		PRODUCT_COMMISSIONING_DISABLED;
	config->commissioning.sensorless_validation =
		PRODUCT_COMMISSIONING_REQUIRED;
}

int ProductConfig_RunHostTests(void)
{
	const ProductConfig *catalog_config = ProductCatalog_GetCurrent();
	ProductConfig config;
	ProductBoardDesign board;
	ProductConfigValidationResult result;
	ProductConfigRuntimeError runtime_error;

	/* Current catalog: one physical angle sensor, internal temperature,
	 * classic CAN and low-side three-shunt current acquisition. */
	TEST_CHECK(catalog_config != 0);
	TEST_CHECK(ProductConfig_Validate(catalog_config, &result));
	TEST_CHECK(ProductConfig_ValidateRuntime(catalog_config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_OK);
	TEST_CHECK(result.total_error_count == 0U);
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_PHASE_CURRENT_FEEDBACK) != 0U);
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK) != 0U);
	TEST_CHECK((result.derived.commissioning_steps &
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION) != 0U);
	TEST_CHECK(catalog_config->can.endpoint != PRODUCT_CONFIG_ENDPOINT_NONE);
	TEST_CHECK(catalog_config->service_stream.enabled);
	TEST_CHECK(catalog_config->service_stream.endpoint !=
		PRODUCT_CONFIG_ENDPOINT_NONE);
	TEST_CHECK(!catalog_config->board->require_hardware_shutdown);

	/* Communication policy accepts both the catalog's Classic CAN profile and
	 * an explicit CAN FD profile with data-phase bit-rate switching. */
	config = *catalog_config;
	config.can.mode = PRODUCT_CAN_MODE_FD;
	config.can.data_bitrate_kbps = 5000U;
	config.can.bit_rate_switching = true;
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	config.can.maximum_payload_bytes = 65U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CAN_PAYLOAD_INVALID));

	config = *catalog_config;
	config.can.mode = PRODUCT_CAN_MODE_FD;
	config.can.data_bitrate_kbps = 5000U;
	config.can.bit_rate_switching = true;
	config.can.nominal_bitrate_kbps = UINT32_MAX;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID));

	/* Zero physical angle sensors without a sensorless route cannot satisfy
	 * the required speed and position features. */
	config = *catalog_config;
	ProductConfigTests_DisablePhysicalAngleSensors(&config);
	config.features.sensorless_control = PRODUCT_FEATURE_OFF;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_SPEED_FEEDBACK_UNAVAILABLE));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_POSITION_FEEDBACK_UNAVAILABLE));

	/* A zero-encoder product is valid when its feedback graph and feature /
	 * commissioning policies explicitly select the sensorless path. */
	config = *catalog_config;
	ProductConfigTests_ConfigureSensorlessOnly(&config);
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_SENSORLESS_OBSERVER) != 0U);
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_MOTOR_POSITION_FEEDBACK) == 0U);
	TEST_CHECK((result.derived.commissioning_steps &
		PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION) != 0U);

	/* Two installed sensors can map motor and output shaft independently. */
	config = *catalog_config;
	config.angle_sensor_count = 2U;
	config.angle_sensors[1].instance_id = 2U;
	config.angle_sensors[1].design =
		&ProductCatalog_AbsoluteSerial16BitAngleSensor;
	config.angle_sensors[1].source =
		PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL;
	config.angle_sensors[1].role = PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
	config.angle_sensors[1].endpoint = 0x0202U;
	config.feedback.output_position = ProductConfigTests_AngleFeedback(1U);
	config.features.output_position_control = PRODUCT_FEATURE_REQUIRED;
	config.commissioning.dual_angle_alignment =
		PRODUCT_COMMISSIONING_REQUIRED;
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT) != 0U);
	TEST_CHECK(result.derived.mechanical_zero_sensor_mask == 0x03U);

	/* Required thermal protection cannot silently disappear with the sensor. */
	config = *catalog_config;
	memset(config.temperature_sensors, 0,
		sizeof(config.temperature_sensors));
	config.temperature_sensor_count = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE));

	/* A motor-rotor sensor cannot be routed directly as output-shaft feedback. */
	config = *catalog_config;
	config.feedback.output_position = ProductConfigTests_AngleFeedback(0U);
	config.features.output_position_control = PRODUCT_FEATURE_REQUIRED;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_FEEDBACK_ROLE_MISMATCH));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_OUTPUT_POSITION_UNAVAILABLE));

	/* Single-shunt is a closed topology: two synchronized samples, PWM sector
	 * capture and sample-window compensation are all mandatory. */
	config = *catalog_config;
	board = *catalog_config->board;
	config.board = &board;
	board.current_sense.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT;
	board.current_sense.physical_channel_count = 1U;
	board.current_sense.channel_endpoints[1] = PRODUCT_CONFIG_ENDPOINT_NONE;
	board.current_sense.channel_endpoints[2] = PRODUCT_CONFIG_ENDPOINT_NONE;
	board.current_sense.current_a_per_count[1] = 0.0f;
	board.current_sense.current_a_per_count[2] = 0.0f;
	board.current_sense.pwm_synchronized = false;
	board.current_sense.samples_per_pwm_period = 1U;
	board.current_sense.captures_pwm_sector = false;
	board.current_sense.supports_sample_window_compensation = false;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(result.total_error_count >= 4U);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CURRENT_PWM_SYNC_REQUIRED));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_TWO_SAMPLES_REQUIRED));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_PWM_SECTOR_REQUIRED));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_WINDOW_COMPENSATION_REQUIRED));

	/* Correct acquisition requirements make the topology structurally valid;
	 * phase-resistance identification remains explicitly disabled until a
	 * topology-specific identification strategy is provided. */
	board.current_sense.pwm_synchronized = true;
	board.current_sense.samples_per_pwm_period = 2U;
	board.current_sense.captures_pwm_sector = true;
	board.current_sense.supports_sample_window_compensation = true;
	config.commissioning.phase_resistance = PRODUCT_COMMISSIONING_DISABLED;
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_SINGLE_SHUNT_RECONSTRUCTION) != 0U);
	TEST_CHECK((result.derived.capabilities &
		PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION) == 0U);

	config = *catalog_config;
	config.identity.configuration_fingerprint = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_IDENTITY);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_INVALID_IDENTITY));

	config = *catalog_config;
	config.service_stream.endpoint = PRODUCT_CONFIG_ENDPOINT_NONE;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_COMMUNICATION);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID));

	return 0;
}
