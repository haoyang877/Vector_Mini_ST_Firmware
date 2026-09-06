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

static bool ProductConfigTests_NearlyEqual(float first, float second,
	float tolerance)
{
	float difference = first - second;

	if (difference < 0.0f)
		difference = -difference;
	return difference <= tolerance;
}

static float ProductConfigTests_QuietNaN(void)
{
	uint32_t bits = UINT32_C(0x7FC00000);
	float value;

	memcpy(&value, &bits, sizeof(value));
	return value;
}

static bool ProductConfigTests_Rejects(const ProductConfig *config,
	ProductConfigErrorCode expected_static_error,
	ProductConfigRuntimeError expected_runtime_error)
{
	ProductConfigValidationResult result;
	ProductConfigRuntimeError runtime_error;

	return !ProductConfig_Validate(config, &result) &&
		ProductConfigTests_HasError(&result, expected_static_error) &&
		!ProductConfig_ValidateRuntime(config, &runtime_error) &&
		runtime_error == expected_runtime_error;
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

static void ProductConfigTests_ClearCurrentChannel(
	ProductCurrentSenseConfig *current_sense, uint8_t channel)
{
	current_sense->channel_roles[channel] =
		PRODUCT_CURRENT_CHANNEL_ROLE_INVALID;
	current_sense->channel_polarities[channel] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_INVALID;
	current_sense->channel_endpoints[channel] = PRODUCT_CONFIG_ENDPOINT_NONE;
	current_sense->current_a_per_count[channel] = 0.0f;
	current_sense->default_offset_count[channel] = 0U;
	current_sense->minimum_valid_offset_count[channel] = 0U;
	current_sense->maximum_valid_offset_count[channel] = 0U;
}

int ProductConfig_RunHostTests(void)
{
	const ProductCatalogEntry *catalog_entry = ProductCatalog_GetCurrent();
	const ProductConfig *catalog_config = catalog_entry != 0 ?
		catalog_entry->config : 0;
	ProductConfig config;
	ProductBoardDesign board;
	ProductConfigValidationResult result;
	ProductConfigRuntimeError runtime_error;
	uint8_t index;

#if defined(PRODUCT_CATALOG_INCLUDE_ALL)
	{
		const ProductCatalogEntry *no_damper = ProductCatalog_GetByVariant(
			PRODUCT_CATALOG_VARIANT_NO_DAMPER);
		const ProductCatalogEntry *damped = ProductCatalog_GetByVariant(
			PRODUCT_CATALOG_VARIANT_DAMPED);
		TEST_CHECK(no_damper != 0 && no_damper->config != 0);
		TEST_CHECK(damped != 0 && damped->config != 0);
		TEST_CHECK(ProductConfig_Validate(no_damper->config, &result));
		TEST_CHECK(ProductConfig_ValidateRuntime(no_damper->config,
			&runtime_error));
		TEST_CHECK(ProductConfig_Validate(damped->config, &result));
		TEST_CHECK(ProductConfig_ValidateRuntime(damped->config,
			&runtime_error));
		TEST_CHECK(!no_damper->persistence.
			allow_erased_fingerprint_migration);
		TEST_CHECK(damped->persistence.allow_erased_fingerprint_migration);
	}
#endif

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

	/* ProductCatalog is the deployed board configuration source. These golden
	 * checks prevent an accidental hardware-policy change. */
	TEST_CHECK(catalog_config->board->current_sense.nominal_shunt_milliohm ==
		6U);
	for (index = 0U; index < PRODUCT_CONFIG_MAX_CURRENT_CHANNELS; index++)
	{
		TEST_CHECK(catalog_config->board->current_sense.channel_roles[index] ==
			(ProductCurrentChannelRole)(
				PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A + index));
		TEST_CHECK(
			catalog_config->board->current_sense.channel_polarities[index] ==
				PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED);
		TEST_CHECK(catalog_config->board->current_sense.default_offset_count[index] ==
			2048U);
		TEST_CHECK(catalog_config->board->current_sense.minimum_valid_offset_count[index] ==
			1948U);
		TEST_CHECK(catalog_config->board->current_sense.maximum_valid_offset_count[index] ==
			2148U);
		TEST_CHECK(ProductConfigTests_NearlyEqual(
			catalog_config->board->current_sense.current_a_per_count[index],
			0.0134310134f, 1.0e-9f));
	}
	TEST_CHECK(catalog_config->board->current_sense.offset_calibration_sample_count ==
		20000U);
	TEST_CHECK(ProductConfigTests_NearlyEqual(
		catalog_config->board->phase_resistance_path_compensation_ohm,
		0.008f, 1.0e-9f));
	TEST_CHECK(ProductConfigTests_NearlyEqual(
		catalog_config->safety.software_overcurrent_trip_a, 18.0f, 1.0e-6f));
	TEST_CHECK(ProductConfigTests_NearlyEqual(
		catalog_config->safety.undervoltage_trip_v, 10.0f, 1.0e-6f));
	TEST_CHECK(ProductConfigTests_NearlyEqual(
		catalog_config->safety.overvoltage_trip_v, 30.0f, 1.0e-6f));
	TEST_CHECK(ProductConfigTests_NearlyEqual(
		catalog_config->safety.bus_voltage_filter_alpha, 0.05f, 1.0e-7f));
	TEST_CHECK(catalog_config->safety.overcurrent_confirm_cycles == 5U);
	TEST_CHECK(catalog_config->safety.voltage_confirm_cycles == 10000U);
	TEST_CHECK(!catalog_config->safety.temperature_invalid_is_fault);
	TEST_CHECK(catalog_config->temperature_sensors[0].sample_period_ms == 1U);
	TEST_CHECK(catalog_config->temperature_sensors[0].pending_timeout_ms == 2U);
	TEST_CHECK(!catalog_config->temperature_sensors[0].protection_enabled);
	TEST_CHECK(catalog_config->features.temperature_monitoring ==
		PRODUCT_FEATURE_REQUIRED);
	TEST_CHECK(catalog_config->features.temperature_protection ==
		PRODUCT_FEATURE_OFF);
	TEST_CHECK(catalog_config->features.required_monitored_temperature_zones ==
		PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU));
	TEST_CHECK(catalog_config->features.required_protected_temperature_zones ==
		0U);
	TEST_CHECK(catalog_config->can.minimum_heartbeat_ms == 500U);
	TEST_CHECK(catalog_config->can.maximum_heartbeat_ms == 1000U);
	TEST_CHECK(catalog_config->control.speed_loop_frequency_hz == 2000U);
	TEST_CHECK(catalog_config->control.position_loop_frequency_hz == 1000U);
	TEST_CHECK(catalog_config->control.cascade_position_loop_frequency_hz ==
		5000U);

	/* Exhaustive product-policy validation rejects non-finite values, invalid
	 * ranges and defaults that cannot be represented safely by the runtime. */
	config = *catalog_config;
	config.motor_acceptance.phase_resistance_min_ohm =
		ProductConfigTests_QuietNaN();
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_MOTOR_ACCEPTANCE_RANGE_INVALID,
		PRODUCT_CONFIG_RUNTIME_MOTOR_ACCEPTANCE));

	config = *catalog_config;
	config.motor_acceptance.phase_resistance_max_ohm =
		catalog_config->motor->phase_resistance_ohm * 0.5f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_MOTOR_OUTSIDE_ACCEPTANCE,
		PRODUCT_CONFIG_RUNTIME_MOTOR_ACCEPTANCE));

	config = *catalog_config;
	config.control.speed_loop_frequency_hz = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.speed_loop_frequency_hz = 3000U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.speed_loop_frequency_hz = 1U;
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfig_ValidateRuntime(&config, &runtime_error));

	config = *catalog_config;
	config.control.position_loop_frequency_hz = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.position_loop_frequency_hz = 3000U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.cascade_position_loop_frequency_hz = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.cascade_position_loop_frequency_hz = 3000U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	board = *catalog_config->board;
	board.control_frequency_hz = 65536U;
	config = *catalog_config;
	config.board = &board;
	config.control.speed_loop_frequency_hz = 1U;
	config.control.position_loop_frequency_hz = 1U;
	config.control.cascade_position_loop_frequency_hz = 1U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.speed_kp = ProductConfigTests_QuietNaN();
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_PARAMETER_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.parameter_limits.position_kp_limit_a_per_rad = 0.0f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_LIMIT_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.default_speed_limit_rad_s =
		catalog_config->motor->speed_limit_rad_s + 1.0f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CONTROL_DEFAULT_EXCEEDS_LIMIT,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.sensorless.startup.observer_lock_ratio = 1.01f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_SENSORLESS_CONTROL_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.sensorless.speed_feedback_lpf_alpha = 0.0f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_SENSORLESS_CONTROL_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.control.position_friction.enabled = true;
	config.control.position_friction.friction_positive_current_a = -0.1f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_POSITION_FRICTION_CONTROL_INVALID,
		PRODUCT_CONFIG_RUNTIME_CONTROL));

	config = *catalog_config;
	config.commissioning_tuning.phase_resistance.test_current_low_a =
		config.commissioning_tuning.phase_resistance.test_current_high_a;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_PHASE_RESISTANCE_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.phase_resistance.ramp_time_ms = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_PHASE_RESISTANCE_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.phase_resistance.voltage_tolerance_v =
		ProductConfigTests_QuietNaN();
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_PHASE_RESISTANCE_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.phase_resistance.test_current_max_a =
		catalog_config->motor->calibration_current_a + 0.5f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_COMMISSIONING_EXCEEDS_LIMIT,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.angle.calibration_mechanical_turns = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.angle.calibration_min_samples_per_bin = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.angle.calibration_speed_error_ratio = 1.01f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.angle.calibration_sample_timeout_s = 0.0f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.angle.startup.align_current_a =
		catalog_config->motor->current_limit_a + 0.5f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_COMMISSIONING_EXCEEDS_LIMIT,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.friction.speed_point_count =
		PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT + 1U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_FRICTION_IDENTIFICATION_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.friction.speed_points_rad_s[1] =
		config.commissioning_tuning.friction.speed_points_rad_s[0];
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_FRICTION_IDENTIFICATION_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.friction.current_ratio_max = 1.01f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_FRICTION_IDENTIFICATION_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.cogging.turns = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_COGGING_IDENTIFICATION_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.cogging.minimum_samples_per_bin = 0U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_COGGING_IDENTIFICATION_TUNING_INVALID,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	config = *catalog_config;
	config.commissioning_tuning.cogging.maximum_current_a =
		catalog_config->motor->calibration_current_a + 0.5f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_COMMISSIONING_EXCEEDS_LIMIT,
		PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING));

	/* Current-sense calibration metadata must be complete for every physical
	 * channel; invalid defaults must never reach the runtime projection. */
	config = *catalog_config;
	board = *catalog_config->board;
	config.board = &board;
	board.current_sense.nominal_shunt_milliohm = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CURRENT_SHUNT_INVALID));

	board = *catalog_config->board;
	board.current_sense.minimum_valid_offset_count[1] = 2149U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_RANGE_INVALID));

	board = *catalog_config->board;
	board.current_sense.default_offset_count[2] = 2149U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_DEFAULT_INVALID));

	board = *catalog_config->board;
	board.current_sense.offset_calibration_sample_count = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_CALIBRATION_INVALID));

	/* Channel meaning and sign are explicit configuration. All validators must
	 * reject implicit, ambiguous or physically impossible mappings. */
	board = *catalog_config->board;
	board.current_sense.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_INVALID;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_INVALID,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.channel_roles[1] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_DUPLICATE,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.channel_roles[2] =
		PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_TOPOLOGY_MISMATCH,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.channel_polarities[1] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_INVALID;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_POLARITY_INVALID,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.current_a_per_count[0] = -0.0134310134f;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_SCALE_INVALID,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.channel_endpoints[2] =
		board.current_sense.channel_endpoints[0];
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_DUPLICATE,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	/* A two-shunt mapping may use any two different phases in any endpoint
	 * order. This proves the array index no longer defines phase identity. */
	board = *catalog_config->board;
	board.current_sense.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT;
	board.current_sense.physical_channel_count = 2U;
	board.current_sense.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C;
	board.current_sense.channel_roles[1] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A;
	board.current_sense.channel_polarities[0] =
		PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL;
	ProductConfigTests_ClearCurrentChannel(&board.current_sense, 2U);
	TEST_CHECK(ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfig_ValidateRuntime(&config, &runtime_error));

	board.current_sense.channel_roles[1] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_DUPLICATE,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.current_sense.topology =
		PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT;
	board.current_sense.physical_channel_count = 2U;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_UNUSED_CHANNEL_CONFIGURED,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

	board = *catalog_config->board;
	board.phase_resistance_path_compensation_ohm = -0.001f;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_DESIGN);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_BOARD_PATH_COMPENSATION_INVALID));

	/* Software protection thresholds and debounce periods are product policy,
	 * independent of the board's absolute measurable-current limit. */
	config = *catalog_config;
	config.safety.software_overcurrent_trip_a = 21.0f;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_SAFETY);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SAFETY_LIMIT_INVALID));

	config = *catalog_config;
	config.safety.bus_voltage_filter_alpha = 0.0f;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SAFETY_LIMIT_INVALID));

	config = *catalog_config;
	config.safety.voltage_confirm_cycles = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SAFETY_CONFIRMATION_INVALID));

	config = *catalog_config;
	config.temperature_sensors[0].sample_period_ms = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_TEMPERATURE_SAMPLE_PERIOD_INVALID));

	config = *catalog_config;
	config.temperature_sensors[0].pending_timeout_ms = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_TEMPERATURE_PENDING_TIMEOUT_INVALID));

	config = *catalog_config;
	config.can.default_node_id = 8U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CAN_NODE_ID_INVALID));

	config = *catalog_config;
	config.can.heartbeat_ms = 499U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_COMMUNICATION);
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID));

	config = *catalog_config;
	config.can.minimum_heartbeat_ms = 1001U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID));

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

	config = *catalog_config;
	config.can.mode = PRODUCT_CAN_MODE_FD;
	config.can.data_bitrate_kbps = 5000U;
	config.can.bit_rate_switching = false;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(!ProductConfig_ValidateRuntime(&config, &runtime_error));
	TEST_CHECK(runtime_error == PRODUCT_CONFIG_RUNTIME_COMMUNICATION);
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
		&ProductCatalog_Tle5012bAngleSensor;
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

	/* Schema 10 owns one encoder LUT.  Two motor-rotor devices which both
	 * advertise linearization must not silently share that persisted table. */
	config = *catalog_config;
	config.angle_sensor_count = 2U;
	config.angle_sensors[1].instance_id = 2U;
	config.angle_sensors[1].design =
		&ProductCatalog_Tle5012bAngleSensor;
	config.angle_sensors[1].source =
		PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL;
	config.angle_sensors[1].role =
		PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT;
	config.angle_sensors[1].endpoint = 0x0202U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_SECONDARY_ROTOR_LUT_UNSUPPORTED));

	/* Required diagnostic temperature monitoring cannot silently disappear. */
	config = *catalog_config;
	memset(config.temperature_sensors, 0,
		sizeof(config.temperature_sensors));
	config.temperature_sensor_count = 0U;
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE));
	TEST_CHECK(ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE));
	TEST_CHECK(!ProductConfigTests_HasError(&result,
		PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE));

	/* Protection is a separate, explicit product policy. Merely monitoring a
	 * zone must not be mistaken for having a safe shutdown threshold. */
	config = *catalog_config;
	config.features.temperature_protection = PRODUCT_FEATURE_REQUIRED;
	config.features.required_protected_temperature_zones =
		PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU);
	TEST_CHECK(!ProductConfig_Validate(&config, &result));
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
	board.current_sense.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK;
	ProductConfigTests_ClearCurrentChannel(&board.current_sense, 1U);
	ProductConfigTests_ClearCurrentChannel(&board.current_sense, 2U);
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

	board.current_sense.channel_roles[0] =
		PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A;
	TEST_CHECK(ProductConfigTests_Rejects(&config,
		PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_TOPOLOGY_MISMATCH,
		PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE));

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
