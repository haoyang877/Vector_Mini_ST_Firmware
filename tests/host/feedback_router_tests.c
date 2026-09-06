#include "feedback_router.h"

#include <stdint.h>
#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define TEST_EPSILON (0.00001f)

static FeedbackRouterSourceRef FeedbackRouterTest_NoSource(void)
{
	FeedbackRouterSourceRef source;

	source.kind = FEEDBACK_ROUTER_SOURCE_NONE;
	source.index = FEEDBACK_ROUTER_SOURCE_INDEX_NONE;
	return source;
}

static FeedbackRouterSourceRef FeedbackRouterTest_AngleSource(uint8_t index)
{
	FeedbackRouterSourceRef source;

	source.kind = FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR;
	source.index = index;
	return source;
}

static FeedbackRouterSourceRef FeedbackRouterTest_ObserverSource(void)
{
	FeedbackRouterSourceRef source;

	source.kind = FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER;
	source.index = 0U;
	return source;
}

static void FeedbackRouterTest_InitConfig(FeedbackRouterConfig *config)
{
	FeedbackRouterSourceRef no_source = FeedbackRouterTest_NoSource();

	memset(config, 0, sizeof(*config));
	config->electrical_angle = no_source;
	config->motor_velocity = no_source;
	config->motor_position = no_source;
	config->output_position = no_source;
	config->calibration_reference = no_source;
	config->fallback_electrical_angle = no_source;
}

static bool FeedbackRouterTest_Near(float left, float right)
{
	float delta = left - right;

	if (delta < 0.0f)
		delta = -delta;
	return delta <= TEST_EPSILON;
}

static float FeedbackRouterTest_Nan(void)
{
	union
	{
		uint32_t bits;
		float value;
	} result;

	result.bits = 0x7FC00000UL;
	return result.value;
}

static int FeedbackRouterTest_DisabledConfiguration(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterInput input;
	FeedbackRouterOutput output;
	FeedbackRouterValidationIssue issue;

	FeedbackRouterTest_InitConfig(&config);
	memset(&input, 0, sizeof(input));
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(issue.status == FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(FeedbackRouter_Configure(&router, &config, &issue) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_DISABLED);
	TEST_CHECK(output.configured_signals == 0U);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_DISABLED);
	TEST_CHECK(output.output_position.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_DISABLED);
	return 0;
}

static int FeedbackRouterTest_RejectsInvalidConfigurations(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterValidationIssue issue;
	FeedbackRouterStatus status;

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_count = FEEDBACK_ROUTER_MAX_ANGLE_SENSORS + 1U;
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_ANGLE_SENSOR_COUNT_EXCEEDED);

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_capabilities[1] =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION);
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK);
	TEST_CHECK(issue.source.kind == FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR);
	TEST_CHECK(issue.source.index == 1U);

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_count = 1U;
	config.angle_sensor_capabilities[0] =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION);
	config.electrical_angle = FeedbackRouterTest_AngleSource(0U);
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_SOURCE_CAPABILITY_MISMATCH);
	TEST_CHECK(issue.signal == FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);

	config.electrical_angle = FeedbackRouterTest_AngleSource(1U);
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX);

	config.electrical_angle.kind = (FeedbackRouterSourceKind)99;
	config.electrical_angle.index = 0U;
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_KIND);

	FeedbackRouterTest_InitConfig(&config);
	config.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK);

	FeedbackRouterTest_InitConfig(&config);
	config.sensorless_observer_available = true;
	config.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	config.electrical_angle = FeedbackRouterTest_ObserverSource();
	config.electrical_angle.index = 1U;
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX);

	FeedbackRouterTest_InitConfig(&config);
	config.sensorless_observer_available = true;
	config.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	config.fallback_electrical_angle = FeedbackRouterTest_ObserverSource();
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_FALLBACK_WITHOUT_PRIMARY);
	TEST_CHECK(issue.fallback_route);

	config.electrical_angle = FeedbackRouterTest_ObserverSource();
	TEST_CHECK(FeedbackRouter_ValidateConfig(&config, &issue) ==
		FEEDBACK_ROUTER_STATUS_FALLBACK_DUPLICATES_PRIMARY);

	FeedbackRouter_Reset(&router);
	FeedbackRouterTest_InitConfig(&config);
	config.electrical_angle.kind = (FeedbackRouterSourceKind)99;
	config.electrical_angle.index = 0U;
	status = FeedbackRouter_Configure(&router, &config, &issue);
	TEST_CHECK(status == FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_KIND);
	TEST_CHECK(!router.is_configured);
	return 0;
}

static int FeedbackRouterTest_ZeroSensorObserverOnly(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterInput input;
	FeedbackRouterOutput output;
	FeedbackRouterSignalMask expected_mask;

	FeedbackRouterTest_InitConfig(&config);
	config.sensorless_observer_available = true;
	config.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	config.electrical_angle = FeedbackRouterTest_ObserverSource();
	config.motor_velocity = FeedbackRouterTest_ObserverSource();
	config.calibration_reference = FeedbackRouterTest_ObserverSource();
	TEST_CHECK(FeedbackRouter_Configure(&router, &config, NULL) ==
		FEEDBACK_ROUTER_STATUS_OK);

	memset(&input, 0, sizeof(input));
	input.sensorless_observer_present = true;
	input.sensorless_observer.available = true;
	input.sensorless_observer.valid_signals =
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	input.sensorless_observer.electrical_angle_rad = 1.25f;
	input.sensorless_observer.motor_velocity_rad_s = 80.0f;
	input.sensorless_observer.calibration_reference_rad = 0.75f;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	expected_mask = FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_HEALTHY);
	TEST_CHECK(output.valid_signals == expected_mask);
	TEST_CHECK(output.fallback_signals == 0U);
	TEST_CHECK(FeedbackRouterTest_Near(output.electrical_angle.value, 1.25f));
	TEST_CHECK(FeedbackRouterTest_Near(output.motor_velocity.value, 80.0f));
	TEST_CHECK(FeedbackRouterTest_Near(
		output.calibration_reference.value, 0.75f));
	TEST_CHECK(output.electrical_angle.selected_source.kind ==
		FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER);
	TEST_CHECK(output.motor_position.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_DISABLED);
	return 0;
}

static int FeedbackRouterTest_PrimaryAndFallbackSwitching(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterInput input;
	FeedbackRouterOutput output;
	FeedbackRouterSignalMask electrical_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	FeedbackRouterSignalMask velocity_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY);

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_count = 1U;
	config.angle_sensor_capabilities[0] = electrical_mask | velocity_mask;
	config.sensorless_observer_available = true;
	config.sensorless_observer_capabilities = electrical_mask;
	config.electrical_angle = FeedbackRouterTest_AngleSource(0U);
	config.motor_velocity = FeedbackRouterTest_AngleSource(0U);
	config.fallback_electrical_angle = FeedbackRouterTest_ObserverSource();
	TEST_CHECK(FeedbackRouter_Configure(&router, &config, NULL) ==
		FEEDBACK_ROUTER_STATUS_OK);

	memset(&input, 0, sizeof(input));
	input.angle_sensor_count = 1U;
	input.angle_sensors[0].available = true;
	input.angle_sensors[0].valid_signals = electrical_mask | velocity_mask;
	input.angle_sensors[0].electrical_angle_rad = 2.0f;
	input.angle_sensors[0].motor_velocity_rad_s = 12.0f;
	input.sensorless_observer_present = true;
	input.sensorless_observer.available = true;
	input.sensorless_observer.valid_signals = electrical_mask;
	input.sensorless_observer.electrical_angle_rad = 2.1f;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_HEALTHY);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_PRIMARY);
	TEST_CHECK(FeedbackRouterTest_Near(output.electrical_angle.value, 2.0f));

	input.angle_sensors[0].valid_signals = velocity_mask;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_DEGRADED);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_FALLBACK);
	TEST_CHECK(output.electrical_angle.selected_source.kind ==
		FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER);
	TEST_CHECK(FeedbackRouterTest_Near(output.electrical_angle.value, 2.1f));
	TEST_CHECK(output.motor_velocity.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_PRIMARY);

	input.angle_sensors[0].valid_signals = electrical_mask | velocity_mask;
	input.angle_sensors[0].electrical_angle_rad = FeedbackRouterTest_Nan();
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_FALLBACK);

	input.sensorless_observer.available = false;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_UNAVAILABLE);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_UNAVAILABLE);
	TEST_CHECK((output.unavailable_signals & electrical_mask) != 0U);
	return 0;
}

static int FeedbackRouterTest_DualSensorMapping(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterInput input;
	FeedbackRouterOutput output;
	FeedbackRouterSignalMask electrical_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	FeedbackRouterSignalMask motor_position_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION);
	FeedbackRouterSignalMask output_position_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION);

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_count = 2U;
	config.angle_sensor_capabilities[0] = electrical_mask |
		motor_position_mask;
	config.angle_sensor_capabilities[1] = electrical_mask |
		output_position_mask;
	config.electrical_angle = FeedbackRouterTest_AngleSource(0U);
	config.motor_position = FeedbackRouterTest_AngleSource(0U);
	config.output_position = FeedbackRouterTest_AngleSource(1U);
	config.fallback_electrical_angle = FeedbackRouterTest_AngleSource(1U);
	TEST_CHECK(FeedbackRouter_Configure(&router, &config, NULL) ==
		FEEDBACK_ROUTER_STATUS_OK);

	memset(&input, 0, sizeof(input));
	input.angle_sensor_count = 2U;
	input.angle_sensors[0].available = true;
	input.angle_sensors[0].valid_signals = electrical_mask |
		motor_position_mask;
	input.angle_sensors[0].electrical_angle_rad = 0.2f;
	input.angle_sensors[0].motor_position_rad = 8.0f;
	input.angle_sensors[1].available = true;
	input.angle_sensors[1].valid_signals = electrical_mask |
		output_position_mask;
	input.angle_sensors[1].electrical_angle_rad = 0.25f;
	input.angle_sensors[1].output_position_rad = 1.5f;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_HEALTHY);
	TEST_CHECK(FeedbackRouterTest_Near(output.motor_position.value, 8.0f));
	TEST_CHECK(FeedbackRouterTest_Near(output.output_position.value, 1.5f));
	TEST_CHECK(output.output_position.selected_source.index == 1U);

	input.angle_sensors[0].valid_signals = motor_position_mask;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_DEGRADED);
	TEST_CHECK(output.electrical_angle.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_FALLBACK);
	TEST_CHECK(output.electrical_angle.selected_source.index == 1U);
	TEST_CHECK(FeedbackRouterTest_Near(output.electrical_angle.value, 0.25f));

	/* A not-yet-published second sensor is a runtime loss, not malformed DTO. */
	input.angle_sensor_count = 1U;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(output.output_position.state ==
		FEEDBACK_ROUTER_SIGNAL_STATE_UNAVAILABLE);
	TEST_CHECK(output.health == FEEDBACK_ROUTER_HEALTH_UNAVAILABLE);
	return 0;
}

static int FeedbackRouterTest_InputValidation(void)
{
	FeedbackRouter router;
	FeedbackRouterConfig config;
	FeedbackRouterInput input;
	FeedbackRouterOutput output;
	FeedbackRouterSignalMask electrical_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);

	FeedbackRouterTest_InitConfig(&config);
	config.angle_sensor_count = 1U;
	config.angle_sensor_capabilities[0] = electrical_mask;
	config.electrical_angle = FeedbackRouterTest_AngleSource(0U);
	TEST_CHECK(FeedbackRouter_Configure(&router, &config, NULL) ==
		FEEDBACK_ROUTER_STATUS_OK);

	memset(&input, 0, sizeof(input));
	input.angle_sensor_count = 2U;
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_INVALID_INPUT_COUNT);

	input.angle_sensor_count = 1U;
	input.angle_sensors[0].available = true;
	input.angle_sensors[0].valid_signals = (1UL << 31);
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_INVALID_INPUT_MASK);

	FeedbackRouter_Reset(&router);
	TEST_CHECK(FeedbackRouter_Process(&router, &input, &output) ==
		FEEDBACK_ROUTER_STATUS_NOT_CONFIGURED);
	return 0;
}

static int FeedbackRouterTest_DualInstanceIsolation(void)
{
	FeedbackRouter sensor_router;
	FeedbackRouter observer_router;
	FeedbackRouterConfig sensor_config;
	FeedbackRouterConfig observer_config;
	FeedbackRouterInput input;
	FeedbackRouterOutput sensor_output;
	FeedbackRouterOutput observer_output;
	FeedbackRouterSignalMask electrical_mask = FEEDBACK_ROUTER_SIGNAL_MASK(
		FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);

	FeedbackRouterTest_InitConfig(&sensor_config);
	sensor_config.angle_sensor_count = 1U;
	sensor_config.angle_sensor_capabilities[0] = electrical_mask;
	sensor_config.electrical_angle = FeedbackRouterTest_AngleSource(0U);
	TEST_CHECK(FeedbackRouter_Configure(&sensor_router, &sensor_config, NULL) ==
		FEEDBACK_ROUTER_STATUS_OK);

	FeedbackRouterTest_InitConfig(&observer_config);
	observer_config.sensorless_observer_available = true;
	observer_config.sensorless_observer_capabilities = electrical_mask;
	observer_config.electrical_angle = FeedbackRouterTest_ObserverSource();
	TEST_CHECK(FeedbackRouter_Configure(&observer_router, &observer_config,
		NULL) == FEEDBACK_ROUTER_STATUS_OK);

	memset(&input, 0, sizeof(input));
	input.angle_sensor_count = 1U;
	input.angle_sensors[0].available = true;
	input.angle_sensors[0].valid_signals = electrical_mask;
	input.angle_sensors[0].electrical_angle_rad = 0.5f;
	input.sensorless_observer_present = true;
	input.sensorless_observer.available = true;
	input.sensorless_observer.valid_signals = electrical_mask;
	input.sensorless_observer.electrical_angle_rad = 1.5f;
	TEST_CHECK(FeedbackRouter_Process(&sensor_router, &input, &sensor_output) ==
		FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(FeedbackRouter_Process(&observer_router, &input,
		&observer_output) == FEEDBACK_ROUTER_STATUS_INVALID_INPUT_COUNT);

	/* The observer-only router correctly accepts the same source data once
	 * the unrelated physical sensor is not declared as part of its input. */
	input.angle_sensor_count = 0U;
	TEST_CHECK(FeedbackRouter_Process(&observer_router, &input,
		&observer_output) == FEEDBACK_ROUTER_STATUS_OK);
	TEST_CHECK(FeedbackRouterTest_Near(sensor_output.electrical_angle.value,
		0.5f));
	TEST_CHECK(FeedbackRouterTest_Near(observer_output.electrical_angle.value,
		1.5f));

	/* Invalid reconfiguration clears only the targeted instance. */
	sensor_config.electrical_angle = FeedbackRouterTest_AngleSource(1U);
	TEST_CHECK(FeedbackRouter_Configure(&sensor_router, &sensor_config, NULL) ==
		FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX);
	TEST_CHECK(!sensor_router.is_configured);
	TEST_CHECK(observer_router.is_configured);
	TEST_CHECK(FeedbackRouter_Process(&observer_router, &input,
		&observer_output) == FEEDBACK_ROUTER_STATUS_OK);
	return 0;
}

int FeedbackRouter_RunHostTests(void)
{
	int result;

	result = FeedbackRouterTest_DisabledConfiguration();
	if (result != 0)
		return result;
	result = FeedbackRouterTest_RejectsInvalidConfigurations();
	if (result != 0)
		return result;
	result = FeedbackRouterTest_ZeroSensorObserverOnly();
	if (result != 0)
		return result;
	result = FeedbackRouterTest_PrimaryAndFallbackSwitching();
	if (result != 0)
		return result;
	result = FeedbackRouterTest_DualSensorMapping();
	if (result != 0)
		return result;
	result = FeedbackRouterTest_InputValidation();
	if (result != 0)
		return result;
	return FeedbackRouterTest_DualInstanceIsolation();
}
