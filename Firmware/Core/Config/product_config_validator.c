#include "product_config.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

enum
{
	PRODUCT_FEATURE_DETAIL_SPEED = 1U,
	PRODUCT_FEATURE_DETAIL_POSITION,
	PRODUCT_FEATURE_DETAIL_OUTPUT_POSITION,
	PRODUCT_FEATURE_DETAIL_SENSORLESS,
	PRODUCT_FEATURE_DETAIL_ANGLE_REDUNDANCY,
	PRODUCT_FEATURE_DETAIL_TEMPERATURE_MONITORING,
	PRODUCT_FEATURE_DETAIL_TEMPERATURE_PROTECTION
};

static bool ProductConfig_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static bool ProductConfig_IsPositiveFinite(float value)
{
	return ProductConfig_IsFinite(value) && value > 0.0f;
}

static uint8_t ProductConfig_BoundedAngleSensorCount(
	const ProductConfig *config)
{
	return config->angle_sensor_count <= PRODUCT_CONFIG_MAX_ANGLE_SENSORS ?
		config->angle_sensor_count : PRODUCT_CONFIG_MAX_ANGLE_SENSORS;
}

static uint8_t ProductConfig_BoundedTemperatureSensorCount(
	const ProductConfig *config)
{
	return config->temperature_sensor_count <=
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS ?
		config->temperature_sensor_count :
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS;
}

static bool ProductConfig_IsMotorRotorRole(ProductAngleSensorRole role)
{
	return role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY ||
		role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT;
}

static uint8_t ProductConfig_CurrentSenseChannelCount(
	ProductCurrentSenseTopology topology)
{
	switch (topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			return 3U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			return 2U;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			return 1U;
		default:
			return 0U;
	}
}

static bool ProductConfig_CurrentSenseIsStructurallyReady(
	const ProductCurrentSenseConfig *current_sense)
{
	uint8_t channel;
	uint8_t other;
	uint8_t required_channel_count;

	if (current_sense == NULL)
		return false;
	required_channel_count = ProductConfig_CurrentSenseChannelCount(
		current_sense->topology);
	if (required_channel_count == 0U)
		return false;
	if (current_sense->physical_channel_count != required_channel_count ||
		!current_sense->pwm_synchronized ||
		current_sense->samples_per_pwm_period == 0U)
		return false;

	for (channel = 0U; channel < required_channel_count; channel++)
	{
		if (current_sense->channel_endpoints[channel] ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			!ProductConfig_IsPositiveFinite(
				current_sense->current_a_per_count[channel]))
			return false;
		for (other = (uint8_t)(channel + 1U);
			other < required_channel_count; other++)
		{
			if (current_sense->channel_endpoints[channel] ==
				current_sense->channel_endpoints[other])
				return false;
		}
	}

	if (current_sense->topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		return current_sense->samples_per_pwm_period >= 2U &&
			current_sense->captures_pwm_sector &&
			current_sense->supports_sample_window_compensation;
	}
	return true;
}

static bool ProductConfig_MotorSupportsObserver(
	const ProductMotorDesign *motor)
{
	return motor != NULL && motor->pole_pairs > 0U &&
		ProductConfig_IsPositiveFinite(motor->phase_resistance_ohm) &&
		ProductConfig_IsPositiveFinite(motor->d_axis_inductance_h) &&
		ProductConfig_IsPositiveFinite(motor->q_axis_inductance_h) &&
		ProductConfig_IsPositiveFinite(motor->flux_weber);
}

static bool ProductConfig_SensorlessIsStructurallyReady(
	const ProductConfig *config)
{
	return config != NULL && config->board != NULL &&
		config->board->bus_voltage_measurement_available &&
		ProductConfig_IsPositiveFinite(config->board->bus_voltage_v_per_count) &&
		ProductConfig_CurrentSenseIsStructurallyReady(
			&config->board->current_sense) &&
		ProductConfig_MotorSupportsObserver(config->motor);
}

static void ProductConfig_AddIssue(ProductConfigValidationResult *result,
	ProductConfigErrorCode code, ProductConfigValidationSubject subject,
	uint8_t instance_index, uint32_t detail)
{
	ProductConfigValidationIssue *issue;

	if (result->stored_error_count < PRODUCT_CONFIG_MAX_VALIDATION_ERRORS)
	{
		issue = &result->issues[result->stored_error_count++];
		issue->code = code;
		issue->subject = subject;
		issue->instance_index = instance_index;
		issue->detail = detail;
	}
	else
	{
		result->truncated = true;
	}
	if (result->total_error_count < UINT16_MAX)
		result->total_error_count++;
}

static bool ProductConfig_FeatureRequirementIsValid(
	ProductFeatureRequirement requirement)
{
	return requirement <= PRODUCT_FEATURE_REQUIRED;
}

static bool ProductConfig_CommissioningRequirementIsValid(
	ProductCommissioningRequirement requirement)
{
	return requirement <= PRODUCT_COMMISSIONING_REQUIRED;
}

static void ProductConfig_ValidateFeaturePolicyValue(
	ProductConfigValidationResult *result,
	ProductFeatureRequirement requirement, uint32_t feature_detail)
{
	if (!ProductConfig_FeatureRequirementIsValid(requirement))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEATURE_POLICY_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, feature_detail);
	}
}

static void ProductConfig_ValidateCommissioningPolicyValue(
	ProductConfigValidationResult *result,
	ProductCommissioningRequirement requirement,
	ProductCommissioningStepMask step)
{
	if (!ProductConfig_CommissioningRequirementIsValid(requirement))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMISSIONING_POLICY_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, step);
	}
}

static void ProductConfig_RequireCommissioningCapability(
	ProductConfigValidationResult *result,
	ProductCommissioningRequirement requirement,
	ProductCapabilityMask capabilities,
	ProductCapabilityMask required_capability,
	ProductCommissioningStepMask step)
{
	if (requirement == PRODUCT_COMMISSIONING_REQUIRED &&
		(capabilities & required_capability) != required_capability)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMISSIONING_STEP_UNSUPPORTED,
			PRODUCT_CONFIG_SUBJECT_COMMISSIONING_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, step);
	}
}

static void ProductConfig_ValidateFeedbackRoute(const ProductConfig *config,
	ProductConfigValidationResult *result, ProductFeedbackSourceRef source,
	ProductFeedbackSignal signal)
{
	const ProductAngleSensorInstanceConfig *sensor;
	uint32_t required_capability = 0U;
	bool role_matches = true;

	if (source.kind == PRODUCT_FEEDBACK_SOURCE_NONE)
		return;
	if (source.kind == PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER)
	{
		if (signal != PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE &&
			signal != PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY &&
			signal != PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		}
		else if (!ProductConfig_SensorlessIsStructurallyReady(config))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		}
		return;
	}
	if (source.kind != PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_SOURCE_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)signal);
		return;
	}
	if (source.angle_sensor_index >=
		ProductConfig_BoundedAngleSensorCount(config))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_SENSOR_INDEX_INVALID,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
		return;
	}
	sensor = &config->angle_sensors[source.angle_sensor_index];
	if (sensor->design == NULL)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
		return;
	}
	switch (signal)
	{
		case PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE:
			required_capability = PRODUCT_ANGLE_CAP_POSITION |
				PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY:
			required_capability = PRODUCT_ANGLE_CAP_VELOCITY;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		case PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = sensor->role ==
				PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT;
			break;
		case PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE:
			required_capability = PRODUCT_ANGLE_CAP_POSITION;
			role_matches = ProductConfig_IsMotorRotorRole(sensor->role);
			break;
		default:
			role_matches = false;
			break;
	}
	if (!role_matches)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_ROLE_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
	}
	if (required_capability == 0U ||
		(sensor->design->capabilities & required_capability) !=
			required_capability)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
			source.angle_sensor_index, (uint32_t)signal);
	}
}

static void ProductConfig_ValidateCurrentSense(
	const ProductCurrentSenseConfig *current_sense,
	ProductConfigValidationResult *result)
{
	uint8_t required_channel_count = 0U;
	uint8_t channel;
	uint8_t other;

	required_channel_count = ProductConfig_CurrentSenseChannelCount(
		current_sense->topology);
	if (required_channel_count == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_TOPOLOGY_INVALID,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			(uint32_t)current_sense->topology);
	}
	if (required_channel_count != 0U &&
		current_sense->physical_channel_count != required_channel_count)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_COUNT_MISMATCH,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			(uint32_t)current_sense->physical_channel_count);
	}
	for (channel = 0U; channel < required_channel_count; channel++)
	{
		if (current_sense->channel_endpoints[channel] ==
			PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
		if (!ProductConfig_IsPositiveFinite(
			current_sense->current_a_per_count[channel]))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CURRENT_SCALE_INVALID,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel, 0U);
		}
		for (other = (uint8_t)(channel + 1U);
			other < required_channel_count; other++)
		{
			if (current_sense->channel_endpoints[channel] !=
					PRODUCT_CONFIG_ENDPOINT_NONE &&
				current_sense->channel_endpoints[channel] ==
					current_sense->channel_endpoints[other])
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE, channel,
					(uint32_t)other);
			}
		}
	}
	if (!current_sense->pwm_synchronized ||
		current_sense->samples_per_pwm_period == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CURRENT_PWM_SYNC_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (current_sense->topology ==
		PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		if (current_sense->samples_per_pwm_period < 2U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_TWO_SAMPLES_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)current_sense->samples_per_pwm_period);
		}
		if (!current_sense->captures_pwm_sector)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_PWM_SECTOR_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (!current_sense->supports_sample_window_compensation)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_WINDOW_COMPENSATION_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
}

bool ProductConfig_Validate(const ProductConfig *config,
	ProductConfigValidationResult *result)
{
	uint8_t index;
	uint8_t other;
	uint8_t angle_count;
	uint8_t temperature_count;
	ProductTemperatureZoneMask zone_mask;

	if (result == NULL)
		return false;
	memset(result, 0, sizeof(*result));
	if (config == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_NULL_CONFIG,
			PRODUCT_CONFIG_SUBJECT_CONFIG,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		return false;
	}
	(void)ProductConfig_Derive(config, &result->derived);

	if (config->identity.product_id == 0U ||
		config->identity.variant_id == 0U ||
		config->identity.platform_id == 0U ||
		config->identity.configuration_fingerprint == 0U ||
		config->identity.configuration_schema_version !=
			PRODUCT_CONFIG_SCHEMA_VERSION)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_INVALID_IDENTITY,
			PRODUCT_CONFIG_SUBJECT_IDENTITY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->board == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_BOARD_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_BOARD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else
	{
		if (config->board->design_id == 0U ||
			config->board->platform_id == 0U ||
			config->board->motor_drive_endpoint == PRODUCT_CONFIG_ENDPOINT_NONE ||
			config->board->control_frequency_hz == 0U ||
			!ProductConfig_IsPositiveFinite(
				config->board->reliable_phase_current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->board->command_phase_current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->board->calibration_phase_current_limit_a) ||
			config->board->command_phase_current_limit_a >
				config->board->reliable_phase_current_limit_a ||
			config->board->calibration_phase_current_limit_a >
				config->board->reliable_phase_current_limit_a ||
			(config->board->bus_voltage_measurement_available &&
			 !ProductConfig_IsPositiveFinite(
				 config->board->bus_voltage_v_per_count)))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_INVALID_BOARD_LIMIT,
				PRODUCT_CONFIG_SUBJECT_BOARD,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->identity.platform_id != config->board->platform_id)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_PLATFORM_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_IDENTITY,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				config->board->platform_id);
		}
		ProductConfig_ValidateCurrentSense(&config->board->current_sense,
			result);
	}

	if (config->motor == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_MOTOR_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_MOTOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else
	{
		if (config->motor->design_id == 0U || config->motor->pole_pairs == 0U ||
			!ProductConfig_IsPositiveFinite(
				config->motor->phase_resistance_ohm) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->d_axis_inductance_h) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->q_axis_inductance_h) ||
			!ProductConfig_IsPositiveFinite(config->motor->flux_weber) ||
			!ProductConfig_IsPositiveFinite(config->motor->current_limit_a) ||
			!ProductConfig_IsPositiveFinite(
				config->motor->calibration_current_a) ||
			!ProductConfig_IsPositiveFinite(config->motor->speed_limit_rad_s) ||
			config->motor->calibration_current_a >
				config->motor->current_limit_a)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_INVALID_MOTOR_DESIGN,
				PRODUCT_CONFIG_SUBJECT_MOTOR,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board != NULL &&
			(config->motor->current_limit_a >
				config->board->command_phase_current_limit_a ||
			 config->motor->calibration_current_a >
				config->board->calibration_phase_current_limit_a))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_MOTOR_EXCEEDS_BOARD_LIMIT,
				PRODUCT_CONFIG_SUBJECT_MOTOR,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
	if (config->load == NULL)
	{
		ProductConfig_AddIssue(result, PRODUCT_CONFIG_ERROR_LOAD_REQUIRED,
			PRODUCT_CONFIG_SUBJECT_LOAD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	else if (config->load->design_id == 0U ||
		!ProductConfig_IsPositiveFinite(config->load->transmission_ratio) ||
		!ProductConfig_IsPositiveFinite(
			config->load->maximum_output_speed_rad_s))
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_INVALID_LOAD_DESIGN,
			PRODUCT_CONFIG_SUBJECT_LOAD,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}

	if (config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_COUNT_EXCEEDED,
			PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, config->angle_sensor_count);
	}
	angle_count = ProductConfig_BoundedAngleSensorCount(config);
	for (index = 0U; index < angle_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[index];
		if (sensor->design == NULL)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_DESIGN_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		if (sensor->instance_id == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		if (sensor->source == PRODUCT_ANGLE_SENSOR_SOURCE_INVALID ||
			(sensor->design != NULL && sensor->source != sensor->design->source))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_SOURCE_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index,
				(uint32_t)sensor->source);
		}
		if (sensor->role < PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY ||
			sensor->role > PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index,
				(uint32_t)sensor->role);
		}
		if (sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, 0U);
		}
		for (other = (uint8_t)(index + 1U); other < angle_count; other++)
		{
			if (sensor->instance_id != 0U && sensor->instance_id ==
				config->angle_sensors[other].instance_id)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, other);
			}
			if (sensor->role != PRODUCT_ANGLE_SENSOR_ROLE_INVALID &&
				sensor->role == config->angle_sensors[other].role)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR, index, other);
			}
		}
	}

	if (config->temperature_sensor_count >
		PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_COUNT_EXCEEDED,
			PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE,
			config->temperature_sensor_count);
	}
	temperature_count = ProductConfig_BoundedTemperatureSensorCount(config);
	for (index = 0U; index < temperature_count; index++)
	{
		const ProductTemperatureSensorInstanceConfig *sensor =
			&config->temperature_sensors[index];
		if (sensor->design == NULL)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_DESIGN_REQUIRED,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->instance_id == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->source == PRODUCT_TEMPERATURE_SENSOR_SOURCE_INVALID ||
			(sensor->design != NULL && sensor->source != sensor->design->source))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_SOURCE_MISMATCH,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index,
				(uint32_t)sensor->source);
		}
		if (sensor->zone < PRODUCT_TEMPERATURE_ZONE_MCU ||
			sensor->zone > PRODUCT_TEMPERATURE_ZONE_AMBIENT)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ZONE_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index,
				(uint32_t)sensor->zone);
		}
		if (sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		if (sensor->protection_enabled &&
			(sensor->design == NULL ||
			 !ProductConfig_IsFinite(sensor->protection_limit_c) ||
			 sensor->protection_limit_c <
				sensor->design->minimum_temperature_c ||
			 sensor->protection_limit_c >
				sensor->design->maximum_temperature_c))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_TEMPERATURE_LIMIT_INVALID,
				PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, 0U);
		}
		for (other = (uint8_t)(index + 1U); other < temperature_count;
			other++)
		{
			if (sensor->instance_id != 0U && sensor->instance_id ==
				config->temperature_sensors[other].instance_id)
			{
				ProductConfig_AddIssue(result,
					PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_DUPLICATE,
					PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR, index, other);
			}
		}
	}

	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.electrical_angle,
		PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.motor_velocity,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.motor_position,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.output_position,
		PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.calibration_reference,
		PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE);
	ProductConfig_ValidateFeedbackRoute(config, result,
		config->feedback.fallback_electrical_angle,
		PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE);

	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.speed_control, PRODUCT_FEATURE_DETAIL_SPEED);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.position_control, PRODUCT_FEATURE_DETAIL_POSITION);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.output_position_control,
		PRODUCT_FEATURE_DETAIL_OUTPUT_POSITION);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.sensorless_control,
		PRODUCT_FEATURE_DETAIL_SENSORLESS);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.angle_redundancy_monitor,
		PRODUCT_FEATURE_DETAIL_ANGLE_REDUNDANCY);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.temperature_monitoring,
		PRODUCT_FEATURE_DETAIL_TEMPERATURE_MONITORING);
	ProductConfig_ValidateFeaturePolicyValue(result,
		config->features.temperature_protection,
		PRODUCT_FEATURE_DETAIL_TEMPERATURE_PROTECTION);

	if (config->features.speed_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_SPEED_FEEDBACK_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.position_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 (PRODUCT_CAP_MOTOR_POSITION_FEEDBACK |
		  PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK)) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_POSITION_FEEDBACK_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.output_position_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_OUTPUT_POSITION_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.sensorless_control == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities & PRODUCT_CAP_SENSORLESS_OBSERVER) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_SENSORLESS_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.angle_redundancy_monitor ==
			PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities & PRODUCT_CAP_ANGLE_REDUNDANCY) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_REDUNDANCY_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.temperature_monitoring == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_TEMPERATURE_MONITORING) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	if (config->features.temperature_protection == PRODUCT_FEATURE_REQUIRED &&
		(result->derived.capabilities &
		 PRODUCT_CAP_TEMPERATURE_PROTECTION) == 0U)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE,
			PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
	}
	for (index = PRODUCT_TEMPERATURE_ZONE_MCU;
		index <= PRODUCT_TEMPERATURE_ZONE_AMBIENT; index++)
	{
		zone_mask = PRODUCT_TEMPERATURE_ZONE_MASK(index);
		if ((config->features.required_temperature_zones & zone_mask) != 0U &&
			(result->derived.protected_temperature_zones & zone_mask) == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE,
				PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY, index, zone_mask);
		}
	}

#define VALIDATE_COMMISSIONING_POLICY(field, step) \
	ProductConfig_ValidateCommissioningPolicyValue(result, \
		config->commissioning.field, step)
	VALIDATE_COMMISSIONING_POLICY(current_offset,
		PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET);
	VALIDATE_COMMISSIONING_POLICY(phase_resistance,
		PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE);
	VALIDATE_COMMISSIONING_POLICY(angle_direction,
		PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION);
	VALIDATE_COMMISSIONING_POLICY(angle_linearization,
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION);
	VALIDATE_COMMISSIONING_POLICY(electrical_zero,
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO);
	VALIDATE_COMMISSIONING_POLICY(mechanical_zero,
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO);
	VALIDATE_COMMISSIONING_POLICY(dual_angle_alignment,
		PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT);
	VALIDATE_COMMISSIONING_POLICY(friction_identification,
		PRODUCT_COMMISSIONING_STEP_FRICTION);
	VALIDATE_COMMISSIONING_POLICY(cogging_identification,
		PRODUCT_COMMISSIONING_STEP_COGGING);
	VALIDATE_COMMISSIONING_POLICY(sensorless_validation,
		PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION);
	VALIDATE_COMMISSIONING_POLICY(save_results,
		PRODUCT_COMMISSIONING_STEP_SAVE);
#undef VALIDATE_COMMISSIONING_POLICY

#define REQUIRE_COMMISSIONING(field, capability, step) \
	ProductConfig_RequireCommissioningCapability(result, \
		config->commissioning.field, result->derived.capabilities, \
		capability, step)
	REQUIRE_COMMISSIONING(current_offset,
		PRODUCT_CAP_CURRENT_OFFSET_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET);
	REQUIRE_COMMISSIONING(phase_resistance,
		PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE);
	REQUIRE_COMMISSIONING(angle_direction,
		PRODUCT_CAP_ANGLE_DIRECTION_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION);
	REQUIRE_COMMISSIONING(angle_linearization,
		PRODUCT_CAP_ANGLE_LINEARIZATION_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION);
	REQUIRE_COMMISSIONING(electrical_zero,
		PRODUCT_CAP_ELECTRICAL_ZERO_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO);
	REQUIRE_COMMISSIONING(mechanical_zero,
		PRODUCT_CAP_MECHANICAL_ZERO_CALIBRATION,
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO);
	REQUIRE_COMMISSIONING(dual_angle_alignment,
		PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT,
		PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT);
	REQUIRE_COMMISSIONING(friction_identification,
		PRODUCT_CAP_FRICTION_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_FRICTION);
	REQUIRE_COMMISSIONING(cogging_identification,
		PRODUCT_CAP_COGGING_IDENTIFICATION,
		PRODUCT_COMMISSIONING_STEP_COGGING);
	REQUIRE_COMMISSIONING(sensorless_validation,
		PRODUCT_CAP_SENSORLESS_OBSERVER,
		PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION);
#undef REQUIRE_COMMISSIONING

	if (config->can.mode > PRODUCT_CAN_MODE_FD)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_CAN_MODE_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, (uint32_t)config->can.mode);
	}
	else if (config->can.mode == PRODUCT_CAN_MODE_CLASSIC)
	{
		if (config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board == NULL || !config->board->classic_can_supported)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)config->can.mode);
		}
		if (config->can.nominal_bitrate_kbps == 0U ||
			config->can.data_bitrate_kbps != 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->can.bit_rate_switching)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BRS_REQUIRES_FD,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
	else if (config->can.mode == PRODUCT_CAN_MODE_FD)
	{
		if (config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
		if (config->board == NULL || !config->board->can_fd_supported ||
			(config->can.bit_rate_switching &&
			 !config->board->can_brs_supported))
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE,
				(uint32_t)config->can.mode);
		}
		if (config->can.nominal_bitrate_kbps == 0U ||
			config->can.data_bitrate_kbps == 0U)
		{
			ProductConfig_AddIssue(result,
				PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID,
				PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
				PRODUCT_CONFIG_SENSOR_INDEX_NONE, 0U);
		}
	}
	if (config->service_stream.enabled &&
		config->service_stream.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
	{
		ProductConfig_AddIssue(result,
			PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
			PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
			PRODUCT_CONFIG_SENSOR_INDEX_NONE, 1U);
	}

	return result->total_error_count == 0U;
}
