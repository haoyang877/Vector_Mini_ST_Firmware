#include "product_config.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

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

static const ProductAngleSensorInstanceConfig *
ProductConfig_GetAngleSensor(const ProductConfig *config,
	ProductFeedbackSourceRef source)
{
	if (config == NULL || source.kind !=
			PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR ||
		source.angle_sensor_index >= ProductConfig_BoundedAngleSensorCount(config))
		return NULL;
	return &config->angle_sensors[source.angle_sensor_index];
}

static bool ProductConfig_AngleSensorSupportsSignal(
	const ProductAngleSensorInstanceConfig *sensor,
	ProductFeedbackSignal signal)
{
	uint32_t required_capability;

	if (sensor == NULL || sensor->design == NULL ||
		sensor->source == PRODUCT_ANGLE_SENSOR_SOURCE_INVALID ||
		sensor->source != sensor->design->source)
		return false;

	switch (signal)
	{
		case PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE:
			required_capability = PRODUCT_ANGLE_CAP_POSITION |
				PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION;
			return ProductConfig_IsMotorRotorRole(sensor->role) &&
				(sensor->design->capabilities & required_capability) ==
				required_capability;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY:
			return ProductConfig_IsMotorRotorRole(sensor->role) &&
				(sensor->design->capabilities &
					PRODUCT_ANGLE_CAP_VELOCITY) != 0U;
		case PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION:
			return ProductConfig_IsMotorRotorRole(sensor->role) &&
				(sensor->design->capabilities &
					PRODUCT_ANGLE_CAP_POSITION) != 0U;
		case PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION:
			return sensor->role == PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT &&
				(sensor->design->capabilities &
					PRODUCT_ANGLE_CAP_POSITION) != 0U;
		case PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE:
			return ProductConfig_IsMotorRotorRole(sensor->role) &&
				(sensor->design->capabilities &
					PRODUCT_ANGLE_CAP_POSITION) != 0U;
		default:
			return false;
	}
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

static bool ProductConfig_FeedbackSourceSupportsSignal(
	const ProductConfig *config, ProductFeedbackSourceRef source,
	ProductFeedbackSignal signal)
{
	if (source.kind == PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR)
	{
		return ProductConfig_AngleSensorSupportsSignal(
			ProductConfig_GetAngleSensor(config, source), signal);
	}
	if (source.kind == PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER)
	{
		return ProductConfig_SensorlessIsStructurallyReady(config) &&
			(signal == PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE ||
			 signal == PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY ||
			 signal == PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE);
	}
	return false;
}

static uint8_t ProductConfig_PhysicalFeedbackSensorMask(
	const ProductConfig *config, ProductFeedbackSourceRef source,
	uint32_t required_capability)
{
	const ProductAngleSensorInstanceConfig *sensor =
		ProductConfig_GetAngleSensor(config, source);

	if (sensor == NULL || sensor->design == NULL ||
		(sensor->design->capabilities & required_capability) == 0U)
		return 0U;
	return (uint8_t)(1U << source.angle_sensor_index);
}

static bool ProductConfig_CommissioningEnabled(
	ProductCommissioningRequirement requirement,
	ProductCapabilityMask capabilities,
	ProductCapabilityMask required_capability)
{
	return requirement != PRODUCT_COMMISSIONING_DISABLED &&
		(capabilities & required_capability) == required_capability;
}

bool ProductConfig_Derive(const ProductConfig *config,
	ProductConfigDerived *derived)
{
	uint8_t sensor_index;
	uint8_t angle_count;
	uint8_t temperature_count;
	uint8_t motor_rotor_sensor_count = 0U;
	bool has_output_sensor = false;

	if (config == NULL || derived == NULL)
		return false;
	memset(derived, 0, sizeof(*derived));

	if (config->board != NULL &&
		ProductConfig_CurrentSenseIsStructurallyReady(
			&config->board->current_sense))
	{
		derived->capabilities |= PRODUCT_CAP_PHASE_CURRENT_FEEDBACK;
		if (config->board->current_sense.offset_calibration_supported)
			derived->capabilities |=
				PRODUCT_CAP_CURRENT_OFFSET_CALIBRATION;
		if (config->board->current_sense.topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
		{
			derived->capabilities |=
				PRODUCT_CAP_SINGLE_SHUNT_RECONSTRUCTION;
		}
	}
	if (config->board != NULL &&
		config->board->bus_voltage_measurement_available &&
		ProductConfig_IsPositiveFinite(config->board->bus_voltage_v_per_count))
	{
		derived->capabilities |= PRODUCT_CAP_BUS_VOLTAGE_FEEDBACK;
	}
	if (ProductConfig_SensorlessIsStructurallyReady(config))
		derived->capabilities |= PRODUCT_CAP_SENSORLESS_OBSERVER;

	angle_count = ProductConfig_BoundedAngleSensorCount(config);
	for (sensor_index = 0U; sensor_index < angle_count; sensor_index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[sensor_index];
		uint32_t sensor_capabilities;

		if (sensor->design == NULL || sensor->source != sensor->design->source)
			continue;
		sensor_capabilities = sensor->design->capabilities;
		if (ProductConfig_IsMotorRotorRole(sensor->role))
			motor_rotor_sensor_count++;
		else if (sensor->role == PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT)
			has_output_sensor = true;
		if ((sensor_capabilities &
			PRODUCT_ANGLE_CAP_DIRECTION_CALIBRATION) != 0U &&
			ProductConfig_IsMotorRotorRole(sensor->role))
		{
			derived->direction_sensor_mask |= (uint8_t)(1U << sensor_index);
		}
		if ((sensor_capabilities &
			PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION) != 0U)
		{
			derived->linearization_sensor_mask |=
				(uint8_t)(1U << sensor_index);
		}
	}
	if (derived->direction_sensor_mask != 0U)
		derived->capabilities |= PRODUCT_CAP_ANGLE_DIRECTION_CALIBRATION;
	if (derived->linearization_sensor_mask != 0U)
		derived->capabilities |= PRODUCT_CAP_ANGLE_LINEARIZATION_CALIBRATION;
	if (angle_count == PRODUCT_CONFIG_MAX_ANGLE_SENSORS &&
		motor_rotor_sensor_count > 0U &&
		(has_output_sensor || motor_rotor_sensor_count == 2U))
	{
		derived->capabilities |= PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT;
	}
	if (motor_rotor_sensor_count == 2U)
		derived->capabilities |= PRODUCT_CAP_ANGLE_REDUNDANCY;

	temperature_count = ProductConfig_BoundedTemperatureSensorCount(config);
	for (sensor_index = 0U; sensor_index < temperature_count; sensor_index++)
	{
		const ProductTemperatureSensorInstanceConfig *sensor =
			&config->temperature_sensors[sensor_index];
		ProductTemperatureZoneMask zone_mask;

		if (sensor->design == NULL || sensor->source != sensor->design->source ||
			sensor->zone < PRODUCT_TEMPERATURE_ZONE_MCU ||
			sensor->zone > PRODUCT_TEMPERATURE_ZONE_AMBIENT ||
			sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
			continue;
		zone_mask = PRODUCT_TEMPERATURE_ZONE_MASK(sensor->zone);
		derived->monitored_temperature_zones |= zone_mask;
		if (sensor->protection_enabled &&
			ProductConfig_IsFinite(sensor->protection_limit_c) &&
			sensor->protection_limit_c >=
				sensor->design->minimum_temperature_c &&
			sensor->protection_limit_c <=
				sensor->design->maximum_temperature_c)
		{
			derived->protected_temperature_zones |= zone_mask;
		}
	}
	if (derived->monitored_temperature_zones != 0U)
		derived->capabilities |= PRODUCT_CAP_TEMPERATURE_MONITORING;
	if (derived->protected_temperature_zones != 0U)
		derived->capabilities |= PRODUCT_CAP_TEMPERATURE_PROTECTION;

	if (ProductConfig_FeedbackSourceSupportsSignal(config,
		config->feedback.electrical_angle,
		PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE))
	{
		derived->capabilities |= PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK;
	}
	if (ProductConfig_FeedbackSourceSupportsSignal(config,
		config->feedback.motor_velocity,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY))
	{
		derived->capabilities |= PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK;
	}
	if (ProductConfig_FeedbackSourceSupportsSignal(config,
		config->feedback.motor_position,
		PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION))
	{
		derived->capabilities |= PRODUCT_CAP_MOTOR_POSITION_FEEDBACK;
	}
	if (ProductConfig_FeedbackSourceSupportsSignal(config,
		config->feedback.output_position,
		PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION))
	{
		derived->capabilities |= PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK;
	}

	derived->electrical_zero_sensor_mask =
		ProductConfig_PhysicalFeedbackSensorMask(config,
			config->feedback.electrical_angle,
			PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION);
	derived->mechanical_zero_sensor_mask =
		ProductConfig_PhysicalFeedbackSensorMask(config,
			config->feedback.motor_position,
			PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION) |
		ProductConfig_PhysicalFeedbackSensorMask(config,
			config->feedback.output_position,
			PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION);
	if (derived->electrical_zero_sensor_mask != 0U)
		derived->capabilities |= PRODUCT_CAP_ELECTRICAL_ZERO_CALIBRATION;
	if (derived->mechanical_zero_sensor_mask != 0U)
		derived->capabilities |= PRODUCT_CAP_MECHANICAL_ZERO_CALIBRATION;

	if ((derived->capabilities &
		(PRODUCT_CAP_PHASE_CURRENT_FEEDBACK |
		 PRODUCT_CAP_BUS_VOLTAGE_FEEDBACK)) ==
		(PRODUCT_CAP_PHASE_CURRENT_FEEDBACK |
		 PRODUCT_CAP_BUS_VOLTAGE_FEEDBACK) && config->board != NULL &&
		config->board->current_sense.topology !=
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		derived->capabilities |=
			PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION;
	}
	if (config->load != NULL && config->load->friction_identification_allowed &&
		(derived->capabilities &
		 (PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK |
		  PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK |
		  PRODUCT_CAP_MOTOR_POSITION_FEEDBACK)) ==
		 (PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK |
		  PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK |
		  PRODUCT_CAP_MOTOR_POSITION_FEEDBACK))
	{
		derived->capabilities |= PRODUCT_CAP_FRICTION_IDENTIFICATION;
	}
	if (config->load != NULL && config->load->cogging_identification_allowed &&
		(derived->capabilities &
		 (PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK |
		  PRODUCT_CAP_MOTOR_POSITION_FEEDBACK)) ==
		 (PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK |
		  PRODUCT_CAP_MOTOR_POSITION_FEEDBACK))
	{
		const ProductAngleSensorInstanceConfig *position_sensor =
			ProductConfig_GetAngleSensor(config,
				config->feedback.motor_position);
		if (position_sensor != NULL && position_sensor->design != NULL &&
			(position_sensor->design->capabilities &
			 PRODUCT_ANGLE_CAP_REPEATABLE_ABSOLUTE_FRAME) != 0U)
		{
			derived->capabilities |= PRODUCT_CAP_COGGING_IDENTIFICATION;
		}
	}

	if (ProductConfig_CommissioningEnabled(config->commissioning.current_offset,
		derived->capabilities, PRODUCT_CAP_CURRENT_OFFSET_CALIBRATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET;
	if (ProductConfig_CommissioningEnabled(config->commissioning.phase_resistance,
		derived->capabilities, PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE;
	if (ProductConfig_CommissioningEnabled(config->commissioning.angle_direction,
		derived->capabilities, PRODUCT_CAP_ANGLE_DIRECTION_CALIBRATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION;
	if (ProductConfig_CommissioningEnabled(
		config->commissioning.angle_linearization, derived->capabilities,
		PRODUCT_CAP_ANGLE_LINEARIZATION_CALIBRATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION;
	if (ProductConfig_CommissioningEnabled(config->commissioning.electrical_zero,
		derived->capabilities, PRODUCT_CAP_ELECTRICAL_ZERO_CALIBRATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO;
	if (ProductConfig_CommissioningEnabled(config->commissioning.mechanical_zero,
		derived->capabilities, PRODUCT_CAP_MECHANICAL_ZERO_CALIBRATION))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO;
	if (ProductConfig_CommissioningEnabled(
		config->commissioning.dual_angle_alignment, derived->capabilities,
		PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT;
	if (ProductConfig_CommissioningEnabled(
		config->commissioning.friction_identification, derived->capabilities,
		PRODUCT_CAP_FRICTION_IDENTIFICATION))
		derived->commissioning_steps |= PRODUCT_COMMISSIONING_STEP_FRICTION;
	if (ProductConfig_CommissioningEnabled(
		config->commissioning.cogging_identification, derived->capabilities,
		PRODUCT_CAP_COGGING_IDENTIFICATION))
		derived->commissioning_steps |= PRODUCT_COMMISSIONING_STEP_COGGING;
	if (ProductConfig_CommissioningEnabled(
		config->commissioning.sensorless_validation, derived->capabilities,
		PRODUCT_CAP_SENSORLESS_OBSERVER))
		derived->commissioning_steps |=
			PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION;
	if (config->commissioning.save_results !=
		PRODUCT_COMMISSIONING_DISABLED)
		derived->commissioning_steps |= PRODUCT_COMMISSIONING_STEP_SAVE;
	return true;
}
