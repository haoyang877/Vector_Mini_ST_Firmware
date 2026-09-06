#include "Bsp/Boards/VectorMiniSt/Bootstrap/product_config_bridge.h"

#include <math.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static bool ProductConfigBridge_EntryIsConsistent(
	const ProductCatalogEntry *entry)
{
	const ProductConfig *config = entry != 0 ? entry->config : 0;

	return config != 0 && config->board != 0 && config->motor != 0 &&
		config->load != 0 && config->identity.product_id != 0U &&
		config->identity.variant_id == PRODUCT_CATALOG_ACTIVE_VARIANT &&
		config->identity.platform_id != 0U &&
		config->identity.configuration_fingerprint != 0U &&
		config->identity.configuration_schema_version ==
			PRODUCT_CONFIG_SCHEMA_VERSION &&
		config->identity.platform_id == config->board->platform_id &&
		entry->manifest.product_id == config->identity.product_id &&
		entry->manifest.mcu_id == config->identity.platform_id &&
		entry->manifest.hardware_revision ==
			config->identity.hardware_revision &&
		entry->manifest.configuration_fingerprint ==
			config->identity.configuration_fingerprint &&
		entry->manifest.parameter_schema_version ==
			PRODUCT_PARAMETER_SCHEMA_VERSION &&
		entry->manifest.hardware_profile_id ==
			entry->persistence.hardware_compatibility_id &&
		entry->manifest.motor_profile_id ==
			entry->persistence.motor_compatibility_id &&
		entry->manifest.encoder_profile_id ==
			entry->persistence.encoder_compatibility_id &&
		entry->manifest.mechanical_load_profile_id ==
			entry->persistence.mechanical_load_compatibility_id &&
		entry->manifest.control_tuning_profile_id ==
			entry->persistence.control_compatibility_id &&
		entry->manifest.memory_layout_profile_id ==
			entry->persistence.storage_layout_compatibility_id;
}

static uint8_t ProductConfigBridge_CurrentChannelCount(
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

static bool ProductConfigBridge_MapCurrentTopology(
	ProductCurrentSenseTopology source, PhaseCurrentTopology *target)
{
	if (target == 0)
		return false;
	switch (source)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
			*target = PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			*target = PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			*target = PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			*target = PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT;
			return true;
		default:
			return false;
	}
}

static bool ProductConfigBridge_MapBspCurrentTopology(
	ProductCurrentSenseTopology source, BspCurrentSenseTopology *topology,
	BspCurrentSamplingMode *sampling_mode)
{
	if (topology == 0 || sampling_mode == 0)
		return false;
	switch (source)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
			*topology = BSP_CURRENT_SENSE_PHASE_INLINE_THREE_SENSOR;
			*sampling_mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			*topology = BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
			*sampling_mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			*topology = BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT;
			*sampling_mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
			return true;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			*topology = BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT;
			*sampling_mode = BSP_CURRENT_SAMPLING_MODE_DYNAMIC;
			return true;
		default:
			return false;
	}
}

static bool ProductConfigBridge_MapCurrentRole(
	ProductCurrentSenseTopology topology, ProductCurrentChannelRole source,
	MeasurementCurrentChannelRole *target, uint8_t *flash_slot)
{
	if (target == 0 || flash_slot == 0)
		return false;
	if (topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		if (source != PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK)
			return false;
		*target = MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK;
		*flash_slot = 0U;
		return true;
	}

	switch (source)
	{
		case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A:
			*target = MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A;
			*flash_slot = 0U;
			return true;
		case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B:
			*target = MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B;
			*flash_slot = 1U;
			return true;
		case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C:
			*target = MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C;
			*flash_slot = 2U;
			return true;
		default:
			return false;
	}
}

bool ProductConfigBridge_ProjectCurrentSense(
	const ProductCurrentSenseConfig *current_sense,
	const BspMotorDriveEndpointCapabilities *motor_drive_capabilities,
	ProductCurrentSenseProjection *projection)
{
	ProductCurrentSenseProjection candidate;
	BspCurrentSenseTopology bsp_topology;
	BspCurrentSamplingMode sampling_mode;
	uint8_t acquisition_indices[BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT];
	uint8_t expected_channel_count;
	uint8_t used_flash_slots = 0U;
	uint8_t channel;

	if (current_sense == 0 || motor_drive_capabilities == 0 ||
		projection == 0)
	{
		return false;
	}
	memset(&candidate, 0, sizeof(candidate));
	expected_channel_count = ProductConfigBridge_CurrentChannelCount(
		current_sense->topology);
	if (expected_channel_count == 0U ||
		current_sense->physical_channel_count != expected_channel_count ||
		motor_drive_capabilities->endpoint_id == 0U ||
		motor_drive_capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		!ProductConfigBridge_MapBspCurrentTopology(current_sense->topology,
			&bsp_topology, &sampling_mode) ||
		(motor_drive_capabilities->supported_current_sense_topologies &
		 BSP_CURRENT_SENSE_TOPOLOGY_BIT(bsp_topology)) == 0U ||
		motor_drive_capabilities->current_sensor_capacity <
			expected_channel_count ||
		!current_sense->pwm_synchronized ||
		!motor_drive_capabilities->supports_synchronized_sampling ||
		(motor_drive_capabilities->supported_sampling_modes &
		 BSP_CURRENT_SAMPLING_MODE_BIT(sampling_mode)) == 0U ||
		current_sense->samples_per_pwm_period == 0U ||
		current_sense->samples_per_pwm_period >
			BSP_MOTOR_MAX_SAMPLING_POINT_COUNT ||
		(sampling_mode == BSP_CURRENT_SAMPLING_MODE_FIXED &&
		 current_sense->samples_per_pwm_period != 1U) ||
		(sampling_mode == BSP_CURRENT_SAMPLING_MODE_DYNAMIC &&
		 (current_sense->samples_per_pwm_period < 2U ||
		  !current_sense->captures_pwm_sector ||
		  !current_sense->supports_sample_window_compensation)) ||
		!ProductConfigBridge_MapCurrentTopology(current_sense->topology,
			&candidate.measurement.topology) ||
		!BspBoard_ResolveCurrentAcquisitionIndices(motor_drive_capabilities,
			current_sense->channel_endpoints, expected_channel_count,
			acquisition_indices))
	{
		return false;
	}
	candidate.measurement.physical_channel_count = expected_channel_count;

	for (channel = 0U; channel < expected_channel_count; channel++)
	{
		MeasurementCurrentChannelConfig *target =
			&candidate.measurement.channels[channel];
		uint8_t flash_slot;
		float polarity;

		if (!ProductConfigBridge_MapCurrentRole(current_sense->topology,
				current_sense->channel_roles[channel], &target->role,
				&flash_slot) ||
			(used_flash_slots & (uint8_t)(1U << flash_slot)) != 0U ||
			current_sense->minimum_valid_offset_count[channel] >
				current_sense->maximum_valid_offset_count[channel] ||
			current_sense->default_offset_count[channel] <
				current_sense->minimum_valid_offset_count[channel] ||
			current_sense->default_offset_count[channel] >
				current_sense->maximum_valid_offset_count[channel] ||
			!isfinite(current_sense->current_a_per_count[channel]) ||
			current_sense->current_a_per_count[channel] <= 0.0f)
		{
			return false;
		}
		if (current_sense->channel_polarities[channel] ==
			PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL)
		{
			polarity = 1.0f;
		}
		else if (current_sense->channel_polarities[channel] ==
			PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED)
		{
			polarity = -1.0f;
		}
		else
		{
			return false;
		}

		used_flash_slots = (uint8_t)(used_flash_slots |
			(uint8_t)(1U << flash_slot));
		target->acquisition_index = acquisition_indices[channel];
		target->offset_adc = current_sense->default_offset_count[channel];
		target->minimum_valid_offset_adc =
			current_sense->minimum_valid_offset_count[channel];
		target->maximum_valid_offset_adc =
			current_sense->maximum_valid_offset_count[channel];
		target->current_a_per_count = polarity *
			current_sense->current_a_per_count[channel];
		candidate.default_offset_adc[flash_slot] =
			current_sense->default_offset_count[channel];
		candidate.minimum_offset_adc[flash_slot] =
			current_sense->minimum_valid_offset_count[channel];
		candidate.maximum_offset_adc[flash_slot] =
			current_sense->maximum_valid_offset_count[channel];
	}

	if (expected_channel_count == 3U && used_flash_slots != 0x07U)
		return false;
	*projection = candidate;
	return true;
}

static bool ProductConfigBridge_MapFeedbackSource(
	ProductFeedbackSourceRef source, FeedbackRouterSourceRef *target)
{
	if (target == 0)
		return false;
	switch (source.kind)
	{
		case PRODUCT_FEEDBACK_SOURCE_NONE:
			target->kind = FEEDBACK_ROUTER_SOURCE_NONE;
			target->index = FEEDBACK_ROUTER_SOURCE_INDEX_NONE;
			return source.angle_sensor_index == PRODUCT_CONFIG_SENSOR_INDEX_NONE;
		case PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR:
			target->kind = FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR;
			target->index = source.angle_sensor_index;
			return true;
		case PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER:
			target->kind = FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER;
			target->index = 0U;
			return source.angle_sensor_index == PRODUCT_CONFIG_SENSOR_INDEX_NONE;
		default:
			return false;
	}
}

static bool ProductConfigBridge_RouteMatchesRuntimeRole(
	const ProductConfig *config, ProductFeedbackSourceRef source,
	ProductFeedbackSignal signal, uint8_t primary_encoder_index)
{
	if (source.kind != PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR)
		return true;
	if (source.angle_sensor_index >= config->angle_sensor_count)
		return false;
	if (signal == PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION)
	{
		return config->angle_sensors[source.angle_sensor_index].role ==
			PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT &&
			source.angle_sensor_index != primary_encoder_index;
	}
	return source.angle_sensor_index == primary_encoder_index;
}

bool ProductConfigBridge_ProjectFeedback(const ProductConfig *config,
	RotorFeedbackRuntimeConfig *projection)
{
	RotorFeedbackRuntimeConfig candidate;
	FeedbackRouterValidationIssue issue;
	uint8_t primary_encoder_index = ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE;
	bool observer_requested = false;
	uint8_t index;

	if (config == 0 || projection == 0 || config->board == 0 ||
		config->board->control_frequency_hz == 0U ||
		config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS ||
		config->features.angle_redundancy_monitor != PRODUCT_FEATURE_OFF ||
		config->feedback.fallback_electrical_angle.kind !=
			PRODUCT_FEEDBACK_SOURCE_NONE)
	{
		return false;
	}
	memset(&candidate, 0, sizeof(candidate));
	candidate.primary_encoder_index = ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE;
	candidate.sample_period_s = 1.0f /
		(float)config->board->control_frequency_hz;
	candidate.routing.angle_sensor_count = config->angle_sensor_count;
	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[index];
		FeedbackRouterSignalMask capabilities = 0U;

		if (sensor->design == 0)
			return false;
		/* The deployed runtime and Flash ABI own one motor-rotor EncoderContext.
		 * Output position remains a lightweight secondary route, but redundant
		 * motor-rotor feedback needs a different runtime and is rejected. */
		if (sensor->role ==
			PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT)
		{
			return false;
		}
		if (sensor->role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY)
		{
			if (primary_encoder_index != ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE)
				return false;
			primary_encoder_index = index;
		}
		if (sensor->role == PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY)
		{
			if ((sensor->design->capabilities & PRODUCT_ANGLE_CAP_POSITION) != 0U)
			{
				capabilities |= FEEDBACK_ROUTER_SIGNAL_MASK(
					FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION) |
					FEEDBACK_ROUTER_SIGNAL_MASK(
						FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE);
			}
			if ((sensor->design->capabilities & PRODUCT_ANGLE_CAP_VELOCITY) != 0U)
			{
				capabilities |= FEEDBACK_ROUTER_SIGNAL_MASK(
					FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY);
			}
			if ((sensor->design->capabilities &
				(PRODUCT_ANGLE_CAP_POSITION |
				 PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION)) ==
				(PRODUCT_ANGLE_CAP_POSITION |
				 PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION))
			{
				capabilities |= FEEDBACK_ROUTER_SIGNAL_MASK(
					FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
			}
		}
		else if (sensor->role == PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT &&
			(sensor->design->capabilities & PRODUCT_ANGLE_CAP_POSITION) != 0U)
		{
			capabilities |= FEEDBACK_ROUTER_SIGNAL_MASK(
				FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION);
		}
		candidate.routing.angle_sensor_capabilities[index] = capabilities;
	}
	candidate.primary_encoder_index = primary_encoder_index;

	if (!ProductConfigBridge_MapFeedbackSource(config->feedback.electrical_angle,
			&candidate.routing.electrical_angle) ||
		!ProductConfigBridge_MapFeedbackSource(config->feedback.motor_velocity,
			&candidate.routing.motor_velocity) ||
		!ProductConfigBridge_MapFeedbackSource(config->feedback.motor_position,
			&candidate.routing.motor_position) ||
		!ProductConfigBridge_MapFeedbackSource(config->feedback.output_position,
			&candidate.routing.output_position) ||
		!ProductConfigBridge_MapFeedbackSource(
			config->feedback.calibration_reference,
			&candidate.routing.calibration_reference) ||
		!ProductConfigBridge_MapFeedbackSource(
			config->feedback.fallback_electrical_angle,
			&candidate.routing.fallback_electrical_angle) ||
		!ProductConfigBridge_RouteMatchesRuntimeRole(config,
			config->feedback.electrical_angle,
			PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE,
			primary_encoder_index) ||
		!ProductConfigBridge_RouteMatchesRuntimeRole(config,
			config->feedback.motor_velocity,
			PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY,
			primary_encoder_index) ||
		!ProductConfigBridge_RouteMatchesRuntimeRole(config,
			config->feedback.motor_position,
			PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION,
			primary_encoder_index) ||
		!ProductConfigBridge_RouteMatchesRuntimeRole(config,
			config->feedback.output_position,
			PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION,
			primary_encoder_index) ||
		!ProductConfigBridge_RouteMatchesRuntimeRole(config,
			config->feedback.calibration_reference,
			PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE,
			primary_encoder_index))
	{
		return false;
	}

	observer_requested =
		config->feedback.electrical_angle.kind ==
			PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER ||
		config->feedback.motor_velocity.kind ==
			PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER ||
		config->feedback.calibration_reference.kind ==
			PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER ||
		config->features.sensorless_control != PRODUCT_FEATURE_OFF ||
		config->commissioning.sensorless_validation !=
			PRODUCT_COMMISSIONING_DISABLED;
	candidate.routing.sensorless_observer_available = observer_requested;
	candidate.routing.sensorless_observer_capabilities = observer_requested ?
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK : 0U;
	candidate.use_output_position_for_position_control =
		config->features.output_position_control != PRODUCT_FEATURE_OFF &&
		config->feedback.output_position.kind != PRODUCT_FEEDBACK_SOURCE_NONE;
	if ((config->features.output_position_control == PRODUCT_FEATURE_REQUIRED &&
		 config->feedback.output_position.kind == PRODUCT_FEEDBACK_SOURCE_NONE) ||
		(candidate.use_output_position_for_position_control &&
		 primary_encoder_index == ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE))
	{
		return false;
	}
	if (config->angle_sensor_count > 1U &&
		primary_encoder_index == ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE)
		return false;
	if (FeedbackRouter_ValidateConfig(&candidate.routing, &issue) !=
		FEEDBACK_ROUTER_STATUS_OK)
	{
		return false;
	}
	*projection = candidate;
	return true;
}

static bool ProductConfigBridge_FeatureRequirementIsValid(
	ProductFeatureRequirement requirement)
{
	return requirement <= PRODUCT_FEATURE_REQUIRED;
}

static bool ProductConfigBridge_RouteUsesPrimaryPhysicalSensor(
	const RotorFeedbackRuntimeConfig *feedback,
	FeedbackRouterSourceRef source, FeedbackRouterSignal signal)
{
	uint8_t primary;

	if (feedback == 0 || signal >= FEEDBACK_ROUTER_SIGNAL_COUNT)
		return false;
	primary = feedback->primary_encoder_index;
	return primary != ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE &&
		primary < feedback->routing.angle_sensor_count &&
		source.kind == FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR &&
		source.index == primary &&
		(feedback->routing.angle_sensor_capabilities[primary] &
		 FEEDBACK_ROUTER_SIGNAL_MASK(signal)) != 0U;
}

static bool ProductConfigBridge_SelectedPositionIsPhysical(
	const RotorFeedbackRuntimeConfig *feedback)
{
	FeedbackRouterSourceRef route;
	FeedbackRouterSignal signal;

	if (feedback == 0)
		return false;
	if (feedback->use_output_position_for_position_control)
	{
		route = feedback->routing.output_position;
		signal = FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION;
		return route.kind == FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR &&
			route.index < feedback->routing.angle_sensor_count &&
			(feedback->routing.angle_sensor_capabilities[route.index] &
			 FEEDBACK_ROUTER_SIGNAL_MASK(signal)) != 0U;
	}
	return ProductConfigBridge_RouteUsesPrimaryPhysicalSensor(feedback,
		feedback->routing.motor_position,
		FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION);
}

static bool ProductConfigBridge_SensorlessStructureIsAvailable(
	const ProductConfig *config, const RotorFeedbackRuntimeConfig *feedback)
{
	const ProductCurrentSenseConfig *current;
	uint8_t current_channel_count;
	uint8_t channel;

	if (config == 0 || feedback == 0 || config->board == 0 ||
		config->motor == 0 ||
		!feedback->routing.sensorless_observer_available ||
		(feedback->routing.sensorless_observer_capabilities &
		 (FEEDBACK_ROUTER_SIGNAL_MASK(
			FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE) |
		  FEEDBACK_ROUTER_SIGNAL_MASK(
			FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY))) !=
		 (FEEDBACK_ROUTER_SIGNAL_MASK(
			FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE) |
		  FEEDBACK_ROUTER_SIGNAL_MASK(
			FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY)) ||
		!config->board->bus_voltage_measurement_available ||
		!isfinite(config->board->bus_voltage_v_per_count) ||
		config->board->bus_voltage_v_per_count <= 0.0f ||
		config->motor->pole_pairs == 0U ||
		!isfinite(config->motor->phase_resistance_ohm) ||
		config->motor->phase_resistance_ohm <= 0.0f ||
		!isfinite(config->motor->d_axis_inductance_h) ||
		config->motor->d_axis_inductance_h <= 0.0f ||
		!isfinite(config->motor->q_axis_inductance_h) ||
		config->motor->q_axis_inductance_h <= 0.0f ||
		!isfinite(config->motor->flux_weber) ||
		config->motor->flux_weber <= 0.0f ||
		!isfinite(config->motor->current_limit_a) ||
		config->motor->current_limit_a <= 0.0f)
	{
		return false;
	}

	current = &config->board->current_sense;
	current_channel_count = ProductConfigBridge_CurrentChannelCount(
		current->topology);
	if (current_channel_count == 0U ||
		current->physical_channel_count != current_channel_count ||
		!current->pwm_synchronized ||
		current->samples_per_pwm_period == 0U)
	{
		return false;
	}
	if (current->topology == PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT &&
		(current->samples_per_pwm_period < 2U ||
		 !current->captures_pwm_sector ||
		 !current->supports_sample_window_compensation))
	{
		return false;
	}
	for (channel = 0U; channel < current_channel_count; channel++)
	{
		if (current->channel_endpoints[channel] ==
				PRODUCT_CONFIG_ENDPOINT_NONE ||
			!isfinite(current->current_a_per_count[channel]) ||
			current->current_a_per_count[channel] <= 0.0f)
		{
			return false;
		}
	}
	return true;
}

bool ProductConfigBridge_ProjectControlModes(const ProductConfig *config,
	const RotorFeedbackRuntimeConfig *feedback,
	MotorControlModeMask *projection)
{
	MotorControlModeMask candidate = MOTOR_CONTROL_MODE_MASK(
		MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP);
	bool physical_electrical;
	bool physical_velocity;
	bool sensorless_available;
	bool position_available;

	if (config == 0 || feedback == 0 || projection == 0 ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.speed_control) ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.position_control) ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.output_position_control) ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.sensorless_control))
	{
		return false;
	}

	physical_electrical =
		ProductConfigBridge_RouteUsesPrimaryPhysicalSensor(feedback,
			feedback->routing.electrical_angle,
			FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE);
	physical_velocity =
		ProductConfigBridge_RouteUsesPrimaryPhysicalSensor(feedback,
			feedback->routing.motor_velocity,
			FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY);
	if (physical_electrical)
	{
		candidate |= MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_CURRENT) |
			MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_VQ);
	}
	if (config->features.speed_control != PRODUCT_FEATURE_OFF &&
		physical_electrical && physical_velocity)
	{
		candidate |= MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_SPEED);
	}

	sensorless_available =
		config->features.sensorless_control != PRODUCT_FEATURE_OFF &&
		ProductConfigBridge_SensorlessStructureIsAvailable(config, feedback);
	if (sensorless_available)
	{
		candidate |= MOTOR_CONTROL_MODE_MASK(
			MOTOR_CONTROL_MODE_SENSORLESS_SPEED);
	}

	position_available = physical_electrical && physical_velocity &&
		ProductConfigBridge_SelectedPositionIsPhysical(feedback);
	if (config->features.position_control != PRODUCT_FEATURE_OFF &&
		position_available)
	{
		candidate |= MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_POSITION_CASCADE) |
			MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_POSITION_IMPEDANCE);
	}

	if ((config->features.speed_control == PRODUCT_FEATURE_REQUIRED &&
		 (candidate & (MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_SPEED) |
		  MOTOR_CONTROL_MODE_MASK(MOTOR_CONTROL_MODE_SENSORLESS_SPEED))) == 0U) ||
		(config->features.sensorless_control == PRODUCT_FEATURE_REQUIRED &&
		 !sensorless_available) ||
		(config->features.position_control == PRODUCT_FEATURE_REQUIRED &&
		 !position_available) ||
		(config->features.output_position_control == PRODUCT_FEATURE_REQUIRED &&
		 (!feedback->use_output_position_for_position_control ||
		  !position_available)) ||
		(candidate & ~MOTOR_CONTROL_SUPPORTED_MODE_MASK) != 0U)
	{
		return false;
	}
	*projection = candidate;
	return true;
}

bool ProductConfigBridge_ProjectTemperature(const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities,
	ProductTemperatureRuntimeProjection *projection)
{
	ProductTemperatureRuntimeProjection candidate =
	{
		PRODUCT_CONFIG_BRIDGE_TEMPERATURE_SENSOR_NONE,
		false,
		false
	};
	const ProductTemperatureSensorInstanceConfig *sensor;
	const BspTemperatureEndpointCapabilities *capabilities;
	const ProductTemperatureZoneMask supported_zone =
		PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU);
	bool protection_valid;

	if (config == 0 || board_capabilities == 0 || projection == 0 ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.temperature_monitoring) ||
		!ProductConfigBridge_FeatureRequirementIsValid(
			config->features.temperature_protection) ||
		(config->features.required_monitored_temperature_zones &
		 ~supported_zone) != 0U ||
		(config->features.required_protected_temperature_zones &
		 ~supported_zone) != 0U)
	{
		return false;
	}

	if (config->features.temperature_monitoring == PRODUCT_FEATURE_OFF &&
		config->features.temperature_protection == PRODUCT_FEATURE_OFF)
	{
		if (config->features.required_monitored_temperature_zones != 0U ||
			config->features.required_protected_temperature_zones != 0U)
		{
			return false;
		}
		*projection = candidate;
		return true;
	}
	if ((config->features.temperature_monitoring == PRODUCT_FEATURE_OFF &&
		 config->features.required_monitored_temperature_zones != 0U) ||
		(config->features.temperature_protection == PRODUCT_FEATURE_OFF &&
		 config->features.required_protected_temperature_zones != 0U))
	{
		return false;
	}
	if (config->temperature_sensor_count == 0U)
	{
		if (config->features.temperature_monitoring == PRODUCT_FEATURE_REQUIRED ||
			config->features.temperature_protection == PRODUCT_FEATURE_REQUIRED ||
			config->features.required_monitored_temperature_zones != 0U ||
			config->features.required_protected_temperature_zones != 0U)
		{
			return false;
		}
		*projection = candidate;
		return true;
	}
	if (config->temperature_sensor_count != 1U)
		return false;

	sensor = &config->temperature_sensors[0];
	if (sensor->instance_id == 0U || sensor->design == 0 ||
		sensor->design->design_id != PRODUCT_CATALOG_TEMPERATURE_INTERNAL ||
		sensor->design->source !=
			PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL ||
		sensor->source != PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL ||
		sensor->zone != PRODUCT_TEMPERATURE_ZONE_MCU ||
		sensor->endpoint == PRODUCT_CONFIG_ENDPOINT_NONE ||
		sensor->sample_period_ms == 0U || sensor->pending_timeout_ms == 0U)
	{
		return false;
	}
	capabilities = BspBoard_FindTemperatureEndpoint(board_capabilities,
		sensor->endpoint);
	if (capabilities == 0 ||
		capabilities->source_kind != BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE ||
		capabilities->location != BSP_TEMPERATURE_LOCATION_PROCESSOR ||
		capabilities->supports_open_circuit_diagnostic ||
		capabilities->supports_short_circuit_diagnostic)
	{
		return false;
	}
	if (capabilities->availability != BSP_ENDPOINT_AVAILABLE)
	{
		if (config->features.temperature_monitoring == PRODUCT_FEATURE_REQUIRED ||
			config->features.temperature_protection == PRODUCT_FEATURE_REQUIRED ||
			config->features.required_monitored_temperature_zones != 0U ||
			config->features.required_protected_temperature_zones != 0U)
		{
			return false;
		}
		*projection = candidate;
		return true;
	}

	candidate.sensor_index = 0U;
	candidate.supervision_enabled = true;
	protection_valid = sensor->protection_enabled &&
		isfinite(sensor->protection_limit_c) &&
		isfinite(sensor->design->minimum_temperature_c) &&
		isfinite(sensor->design->maximum_temperature_c) &&
		sensor->design->minimum_temperature_c <=
			sensor->design->maximum_temperature_c &&
		sensor->protection_limit_c >= sensor->design->minimum_temperature_c &&
		sensor->protection_limit_c <= sensor->design->maximum_temperature_c;
	if (config->features.temperature_protection != PRODUCT_FEATURE_OFF &&
		protection_valid)
	{
		candidate.protection_enabled = true;
	}
	if ((config->features.temperature_protection == PRODUCT_FEATURE_REQUIRED &&
		 !candidate.protection_enabled) ||
		(config->features.required_monitored_temperature_zones &
		 supported_zone) !=
		 config->features.required_monitored_temperature_zones ||
		(config->features.required_protected_temperature_zones != 0U &&
		 !candidate.protection_enabled))
	{
		return false;
	}
	*projection = candidate;
	return true;
}

static bool ProductConfigBridge_CommissioningRequirementIsValid(
	ProductCommissioningRequirement requirement)
{
	return requirement <= PRODUCT_COMMISSIONING_REQUIRED;
}

static bool ProductConfigBridge_SelectCommissioningStep(
	ProductCommissioningRequirement requirement, bool capability_available,
	ProductCommissioningStepMask step,
	ProductCommissioningStepMask *selected_steps)
{
	if (requirement == PRODUCT_COMMISSIONING_DISABLED)
		return true;
	if (!capability_available)
		return requirement == PRODUCT_COMMISSIONING_AUTO;
	*selected_steps |= step;
	return true;
}

static uint8_t ProductConfigBridge_CommissioningRouteSensorMask(
	const ProductConfig *config, ProductFeedbackSourceRef source,
	uint32_t required_capability)
{
	const ProductAngleSensorInstanceConfig *sensor;

	if (source.kind != PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR ||
		source.angle_sensor_index >= config->angle_sensor_count)
	{
		return 0U;
	}
	sensor = &config->angle_sensors[source.angle_sensor_index];
	if (sensor->design == 0 || sensor->source != sensor->design->source ||
		(sensor->design->capabilities & required_capability) !=
			required_capability)
	{
		return 0U;
	}
	return (uint8_t)(1U << source.angle_sensor_index);
}

bool ProductConfigBridge_ProjectCommissioning(const ProductConfig *config,
	MotorCommissioningStageMask *projection)
{
	const ProductCommissioningPolicy *policy;
	const ProductCurrentSenseConfig *current;
	MotorCommissioningStageMask candidate = 0U;
	ProductCommissioningStepMask selected_steps = 0U;
	bool electrical_zero_enabled;
	bool mechanical_zero_enabled;
	bool electrical_zero_selected;
	bool mechanical_zero_selected;
	bool current_feedback_available = false;
	bool friction_available;
	bool cogging_available;
	uint8_t primary_sensor_mask = 0U;
	uint8_t direction_sensor_mask = 0U;
	uint8_t linearization_sensor_mask = 0U;
	uint8_t electrical_zero_sensor_mask;
	uint8_t mechanical_zero_sensor_mask;
	uint8_t cogging_sensor_mask;
	uint8_t index;

	if (config == 0 || projection == 0 ||
		config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS)
		return false;
	policy = &config->commissioning;
#define REQUIRE_VALID_COMMISSIONING_POLICY(field_) \
	if (!ProductConfigBridge_CommissioningRequirementIsValid(policy->field_)) \
		return false
	REQUIRE_VALID_COMMISSIONING_POLICY(current_offset);
	REQUIRE_VALID_COMMISSIONING_POLICY(phase_resistance);
	REQUIRE_VALID_COMMISSIONING_POLICY(angle_direction);
	REQUIRE_VALID_COMMISSIONING_POLICY(angle_linearization);
	REQUIRE_VALID_COMMISSIONING_POLICY(electrical_zero);
	REQUIRE_VALID_COMMISSIONING_POLICY(mechanical_zero);
	REQUIRE_VALID_COMMISSIONING_POLICY(dual_angle_alignment);
	REQUIRE_VALID_COMMISSIONING_POLICY(friction_identification);
	REQUIRE_VALID_COMMISSIONING_POLICY(cogging_identification);
	REQUIRE_VALID_COMMISSIONING_POLICY(sensorless_validation);
	REQUIRE_VALID_COMMISSIONING_POLICY(save_results);
#undef REQUIRE_VALID_COMMISSIONING_POLICY

	/* These stages have no production ServiceProcedure yet.  Never accept a
	 * declaration that would otherwise be silently skipped. */
	if (policy->dual_angle_alignment != PRODUCT_COMMISSIONING_DISABLED ||
		policy->sensorless_validation != PRODUCT_COMMISSIONING_DISABLED)
	{
		return false;
	}

	electrical_zero_enabled = policy->electrical_zero !=
		PRODUCT_COMMISSIONING_DISABLED;
	mechanical_zero_enabled = policy->mechanical_zero !=
		PRODUCT_COMMISSIONING_DISABLED;
	if (electrical_zero_enabled != mechanical_zero_enabled)
		return false;

	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[index];
		uint8_t sensor_mask = (uint8_t)(1U << index);

		if (sensor->role ==
			PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY)
		{
			if (primary_sensor_mask != 0U)
				return false;
			primary_sensor_mask = sensor_mask;
		}
		if (sensor->design == 0 || sensor->source != sensor->design->source)
			continue;
		if ((sensor->role ==
				PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY ||
			 sensor->role ==
				PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT) &&
			(sensor->design->capabilities &
				PRODUCT_ANGLE_CAP_DIRECTION_CALIBRATION) != 0U)
		{
			direction_sensor_mask |= sensor_mask;
		}
		if ((sensor->design->capabilities &
			PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION) != 0U)
		{
			linearization_sensor_mask |= sensor_mask;
		}
	}

	electrical_zero_sensor_mask =
		ProductConfigBridge_CommissioningRouteSensorMask(config,
			config->feedback.electrical_angle,
			PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION);
	mechanical_zero_sensor_mask =
		ProductConfigBridge_CommissioningRouteSensorMask(config,
			config->feedback.motor_position,
			PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION) |
		ProductConfigBridge_CommissioningRouteSensorMask(config,
			config->feedback.output_position,
			PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION);
	cogging_sensor_mask = ProductConfigBridge_CommissioningRouteSensorMask(
		config, config->feedback.motor_position,
		PRODUCT_ANGLE_CAP_REPEATABLE_ABSOLUTE_FRAME);
	if (config->board != 0)
	{
		uint8_t required_channel_count;

		current = &config->board->current_sense;
		required_channel_count = ProductConfigBridge_CurrentChannelCount(
			current->topology);
		current_feedback_available = required_channel_count != 0U &&
			current->physical_channel_count == required_channel_count &&
			current->pwm_synchronized &&
			current->samples_per_pwm_period != 0U;
	}
	friction_available = config->load != 0 &&
		config->load->friction_identification_allowed &&
		primary_sensor_mask != 0U &&
		config->feedback.electrical_angle.kind != PRODUCT_FEEDBACK_SOURCE_NONE &&
		config->feedback.motor_velocity.kind != PRODUCT_FEEDBACK_SOURCE_NONE &&
		config->feedback.motor_position.kind != PRODUCT_FEEDBACK_SOURCE_NONE;
	cogging_available = config->load != 0 &&
		config->load->cogging_identification_allowed &&
		(cogging_sensor_mask & primary_sensor_mask) != 0U &&
		config->feedback.electrical_angle.kind != PRODUCT_FEEDBACK_SOURCE_NONE;

#define SELECT_COMMISSIONING_STEP(field_, available_, step_) \
	if (!ProductConfigBridge_SelectCommissioningStep(policy->field_, \
		(available_), (step_), &selected_steps)) \
		return false
	SELECT_COMMISSIONING_STEP(current_offset,
		current_feedback_available && config->board->current_sense.
			offset_calibration_supported,
		PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET);
	SELECT_COMMISSIONING_STEP(phase_resistance,
		current_feedback_available &&
		config->board->bus_voltage_measurement_available &&
		config->board->current_sense.topology !=
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT,
		PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE);
	SELECT_COMMISSIONING_STEP(angle_direction, direction_sensor_mask != 0U,
		PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION);
	SELECT_COMMISSIONING_STEP(angle_linearization,
		linearization_sensor_mask != 0U,
		PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION);
	SELECT_COMMISSIONING_STEP(electrical_zero,
		electrical_zero_sensor_mask != 0U,
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO);
	SELECT_COMMISSIONING_STEP(mechanical_zero,
		mechanical_zero_sensor_mask != 0U,
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO);
	SELECT_COMMISSIONING_STEP(friction_identification, friction_available,
		PRODUCT_COMMISSIONING_STEP_FRICTION);
	SELECT_COMMISSIONING_STEP(cogging_identification, cogging_available,
		PRODUCT_COMMISSIONING_STEP_COGGING);
	SELECT_COMMISSIONING_STEP(save_results, true,
		PRODUCT_COMMISSIONING_STEP_SAVE);
#undef SELECT_COMMISSIONING_STEP

	electrical_zero_selected = (selected_steps &
		PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO) != 0U;
	mechanical_zero_selected = (selected_steps &
		PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO) != 0U;
	if (electrical_zero_selected != mechanical_zero_selected)
		return false;
	/* The deployed calibration ABI owns exactly one EncoderContext.  Reject
	 * any selected stage whose target set contains a secondary/output
	 * sensor rather than pretending the primary result calibrated them all. */
	if (((selected_steps & PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION) != 0U &&
		 (direction_sensor_mask == 0U ||
		  (direction_sensor_mask & ~primary_sensor_mask) != 0U)) ||
		((selected_steps & PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION) != 0U &&
		 (linearization_sensor_mask == 0U ||
		  (linearization_sensor_mask & ~primary_sensor_mask) != 0U)) ||
		(electrical_zero_selected &&
		 (electrical_zero_sensor_mask == 0U ||
		  (electrical_zero_sensor_mask & ~primary_sensor_mask) != 0U)) ||
		(mechanical_zero_selected &&
		 (mechanical_zero_sensor_mask == 0U ||
		  (mechanical_zero_sensor_mask & ~primary_sensor_mask) != 0U)))
	{
		return false;
	}

#define MAP_COMMISSIONING_STAGE(step_, stage_) \
	if ((selected_steps & (step_)) != 0U) \
		candidate |= MOTOR_COMMISSIONING_STAGE_MASK(stage_)
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET,
		COMMISSIONING_STAGE_CURRENT_OFFSET);
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE,
		COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK);
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION,
		COMMISSIONING_STAGE_ENCODER_DIRECTION);
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION,
		COMMISSIONING_STAGE_ENCODER_LUT);
	if (electrical_zero_selected)
	{
		candidate |= MOTOR_COMMISSIONING_STAGE_MASK(
			COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO);
	}
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_FRICTION,
		COMMISSIONING_STAGE_FRICTION);
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_COGGING,
		COMMISSIONING_STAGE_COGGING);
	MAP_COMMISSIONING_STAGE(PRODUCT_COMMISSIONING_STEP_SAVE,
		COMMISSIONING_STAGE_SAVE);
#undef MAP_COMMISSIONING_STAGE

	if ((candidate & ~MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK) != 0U)
	{
		return false;
	}
	*projection = candidate;
	return true;
}

static bool ProductConfigBridge_RuntimeShapeIsSane(
	const ProductConfig *config)
{
	uint8_t expected_current_channels;
	uint8_t index;
	const ProductFeedbackSourceRef *routes[6];

	expected_current_channels = ProductConfigBridge_CurrentChannelCount(
		config->board->current_sense.topology);
	if (expected_current_channels == 0U ||
		config->board->current_sense.physical_channel_count !=
			expected_current_channels ||
		config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS ||
		config->temperature_sensor_count >
			PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS)
		return false;
	for (index = 0U; index < expected_current_channels; index++)
	{
		if (config->board->current_sense.channel_endpoints[index] ==
			PRODUCT_CONFIG_ENDPOINT_NONE)
			return false;
	}
	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		if (config->angle_sensors[index].design == 0 ||
			config->angle_sensors[index].endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
			return false;
	}
	for (index = 0U; index < config->temperature_sensor_count; index++)
	{
		if (config->temperature_sensors[index].design == 0 ||
			config->temperature_sensors[index].endpoint ==
				PRODUCT_CONFIG_ENDPOINT_NONE)
			return false;
	}
	routes[0] = &config->feedback.electrical_angle;
	routes[1] = &config->feedback.motor_velocity;
	routes[2] = &config->feedback.motor_position;
	routes[3] = &config->feedback.output_position;
	routes[4] = &config->feedback.calibration_reference;
	routes[5] = &config->feedback.fallback_electrical_angle;
	for (index = 0U; index < 6U; index++)
	{
		if (routes[index]->kind == PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR &&
			routes[index]->angle_sensor_index >= config->angle_sensor_count)
			return false;
	}
	return true;
}

static bool ProductConfigBridge_TargetMotorDriveIsSupported(
	const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities)
{
	const ProductCurrentSenseConfig *current = &config->board->current_sense;
	const BspMotorDriveEndpointCapabilities *capabilities =
		BspBoard_FindMotorDriveEndpoint(board_capabilities,
			config->board->motor_drive_endpoint);
	ProductCurrentSenseProjection projection;

	return capabilities != 0 &&
		capabilities->supported_current_sense_topologies ==
			BSP_CURRENT_SENSE_TOPOLOGY_BIT(
				BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT) &&
		capabilities->current_sensor_capacity == 3U &&
		capabilities->supports_synchronized_sampling &&
		capabilities->supported_sampling_modes ==
			BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_FIXED) &&
		current->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT &&
		current->physical_channel_count == 3U &&
		current->samples_per_pwm_period == 1U &&
		config->board->control_frequency_hz == UINT32_C(20000) &&
		(!config->board->require_hardware_shutdown ||
		 capabilities->supports_hardware_shutdown) &&
		ProductConfigBridge_ProjectCurrentSense(current, capabilities,
			&projection);
}

static bool ProductConfigBridge_TargetAngleSensorsAreSupported(
	const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities)
{
	uint8_t index;

	for (index = 0U; index < config->angle_sensor_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&config->angle_sensors[index];
		const BspAngleSensorEndpointCapabilities *capabilities =
			BspBoard_FindAngleSensorEndpoint(board_capabilities,
				sensor->endpoint);

		if (sensor->instance_id == 0U || sensor->design == 0 ||
			sensor->design->design_id !=
				PRODUCT_CATALOG_ANGLE_TLE5012B ||
			sensor->design->source !=
				PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL ||
			sensor->design->counts_per_turn != UINT32_C(65536) ||
			(sensor->design->capabilities &
			 ~ProductCatalog_Tle5012bAngleSensor.capabilities) != 0U ||
			sensor->source != PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL ||
			(sensor->role !=
				 PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY &&
			 sensor->role != PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT) ||
			capabilities == 0 ||
			capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
			capabilities->kind != BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL ||
			capabilities->maximum_transfer_word_bits < 16U ||
			!capabilities->has_dedicated_select)
		{
			return false;
		}
	}
	return true;
}

static bool ProductConfigBridge_TargetCanIsSupported(
	const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities)
{
	const BspCommunicationEndpointCapabilities *capabilities;
	uint32_t nominal_bit_rate;
	uint32_t data_bit_rate = 0U;

	if (config->can.mode == PRODUCT_CAN_MODE_DISABLED ||
		config->can.mode > PRODUCT_CAN_MODE_FD ||
		config->can.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE ||
		config->can.nominal_bitrate_kbps == 0U ||
		config->can.nominal_bitrate_kbps > UINT32_MAX / UINT32_C(1000) ||
		config->can.maximum_payload_bytes == 0U ||
		config->can.default_node_id > 7U ||
		config->can.minimum_heartbeat_ms > config->can.maximum_heartbeat_ms ||
		(config->can.heartbeat_ms != 0U &&
		 (config->can.heartbeat_ms < config->can.minimum_heartbeat_ms ||
		  config->can.heartbeat_ms > config->can.maximum_heartbeat_ms)))
	{
		return false;
	}
	capabilities = BspBoard_FindCommunicationEndpoint(board_capabilities,
		config->can.endpoint);
	if (capabilities == 0 ||
		capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		capabilities->kind != BSP_COMMUNICATION_CAN ||
		capabilities->features !=
			(BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
			 BSP_COMMUNICATION_FEATURE_CAN_FD |
			 BSP_COMMUNICATION_FEATURE_CAN_BRS) ||
		capabilities->maximum_payload_bytes !=
			BSP_CAN_CLASSIC_MAX_DATA_LENGTH ||
		capabilities->maximum_nominal_bit_rate != UINT32_C(1000000) ||
		capabilities->maximum_data_bit_rate != UINT32_C(5000000) ||
		config->can.maximum_payload_bytes > capabilities->maximum_payload_bytes)
	{
		return false;
	}
	nominal_bit_rate = config->can.nominal_bitrate_kbps * UINT32_C(1000);
	if (nominal_bit_rate > capabilities->maximum_nominal_bit_rate)
		return false;

	if (config->can.mode == PRODUCT_CAN_MODE_CLASSIC)
	{
		return config->board->classic_can_supported &&
			(capabilities->features &
			 BSP_COMMUNICATION_FEATURE_CAN_CLASSIC) != 0U &&
			!config->can.bit_rate_switching &&
			config->can.data_bitrate_kbps == 0U &&
			config->can.maximum_payload_bytes <=
				BSP_CAN_CLASSIC_MAX_DATA_LENGTH;
	}
	if (!config->board->can_fd_supported ||
		(capabilities->features & BSP_COMMUNICATION_FEATURE_CAN_FD) == 0U ||
		config->can.data_bitrate_kbps == 0U ||
		config->can.data_bitrate_kbps > UINT32_MAX / UINT32_C(1000))
	{
		return false;
	}
	data_bit_rate = config->can.data_bitrate_kbps * UINT32_C(1000);
	return data_bit_rate <= capabilities->maximum_data_bit_rate &&
		(config->can.bit_rate_switching ?
		 (config->board->can_brs_supported &&
		  (capabilities->features & BSP_COMMUNICATION_FEATURE_CAN_BRS) != 0U) :
		 (data_bit_rate == nominal_bit_rate));
}

static bool ProductConfigBridge_TargetServiceStreamIsSupported(
	const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities)
{
	const BspCommunicationEndpointCapabilities *capabilities;
	const BspCommunicationFeatureSet required_features =
		BSP_COMMUNICATION_FEATURE_BYTE_STREAM |
		BSP_COMMUNICATION_FEATURE_FULL_DUPLEX;

	if (!config->service_stream.enabled ||
		config->service_stream.endpoint == PRODUCT_CONFIG_ENDPOINT_NONE)
	{
		return false;
	}
	capabilities = BspBoard_FindCommunicationEndpoint(board_capabilities,
		config->service_stream.endpoint);
	return capabilities != 0 &&
		capabilities->availability == BSP_ENDPOINT_AVAILABLE &&
		capabilities->kind == BSP_COMMUNICATION_BYTE_STREAM &&
		capabilities->features == required_features &&
		capabilities->maximum_payload_bytes == 64U &&
		capabilities->maximum_nominal_bit_rate == 0U &&
		capabilities->maximum_data_bit_rate == 0U;
}

static bool ProductConfigBridge_TargetRuntimeIsSupported(
	const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities)
{
	RotorFeedbackRuntimeConfig feedback;
	MotorControlModeMask control_modes;
	ProductTemperatureRuntimeProjection temperature;

	return ProductConfigBridge_TargetMotorDriveIsSupported(config,
			board_capabilities) &&
		ProductConfigBridge_TargetAngleSensorsAreSupported(config,
			board_capabilities) &&
		ProductConfigBridge_ProjectFeedback(config, &feedback) &&
		ProductConfigBridge_ProjectControlModes(config, &feedback,
			&control_modes) &&
		ProductConfigBridge_TargetCanIsSupported(config, board_capabilities) &&
		ProductConfigBridge_TargetServiceStreamIsSupported(config,
			board_capabilities) &&
		ProductConfigBridge_ProjectTemperature(config, board_capabilities,
			&temperature);
}

bool ProductConfigBridge_ValidateRuntime(const ProductCatalogEntry *entry,
	const BspBoardRuntimeIdentity *board_identity,
	const BspBoardCapabilities *board_capabilities,
	ProductConfigBridgeStatus *bridge_status)
{
	const ProductConfig *config = entry != 0 ? entry->config : 0;

	if (bridge_status != 0)
		*bridge_status = PRODUCT_CONFIG_BRIDGE_OK;
	if (!ProductConfigBridge_EntryIsConsistent(entry))
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return false;
	}
	if (board_identity == 0 || board_capabilities == 0 ||
		board_identity->board_id == 0U ||
		board_identity->binding_fingerprint == 0U ||
		board_capabilities->board_id == 0U ||
		board_capabilities->binding_fingerprint == 0U)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BOARD_INVALID;
		return false;
	}
	if (board_identity->board_id != board_capabilities->board_id ||
		board_identity->binding_fingerprint !=
			board_capabilities->binding_fingerprint ||
		config->board->design_id != board_identity->board_id ||
		config->board->bsp_binding_fingerprint !=
			board_identity->binding_fingerprint)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return false;
	}
	/* The rich Product/BSP validator is a build/host gate. On target, retain a
	 * compact structural check; every concrete endpoint is then resolved and
	 * initialized fail-closed by the composition root and its leaf factory. */
	if (!ProductConfigBridge_RuntimeShapeIsSane(config))
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return false;
	}
	if (!ProductConfigBridge_TargetRuntimeIsSupported(config,
			board_capabilities))
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED;
		return false;
	}
	return true;
}
