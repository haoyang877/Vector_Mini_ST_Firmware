#include "product_variant.h"

#include <math.h>

#include "control_loop_config.h"

bool ProductVariant_GetActive(ProductVariant *variant)
{
	if (variant == 0)
		return false;
	variant->identity = ProductManifest_Get();
	variant->board = BoardProfile_GetActive();
	variant->motor = MotorProfile_GetActive();
	variant->encoder = EncoderProfile_GetActive();
	variant->mechanical_load = MechanicalLoadProfile_GetActive();
	variant->control_tuning = ControlTuningProfile_GetActive();
	variant->memory_layout = MemoryLayoutProfile_GetActive();
	variant->configuration_fingerprint = PRODUCT_CONFIGURATION_FINGERPRINT;
	/* One-time migration applies only to the already-deployed product tuple. */
#if ACTIVE_BOARD_PROFILE == BOARD_PROFILE_VECTOR_MINI_ST && \
	ACTIVE_MOTOR_PROFILE == MOTOR_PROFILE_HT8115_4 && \
	ACTIVE_ENCODER_PROFILE == ENCODER_PROFILE_TLE5012B_16BIT && \
	ACTIVE_MECHANICAL_LOAD_PROFILE == MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM && \
	CURRENT_SENSE_SHUNT_MILLIOHM == CURRENT_SENSE_SHUNT_6_MILLIOHM
	variant->allow_legacy_parameter_migration = true;
#else
	variant->allow_legacy_parameter_migration = false;
#endif
	return ProductVariant_Validate(variant);
}

bool ProductVariant_Validate(const ProductVariant *variant)
{
	const BoardProfile *board;
	const MotorProfile *motor;

	if (variant == 0 || variant->identity == 0 || variant->board == 0 ||
		variant->motor == 0 || variant->encoder == 0 ||
		variant->mechanical_load == 0 || variant->control_tuning == 0 ||
		variant->memory_layout == 0)
		return false;
	board = variant->board;
	motor = variant->motor;
	return variant->configuration_fingerprint != 0U &&
		variant->identity->configuration_fingerprint ==
			variant->configuration_fingerprint &&
		variant->identity->hardware_profile_id == board->profile_id &&
		variant->identity->motor_profile_id == motor->profile_id &&
		variant->identity->encoder_profile_id == variant->encoder->profile_id &&
		variant->identity->mechanical_load_profile_id ==
			variant->mechanical_load->profile_id &&
		variant->identity->control_tuning_profile_id ==
			variant->control_tuning->profile_id &&
		variant->identity->memory_layout_profile_id ==
			variant->memory_layout->profile_id &&
		board->control_frequency_hz == CONTROL_LOOP_FREQUENCY_HZ &&
		board->minimum_current_offset_adc <= board->maximum_current_offset_adc &&
		board->default_phase_a_current_offset_adc >= board->minimum_current_offset_adc &&
		board->default_phase_a_current_offset_adc <= board->maximum_current_offset_adc &&
		board->default_phase_b_current_offset_adc >= board->minimum_current_offset_adc &&
		board->default_phase_b_current_offset_adc <= board->maximum_current_offset_adc &&
		board->default_phase_c_current_offset_adc >= board->minimum_current_offset_adc &&
		board->default_phase_c_current_offset_adc <= board->maximum_current_offset_adc &&
		isfinite(board->current_sense_reliable_limit_a) &&
		board->current_command_limit_a <= board->current_sense_reliable_limit_a &&
		board->calibration_current_limit_a <= board->current_sense_reliable_limit_a &&
		board->overcurrent_trip_a <= board->current_sense_reliable_limit_a &&
		motor->current_limit_a <= board->current_command_limit_a &&
		motor->calibration_current_a <= board->calibration_current_limit_a &&
		variant->mechanical_load->default_speed_limit_rps *
			6.28318530717958647692f <= motor->speed_limit_max_rad_s &&
		variant->mechanical_load->default_position_max_speed_rps <=
			motor->position_speed_limit_rps &&
		variant->mechanical_load->default_position_max_speed_rps <=
			variant->mechanical_load->default_speed_limit_rps &&
		variant->mechanical_load->encoder_calibration_startup.minimum_current_limit_a <=
			board->current_command_limit_a &&
		board->default_can_node_id <= 7U &&
		board->minimum_can_heartbeat_ms <= board->maximum_can_heartbeat_ms &&
		(board->default_can_heartbeat_ms == 0U ||
		 (board->default_can_heartbeat_ms >= board->minimum_can_heartbeat_ms &&
		  board->default_can_heartbeat_ms <= board->maximum_can_heartbeat_ms)) &&
		(!board->temperature_protection_enabled ||
		 board->temperature_sensor != TEMPERATURE_SENSOR_NONE) &&
		(board->inverter_deadtime_source ==
			POWER_STAGE_DEADTIME_EXTERNAL_GATE_DRIVER ||
		 board->inverter_deadtime_s > 0.0f) &&
		variant->memory_layout->application_start_address +
			variant->memory_layout->application_size_bytes <=
			variant->memory_layout->parameter_slot_0_address &&
		variant->memory_layout->parameter_slot_0_address +
			variant->memory_layout->parameter_slot_size_bytes <=
			variant->memory_layout->parameter_slot_1_address;
}
