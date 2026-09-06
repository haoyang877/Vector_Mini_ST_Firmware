#include "product_config.h"

const char *ProductConfig_ErrorCodeName(ProductConfigErrorCode code)
{
	switch (code)
	{
		case PRODUCT_CONFIG_ERROR_NONE: return "none";
		case PRODUCT_CONFIG_ERROR_NULL_CONFIG: return "null_config";
		case PRODUCT_CONFIG_ERROR_INVALID_IDENTITY: return "invalid_identity";
		case PRODUCT_CONFIG_ERROR_BOARD_REQUIRED: return "board_required";
		case PRODUCT_CONFIG_ERROR_MOTOR_REQUIRED: return "motor_required";
		case PRODUCT_CONFIG_ERROR_LOAD_REQUIRED: return "load_required";
		case PRODUCT_CONFIG_ERROR_PLATFORM_MISMATCH: return "platform_mismatch";
		case PRODUCT_CONFIG_ERROR_INVALID_BOARD_LIMIT: return "invalid_board_limit";
		case PRODUCT_CONFIG_ERROR_INVALID_MOTOR_DESIGN: return "invalid_motor_design";
		case PRODUCT_CONFIG_ERROR_MOTOR_EXCEEDS_BOARD_LIMIT: return "motor_exceeds_board_limit";
		case PRODUCT_CONFIG_ERROR_INVALID_LOAD_DESIGN: return "invalid_load_design";
		case PRODUCT_CONFIG_ERROR_CURRENT_TOPOLOGY_INVALID: return "current_topology_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_COUNT_MISMATCH: return "current_channel_count_mismatch";
		case PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_INVALID: return "current_endpoint_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_DUPLICATE: return "current_endpoint_duplicate";
		case PRODUCT_CONFIG_ERROR_CURRENT_SCALE_INVALID: return "current_scale_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_PWM_SYNC_REQUIRED: return "current_pwm_sync_required";
		case PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_TWO_SAMPLES_REQUIRED: return "single_shunt_two_samples_required";
		case PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_PWM_SECTOR_REQUIRED: return "single_shunt_pwm_sector_required";
		case PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_WINDOW_COMPENSATION_REQUIRED: return "single_shunt_window_compensation_required";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_COUNT_EXCEEDED: return "angle_sensor_count_exceeded";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_DESIGN_REQUIRED: return "angle_sensor_design_required";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_INVALID: return "angle_sensor_id_invalid";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_DUPLICATE: return "angle_sensor_id_duplicate";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_SOURCE_MISMATCH: return "angle_sensor_source_mismatch";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_INVALID: return "angle_sensor_role_invalid";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_DUPLICATE: return "angle_sensor_role_duplicate";
		case PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ENDPOINT_INVALID: return "angle_sensor_endpoint_invalid";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_COUNT_EXCEEDED: return "temperature_sensor_count_exceeded";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_DESIGN_REQUIRED: return "temperature_sensor_design_required";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_INVALID: return "temperature_sensor_id_invalid";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_DUPLICATE: return "temperature_sensor_id_duplicate";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_SOURCE_MISMATCH: return "temperature_sensor_source_mismatch";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ZONE_INVALID: return "temperature_sensor_zone_invalid";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ENDPOINT_INVALID: return "temperature_sensor_endpoint_invalid";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_LIMIT_INVALID: return "temperature_limit_invalid";
		case PRODUCT_CONFIG_ERROR_FEEDBACK_SOURCE_INVALID: return "feedback_source_invalid";
		case PRODUCT_CONFIG_ERROR_FEEDBACK_SENSOR_INDEX_INVALID: return "feedback_sensor_index_invalid";
		case PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH: return "feedback_capability_mismatch";
		case PRODUCT_CONFIG_ERROR_FEEDBACK_ROLE_MISMATCH: return "feedback_role_mismatch";
		case PRODUCT_CONFIG_ERROR_FEATURE_POLICY_INVALID: return "feature_policy_invalid";
		case PRODUCT_CONFIG_ERROR_REQUIRED_SPEED_FEEDBACK_UNAVAILABLE: return "required_speed_feedback_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_POSITION_FEEDBACK_UNAVAILABLE: return "required_position_feedback_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_OUTPUT_POSITION_UNAVAILABLE: return "required_output_position_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_SENSORLESS_UNAVAILABLE: return "required_sensorless_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_REDUNDANCY_UNAVAILABLE: return "required_redundancy_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE: return "required_temperature_monitor_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE: return "required_temperature_protection_unavailable";
		case PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE: return "required_temperature_zone_unavailable";
		case PRODUCT_CONFIG_ERROR_COMMISSIONING_POLICY_INVALID: return "commissioning_policy_invalid";
		case PRODUCT_CONFIG_ERROR_COMMISSIONING_STEP_UNSUPPORTED: return "commissioning_step_unsupported";
		case PRODUCT_CONFIG_ERROR_CAN_MODE_INVALID: return "can_mode_invalid";
		case PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED: return "can_mode_unsupported";
		case PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID: return "can_bitrate_invalid";
		case PRODUCT_CONFIG_ERROR_CAN_BRS_REQUIRES_FD: return "can_brs_requires_fd";
		case PRODUCT_CONFIG_ERROR_CAN_PAYLOAD_INVALID: return "can_payload_invalid";
		case PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID: return "communication_endpoint_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_SHUNT_INVALID: return "current_shunt_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_RANGE_INVALID: return "current_offset_range_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_DEFAULT_INVALID: return "current_offset_default_invalid";
		case PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_CALIBRATION_INVALID: return "current_offset_calibration_invalid";
		case PRODUCT_CONFIG_ERROR_BOARD_PATH_COMPENSATION_INVALID: return "board_path_compensation_invalid";
		case PRODUCT_CONFIG_ERROR_SAFETY_LIMIT_INVALID: return "safety_limit_invalid";
		case PRODUCT_CONFIG_ERROR_SAFETY_CONFIRMATION_INVALID: return "safety_confirmation_invalid";
		case PRODUCT_CONFIG_ERROR_TEMPERATURE_SAMPLE_DIVIDER_INVALID: return "temperature_sample_divider_invalid";
		case PRODUCT_CONFIG_ERROR_CAN_NODE_ID_INVALID: return "can_node_id_invalid";
		case PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID: return "can_heartbeat_invalid";
		default: return "unknown";
	}
}
