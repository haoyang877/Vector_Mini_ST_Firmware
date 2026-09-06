#include "product_catalog.h"

#include "Core/Config/product_manifest.h"

#define PRODUCT_TWO_PI 6.28318530717958647692f

const ProductBoardDesign ProductCatalog_VectorMiniStBoard =
{
	.design_id = PRODUCT_CATALOG_BOARD_CURRENT,
	.platform_id = PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET,
	.bsp_binding_fingerprint = PRODUCT_CATALOG_BSP_BINDING_FINGERPRINT,
	.motor_drive_endpoint = 0x0100U,
	.require_hardware_shutdown = false,
	.control_frequency_hz = 20000U,
	.reliable_phase_current_limit_a = 20.0f,
	.command_phase_current_limit_a = 10.0f,
	.calibration_phase_current_limit_a = 10.0f,
	.phase_resistance_path_compensation_ohm = 0.008f,
	.bus_voltage_measurement_available = true,
	.bus_voltage_v_per_count = 0.0088644689f,
	.classic_can_supported = true,
	.can_fd_supported = true,
	.can_brs_supported = true,
	.current_sense =
	{
		.topology = PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT,
		.physical_channel_count = 3U,
		.channel_roles =
		{
			PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A,
			PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B,
			PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C
		},
		/* Existing Vector Mini amplifiers report positive phase current as a
		 * falling ADC delta: current = -(raw - offset) * scale. */
		.channel_polarities =
		{
			PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED,
			PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED,
			PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED
		},
		.channel_endpoints =
		{
			PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_A,
			PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_B,
			PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_C
		},
		.current_a_per_count =
		{
			0.0134310134f,
			0.0134310134f,
			0.0134310134f
		},
		.default_offset_count = {2048U, 2048U, 2048U},
		.nominal_shunt_milliohm = 6U,
		.minimum_valid_offset_count = {1948U, 1948U, 1948U},
		.maximum_valid_offset_count = {2148U, 2148U, 2148U},
		.offset_calibration_sample_count = 20000U,
		.pwm_synchronized = true,
		.samples_per_pwm_period = 1U,
		.captures_pwm_sector = true,
		.supports_sample_window_compensation = false,
		.offset_calibration_supported = true
	}
};

const ProductMotorDesign ProductCatalog_Ht8115_4Motor =
{
	.design_id = PRODUCT_CATALOG_MOTOR_CURRENT,
	.pole_pairs = 21U,
	.phase_resistance_ohm = 1.905f,
	.d_axis_inductance_h = 0.001635f,
	.q_axis_inductance_h = 0.001635f,
	.flux_weber = 0.0175025f,
	.current_limit_a = 6.0f,
	.calibration_current_a = 3.0f,
	.speed_limit_rad_s = 38.9557489f
};

const ProductAngleSensorDesign
	ProductCatalog_Tle5012bAngleSensor =
{
	.design_id = PRODUCT_CATALOG_ANGLE_TLE5012B,
	.source = PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL,
	.counts_per_turn = 65536UL,
	.capabilities = PRODUCT_ANGLE_CAP_POSITION |
		PRODUCT_ANGLE_CAP_VELOCITY |
		PRODUCT_ANGLE_CAP_REPEATABLE_ABSOLUTE_FRAME |
		PRODUCT_ANGLE_CAP_DIRECTION_CALIBRATION |
		PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION |
		PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION |
		PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION |
		PRODUCT_ANGLE_CAP_DIAGNOSTICS
};

const ProductTemperatureSensorDesign ProductCatalog_InternalTemperatureSensor =
{
	.design_id = PRODUCT_CATALOG_TEMPERATURE_INTERNAL,
	.source = PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL,
	.minimum_temperature_c = -40.0f,
	.maximum_temperature_c = 150.0f
};

#define PRODUCT_MOTOR_ACCEPTANCE_INITIALIZER \
	{ \
		.phase_resistance_min_ohm = 0.0001f, \
		.phase_resistance_max_ohm = 5.0f, \
		.inductance_min_h = 1.0e-6f, \
		.inductance_max_h = 5.0e-3f, \
		.flux_min_weber = 1.0e-5f, \
		.flux_max_weber = 1.0f \
	}

#define PRODUCT_SENSORLESS_STARTUP_INITIALIZER \
	{ \
		.align_current_ramp_time_s = 0.50f, \
		.align_hold_time_s = 0.30f, \
		.align_current_a = 2.0f, \
		.startup_iq_initial_a = 0.15f, \
		.startup_iq_a = 0.50f, \
		.startup_iq_ramp_time_s = 2.00f, \
		.startup_id_a = 0.50f, \
		.minimum_current_limit_a = 0.0f, \
		.minimum_electrical_velocity_rad_s = 250.0f, \
		.target_electrical_velocity_rad_s = 420.0f, \
		.startup_ramp_time_s = 2.00f, \
		.speed_lock_time_s = 0.20f, \
		.speed_lock_filter_alpha = 1.0f, \
		.observer_lock_ratio = 0.25f, \
		.angle_handoff_time_s = 0.10f, \
		.lock_timeout_s = 3.00f, \
		.id_ramp_down_time_s = 0.50f, \
		.observer_loss_time_s = 0.20f \
	}

#define PRODUCT_ENCODER_STARTUP_NO_DAMPER_INITIALIZER \
	PRODUCT_SENSORLESS_STARTUP_INITIALIZER

#define PRODUCT_ENCODER_STARTUP_DAMPED_INITIALIZER \
	{ \
		.align_current_ramp_time_s = 0.80f, \
		.align_hold_time_s = 0.50f, \
		.align_current_a = 3.50f, \
		.startup_iq_initial_a = 3.50f, \
		.startup_iq_a = 4.50f, \
		.startup_iq_ramp_time_s = 1.00f, \
		.startup_id_a = 0.50f, \
		.minimum_current_limit_a = 5.50f, \
		.minimum_electrical_velocity_rad_s = 250.0f, \
		.target_electrical_velocity_rad_s = 420.0f, \
		.startup_ramp_time_s = 2.00f, \
		.speed_lock_time_s = 0.20f, \
		.speed_lock_filter_alpha = 0.01f, \
		.observer_lock_ratio = 0.25f, \
		.angle_handoff_time_s = 0.10f, \
		.lock_timeout_s = 5.00f, \
		.id_ramp_down_time_s = 0.50f, \
		.observer_loss_time_s = 0.20f \
	}

#define PRODUCT_PARAMETER_LIMITS_INITIALIZER \
	{ \
		.speed_limit_max_rad_s = 400.0f * PRODUCT_TWO_PI, \
		.speed_ramp_max_rad_s2 = 1000.0f * PRODUCT_TWO_PI, \
		.position_ramp_max_rad_s2 = 200.0f * PRODUCT_TWO_PI, \
		.position_speed_limit_rad_s = (372.0f / 60.0f) * PRODUCT_TWO_PI, \
		.position_kp_limit_a_per_rad = 50.0f, \
		.position_kd_limit_a_per_rad_s = 10.0f, \
		.position_ki_limit_a_per_rad_s = 10.0f, \
		.cascade_position_kp_limit_per_s = 50.0f, \
		.cascade_position_kd_limit = 10.0f \
	}

#define PRODUCT_SENSORLESS_CONTROL_INITIALIZER \
	{ \
		.startup = PRODUCT_SENSORLESS_STARTUP_INITIALIZER, \
		.observer_max_electrical_velocity_rad_s = 5000.0f, \
		.speed_feedback_lpf_alpha = 0.1042f, \
		.flux_observer_gamma = 800000.0f, \
		.flux_observer_resistance_scale = 0.4199475f, \
		.flux_observer_max_correction_step_rad = 0.2f, \
		.flux_observer_minimum_flux_weber = 1.0e-6f, \
		.flux_observer_velocity_lpf_alpha = 0.1f, \
		.flux_observer_angle_wrap_threshold_rad = 4.0f \
	}

#define PRODUCT_POSITION_FRICTION_NO_DAMPER_INITIALIZER \
	{ \
		.enabled = false, \
		.friction_positive_current_a = 0.0f, \
		.friction_negative_current_a = 0.0f, \
		.breakaway_positive_current_a = 0.0f, \
		.breakaway_negative_current_a = 0.0f, \
		.current_slew_rate_a_per_s = 15.0f, \
		.position_enter_rad = 0.001f, \
		.position_exit_rad = 0.003f, \
		.reference_speed_rad_s = 0.03f, \
		.stop_speed_rad_s = 0.02f, \
		.move_speed_rad_s = 0.05f, \
		.stuck_time_s = 0.05f, \
		.landing_position_rad = 0.006f, \
		.landing_speed_rad_s = 0.10f, \
		.recovery_delay_s = 0.20f, \
		.recovery_pulse_time_s = 0.05f, \
		.recovery_cooldown_s = 0.20f \
	}

#define PRODUCT_POSITION_FRICTION_DAMPED_INITIALIZER \
	{ \
		.enabled = true, \
		.friction_positive_current_a = 1.55f, \
		.friction_negative_current_a = 1.50f, \
		.breakaway_positive_current_a = 1.85f, \
		.breakaway_negative_current_a = 1.80f, \
		.current_slew_rate_a_per_s = 15.0f, \
		.position_enter_rad = 0.001f, \
		.position_exit_rad = 0.003f, \
		.reference_speed_rad_s = 0.03f, \
		.stop_speed_rad_s = 0.02f, \
		.move_speed_rad_s = 0.05f, \
		.stuck_time_s = 0.05f, \
		.landing_position_rad = 0.006f, \
		.landing_speed_rad_s = 0.10f, \
		.recovery_delay_s = 0.20f, \
		.recovery_pulse_time_s = 0.05f, \
		.recovery_cooldown_s = 0.20f \
	}

#define PRODUCT_POSITION_FRICTION_INITIALIZER_(tag_) \
	PRODUCT_POSITION_FRICTION_##tag_##_INITIALIZER
#define PRODUCT_POSITION_FRICTION_INITIALIZER(tag_) \
	PRODUCT_POSITION_FRICTION_INITIALIZER_(tag_)

#define PRODUCT_CONTROL_INITIALIZER(default_speed_, default_position_speed_, \
	load_tag_) \
	{ \
		.speed_loop_frequency_hz = 2000U, \
		.position_loop_frequency_hz = 1000U, \
		.cascade_position_loop_frequency_hz = 5000U, \
		.current_loop_bandwidth_rad_s = 500.0f * PRODUCT_TWO_PI, \
		.open_loop_voltage_v = 1.0f, \
		.open_loop_electrical_velocity_rad_s = 12.0f, \
		.open_loop_initial_theta_rad = 0.0f, \
		.default_speed_limit_rad_s = (default_speed_), \
		.speed_acceleration_rad_s2 = 50.0f * PRODUCT_TWO_PI, \
		.speed_deceleration_rad_s2 = 50.0f * PRODUCT_TWO_PI, \
		.speed_kp = 0.05f, \
		.speed_ki = 0.5f, \
		.position_acceleration_rad_s2 = 0.125f * PRODUCT_TWO_PI, \
		.position_deceleration_rad_s2 = 0.125f * PRODUCT_TWO_PI, \
		.default_position_max_speed_rad_s = (default_position_speed_), \
		.position_kp_a_per_rad = 8.0f, \
		.position_kd_a_per_rad_s = 0.50f, \
		.position_ki_a_per_rad_s = 10.0f, \
		.position_integral_limit_a = 5.0f, \
		.position_error_window_rad = 0.001f, \
		.cascade_position_kp_per_s = 0.05f, \
		.cascade_position_kd = 0.50f, \
		.parameter_limits = PRODUCT_PARAMETER_LIMITS_INITIALIZER, \
		.sensorless = PRODUCT_SENSORLESS_CONTROL_INITIALIZER, \
		.position_friction = PRODUCT_POSITION_FRICTION_INITIALIZER(load_tag_) \
	}

#define PRODUCT_PHASE_RESISTANCE_INITIALIZER \
	{ \
		.test_current_low_a = 1.0f, \
		.test_current_high_a = 2.0f, \
		.test_current_max_a = 3.0f, \
		.test_current_min_a = 0.5f, \
		.current_tolerance_a = 0.10f, \
		.q_current_tolerance_a = 0.10f, \
		.voltage_tolerance_v = 0.01f, \
		.voltage_min_delta_v = 0.005f, \
		.voltage_filter_alpha = 0.02f, \
		.ramp_time_ms = 200U, \
		.settle_time_ms = 200U, \
		.sample_time_ms = 100U, \
		.pause_time_ms = 100U, \
		.timeout_ms = 3000U, \
		.balance_warning_pct = 3.0f, \
		.balance_fault_pct = 5.0f, \
		.design_tolerance_pct = 20.0f \
	}

#define PRODUCT_ENCODER_STARTUP_INITIALIZER_(tag_) \
	PRODUCT_ENCODER_STARTUP_##tag_##_INITIALIZER
#define PRODUCT_ENCODER_STARTUP_INITIALIZER(tag_) \
	PRODUCT_ENCODER_STARTUP_INITIALIZER_(tag_)

#define PRODUCT_ANGLE_COMMISSIONING_INITIALIZER(load_tag_, stable_timeout_, \
	turns_, startup_timeout_, stop_current_ramp_, minimum_align_current_) \
	{ \
		.startup = PRODUCT_ENCODER_STARTUP_INITIALIZER(load_tag_), \
		.linearization_align_time_s = 1.5f, \
		.linearization_ramp_time_s = 3.0f, \
		.linearization_speed_electrical_rad_s = 20.0f * PRODUCT_TWO_PI, \
		.linearization_timeout_factor = 1.5f, \
		.linearization_unlock_timeout_s = 1.0f, \
		.calibration_speed_mechanical_rad_s = 20.0f, \
		.calibration_speed_error_ratio = 0.20f, \
		.calibration_speed_stable_time_s = 0.50f, \
		.calibration_speed_stable_timeout_s = (stable_timeout_), \
		.calibration_align_sample_time_s = 0.10f, \
		.calibration_mechanical_turns = (turns_), \
		.calibration_verify_mechanical_turns = 1U, \
		.calibration_min_samples_per_bin = 16U, \
		.calibration_lut_build_bins_per_cycle = 8U, \
		.calibration_find_origin_timeout_s = 1.50f, \
		.calibration_sample_timeout_s = 12.00f, \
		.calibration_verify_timeout_s = 4.00f, \
		.calibration_max_rms_residual_q15 = 256U, \
		.calibration_max_peak_residual_q15 = 1024U, \
		.calibration_startup_timeout_s = (startup_timeout_), \
		.calibration_stop_speed_margin = 1.05f, \
		.calibration_stop_deceleration_time_s = 1.00f, \
		.calibration_stop_deceleration_timeout_s = 3.00f, \
		.calibration_stop_speed_tolerance_ratio = 0.10f, \
		.calibration_stop_current_ramp_time_s = (stop_current_ramp_), \
		.electrical_zero_current_ramp_time_s = 0.50f, \
		.electrical_zero_hold_time_s = 1.00f, \
		.electrical_zero_min_align_current_a = (minimum_align_current_), \
		.direction_align_time_s = 1.00f, \
		.direction_speed_electrical_rad_s = 20.0f \
	}

#define PRODUCT_FRICTION_IDENTIFICATION_INITIALIZER \
	{ \
		.speed_points_rad_s = \
			{0.62831853f, 1.25663706f, 2.51327412f, 3.14159265f}, \
		.speed_point_count = 4U, \
		.stable_time_s = 0.75f, \
		.track_timeout_s = 8.0f, \
		.sample_timeout_s = 15.0f, \
		.stop_hold_time_s = 0.30f, \
		.stop_timeout_s = 5.0f, \
		.speed_tolerance_ratio = 0.05f, \
		.minimum_speed_tolerance_rad_s = 0.08f, \
		.stop_speed_rad_s = 0.12f, \
		.sample_turns = 1.0f, \
		.minimum_sample_time_s = 0.50f, \
		.current_ratio_max = 0.90f, \
		.saturation_time_s = 0.25f, \
		.rmse_floor_a = 0.05f, \
		.rmse_ratio_max = 0.25f \
	}

#define PRODUCT_COGGING_INITIALIZER(stage_timeout_, tolerance_) \
	{ \
		.speed_rad_s = 0.62831853f, \
		.turns = 2U, \
		.stable_time_s = 0.75f, \
		.stage_timeout_s = (stage_timeout_), \
		.speed_tolerance_ratio = (tolerance_), \
		.minimum_samples_per_bin = 4U, \
		.maximum_current_a = 1.0f \
	}

#define PRODUCT_COMMISSIONING_INITIALIZER(load_tag_, stable_timeout_, turns_, \
	startup_timeout_, stop_current_ramp_, minimum_align_current_, \
	cogging_timeout_, cogging_tolerance_) \
	{ \
		.phase_resistance = PRODUCT_PHASE_RESISTANCE_INITIALIZER, \
		.angle = PRODUCT_ANGLE_COMMISSIONING_INITIALIZER(load_tag_, \
			stable_timeout_, turns_, startup_timeout_, stop_current_ramp_, \
			minimum_align_current_), \
		.friction = PRODUCT_FRICTION_IDENTIFICATION_INITIALIZER, \
		.cogging = PRODUCT_COGGING_INITIALIZER(cogging_timeout_, \
			cogging_tolerance_) \
	}

#define PRODUCT_CONFIG_INITIALIZER(variant_, fingerprint_, load_, \
	default_speed_, default_position_speed_, load_tag_, \
	stable_timeout_, turns_, startup_timeout_, stop_current_ramp_, \
	minimum_align_current_, cogging_timeout_, cogging_tolerance_) \
	{ \
		.identity = \
		{ \
			.product_id = PRODUCT_CATALOG_PRODUCT_CURRENT, \
			.variant_id = (variant_), \
			.platform_id = PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET, \
			.configuration_fingerprint = (fingerprint_), \
			.hardware_revision = PRODUCT_HARDWARE_REVISION, \
			.configuration_schema_version = PRODUCT_CONFIG_SCHEMA_VERSION \
		}, \
		.board = &ProductCatalog_VectorMiniStBoard, \
		.motor = &ProductCatalog_Ht8115_4Motor, \
		.load = &(load_), \
		.angle_sensors = \
		{ \
			{ \
				.instance_id = 1U, \
				.design = &ProductCatalog_Tle5012bAngleSensor, \
				.source = PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL, \
				.role = PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY, \
				.endpoint = PRODUCT_CATALOG_ENDPOINT_ANGLE_PRIMARY \
			}, \
			{0} \
		}, \
		.angle_sensor_count = 1U, \
		.temperature_sensors = \
		{ \
			{ \
				.instance_id = 1U, \
				.design = &ProductCatalog_InternalTemperatureSensor, \
				.source = PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL, \
				.zone = PRODUCT_TEMPERATURE_ZONE_MCU, \
				.endpoint = PRODUCT_CATALOG_ENDPOINT_TEMPERATURE_INTERNAL, \
				.sample_period_ms = 1U, \
				.pending_timeout_ms = 2U, \
				.protection_enabled = false, \
				.protection_limit_c = 100.0f \
			}, \
			{0}, \
			{0} \
		}, \
		.temperature_sensor_count = 1U, \
		.safety = \
		{ \
			.software_overcurrent_trip_a = 18.0f, \
			.undervoltage_trip_v = 10.0f, \
			.overvoltage_trip_v = 30.0f, \
			.bus_voltage_filter_alpha = 0.05f, \
			.overcurrent_confirm_cycles = 5U, \
			.voltage_confirm_cycles = 10000U, \
			.temperature_invalid_is_fault = false \
		}, \
		.motor_acceptance = PRODUCT_MOTOR_ACCEPTANCE_INITIALIZER, \
		.control = PRODUCT_CONTROL_INITIALIZER(default_speed_, \
			default_position_speed_, load_tag_), \
		.commissioning_tuning = PRODUCT_COMMISSIONING_INITIALIZER(load_tag_, \
			stable_timeout_, turns_, startup_timeout_, stop_current_ramp_, \
			minimum_align_current_, cogging_timeout_, cogging_tolerance_), \
		.feedback = \
		{ \
			.electrical_angle = {PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U}, \
			.motor_velocity = {PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U}, \
			.motor_position = {PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U}, \
			.output_position = {PRODUCT_FEEDBACK_SOURCE_NONE, \
				PRODUCT_CONFIG_SENSOR_INDEX_NONE}, \
			.calibration_reference = {PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER, \
				PRODUCT_CONFIG_SENSOR_INDEX_NONE}, \
			.fallback_electrical_angle = {PRODUCT_FEEDBACK_SOURCE_NONE, \
				PRODUCT_CONFIG_SENSOR_INDEX_NONE} \
		}, \
		.features = \
		{ \
			.speed_control = PRODUCT_FEATURE_REQUIRED, \
			.position_control = PRODUCT_FEATURE_REQUIRED, \
			.output_position_control = PRODUCT_FEATURE_OFF, \
			.sensorless_control = PRODUCT_FEATURE_OPTIONAL, \
			.angle_redundancy_monitor = PRODUCT_FEATURE_OFF, \
			.temperature_monitoring = PRODUCT_FEATURE_REQUIRED, \
			.temperature_protection = PRODUCT_FEATURE_OFF, \
			.required_monitored_temperature_zones = \
				PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU), \
			.required_protected_temperature_zones = 0U \
		}, \
		.commissioning = \
		{ \
			.current_offset = PRODUCT_COMMISSIONING_REQUIRED, \
			.phase_resistance = PRODUCT_COMMISSIONING_REQUIRED, \
			.angle_direction = PRODUCT_COMMISSIONING_REQUIRED, \
			.angle_linearization = PRODUCT_COMMISSIONING_REQUIRED, \
			.electrical_zero = PRODUCT_COMMISSIONING_REQUIRED, \
			.mechanical_zero = PRODUCT_COMMISSIONING_REQUIRED, \
			.dual_angle_alignment = PRODUCT_COMMISSIONING_DISABLED, \
			.friction_identification = PRODUCT_COMMISSIONING_REQUIRED, \
			.cogging_identification = PRODUCT_COMMISSIONING_REQUIRED, \
			.sensorless_validation = PRODUCT_COMMISSIONING_DISABLED, \
			.save_results = PRODUCT_COMMISSIONING_REQUIRED \
		}, \
		.can = \
		{ \
			.mode = PRODUCT_CAN_MODE_CLASSIC, \
			.endpoint = PRODUCT_CATALOG_ENDPOINT_CAN_CONTROL, \
			.nominal_bitrate_kbps = 1000U, \
			.data_bitrate_kbps = 0U, \
			.bit_rate_switching = false, \
			.maximum_payload_bytes = 8U, \
			.default_node_id = 0U, \
			.heartbeat_ms = 500U, \
			.minimum_heartbeat_ms = 500U, \
			.maximum_heartbeat_ms = 1000U \
		}, \
		.service_stream = \
		{ \
			.enabled = true, \
			.endpoint = PRODUCT_CATALOG_ENDPOINT_SERVICE_STREAM \
		} \
	}

#define PRODUCT_MANIFEST_INITIALIZER(load_compatibility_, fingerprint_) \
	{ \
		.product_id = PRODUCT_CATALOG_PRODUCT_CURRENT, \
		.mcu_id = PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET, \
		.hardware_revision = PRODUCT_HARDWARE_REVISION, \
		.hardware_profile_id = PRODUCT_CATALOG_HARDWARE_COMPATIBILITY_ID, \
		.motor_profile_id = PRODUCT_CATALOG_MOTOR_COMPATIBILITY_ID, \
		.encoder_profile_id = PRODUCT_CATALOG_ENCODER_COMPATIBILITY_ID, \
		.mechanical_load_profile_id = (load_compatibility_), \
		.control_tuning_profile_id = PRODUCT_CATALOG_CONTROL_COMPATIBILITY_ID, \
		.memory_layout_profile_id = PRODUCT_CATALOG_STORAGE_COMPATIBILITY_ID, \
		.parameter_schema_version = PRODUCT_PARAMETER_SCHEMA_VERSION, \
		.boot_image_contract_version = PRODUCT_BOOT_IMAGE_CONTRACT_VERSION, \
		.firmware_version_major = FIRMWARE_VERSION_MAJOR, \
		.firmware_version_minor = FIRMWARE_VERSION_MINOR, \
		.firmware_version_patch = FIRMWARE_VERSION_PATCH, \
		.is_production_release = FIRMWARE_IS_PRODUCTION_RELEASE, \
		.build_number = FIRMWARE_BUILD_NUMBER, \
		.configuration_fingerprint = (fingerprint_) \
	}

#define PRODUCT_PERSISTENCE_INITIALIZER(load_compatibility_, legacy_) \
	{ \
		.hardware_compatibility_id = \
			PRODUCT_CATALOG_HARDWARE_COMPATIBILITY_ID, \
		.motor_compatibility_id = PRODUCT_CATALOG_MOTOR_COMPATIBILITY_ID, \
		.encoder_compatibility_id = PRODUCT_CATALOG_ENCODER_COMPATIBILITY_ID, \
		.mechanical_load_compatibility_id = (load_compatibility_), \
		.control_compatibility_id = PRODUCT_CATALOG_CONTROL_COMPATIBILITY_ID, \
		.storage_layout_compatibility_id = \
			PRODUCT_CATALOG_STORAGE_COMPATIBILITY_ID, \
		.allow_erased_fingerprint_migration = (legacy_) \
	}

#if defined(PRODUCT_CATALOG_INCLUDE_ALL) || \
	PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_NO_DAMPER
static const ProductLoadDesign ProductCatalog_NoDamperLoad =
{
	.design_id = PRODUCT_CATALOG_LOAD_NO_DAMPER,
	.transmission_ratio = 1.0f,
	.maximum_output_speed_rad_s = 38.9557489f,
	.friction_identification_allowed = true,
	.cogging_identification_allowed = true
};

static const ProductConfig ProductCatalog_NoDamperConfig =
	PRODUCT_CONFIG_INITIALIZER(PRODUCT_CATALOG_VARIANT_NO_DAMPER,
		PRODUCT_CATALOG_FINGERPRINT_NO_DAMPER, ProductCatalog_NoDamperLoad,
		(372.0f / 60.0f) * PRODUCT_TWO_PI, 0.125f * PRODUCT_TWO_PI,
		NO_DAMPER, 5.00f, 10U, 8.0f,
		0.50f, 0.0f, 30.0f, 0.10f);

const ProductCatalogEntry ProductCatalog_VectorMiniStHt8115NoDamper =
{
	.config = &ProductCatalog_NoDamperConfig,
	.persistence = PRODUCT_PERSISTENCE_INITIALIZER(
		PRODUCT_CATALOG_LOAD_NO_DAMPER_COMPATIBILITY_ID, false),
	.manifest = PRODUCT_MANIFEST_INITIALIZER(
		PRODUCT_CATALOG_LOAD_NO_DAMPER_COMPATIBILITY_ID,
		PRODUCT_CATALOG_FINGERPRINT_NO_DAMPER)
};
#endif

#if defined(PRODUCT_CATALOG_INCLUDE_ALL) || \
	PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_DAMPED
static const ProductLoadDesign ProductCatalog_DampedLoad =
{
	.design_id = PRODUCT_CATALOG_LOAD_DAMPED,
	.transmission_ratio = 1.0f,
	.maximum_output_speed_rad_s = 3.14159265f,
	.friction_identification_allowed = true,
	.cogging_identification_allowed = true
};

static const ProductConfig ProductCatalog_DampedConfig =
	PRODUCT_CONFIG_INITIALIZER(PRODUCT_CATALOG_VARIANT_DAMPED,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED, ProductCatalog_DampedLoad,
		0.50f * PRODUCT_TWO_PI, 0.50f * PRODUCT_TWO_PI,
		DAMPED, 10.00f, 5U, 12.0f,
		0.10f, 4.50f, 45.0f, 0.25f);

const ProductCatalogEntry ProductCatalog_VectorMiniStHt8115Damped =
{
	.config = &ProductCatalog_DampedConfig,
	.persistence = PRODUCT_PERSISTENCE_INITIALIZER(
		PRODUCT_CATALOG_LOAD_DAMPED_COMPATIBILITY_ID, true),
	.manifest = PRODUCT_MANIFEST_INITIALIZER(
		PRODUCT_CATALOG_LOAD_DAMPED_COMPATIBILITY_ID,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED)
};
#endif

const ProductCatalogEntry *ProductCatalog_GetCurrent(void)
{
#if PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_NO_DAMPER
	return &ProductCatalog_VectorMiniStHt8115NoDamper;
#else
	return &ProductCatalog_VectorMiniStHt8115Damped;
#endif
}

const ProductCatalogEntry *ProductCatalog_GetByVariant(
	ProductComponentId variant_id)
{
#if defined(PRODUCT_CATALOG_INCLUDE_ALL)
	if (variant_id == PRODUCT_CATALOG_VARIANT_NO_DAMPER)
		return &ProductCatalog_VectorMiniStHt8115NoDamper;
	if (variant_id == PRODUCT_CATALOG_VARIANT_DAMPED)
		return &ProductCatalog_VectorMiniStHt8115Damped;
	return 0;
#else
	const ProductCatalogEntry *current = ProductCatalog_GetCurrent();

	return variant_id == current->config->identity.variant_id ? current : 0;
#endif
}
