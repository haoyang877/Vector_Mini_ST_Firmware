#ifndef FIRMWARE_CORE_CONFIG_PRODUCT_CONFIG_H
#define FIRMWARE_CORE_CONFIG_PRODUCT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define PRODUCT_CONFIG_SCHEMA_VERSION                  1U
#define PRODUCT_CONFIG_MAX_CURRENT_CHANNELS            3U
#define PRODUCT_CONFIG_MAX_ANGLE_SENSORS               2U
#define PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS         3U
#define PRODUCT_CONFIG_MAX_VALIDATION_ERRORS          32U
#define PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT 4U
#define PRODUCT_CONFIG_SENSOR_INDEX_NONE             0xFFU
#define PRODUCT_CONFIG_ENDPOINT_NONE                 0x0000U

typedef uint32_t ProductComponentId;
typedef uint16_t ProductEndpointId;
typedef uint16_t ProductInstanceId;
typedef uint64_t ProductCapabilityMask;
typedef uint32_t ProductCommissioningStepMask;
typedef uint32_t ProductTemperatureZoneMask;

typedef enum
{
	PRODUCT_CURRENT_SENSE_TOPOLOGY_INVALID = 0,
	PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT,
	PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT,
	PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT,
	PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT
} ProductCurrentSenseTopology;

/* Logical quantity sampled by one physical current-acquisition endpoint.
 * The role is explicit so neither the endpoint order nor an ADC rank implies
 * which phase a sample belongs to. */
typedef enum
{
	PRODUCT_CURRENT_CHANNEL_ROLE_INVALID = 0,
	PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A,
	PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B,
	PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C,
	PRODUCT_CURRENT_CHANNEL_ROLE_DC_LINK
} ProductCurrentChannelRole;

/* Sign convention from raw ADC delta to positive current. The scale stored in
 * ProductCurrentSenseConfig is always a positive magnitude. */
typedef enum
{
	PRODUCT_CURRENT_CHANNEL_POLARITY_INVALID = 0,
	PRODUCT_CURRENT_CHANNEL_POLARITY_NORMAL,
	PRODUCT_CURRENT_CHANNEL_POLARITY_INVERTED
} ProductCurrentChannelPolarity;

typedef enum
{
	PRODUCT_ANGLE_SENSOR_SOURCE_INVALID = 0,
	PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL,
	PRODUCT_ANGLE_SENSOR_SOURCE_INCREMENTAL_QUADRATURE,
	PRODUCT_ANGLE_SENSOR_SOURCE_HALL,
	PRODUCT_ANGLE_SENSOR_SOURCE_RESOLVER,
	PRODUCT_ANGLE_SENSOR_SOURCE_ANALOG
} ProductAngleSensorSource;

typedef enum
{
	PRODUCT_ANGLE_SENSOR_ROLE_INVALID = 0,
	PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY,
	PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_REDUNDANT,
	PRODUCT_ANGLE_SENSOR_ROLE_OUTPUT_SHAFT
} ProductAngleSensorRole;

typedef enum
{
	PRODUCT_TEMPERATURE_SENSOR_SOURCE_INVALID = 0,
	PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL,
	PRODUCT_TEMPERATURE_SENSOR_SOURCE_ANALOG,
	PRODUCT_TEMPERATURE_SENSOR_SOURCE_DIGITAL
} ProductTemperatureSensorSource;

typedef enum
{
	PRODUCT_TEMPERATURE_ZONE_INVALID = 0,
	PRODUCT_TEMPERATURE_ZONE_MCU,
	PRODUCT_TEMPERATURE_ZONE_PCB,
	PRODUCT_TEMPERATURE_ZONE_POWER_STAGE,
	PRODUCT_TEMPERATURE_ZONE_MOTOR_WINDING,
	PRODUCT_TEMPERATURE_ZONE_AMBIENT
} ProductTemperatureZone;

#define PRODUCT_TEMPERATURE_ZONE_MASK(zone) \
	((ProductTemperatureZoneMask)1UL << (uint32_t)(zone))

typedef enum
{
	PRODUCT_FEEDBACK_SOURCE_NONE = 0,
	PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR,
	PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER
} ProductFeedbackSourceKind;

typedef enum
{
	PRODUCT_FEEDBACK_SIGNAL_ELECTRICAL_ANGLE = 0,
	PRODUCT_FEEDBACK_SIGNAL_MOTOR_VELOCITY,
	PRODUCT_FEEDBACK_SIGNAL_MOTOR_POSITION,
	PRODUCT_FEEDBACK_SIGNAL_OUTPUT_POSITION,
	PRODUCT_FEEDBACK_SIGNAL_CALIBRATION_REFERENCE
} ProductFeedbackSignal;

typedef enum
{
	PRODUCT_FEATURE_OFF = 0,
	PRODUCT_FEATURE_OPTIONAL,
	PRODUCT_FEATURE_REQUIRED
} ProductFeatureRequirement;

typedef enum
{
	PRODUCT_COMMISSIONING_DISABLED = 0,
	PRODUCT_COMMISSIONING_AUTO,
	PRODUCT_COMMISSIONING_REQUIRED
} ProductCommissioningRequirement;

typedef enum
{
	PRODUCT_CAN_MODE_DISABLED = 0,
	PRODUCT_CAN_MODE_CLASSIC,
	PRODUCT_CAN_MODE_FD
} ProductCanMode;

/* Capabilities of an angle-sensor model, independent of its installed role. */
#define PRODUCT_ANGLE_CAP_POSITION                    (1UL << 0)
#define PRODUCT_ANGLE_CAP_VELOCITY                    (1UL << 1)
#define PRODUCT_ANGLE_CAP_REPEATABLE_ABSOLUTE_FRAME   (1UL << 2)
#define PRODUCT_ANGLE_CAP_DIRECTION_CALIBRATION       (1UL << 3)
#define PRODUCT_ANGLE_CAP_LINEARIZATION_CALIBRATION   (1UL << 4)
#define PRODUCT_ANGLE_CAP_ELECTRICAL_ZERO_CALIBRATION (1UL << 5)
#define PRODUCT_ANGLE_CAP_MECHANICAL_ZERO_CALIBRATION (1UL << 6)
#define PRODUCT_ANGLE_CAP_DIAGNOSTICS                 (1UL << 7)

/* Structural capabilities derived from the complete product topology. */
#define PRODUCT_CAP_PHASE_CURRENT_FEEDBACK            (UINT64_C(1) << 0)
#define PRODUCT_CAP_CURRENT_OFFSET_CALIBRATION         (UINT64_C(1) << 1)
#define PRODUCT_CAP_BUS_VOLTAGE_FEEDBACK              (UINT64_C(1) << 2)
#define PRODUCT_CAP_SINGLE_SHUNT_RECONSTRUCTION        (UINT64_C(1) << 3)
#define PRODUCT_CAP_SENSORLESS_OBSERVER                (UINT64_C(1) << 4)
#define PRODUCT_CAP_ELECTRICAL_ANGLE_FEEDBACK          (UINT64_C(1) << 5)
#define PRODUCT_CAP_MOTOR_VELOCITY_FEEDBACK            (UINT64_C(1) << 6)
#define PRODUCT_CAP_MOTOR_POSITION_FEEDBACK            (UINT64_C(1) << 7)
#define PRODUCT_CAP_OUTPUT_POSITION_FEEDBACK           (UINT64_C(1) << 8)
#define PRODUCT_CAP_ANGLE_DIRECTION_CALIBRATION        (UINT64_C(1) << 9)
#define PRODUCT_CAP_ANGLE_LINEARIZATION_CALIBRATION    (UINT64_C(1) << 10)
#define PRODUCT_CAP_ELECTRICAL_ZERO_CALIBRATION        (UINT64_C(1) << 11)
#define PRODUCT_CAP_MECHANICAL_ZERO_CALIBRATION        (UINT64_C(1) << 12)
#define PRODUCT_CAP_DUAL_ANGLE_ALIGNMENT               (UINT64_C(1) << 13)
#define PRODUCT_CAP_ANGLE_REDUNDANCY                    (UINT64_C(1) << 14)
#define PRODUCT_CAP_TEMPERATURE_MONITORING              (UINT64_C(1) << 15)
#define PRODUCT_CAP_TEMPERATURE_PROTECTION              (UINT64_C(1) << 16)
#define PRODUCT_CAP_PHASE_RESISTANCE_IDENTIFICATION     (UINT64_C(1) << 17)
#define PRODUCT_CAP_FRICTION_IDENTIFICATION              (UINT64_C(1) << 18)
#define PRODUCT_CAP_COGGING_IDENTIFICATION               (UINT64_C(1) << 19)

#define PRODUCT_COMMISSIONING_STEP_CURRENT_OFFSET       (1UL << 0)
#define PRODUCT_COMMISSIONING_STEP_PHASE_RESISTANCE     (1UL << 1)
#define PRODUCT_COMMISSIONING_STEP_ANGLE_DIRECTION      (1UL << 2)
#define PRODUCT_COMMISSIONING_STEP_ANGLE_LINEARIZATION  (1UL << 3)
#define PRODUCT_COMMISSIONING_STEP_ELECTRICAL_ZERO      (1UL << 4)
#define PRODUCT_COMMISSIONING_STEP_MECHANICAL_ZERO      (1UL << 5)
#define PRODUCT_COMMISSIONING_STEP_DUAL_ANGLE_ALIGNMENT (1UL << 6)
#define PRODUCT_COMMISSIONING_STEP_FRICTION             (1UL << 7)
#define PRODUCT_COMMISSIONING_STEP_COGGING              (1UL << 8)
#define PRODUCT_COMMISSIONING_STEP_SENSORLESS_VALIDATION (1UL << 9)
#define PRODUCT_COMMISSIONING_STEP_SAVE                 (1UL << 10)

typedef struct
{
	ProductComponentId product_id;
	ProductComponentId variant_id;
	ProductComponentId platform_id;
	uint32_t configuration_fingerprint;
	uint16_t hardware_revision;
	uint16_t configuration_schema_version;
} ProductIdentityConfig;

typedef struct
{
	ProductCurrentSenseTopology topology;
	uint8_t physical_channel_count;
	ProductCurrentChannelRole
		channel_roles[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	ProductCurrentChannelPolarity
		channel_polarities[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	ProductEndpointId channel_endpoints[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	/* Positive magnitude; channel_polarities carries the sign convention. */
	float current_a_per_count[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	uint16_t default_offset_count[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	/* Retained as design metadata and for the deployed Flash compatibility
	 * field. It is never restored from Flash as an operating parameter. */
	uint16_t nominal_shunt_milliohm;
	uint16_t minimum_valid_offset_count[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	uint16_t maximum_valid_offset_count[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
	uint32_t offset_calibration_sample_count;
	bool pwm_synchronized;
	uint8_t samples_per_pwm_period;
	bool captures_pwm_sector;
	bool supports_sample_window_compensation;
	bool offset_calibration_supported;
} ProductCurrentSenseConfig;

typedef struct
{
	ProductComponentId design_id;
	ProductComponentId platform_id;
	uint32_t bsp_binding_fingerprint;
	ProductEndpointId motor_drive_endpoint;
	bool require_hardware_shutdown;
	uint32_t control_frequency_hz;
	float reliable_phase_current_limit_a;
	float command_phase_current_limit_a;
	float calibration_phase_current_limit_a;
	float phase_resistance_path_compensation_ohm;
	bool bus_voltage_measurement_available;
	float bus_voltage_v_per_count;
	bool classic_can_supported;
	bool can_fd_supported;
	bool can_brs_supported;
	ProductCurrentSenseConfig current_sense;
} ProductBoardDesign;

typedef struct
{
	ProductComponentId design_id;
	uint8_t pole_pairs;
	float phase_resistance_ohm;
	float d_axis_inductance_h;
	float q_axis_inductance_h;
	float flux_weber;
	float current_limit_a;
	float calibration_current_a;
	float speed_limit_rad_s;
} ProductMotorDesign;

typedef struct
{
	float phase_resistance_min_ohm;
	float phase_resistance_max_ohm;
	float inductance_min_h;
	float inductance_max_h;
	float flux_min_weber;
	float flux_max_weber;
} ProductMotorAcceptanceConfig;

typedef struct
{
	float align_current_ramp_time_s;
	float align_hold_time_s;
	float align_current_a;
	float startup_iq_initial_a;
	float startup_iq_a;
	float startup_iq_ramp_time_s;
	float startup_id_a;
	float minimum_current_limit_a;
	float minimum_electrical_velocity_rad_s;
	float target_electrical_velocity_rad_s;
	float startup_ramp_time_s;
	float speed_lock_time_s;
	float speed_lock_filter_alpha;
	float observer_lock_ratio;
	float angle_handoff_time_s;
	float lock_timeout_s;
	float id_ramp_down_time_s;
	float observer_loss_time_s;
} ProductSensorlessStartupConfig;

typedef struct
{
	ProductSensorlessStartupConfig startup;
	float observer_max_electrical_velocity_rad_s;
	float speed_feedback_lpf_alpha;
	float flux_observer_gamma;
	float flux_observer_resistance_scale;
	float flux_observer_max_correction_step_rad;
	float flux_observer_minimum_flux_weber;
	float flux_observer_velocity_lpf_alpha;
	float flux_observer_angle_wrap_threshold_rad;
} ProductSensorlessControlConfig;

typedef struct
{
	bool enabled;
	float friction_positive_current_a;
	float friction_negative_current_a;
	float breakaway_positive_current_a;
	float breakaway_negative_current_a;
	float current_slew_rate_a_per_s;
	float position_enter_rad;
	float position_exit_rad;
	float reference_speed_rad_s;
	float stop_speed_rad_s;
	float move_speed_rad_s;
	float stuck_time_s;
	float landing_position_rad;
	float landing_speed_rad_s;
	float recovery_delay_s;
	float recovery_pulse_time_s;
	float recovery_cooldown_s;
} ProductPositionFrictionControlConfig;

typedef struct
{
	float speed_limit_max_rad_s;
	float speed_ramp_max_rad_s2;
	float position_ramp_max_rad_s2;
	float position_speed_limit_rad_s;
	float position_kp_limit_a_per_rad;
	float position_kd_limit_a_per_rad_s;
	float position_ki_limit_a_per_rad_s;
	float cascade_position_kp_limit_per_s;
	float cascade_position_kd_limit;
} ProductControlParameterLimits;

typedef struct
{
	uint32_t speed_loop_frequency_hz;
	uint32_t position_loop_frequency_hz;
	uint32_t cascade_position_loop_frequency_hz;
	float current_loop_bandwidth_rad_s;
	float open_loop_voltage_v;
	float open_loop_electrical_velocity_rad_s;
	float open_loop_initial_theta_rad;
	float default_speed_limit_rad_s;
	float speed_acceleration_rad_s2;
	float speed_deceleration_rad_s2;
	float speed_kp;
	float speed_ki;
	float position_acceleration_rad_s2;
	float position_deceleration_rad_s2;
	float default_position_max_speed_rad_s;
	float position_kp_a_per_rad;
	float position_kd_a_per_rad_s;
	float position_ki_a_per_rad_s;
	float position_integral_limit_a;
	float position_error_window_rad;
	float cascade_position_kp_per_s;
	float cascade_position_kd;
	ProductControlParameterLimits parameter_limits;
	ProductSensorlessControlConfig sensorless;
	ProductPositionFrictionControlConfig position_friction;
} ProductControlConfig;

typedef struct
{
	float test_current_low_a;
	float test_current_high_a;
	float test_current_max_a;
	float test_current_min_a;
	float current_tolerance_a;
	float q_current_tolerance_a;
	float voltage_tolerance_v;
	float voltage_min_delta_v;
	float voltage_filter_alpha;
	uint32_t ramp_time_ms;
	uint32_t settle_time_ms;
	uint32_t sample_time_ms;
	uint32_t pause_time_ms;
	uint32_t timeout_ms;
	float balance_warning_pct;
	float balance_fault_pct;
	float design_tolerance_pct;
} ProductPhaseResistanceCommissioningConfig;

typedef struct
{
	ProductSensorlessStartupConfig startup;
	float linearization_align_time_s;
	float linearization_ramp_time_s;
	float linearization_speed_electrical_rad_s;
	float linearization_timeout_factor;
	float linearization_unlock_timeout_s;
	float calibration_speed_mechanical_rad_s;
	float calibration_speed_error_ratio;
	float calibration_speed_stable_time_s;
	float calibration_speed_stable_timeout_s;
	float calibration_align_sample_time_s;
	uint32_t calibration_mechanical_turns;
	uint32_t calibration_verify_mechanical_turns;
	uint16_t calibration_min_samples_per_bin;
	uint16_t calibration_lut_build_bins_per_cycle;
	float calibration_find_origin_timeout_s;
	float calibration_sample_timeout_s;
	float calibration_verify_timeout_s;
	uint16_t calibration_max_rms_residual_q15;
	uint16_t calibration_max_peak_residual_q15;
	float calibration_startup_timeout_s;
	float calibration_stop_speed_margin;
	float calibration_stop_deceleration_time_s;
	float calibration_stop_deceleration_timeout_s;
	float calibration_stop_speed_tolerance_ratio;
	float calibration_stop_current_ramp_time_s;
	float electrical_zero_current_ramp_time_s;
	float electrical_zero_hold_time_s;
	float electrical_zero_min_align_current_a;
	float direction_align_time_s;
	float direction_speed_electrical_rad_s;
} ProductAngleCommissioningConfig;

typedef struct
{
	float speed_points_rad_s[
		PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT];
	uint32_t speed_point_count;
	float stable_time_s;
	float track_timeout_s;
	float sample_timeout_s;
	float stop_hold_time_s;
	float stop_timeout_s;
	float speed_tolerance_ratio;
	float minimum_speed_tolerance_rad_s;
	float stop_speed_rad_s;
	float sample_turns;
	float minimum_sample_time_s;
	float current_ratio_max;
	float saturation_time_s;
	float rmse_floor_a;
	float rmse_ratio_max;
} ProductFrictionIdentificationConfig;

typedef struct
{
	float speed_rad_s;
	uint32_t turns;
	float stable_time_s;
	float stage_timeout_s;
	float speed_tolerance_ratio;
	uint16_t minimum_samples_per_bin;
	float maximum_current_a;
} ProductCoggingIdentificationConfig;

typedef struct
{
	ProductPhaseResistanceCommissioningConfig phase_resistance;
	ProductAngleCommissioningConfig angle;
	ProductFrictionIdentificationConfig friction;
	ProductCoggingIdentificationConfig cogging;
} ProductCommissioningTuningConfig;

typedef struct
{
	ProductComponentId design_id;
	float transmission_ratio;
	float maximum_output_speed_rad_s;
	bool friction_identification_allowed;
	bool cogging_identification_allowed;
} ProductLoadDesign;

typedef struct
{
	ProductComponentId design_id;
	ProductAngleSensorSource source;
	uint32_t counts_per_turn;
	uint32_t capabilities;
} ProductAngleSensorDesign;

typedef struct
{
	ProductInstanceId instance_id;
	const ProductAngleSensorDesign *design;
	ProductAngleSensorSource source;
	ProductAngleSensorRole role;
	ProductEndpointId endpoint;
} ProductAngleSensorInstanceConfig;

typedef struct
{
	ProductComponentId design_id;
	ProductTemperatureSensorSource source;
	float minimum_temperature_c;
	float maximum_temperature_c;
} ProductTemperatureSensorDesign;

typedef struct
{
	ProductInstanceId instance_id;
	const ProductTemperatureSensorDesign *design;
	ProductTemperatureSensorSource source;
	ProductTemperatureZone zone;
	ProductEndpointId endpoint;
	/* Supervisory wall-clock timing. These values are milliseconds and never
	 * scale with the PWM or motor-control frequency. */
	uint16_t sample_period_ms;
	uint16_t pending_timeout_ms;
	bool protection_enabled;
	float protection_limit_c;
} ProductTemperatureSensorInstanceConfig;

typedef struct
{
	float software_overcurrent_trip_a;
	float undervoltage_trip_v;
	float overvoltage_trip_v;
	/* First-order bus-voltage filter coefficient in the interval (0, 1]. */
	float bus_voltage_filter_alpha;
	uint16_t overcurrent_confirm_cycles;
	uint16_t voltage_confirm_cycles;
	bool temperature_invalid_is_fault;
} ProductSafetyConfig;

typedef struct
{
	ProductFeedbackSourceKind kind;
	uint8_t angle_sensor_index;
} ProductFeedbackSourceRef;

typedef struct
{
	ProductFeedbackSourceRef electrical_angle;
	ProductFeedbackSourceRef motor_velocity;
	ProductFeedbackSourceRef motor_position;
	ProductFeedbackSourceRef output_position;
	ProductFeedbackSourceRef calibration_reference;
	ProductFeedbackSourceRef fallback_electrical_angle;
} ProductFeedbackRoutingConfig;

typedef struct
{
	ProductFeatureRequirement speed_control;
	ProductFeatureRequirement position_control;
	ProductFeatureRequirement output_position_control;
	ProductFeatureRequirement sensorless_control;
	ProductFeatureRequirement angle_redundancy_monitor;
	ProductFeatureRequirement temperature_monitoring;
	ProductFeatureRequirement temperature_protection;
	/* Zones which must be observable even when they are diagnostic-only. */
	ProductTemperatureZoneMask required_monitored_temperature_zones;
	/* Zones which must have an enabled and valid shutdown threshold. */
	ProductTemperatureZoneMask required_protected_temperature_zones;
} ProductFeaturePolicy;

typedef struct
{
	ProductCommissioningRequirement current_offset;
	ProductCommissioningRequirement phase_resistance;
	ProductCommissioningRequirement angle_direction;
	ProductCommissioningRequirement angle_linearization;
	ProductCommissioningRequirement electrical_zero;
	ProductCommissioningRequirement mechanical_zero;
	ProductCommissioningRequirement dual_angle_alignment;
	ProductCommissioningRequirement friction_identification;
	ProductCommissioningRequirement cogging_identification;
	ProductCommissioningRequirement sensorless_validation;
	ProductCommissioningRequirement save_results;
} ProductCommissioningPolicy;

typedef struct
{
	ProductCanMode mode;
	ProductEndpointId endpoint;
	uint32_t nominal_bitrate_kbps;
	uint32_t data_bitrate_kbps;
	bool bit_rate_switching;
	uint8_t maximum_payload_bytes;
	uint8_t default_node_id;
	uint32_t heartbeat_ms;
	uint32_t minimum_heartbeat_ms;
	uint32_t maximum_heartbeat_ms;
} ProductCanConfig;

typedef struct
{
	bool enabled;
	ProductEndpointId endpoint;
} ProductServiceStreamConfig;

typedef struct
{
	ProductIdentityConfig identity;
	const ProductBoardDesign *board;
	const ProductMotorDesign *motor;
	const ProductLoadDesign *load;
	ProductAngleSensorInstanceConfig
		angle_sensors[PRODUCT_CONFIG_MAX_ANGLE_SENSORS];
	uint8_t angle_sensor_count;
	ProductTemperatureSensorInstanceConfig
		temperature_sensors[PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS];
	uint8_t temperature_sensor_count;
	ProductSafetyConfig safety;
	ProductMotorAcceptanceConfig motor_acceptance;
	ProductControlConfig control;
	ProductCommissioningTuningConfig commissioning_tuning;
	ProductFeedbackRoutingConfig feedback;
	ProductFeaturePolicy features;
	ProductCommissioningPolicy commissioning;
	ProductCanConfig can;
	ProductServiceStreamConfig service_stream;
} ProductConfig;

typedef struct
{
	ProductCapabilityMask capabilities;
	ProductCommissioningStepMask commissioning_steps;
	uint8_t direction_sensor_mask;
	uint8_t linearization_sensor_mask;
	uint8_t electrical_zero_sensor_mask;
	uint8_t mechanical_zero_sensor_mask;
	ProductTemperatureZoneMask monitored_temperature_zones;
	ProductTemperatureZoneMask protected_temperature_zones;
} ProductConfigDerived;

typedef enum
{
	PRODUCT_CONFIG_SUBJECT_CONFIG = 0,
	PRODUCT_CONFIG_SUBJECT_IDENTITY,
	PRODUCT_CONFIG_SUBJECT_BOARD,
	PRODUCT_CONFIG_SUBJECT_MOTOR,
	PRODUCT_CONFIG_SUBJECT_LOAD,
	PRODUCT_CONFIG_SUBJECT_CURRENT_SENSE,
	PRODUCT_CONFIG_SUBJECT_ANGLE_SENSOR,
	PRODUCT_CONFIG_SUBJECT_TEMPERATURE_SENSOR,
	PRODUCT_CONFIG_SUBJECT_FEEDBACK_ROUTING,
	PRODUCT_CONFIG_SUBJECT_FEATURE_POLICY,
	PRODUCT_CONFIG_SUBJECT_COMMISSIONING_POLICY,
	PRODUCT_CONFIG_SUBJECT_COMMUNICATION,
	PRODUCT_CONFIG_SUBJECT_SAFETY_POLICY,
	PRODUCT_CONFIG_SUBJECT_MOTOR_ACCEPTANCE,
	PRODUCT_CONFIG_SUBJECT_CONTROL,
	PRODUCT_CONFIG_SUBJECT_COMMISSIONING_TUNING
} ProductConfigValidationSubject;

typedef enum
{
	PRODUCT_CONFIG_ERROR_NONE = 0,
	PRODUCT_CONFIG_ERROR_NULL_CONFIG,
	PRODUCT_CONFIG_ERROR_INVALID_IDENTITY,
	PRODUCT_CONFIG_ERROR_BOARD_REQUIRED,
	PRODUCT_CONFIG_ERROR_MOTOR_REQUIRED,
	PRODUCT_CONFIG_ERROR_LOAD_REQUIRED,
	PRODUCT_CONFIG_ERROR_PLATFORM_MISMATCH,
	PRODUCT_CONFIG_ERROR_INVALID_BOARD_LIMIT,
	PRODUCT_CONFIG_ERROR_INVALID_MOTOR_DESIGN,
	PRODUCT_CONFIG_ERROR_MOTOR_EXCEEDS_BOARD_LIMIT,
	PRODUCT_CONFIG_ERROR_INVALID_LOAD_DESIGN,
	PRODUCT_CONFIG_ERROR_CURRENT_TOPOLOGY_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_COUNT_MISMATCH,
	PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_ENDPOINT_DUPLICATE,
	PRODUCT_CONFIG_ERROR_CURRENT_SCALE_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_PWM_SYNC_REQUIRED,
	PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_TWO_SAMPLES_REQUIRED,
	PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_PWM_SECTOR_REQUIRED,
	PRODUCT_CONFIG_ERROR_SINGLE_SHUNT_WINDOW_COMPENSATION_REQUIRED,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_COUNT_EXCEEDED,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_DESIGN_REQUIRED,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_INVALID,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ID_DUPLICATE,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_SOURCE_MISMATCH,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_INVALID,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ROLE_DUPLICATE,
	PRODUCT_CONFIG_ERROR_ANGLE_SENSOR_ENDPOINT_INVALID,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_COUNT_EXCEEDED,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_DESIGN_REQUIRED,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_INVALID,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ID_DUPLICATE,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_SOURCE_MISMATCH,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ZONE_INVALID,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SENSOR_ENDPOINT_INVALID,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_LIMIT_INVALID,
	PRODUCT_CONFIG_ERROR_FEEDBACK_SOURCE_INVALID,
	PRODUCT_CONFIG_ERROR_FEEDBACK_SENSOR_INDEX_INVALID,
	PRODUCT_CONFIG_ERROR_FEEDBACK_CAPABILITY_MISMATCH,
	PRODUCT_CONFIG_ERROR_FEEDBACK_ROLE_MISMATCH,
	PRODUCT_CONFIG_ERROR_FEATURE_POLICY_INVALID,
	PRODUCT_CONFIG_ERROR_REQUIRED_SPEED_FEEDBACK_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_POSITION_FEEDBACK_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_OUTPUT_POSITION_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_SENSORLESS_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_REDUNDANCY_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_MONITOR_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_PROTECTION_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_REQUIRED_TEMPERATURE_ZONE_UNAVAILABLE,
	PRODUCT_CONFIG_ERROR_COMMISSIONING_POLICY_INVALID,
	PRODUCT_CONFIG_ERROR_COMMISSIONING_STEP_UNSUPPORTED,
	PRODUCT_CONFIG_ERROR_CAN_MODE_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_MODE_UNSUPPORTED,
	PRODUCT_CONFIG_ERROR_CAN_BITRATE_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_BRS_REQUIRES_FD,
	PRODUCT_CONFIG_ERROR_CAN_PAYLOAD_INVALID,
	PRODUCT_CONFIG_ERROR_COMMUNICATION_ENDPOINT_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_SHUNT_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_RANGE_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_DEFAULT_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_OFFSET_CALIBRATION_INVALID,
	PRODUCT_CONFIG_ERROR_BOARD_PATH_COMPENSATION_INVALID,
	PRODUCT_CONFIG_ERROR_SAFETY_LIMIT_INVALID,
	PRODUCT_CONFIG_ERROR_SAFETY_CONFIRMATION_INVALID,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SAMPLE_PERIOD_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_NODE_ID_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID,
	PRODUCT_CONFIG_ERROR_MOTOR_ACCEPTANCE_RANGE_INVALID,
	PRODUCT_CONFIG_ERROR_MOTOR_OUTSIDE_ACCEPTANCE,
	PRODUCT_CONFIG_ERROR_CONTROL_FREQUENCY_INVALID,
	PRODUCT_CONFIG_ERROR_CONTROL_PARAMETER_INVALID,
	PRODUCT_CONFIG_ERROR_CONTROL_LIMIT_INVALID,
	PRODUCT_CONFIG_ERROR_CONTROL_DEFAULT_EXCEEDS_LIMIT,
	PRODUCT_CONFIG_ERROR_SENSORLESS_CONTROL_INVALID,
	PRODUCT_CONFIG_ERROR_POSITION_FRICTION_CONTROL_INVALID,
	PRODUCT_CONFIG_ERROR_PHASE_RESISTANCE_TUNING_INVALID,
	PRODUCT_CONFIG_ERROR_ANGLE_COMMISSIONING_TUNING_INVALID,
	PRODUCT_CONFIG_ERROR_FRICTION_IDENTIFICATION_TUNING_INVALID,
	PRODUCT_CONFIG_ERROR_COGGING_IDENTIFICATION_TUNING_INVALID,
	PRODUCT_CONFIG_ERROR_COMMISSIONING_EXCEEDS_LIMIT,
	/* Appended to preserve all previously assigned diagnostic code values. */
	PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_DUPLICATE,
	PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_ROLE_TOPOLOGY_MISMATCH,
	PRODUCT_CONFIG_ERROR_CURRENT_CHANNEL_POLARITY_INVALID,
	PRODUCT_CONFIG_ERROR_CURRENT_UNUSED_CHANNEL_CONFIGURED,
	PRODUCT_CONFIG_ERROR_TEMPERATURE_PENDING_TIMEOUT_INVALID,
	/* Schema 10 persists exactly one motor-rotor LUT. */
	PRODUCT_CONFIG_ERROR_SECONDARY_ROTOR_LUT_UNSUPPORTED
} ProductConfigErrorCode;

typedef struct
{
	ProductConfigErrorCode code;
	ProductConfigValidationSubject subject;
	uint8_t instance_index;
	uint32_t detail;
} ProductConfigValidationIssue;

typedef struct
{
	ProductConfigValidationIssue
		issues[PRODUCT_CONFIG_MAX_VALIDATION_ERRORS];
	uint16_t stored_error_count;
	uint16_t total_error_count;
	bool truncated;
	ProductConfigDerived derived;
} ProductConfigValidationResult;

typedef enum
{
	PRODUCT_CONFIG_RUNTIME_OK = 0,
	PRODUCT_CONFIG_RUNTIME_NULL,
	PRODUCT_CONFIG_RUNTIME_IDENTITY,
	PRODUCT_CONFIG_RUNTIME_DESIGN,
	PRODUCT_CONFIG_RUNTIME_CURRENT_SENSE,
	PRODUCT_CONFIG_RUNTIME_ANGLE_SENSOR,
	PRODUCT_CONFIG_RUNTIME_TEMPERATURE_SENSOR,
	PRODUCT_CONFIG_RUNTIME_FEEDBACK,
	PRODUCT_CONFIG_RUNTIME_COMMUNICATION,
	PRODUCT_CONFIG_RUNTIME_SAFETY,
	PRODUCT_CONFIG_RUNTIME_MOTOR_ACCEPTANCE,
	PRODUCT_CONFIG_RUNTIME_CONTROL,
	PRODUCT_CONFIG_RUNTIME_COMMISSIONING_TUNING
} ProductConfigRuntimeError;

bool ProductConfig_Derive(const ProductConfig *config,
	ProductConfigDerived *derived);
bool ProductConfig_Validate(const ProductConfig *config,
	ProductConfigValidationResult *result);
bool ProductConfig_ValidateRuntime(const ProductConfig *config,
	ProductConfigRuntimeError *error);
const char *ProductConfig_ErrorCodeName(ProductConfigErrorCode code);

#endif
