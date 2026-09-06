#ifndef FIRMWARE_CORE_CONFIG_PRODUCT_CONFIG_H
#define FIRMWARE_CORE_CONFIG_PRODUCT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define PRODUCT_CONFIG_SCHEMA_VERSION                  1U
#define PRODUCT_CONFIG_MAX_CURRENT_CHANNELS            3U
#define PRODUCT_CONFIG_MAX_ANGLE_SENSORS               2U
#define PRODUCT_CONFIG_MAX_TEMPERATURE_SENSORS         3U
#define PRODUCT_CONFIG_MAX_VALIDATION_ERRORS          32U
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
	ProductEndpointId channel_endpoints[PRODUCT_CONFIG_MAX_CURRENT_CHANNELS];
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
	uint16_t sample_divider;
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
	ProductTemperatureZoneMask required_temperature_zones;
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
	PRODUCT_CONFIG_SUBJECT_SAFETY_POLICY
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
	PRODUCT_CONFIG_ERROR_TEMPERATURE_SAMPLE_DIVIDER_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_NODE_ID_INVALID,
	PRODUCT_CONFIG_ERROR_CAN_HEARTBEAT_INVALID
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
	PRODUCT_CONFIG_RUNTIME_SAFETY
} ProductConfigRuntimeError;

bool ProductConfig_Derive(const ProductConfig *config,
	ProductConfigDerived *derived);
bool ProductConfig_Validate(const ProductConfig *config,
	ProductConfigValidationResult *result);
bool ProductConfig_ValidateRuntime(const ProductConfig *config,
	ProductConfigRuntimeError *error);
const char *ProductConfig_ErrorCodeName(ProductConfigErrorCode code);

#endif
