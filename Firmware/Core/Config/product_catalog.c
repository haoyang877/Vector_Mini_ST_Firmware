#include "product_catalog.h"

const ProductBoardDesign ProductCatalog_CurrentBoard =
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

const ProductMotorDesign ProductCatalog_CurrentMotor =
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

const ProductLoadDesign ProductCatalog_CurrentLoad =
{
	.design_id = PRODUCT_CATALOG_LOAD_DAMPED,
	.transmission_ratio = 1.0f,
	.maximum_output_speed_rad_s = 3.14159265f,
	.friction_identification_allowed = true,
	.cogging_identification_allowed = true
};

const ProductAngleSensorDesign
	ProductCatalog_AbsoluteSerial16BitAngleSensor =
{
	.design_id = PRODUCT_CATALOG_ANGLE_ABSOLUTE_SERIAL_16BIT,
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

const ProductTemperatureSensorDesign
	ProductCatalog_InternalTemperatureSensor =
{
	.design_id = PRODUCT_CATALOG_TEMPERATURE_INTERNAL,
	.source = PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL,
	.minimum_temperature_c = -40.0f,
	.maximum_temperature_c = 150.0f
};

const ProductConfig ProductCatalog_CurrentConfig =
{
	.identity =
	{
		.product_id = PRODUCT_CATALOG_PRODUCT_CURRENT,
		.variant_id = PRODUCT_CATALOG_VARIANT_CURRENT,
		.platform_id = PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET,
		.configuration_fingerprint =
			PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT,
		.hardware_revision = 0U,
		.configuration_schema_version = PRODUCT_CONFIG_SCHEMA_VERSION
	},
	.board = &ProductCatalog_CurrentBoard,
	.motor = &ProductCatalog_CurrentMotor,
	.load = &ProductCatalog_CurrentLoad,
	.angle_sensors =
	{
		{
			.instance_id = 1U,
			.design = &ProductCatalog_AbsoluteSerial16BitAngleSensor,
			.source = PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL,
			.role = PRODUCT_ANGLE_SENSOR_ROLE_MOTOR_ROTOR_PRIMARY,
			.endpoint = PRODUCT_CATALOG_ENDPOINT_ANGLE_PRIMARY
		},
		{0}
	},
	.angle_sensor_count = 1U,
	.temperature_sensors =
	{
		{
			.instance_id = 1U,
			.design = &ProductCatalog_InternalTemperatureSensor,
			.source = PRODUCT_TEMPERATURE_SENSOR_SOURCE_MCU_INTERNAL,
			.zone = PRODUCT_TEMPERATURE_ZONE_MCU,
			.endpoint = PRODUCT_CATALOG_ENDPOINT_TEMPERATURE_INTERNAL,
			.sample_divider = 20U,
			.protection_enabled = true,
			.protection_limit_c = 100.0f
		},
		{0},
		{0}
	},
	.temperature_sensor_count = 1U,
	.safety =
	{
		.software_overcurrent_trip_a = 18.0f,
		.undervoltage_trip_v = 10.0f,
		.overvoltage_trip_v = 30.0f,
		.bus_voltage_filter_alpha = 0.05f,
		.overcurrent_confirm_cycles = 5U,
		.voltage_confirm_cycles = 10000U,
		.temperature_invalid_is_fault = true
	},
	.feedback =
	{
		.electrical_angle =
			{PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U},
		.motor_velocity =
			{PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U},
		.motor_position =
			{PRODUCT_FEEDBACK_SOURCE_ANGLE_SENSOR, 0U},
		.output_position =
			{PRODUCT_FEEDBACK_SOURCE_NONE,
			 PRODUCT_CONFIG_SENSOR_INDEX_NONE},
		.calibration_reference =
			{PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER,
			 PRODUCT_CONFIG_SENSOR_INDEX_NONE},
		.fallback_electrical_angle =
			{PRODUCT_FEEDBACK_SOURCE_SENSORLESS_OBSERVER,
			 PRODUCT_CONFIG_SENSOR_INDEX_NONE}
	},
	.features =
	{
		.speed_control = PRODUCT_FEATURE_REQUIRED,
		.position_control = PRODUCT_FEATURE_REQUIRED,
		.output_position_control = PRODUCT_FEATURE_OFF,
		.sensorless_control = PRODUCT_FEATURE_OPTIONAL,
		.angle_redundancy_monitor = PRODUCT_FEATURE_OFF,
		.temperature_monitoring = PRODUCT_FEATURE_REQUIRED,
		.temperature_protection = PRODUCT_FEATURE_REQUIRED,
		.required_temperature_zones =
			PRODUCT_TEMPERATURE_ZONE_MASK(PRODUCT_TEMPERATURE_ZONE_MCU)
	},
	.commissioning =
	{
		.current_offset = PRODUCT_COMMISSIONING_REQUIRED,
		.phase_resistance = PRODUCT_COMMISSIONING_REQUIRED,
		.angle_direction = PRODUCT_COMMISSIONING_REQUIRED,
		.angle_linearization = PRODUCT_COMMISSIONING_REQUIRED,
		.electrical_zero = PRODUCT_COMMISSIONING_REQUIRED,
		.mechanical_zero = PRODUCT_COMMISSIONING_REQUIRED,
		.dual_angle_alignment = PRODUCT_COMMISSIONING_DISABLED,
		.friction_identification = PRODUCT_COMMISSIONING_REQUIRED,
		.cogging_identification = PRODUCT_COMMISSIONING_REQUIRED,
		.sensorless_validation = PRODUCT_COMMISSIONING_DISABLED,
		.save_results = PRODUCT_COMMISSIONING_REQUIRED
	},
	.can =
	{
		.mode = PRODUCT_CAN_MODE_CLASSIC,
		.endpoint = PRODUCT_CATALOG_ENDPOINT_CAN_CONTROL,
		.nominal_bitrate_kbps = 1000U,
		.data_bitrate_kbps = 0U,
		.bit_rate_switching = false,
		.maximum_payload_bytes = 8U,
		.default_node_id = 0U,
		.heartbeat_ms = 500U,
		.minimum_heartbeat_ms = 500U,
		.maximum_heartbeat_ms = 1000U
	},
	.service_stream =
	{
		.enabled = true,
		.endpoint = PRODUCT_CATALOG_ENDPOINT_SERVICE_STREAM
	}
};

const ProductConfig *ProductCatalog_GetCurrent(void)
{
	return &ProductCatalog_CurrentConfig;
}
