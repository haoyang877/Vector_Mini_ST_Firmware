#ifndef PRODUCT_BOARD_PROFILE_H
#define PRODUCT_BOARD_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_SENSE_SHUNT_2_MILLIOHM 2U
#define CURRENT_SENSE_SHUNT_6_MILLIOHM 6U

#ifndef CURRENT_SENSE_SHUNT_MILLIOHM
#define CURRENT_SENSE_SHUNT_MILLIOHM CURRENT_SENSE_SHUNT_6_MILLIOHM
#endif

typedef enum
{
	TEMPERATURE_SENSOR_NONE = 0U,
	TEMPERATURE_SENSOR_MCU_INTERNAL = 1U,
	TEMPERATURE_SENSOR_POWER_STAGE_NTC = 2U
} TemperatureSensorType;

typedef enum
{
	POWER_STAGE_DEADTIME_TIMER = 0U,
	POWER_STAGE_DEADTIME_EXTERNAL_GATE_DRIVER = 1U
} PowerStageDeadtimeSource;

typedef struct
{
	uint16_t profile_id;
	uint32_t control_frequency_hz;
	uint16_t current_sense_shunt_milliohm;
	float current_sense_amplifier_gain;
	float current_sense_reliable_limit_a;
	uint16_t default_phase_a_current_offset_adc;
	uint16_t default_phase_b_current_offset_adc;
	uint16_t default_phase_c_current_offset_adc;
	uint16_t minimum_current_offset_adc;
	uint16_t maximum_current_offset_adc;
	uint32_t current_offset_calibration_sample_count;
	float current_a_per_adc_count;
	float bus_voltage_v_per_adc_count;
	float current_command_limit_a;
	float calibration_current_limit_a;
	float overcurrent_trip_a;
	float undervoltage_trip_v;
	float overvoltage_trip_v;
	float maximum_temperature_c;
	float inverter_deadtime_s;
	PowerStageDeadtimeSource inverter_deadtime_source;
	bool hardware_break_input_enabled;
	TemperatureSensorType temperature_sensor;
	bool temperature_protection_enabled;
	float phase_resistance_path_compensation_ohm;
	uint16_t overcurrent_confirm_cycles;
	uint16_t voltage_confirm_cycles;
	uint16_t temperature_sample_divider;
	uint8_t default_can_node_id;
	uint32_t default_can_bitrate_kbps;
	uint32_t default_can_heartbeat_ms;
	uint32_t minimum_can_heartbeat_ms;
	uint32_t maximum_can_heartbeat_ms;
	bool can_fd_enabled;
	bool can_brs_enabled;
} BoardProfile;

const BoardProfile *BoardProfile_GetActive(void);

#endif
