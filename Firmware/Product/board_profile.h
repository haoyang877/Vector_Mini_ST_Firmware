#ifndef PRODUCT_BOARD_PROFILE_H
#define PRODUCT_BOARD_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t profile_id;
	uint32_t control_frequency_hz;
	uint16_t current_sense_shunt_milliohm;
	uint16_t default_current_offset_adc;
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
	float thermistor_series_resistance_kohm;
	float thermistor_nominal_resistance_kohm;
	float thermistor_beta_k;
	float thermistor_nominal_temperature_c;
	bool temperature_protection_enabled;
	float phase_resistance_path_compensation_ohm;
	uint16_t overcurrent_confirm_cycles;
	uint16_t voltage_confirm_cycles;
	uint16_t temperature_sample_divider;
	uint8_t default_can_node_id;
	uint32_t default_can_heartbeat_ms;
} BoardProfile;

const BoardProfile *BoardProfile_GetActive(void);

#endif
