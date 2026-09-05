#include "board_profile.h"

#include "current_sense_profile.h"
#include "vector_mini_st_profile.h"

#define BOARD_CONTROL_FREQUENCY_HZ             20000U
#define BOARD_ADC_REFERENCE_V                  3.3f
#define BOARD_ADC_FULL_SCALE_COUNTS            4095.0f
#define BOARD_BUS_DIVIDER_HIGH_KOHM            10.0f
#define BOARD_BUS_DIVIDER_LOW_KOHM             1.0f

static const BoardProfile ActiveBoardProfile =
{
	.profile_id = ACTIVE_BOARD_PROFILE,
	.control_frequency_hz = BOARD_CONTROL_FREQUENCY_HZ,
	.current_sense_shunt_milliohm = CURRENT_SENSE_SHUNT_MILLIOHM,
	.default_current_offset_adc = PARAM_HW_CURRENT_OFFSET_A_COUNTS,
	.minimum_current_offset_adc = 1948U,
	.maximum_current_offset_adc = 2148U,
	.current_offset_calibration_sample_count = 20000U,
	.current_a_per_adc_count = BOARD_ADC_REFERENCE_V / BOARD_ADC_FULL_SCALE_COUNTS /
		CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN /
		CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM,
	.bus_voltage_v_per_adc_count = BOARD_ADC_REFERENCE_V /
		BOARD_ADC_FULL_SCALE_COUNTS *
		(BOARD_BUS_DIVIDER_HIGH_KOHM + BOARD_BUS_DIVIDER_LOW_KOHM) /
		BOARD_BUS_DIVIDER_LOW_KOHM,
	.current_command_limit_a = CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A,
	.calibration_current_limit_a = CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A,
	.overcurrent_trip_a = CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A,
	.undervoltage_trip_v = 10.0f,
	.overvoltage_trip_v = 30.0f,
	.maximum_temperature_c = 100.0f,
	.inverter_deadtime_s = 2.1e-7f,
	.thermistor_series_resistance_kohm = 3.3f,
	.thermistor_nominal_resistance_kohm = 10.0f,
	.thermistor_beta_k = 3455.0f,
	.thermistor_nominal_temperature_c = 25.0f,
	.temperature_protection_enabled =
		PARAM_HW_TEMPERATURE_PROTECTION_ENABLED != 0U,
	.phase_resistance_path_compensation_ohm =
		PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM,
	.overcurrent_confirm_cycles = 5U,
	.voltage_confirm_cycles = 10000U,
	.temperature_sample_divider = 20U,
	.default_can_node_id = PARAM_HW_CAN_NODE_ID,
	.default_can_heartbeat_ms = PARAM_HW_CAN_HEARTBEAT_MS
};

const BoardProfile *BoardProfile_GetActive(void)
{
	return &ActiveBoardProfile;
}
