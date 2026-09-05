#include "board_profile.h"

#include "control_loop_config.h"
#include "vector_mini_st_profile.h"

#define BOARD_ADC_REFERENCE_V                  3.3f
#define BOARD_ADC_FULL_SCALE_COUNTS            4095.0f
#define BOARD_BUS_DIVIDER_HIGH_KOHM            10.0f
#define BOARD_BUS_DIVIDER_LOW_KOHM             1.0f
#define CURRENT_SENSE_AMPLIFIER_GAIN           10.0f

#if CURRENT_SENSE_SHUNT_MILLIOHM == CURRENT_SENSE_SHUNT_2_MILLIOHM
#define CURRENT_SENSE_SHUNT_RESISTANCE_OHM     0.002f
#define CURRENT_SENSE_RELIABLE_LIMIT_A         60.0f
#define CURRENT_SENSE_COMMAND_LIMIT_A          30.0f
#define CURRENT_SENSE_CALIBRATION_LIMIT_A      30.0f
#define CURRENT_SENSE_OVERCURRENT_TRIP_A       40.0f
#define CURRENT_SENSE_PATH_COMPENSATION_OHM    0.004f
#elif CURRENT_SENSE_SHUNT_MILLIOHM == CURRENT_SENSE_SHUNT_6_MILLIOHM
#define CURRENT_SENSE_SHUNT_RESISTANCE_OHM     0.006f
#define CURRENT_SENSE_RELIABLE_LIMIT_A         20.0f
#define CURRENT_SENSE_COMMAND_LIMIT_A          10.0f
#define CURRENT_SENSE_CALIBRATION_LIMIT_A      10.0f
#define CURRENT_SENSE_OVERCURRENT_TRIP_A       18.0f
#define CURRENT_SENSE_PATH_COMPENSATION_OHM    0.008f
#else
#error "Unsupported CURRENT_SENSE_SHUNT_MILLIOHM; select 2U or 6U"
#endif

static const BoardProfile ActiveBoardProfile =
{
	.profile_id = ACTIVE_BOARD_PROFILE,
	.control_frequency_hz = CONTROL_LOOP_FREQUENCY_HZ,
	.current_sense_shunt_milliohm = CURRENT_SENSE_SHUNT_MILLIOHM,
	.current_sense_amplifier_gain = CURRENT_SENSE_AMPLIFIER_GAIN,
	.current_sense_reliable_limit_a = CURRENT_SENSE_RELIABLE_LIMIT_A,
	.default_phase_a_current_offset_adc = PARAM_HW_CURRENT_OFFSET_A_COUNTS,
	.default_phase_b_current_offset_adc = PARAM_HW_CURRENT_OFFSET_B_COUNTS,
	.default_phase_c_current_offset_adc = PARAM_HW_CURRENT_OFFSET_C_COUNTS,
	.minimum_current_offset_adc = 1948U,
	.maximum_current_offset_adc = 2148U,
	.current_offset_calibration_sample_count = 20000U,
	.current_a_per_adc_count = BOARD_ADC_REFERENCE_V / BOARD_ADC_FULL_SCALE_COUNTS /
		CURRENT_SENSE_AMPLIFIER_GAIN /
		CURRENT_SENSE_SHUNT_RESISTANCE_OHM,
	.bus_voltage_v_per_adc_count = BOARD_ADC_REFERENCE_V /
		BOARD_ADC_FULL_SCALE_COUNTS *
		(BOARD_BUS_DIVIDER_HIGH_KOHM + BOARD_BUS_DIVIDER_LOW_KOHM) /
		BOARD_BUS_DIVIDER_LOW_KOHM,
	.current_command_limit_a = CURRENT_SENSE_COMMAND_LIMIT_A,
	.calibration_current_limit_a = CURRENT_SENSE_CALIBRATION_LIMIT_A,
	.overcurrent_trip_a = CURRENT_SENSE_OVERCURRENT_TRIP_A,
	.undervoltage_trip_v = 10.0f,
	.overvoltage_trip_v = 30.0f,
	.maximum_temperature_c = 100.0f,
	.inverter_deadtime_s = 2.1e-7f,
	.inverter_deadtime_source = POWER_STAGE_DEADTIME_EXTERNAL_GATE_DRIVER,
	.hardware_break_input_enabled = false,
	.temperature_sensor = TEMPERATURE_SENSOR_MCU_INTERNAL,
	.temperature_protection_enabled =
		PARAM_HW_TEMPERATURE_PROTECTION_ENABLED != 0U,
	.phase_resistance_path_compensation_ohm =
		CURRENT_SENSE_PATH_COMPENSATION_OHM,
	.overcurrent_confirm_cycles = 5U,
	.voltage_confirm_cycles = 10000U,
	.temperature_sample_divider = 20U,
	.default_can_node_id = PARAM_HW_CAN_NODE_ID,
	.default_can_bitrate_kbps = 1000U,
	.default_can_heartbeat_ms = PARAM_HW_CAN_HEARTBEAT_MS,
	.minimum_can_heartbeat_ms = 500U,
	.maximum_can_heartbeat_ms = 1000U,
	.can_fd_enabled = PARAM_HW_CAN_FD_ENABLED != 0U,
	.can_brs_enabled = PARAM_HW_CAN_BRS_ENABLED != 0U
};

const BoardProfile *BoardProfile_GetActive(void)
{
	return &ActiveBoardProfile;
}
