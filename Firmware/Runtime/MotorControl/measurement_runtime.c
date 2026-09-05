#include "measurement_runtime.h"

#include "motor_state_runtime.h"

bool Measurement_Capture(CurrentControlContext *CurrentControl)
{
    if (CurrentControl == 0 || CurrentControl->measurement_port.read_raw_sample == 0)
        return false;
    return CurrentControl->measurement_port.read_raw_sample(
        CurrentControl->measurement_port.context, &CurrentControl->measurement_raw);
}

bool Measurement_Configure(MeasurementModelContext *context,
    const MotorControlContext *motor, const BoardProfile *board_profile)
{
    MeasurementModelConfig config;

    if (context == 0 || motor == 0 || board_profile == 0)
        return false;

    config.phase_a_offset_adc = motor->configuration.phase_a_current_offset_adc;
    config.phase_b_offset_adc = motor->configuration.phase_b_current_offset_adc;
    config.phase_c_offset_adc = motor->configuration.phase_c_current_offset_adc;
	config.minimum_valid_offset_adc = board_profile->minimum_current_offset_adc;
	config.maximum_valid_offset_adc = board_profile->maximum_current_offset_adc;
    config.current_a_per_count = board_profile->current_a_per_adc_count;
    config.bus_voltage_v_per_count = board_profile->bus_voltage_v_per_adc_count;
    config.overcurrent_trip_a = board_profile->overcurrent_trip_a;
    config.overvoltage_trip_v = board_profile->overvoltage_trip_v;
    config.undervoltage_trip_v = board_profile->undervoltage_trip_v;
    config.thermistor_series_resistance_kohm =
        board_profile->thermistor_series_resistance_kohm;
    config.thermistor_nominal_resistance_kohm =
        board_profile->thermistor_nominal_resistance_kohm;
    config.thermistor_beta_k = board_profile->thermistor_beta_k;
    config.thermistor_nominal_temperature_c =
        board_profile->thermistor_nominal_temperature_c;
    config.maximum_temperature_c = board_profile->maximum_temperature_c;
	config.temperature_protection_enabled =
		board_profile->temperature_protection_enabled;
    config.overcurrent_confirm_cycles =
        board_profile->overcurrent_confirm_cycles;
    config.voltage_confirm_cycles = board_profile->voltage_confirm_cycles;
    config.temperature_sample_divider =
        board_profile->temperature_sample_divider;
    return MeasurementModel_Configure(context, &config);
}

bool Measurement_Process(MeasurementModelContext *context,
    CurrentControlContext *CurrentControl, bool protection_is_active)
{
    MeasurementModelInput input;
    MeasurementModelOutput output;

    if (context == 0 || CurrentControl == 0)
        return false;

    input.phase_a_adc = CurrentControl->measurement_raw.phase_a_adc;
    input.phase_b_adc = CurrentControl->measurement_raw.phase_b_adc;
    input.phase_c_adc = CurrentControl->measurement_raw.phase_c_adc;
    input.bus_voltage_adc = CurrentControl->measurement_raw.bus_voltage_adc;
    input.temperature_adc = CurrentControl->measurement_raw.temperature_adc;
    input.protection_is_active = protection_is_active;

    if (!MeasurementModel_Update(context, &input, &output))
        return false;

    CurrentControl->phase_a_current_a = output.phase_a_current_a;
    CurrentControl->phase_b_current_a = output.phase_b_current_a;
    CurrentControl->phase_c_current_a = output.phase_c_current_a;
    CurrentControl->bus_voltage_v = output.bus_voltage_v;
    CurrentControl->filtered_bus_voltage_v = output.bus_voltage_filtered_v;
    CurrentControl->temperature_c = output.temperature_c;

    if ((output.faults & MEASUREMENT_FAULT_OVER_VOLTAGE) != 0U)
        MotorState_RaiseFault(MOTOR_FAULT_OVER_VOLTAGE);
    if ((output.faults & MEASUREMENT_FAULT_UNDER_VOLTAGE) != 0U)
        MotorState_RaiseFault(MOTOR_FAULT_UNDER_VOLTAGE);
    if ((output.faults & MEASUREMENT_FAULT_CURRENT_OFFSET) != 0U)
        MotorState_RaiseFault(MOTOR_FAULT_CURRENT_OFFSET);
    if ((output.faults & MEASUREMENT_FAULT_OVER_CURRENT) != 0U)
        MotorState_RaiseFault(MOTOR_FAULT_OVER_CURRENT);
    if ((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) != 0U)
        MotorState_RaiseFault(MOTOR_FAULT_HIGH_TEMPERATURE);
    return true;
}
