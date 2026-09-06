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
    const MotorControlContext *motor,
    const MeasurementModelConfig *design_config)
{
    MeasurementModelConfig config;

    if (context == 0 || motor == 0 || design_config == 0)
        return false;

	config = *design_config;
    config.phase_a_offset_adc = motor->configuration.phase_a_current_offset_adc;
    config.phase_b_offset_adc = motor->configuration.phase_b_current_offset_adc;
    config.phase_c_offset_adc = motor->configuration.phase_c_current_offset_adc;
    return MeasurementModel_Configure(context, &config);
}

bool Measurement_UpdateCurrentOffsets(MeasurementModelContext *context,
    const MotorControlContext *motor)
{
	if (context == 0 || !context->is_configured)
		return false;
	return Measurement_Configure(context, motor, &context->config);
}

bool Measurement_Process(MeasurementModelContext *context,
	CurrentControlContext *CurrentControl, bool protection_is_active,
	MotorStateContext *motor_state)
{
    MeasurementModelInput input;
    MeasurementModelOutput output;

    if (context == 0 || CurrentControl == 0)
        return false;

    input.phase_a_adc = CurrentControl->measurement_raw.phase_a_adc;
    input.phase_b_adc = CurrentControl->measurement_raw.phase_b_adc;
    input.phase_c_adc = CurrentControl->measurement_raw.phase_c_adc;
    input.bus_voltage_adc = CurrentControl->measurement_raw.bus_voltage_adc;
    input.temperature_c = CurrentControl->measurement_raw.temperature_c;
    input.temperature_valid = CurrentControl->measurement_raw.temperature_valid;
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
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_OVER_VOLTAGE);
    if ((output.faults & MEASUREMENT_FAULT_UNDER_VOLTAGE) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_UNDER_VOLTAGE);
    if ((output.faults & MEASUREMENT_FAULT_CURRENT_OFFSET) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
    if ((output.faults & MEASUREMENT_FAULT_OVER_CURRENT) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_OVER_CURRENT);
    if ((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_HIGH_TEMPERATURE);
    return true;
}
