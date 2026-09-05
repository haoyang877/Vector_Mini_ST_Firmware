#ifndef RUNTIME_CURRENT_CONTROL_H
#define RUNTIME_CURRENT_CONTROL_H

#include "motor_control_types.h"
#include "pi_controller.h"
#include "power_stage.h"
#include "measurement_port.h"

typedef struct
{
	float bus_voltage_v, filtered_bus_voltage_v;
	float bus_current_a, filtered_bus_current_a;
	float filtered_power_w;
	float alpha_voltage_v, beta_voltage_v;
	float d_axis_voltage_v, q_axis_voltage_v;
	float phase_a_current_a, phase_b_current_a, phase_c_current_a;
	float alpha_current_a, beta_current_a;
	float filtered_alpha_current_a, filtered_beta_current_a;
	float d_axis_current_a, q_axis_current_a;
	float filtered_d_axis_current_a, filtered_q_axis_current_a;
	PiController id_pi;
	PiController iq_pi;
	/*dq voltage of p.u.*/
	float d_axis_modulation, q_axis_modulation;
	/*alpha-beta voltage of p.u.*/
	float alpha_modulation, beta_modulation;
	/*duty of voltage utilization */
	float modulation_utilization;
	/*duty cycle of abc*/
	float phase_a_duty, phase_b_duty, phase_c_duty;
	int32_t sector;
	/*temprature of power board*/
	float temperature_c;
	PowerStageContext *power_stage;
	MeasurementPort measurement_port;
	MeasurementRawSample measurement_raw;
}CurrentControlContext;

void CurrentControlRuntime_RunVoltage(CurrentControlContext *CurrentControl, float Vd_set, float Vq_set, float phase);
void CurrentControlRuntime_RunClosedLoop(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, float phase, float phase_vel);
void CurrentControlRuntime_ResetControllers(CurrentControlContext *CurrentControl);
void CurrentControlRuntime_RunQVoltage(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, float phase, float phase_vel);

void CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControlContext *CurrentControl);
void CurrentControlRuntime_ApplyLowSideZeroVector(CurrentControlContext *CurrentControl);
#endif
