#ifndef CORE_APPLICATION_MOTOR_CONTROL_SENSORLESS_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_SENSORLESS_RUNTIME_H

#include "motor_control_types.h"
#include "current_control_runtime.h"
#include "Core/Config/product_config.h"

typedef struct
{
	float alpha_voltage_v, beta_voltage_v;
	float alpha_current_a, beta_current_a;

	float sin,cos;

	float gamma;
	float resistance_scale;
	float maximum_correction_step_rad;
	float minimum_flux_weber;
	float velocity_lpf_alpha;
	float angle_wrap_threshold_rad;
	float current_sample_period_s;
	float effective_resistance_ohm;
	float stator_inductance_h;
	float flux_weber;
	float inverse_flux_per_weber;
	float flux_squared_weber2;
	float bounded_gamma;
	uint8_t motor_parameters_valid;

	float y1_last,y2_last;
	float etax1,etax2;
	float phi_err;
	float x1_last,x2_last;
	float x1,x2;
	float theta_e,omega_e;
	float theta_last,omega_last;
	float theta_e_unwrapped;
	uint32_t position_epoch;
}FluxObserverContext;

typedef enum
{
	SENSORLESS_STARTUP_IDLE = 0,
	SENSORLESS_STARTUP_ALIGN,
	SENSORLESS_STARTUP_OPEN_LOOP,
	SENSORLESS_STARTUP_SPEED_LOCK,
	SENSORLESS_STARTUP_HANDOFF,
	SENSORLESS_STARTUP_CLOSED_LOOP
}SensorlessStartupState;

typedef struct
{
	SensorlessStartupState state;
	float open_loop_theta;
	float open_loop_omega;
	float handoff_phase_delta;
	float speed_feedback;
	float lock_speed_feedback;
	float direction;
	float speed_pi_output_max;
	uint32_t state_ticks;
	uint32_t speed_loop_ticks;
	uint32_t open_loop_ticks;
	uint32_t lock_ticks;
	uint32_t id_ramp_ticks;
	uint32_t loss_ticks;
}SensorlessStartupContext;

void FluxObserver_Initialize(FluxObserverContext *Fluxobserver,
	const ProductSensorlessControlConfig *config,
	const MotorControlContext *MotorControl);
bool FluxObserver_ConfigureMotor(FluxObserverContext *Fluxobserver,
	const MotorControlContext *MotorControl);
void SensorlessStartup_Reset(SensorlessStartupContext *Startup);
void FluxObserver_Update(CurrentControlContext *CurrentControl,
	FluxObserverContext *Fluxobserver);
float FluxObserver_GetElectricalAngle(FluxObserverContext *Fluxobserver);
float FluxObserver_GetElectricalVelocity(FluxObserverContext *Fluxobserver);
float FluxObserver_GetUnwrappedElectricalPosition(FluxObserverContext *Fluxobserver);
uint32_t FluxObserver_GetPositionEpoch(FluxObserverContext *Fluxobserver);

#endif
