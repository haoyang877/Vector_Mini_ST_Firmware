#include "Core/Application/MotorControl/sensorless_runtime.h"
#include "current_control_math.h"

#include <float.h>

#include "fast_math.h"

static int FluxObserver_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static void FluxObserver_Reset(FluxObserverContext *Fluxobserver)
{
	Fluxobserver->sin = 0.0f;
	Fluxobserver->cos = 1.0f;
	Fluxobserver->y1_last = 0.0f;
	Fluxobserver->y2_last = 0.0f;
	Fluxobserver->etax1 = 0.0f;
	Fluxobserver->etax2 = 0.0f;
	Fluxobserver->phi_err = 0.0f;
	Fluxobserver->x1_last = 0.0f;
	Fluxobserver->x2_last = 0.0f;
	Fluxobserver->x1 = 0.0f;
	Fluxobserver->x2 = 0.0f;
	Fluxobserver->theta_e = 0.0f;
	Fluxobserver->omega_e = 0.0f;
	Fluxobserver->theta_last = 0.0f;
	Fluxobserver->omega_last = 0.0f;
	Fluxobserver->theta_e_unwrapped = 0.0f;
	Fluxobserver->position_epoch++;
}

/**
	* @brief  Initialize flux observer parameters
	* @param  *Fluxobserver: flux observer struct pointer
 **/
void FluxObserver_Initialize(FluxObserverContext *Fluxobserver,
	const ProductSensorlessControlConfig *config,
	const MotorControlContext *MotorControl)
{
	if (Fluxobserver == 0 || config == 0 || MotorControl == 0)
		return;
	FluxObserver_Reset(Fluxobserver);
	Fluxobserver->gamma = config->flux_observer_gamma;
	Fluxobserver->resistance_scale =
		config->flux_observer_resistance_scale;
	Fluxobserver->maximum_correction_step_rad =
		config->flux_observer_max_correction_step_rad;
	Fluxobserver->minimum_flux_weber =
		config->flux_observer_minimum_flux_weber;
	Fluxobserver->velocity_lpf_alpha =
		config->flux_observer_velocity_lpf_alpha;
	Fluxobserver->angle_wrap_threshold_rad =
		config->flux_observer_angle_wrap_threshold_rad;
	Fluxobserver->current_sample_period_s =
		MotorControl->schedule.current_period_s;
	(void)FluxObserver_ConfigureMotor(Fluxobserver, MotorControl);
}

bool FluxObserver_ConfigureMotor(FluxObserverContext *Fluxobserver,
	const MotorControlContext *MotorControl)
{
	float flux_squared;
	float gamma_limit;

	if (Fluxobserver == 0 || MotorControl == 0)
		return false;
	Fluxobserver->effective_resistance_ohm =
		MotorControl->configuration.phase_resistance_ohm *
		Fluxobserver->resistance_scale;
	Fluxobserver->stator_inductance_h =
		(MotorControl->configuration.d_axis_inductance_h +
		 MotorControl->configuration.q_axis_inductance_h) * 0.5f;
	Fluxobserver->flux_weber = MotorControl->configuration.flux_weber;
	Fluxobserver->motor_parameters_valid = 0U;
	if (!FluxObserver_IsFinite(Fluxobserver->effective_resistance_ohm) ||
		Fluxobserver->effective_resistance_ohm <= 0.0f ||
		!FluxObserver_IsFinite(Fluxobserver->resistance_scale) ||
		Fluxobserver->resistance_scale <= 0.0f ||
		!FluxObserver_IsFinite(Fluxobserver->stator_inductance_h) ||
		!FluxObserver_IsFinite(Fluxobserver->flux_weber) ||
		!FluxObserver_IsFinite(Fluxobserver->current_sample_period_s) ||
		Fluxobserver->current_sample_period_s <= 0.0f ||
		Fluxobserver->flux_weber <= Fluxobserver->minimum_flux_weber)
		return false;

	flux_squared = FastMath_Square(Fluxobserver->flux_weber);
	gamma_limit = Fluxobserver->maximum_correction_step_rad /
		(Fluxobserver->current_sample_period_s * flux_squared);
	Fluxobserver->inverse_flux_per_weber = 1.0f / Fluxobserver->flux_weber;
	Fluxobserver->flux_squared_weber2 = flux_squared;
	Fluxobserver->bounded_gamma = FastMath_Min(Fluxobserver->gamma, gamma_limit);
	Fluxobserver->motor_parameters_valid = 1U;
	return true;
}

void SensorlessStartup_Reset(SensorlessStartupContext *Startup)
{
	Startup->state = SENSORLESS_STARTUP_IDLE;
	Startup->open_loop_theta = 0.0f;
	Startup->open_loop_omega = 0.0f;
	Startup->handoff_phase_delta = 0.0f;
	Startup->speed_feedback = 0.0f;
	Startup->lock_speed_feedback = 0.0f;
	Startup->direction = 1.0f;
	Startup->speed_pi_output_max = 1.0f;
	Startup->state_ticks = 0U;
	Startup->speed_loop_ticks = 0U;
	Startup->open_loop_ticks = 0U;
	Startup->lock_ticks = 0U;
	Startup->id_ramp_ticks = 0U;
	Startup->loss_ticks = 0U;
}

/**
	* @brief  Update flux observer
    * @param  *MotorControl: MotorControl struct pointer
	  @param  *Fluxobserver: Fluxobserver struct pointer  
 **/
void FluxObserver_Update(CurrentControlContext *CurrentControl,
	FluxObserverContext *Fluxobserver)
{
	float mod_to_V = CurrentControl->filtered_bus_voltage_v / 1.5f;
	float Rs = Fluxobserver->effective_resistance_ohm;
	float Ls = Fluxobserver->stator_inductance_h;
	float inverse_flux = Fluxobserver->inverse_flux_per_weber;
	float flux_sq = Fluxobserver->flux_squared_weber2;
	float gamma = Fluxobserver->bounded_gamma;
	float delta_theta = 0.0f;

	if (Fluxobserver->motor_parameters_valid == 0U)
	{
		FluxObserver_Reset(Fluxobserver);
		return;
	}

	if(!FluxObserver_IsFinite(Fluxobserver->x1_last) || !FluxObserver_IsFinite(Fluxobserver->x2_last))
		FluxObserver_Reset(Fluxobserver);

	/*update input parameters*/
	Fluxobserver->alpha_current_a = CurrentControl->alpha_current_a;
	Fluxobserver->beta_current_a  = CurrentControl->beta_current_a;
	Fluxobserver->alpha_voltage_v = CurrentControl->alpha_modulation * mod_to_V;
	Fluxobserver->beta_voltage_v  = CurrentControl->beta_modulation  * mod_to_V;
	
	/*flux observer*/
	Fluxobserver->y1_last = -Rs * Fluxobserver->alpha_current_a + Fluxobserver->alpha_voltage_v;
	Fluxobserver->y2_last = -Rs * Fluxobserver->beta_current_a  + Fluxobserver->beta_voltage_v;
	
	Fluxobserver->etax1 = Fluxobserver->x1_last - Ls * Fluxobserver->alpha_current_a;
	Fluxobserver->etax2 = Fluxobserver->x2_last - Ls * Fluxobserver->beta_current_a;
	
	Fluxobserver->phi_err = flux_sq - (FastMath_Square(Fluxobserver->etax1) + FastMath_Square(Fluxobserver->etax2));
	
	Fluxobserver->x1 = Fluxobserver->current_sample_period_s * (Fluxobserver->y1_last + gamma * Fluxobserver->etax1 * Fluxobserver->phi_err) + Fluxobserver->x1_last;
	Fluxobserver->x2 = Fluxobserver->current_sample_period_s * (Fluxobserver->y2_last + gamma * Fluxobserver->etax2 * Fluxobserver->phi_err) + Fluxobserver->x2_last;

	if(!FluxObserver_IsFinite(Fluxobserver->x1) || !FluxObserver_IsFinite(Fluxobserver->x2))
	{
		FluxObserver_Reset(Fluxobserver);
		return;
	}
	
	/*iteration*/
	Fluxobserver->x1_last = Fluxobserver->x1;
	Fluxobserver->x2_last = Fluxobserver->x2;
	
	Fluxobserver->cos = (Fluxobserver->x1 - Ls * Fluxobserver->alpha_current_a) * inverse_flux;
	Fluxobserver->sin = (Fluxobserver->x2 - Ls * Fluxobserver->beta_current_a ) * inverse_flux;
	
	/* FastMath_Atan2 returns [-pi, pi], so a single conditional is sufficient
	 * to map the observer angle to [0, 2pi). */
	Fluxobserver->theta_e = FastMath_Atan2(Fluxobserver->sin, Fluxobserver->cos);
	if (Fluxobserver->theta_e < 0.0f)
		Fluxobserver->theta_e += MATH_TWO_PI;
	if(!FluxObserver_IsFinite(Fluxobserver->theta_e))
	{
		FluxObserver_Reset(Fluxobserver);
		return;
	}
	
	delta_theta = Fluxobserver->theta_e - Fluxobserver->theta_last;
	
	Fluxobserver->theta_last = Fluxobserver->theta_e;
	
	if(delta_theta < -Fluxobserver->angle_wrap_threshold_rad)
		delta_theta += MATH_TWO_PI;
	else if(delta_theta > Fluxobserver->angle_wrap_threshold_rad)
		delta_theta -= MATH_TWO_PI;
	
	Fluxobserver->theta_e_unwrapped += delta_theta;
	FAST_MATH_LOW_PASS(Fluxobserver->omega_e,
		delta_theta / Fluxobserver->current_sample_period_s,
		Fluxobserver->velocity_lpf_alpha);
}

/**
	* @brief  Get observer electrical phase
	* @param  *Fluxobserver: flux observer struct pointer
	* @retval observer electrical phase
 **/
float FluxObserver_GetElectricalAngle(FluxObserverContext *Fluxobserver)
{
	return Fluxobserver->theta_e;
}

/**
	* @brief  Get observer electrical velocity
	* @param  *Fluxobserver: flux observer struct pointer
	* @retval observer electrical velocity
 **/
float FluxObserver_GetElectricalVelocity(FluxObserverContext *Fluxobserver)
{
	return Fluxobserver->omega_e;
}

float FluxObserver_GetUnwrappedElectricalPosition(FluxObserverContext *Fluxobserver)
{
	return Fluxobserver->theta_e_unwrapped;
}

uint32_t FluxObserver_GetPositionEpoch(FluxObserverContext *Fluxobserver)
{
	return Fluxobserver->position_epoch;
}
