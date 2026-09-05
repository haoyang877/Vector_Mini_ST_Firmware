#include "sensorless_runtime.h"
#include "current_control_math.h"

#include <float.h>

#include "fast_math.h"
#include "control_loop_config.h"

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
	const ControlTuningProfile *tuning_profile)
{
	if (Fluxobserver == 0 || tuning_profile == 0)
		return;
	FluxObserver_Reset(Fluxobserver);
	Fluxobserver->gamma = tuning_profile->flux_observer_gamma;
	Fluxobserver->resistance_scale =
		tuning_profile->flux_observer_resistance_scale;
	Fluxobserver->maximum_correction_step_rad =
		tuning_profile->flux_observer_max_correction_step_rad;
	Fluxobserver->minimum_flux_weber =
		tuning_profile->flux_observer_minimum_flux_weber;
	Fluxobserver->velocity_lpf_alpha =
		tuning_profile->flux_observer_velocity_lpf_alpha;
	Fluxobserver->angle_wrap_threshold_rad =
		tuning_profile->flux_observer_angle_wrap_threshold_rad;
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
void FluxObserver_Update(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, FluxObserverContext *Fluxobserver)
{
	float mod_to_V = CurrentControl->filtered_bus_voltage_v / 1.5f;
	float Rs   = MotorControl->configuration.phase_resistance_ohm *
		Fluxobserver->resistance_scale;
	float Ls   = (MotorControl->configuration.d_axis_inductance_h + MotorControl->configuration.q_axis_inductance_h) * 0.5f;
	float flux = MotorControl->configuration.flux_weber;
	float flux_sq;
	float gamma_limit;
	float gamma;
	float delta_theta = 0.0f;

	if(!FluxObserver_IsFinite(Rs) || Rs <= 0.0f ||
	   !FluxObserver_IsFinite(Fluxobserver->resistance_scale) ||
	   Fluxobserver->resistance_scale <= 0.0f ||
	   !FluxObserver_IsFinite(Ls) ||
	   !FluxObserver_IsFinite(flux) ||
	   flux <= Fluxobserver->minimum_flux_weber)
	{
		FluxObserver_Reset(Fluxobserver);
		return;
	}

	if(!FluxObserver_IsFinite(Fluxobserver->x1_last) || !FluxObserver_IsFinite(Fluxobserver->x2_last))
		FluxObserver_Reset(Fluxobserver);

	flux_sq = FastMath_Square(flux);
	gamma_limit = Fluxobserver->maximum_correction_step_rad /
		(CURRENT_LOOP_PERIOD_S * flux_sq);
	gamma = FastMath_Min(Fluxobserver->gamma, gamma_limit);
	
	/*Use phase currents sampled in the current ADC interrupt.*/
	CurrentControl_Clarke(CurrentControl->phase_a_current_a, CurrentControl->phase_b_current_a, CurrentControl->phase_c_current_a, &CurrentControl->alpha_current_a, &CurrentControl->beta_current_a);
	
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
	
	Fluxobserver->x1 = CURRENT_LOOP_PERIOD_S * (Fluxobserver->y1_last + gamma * Fluxobserver->etax1 * Fluxobserver->phi_err) + Fluxobserver->x1_last;
	Fluxobserver->x2 = CURRENT_LOOP_PERIOD_S * (Fluxobserver->y2_last + gamma * Fluxobserver->etax2 * Fluxobserver->phi_err) + Fluxobserver->x2_last;

	if(!FluxObserver_IsFinite(Fluxobserver->x1) || !FluxObserver_IsFinite(Fluxobserver->x2))
	{
		FluxObserver_Reset(Fluxobserver);
		return;
	}
	
	/*iteration*/
	Fluxobserver->x1_last = Fluxobserver->x1;
	Fluxobserver->x2_last = Fluxobserver->x2;
	
	Fluxobserver->cos = (Fluxobserver->x1 - Ls * Fluxobserver->alpha_current_a) / flux;
	Fluxobserver->sin = (Fluxobserver->x2 - Ls * Fluxobserver->beta_current_a ) / flux;
	
	/*calculate angle with atan*/
	Fluxobserver->theta_e = FastMath_NormalizeAngle(FastMath_Atan2(Fluxobserver->sin, Fluxobserver->cos));
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
		delta_theta / CURRENT_LOOP_PERIOD_S,
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
