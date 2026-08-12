#include "foc_sensorless.h"

#include <float.h>

#include "utils.h"
#include "hw_conf.h"

#define FLUX_OBSERVER_DEFAULT_GAMMA        800000.0f
#define FLUX_OBSERVER_MAX_CORRECTION_STEP  0.2f
#define FLUX_OBSERVER_MIN_FLUX             1e-6f

static int Fluxobserver_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static void Fluxobserver_ResetState(Fluxobserver_TypeDef *Fluxobserver)
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
}

/**
	* @brief  Initialize flux observer parameters
	* @param  *Fluxobserver: flux observer struct pointer
 **/
void Fluxobserver_ParamInit(Fluxobserver_TypeDef *Fluxobserver)
{
	Fluxobserver_ResetState(Fluxobserver);
	Fluxobserver->gamma = FLUX_OBSERVER_DEFAULT_GAMMA;
}

void SensorlessStartup_Reset(SensorlessStartup_TypeDef *Startup)
{
	Startup->state = SENSORLESS_STARTUP_IDLE;
	Startup->open_loop_theta = 0.0f;
	Startup->open_loop_omega = 0.0f;
	Startup->handoff_phase_delta = 0.0f;
	Startup->direction = 1.0f;
	Startup->state_ticks = 0U;
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
void Fluxobserver_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Fluxobserver_TypeDef *Fluxobserver)
{
	float mod_to_V = FOC->Vbus_filt / 1.5f;
	float Rs   = MotorControl->motor_phase_resistance;
	float Ls   = (MotorControl->motor_d_inductance + MotorControl->motor_q_inductance) * 0.5f;
	float flux = MotorControl->motor_flux;
	float flux_sq;
	float gamma_limit;
	float gamma;
	float delta_theta = 0.0f;

	if(!Fluxobserver_IsFinite(Rs) || !Fluxobserver_IsFinite(Ls) ||
	   !Fluxobserver_IsFinite(flux) || flux <= FLUX_OBSERVER_MIN_FLUX)
	{
		Fluxobserver_ResetState(Fluxobserver);
		return;
	}

	if(!Fluxobserver_IsFinite(Fluxobserver->x1_last) || !Fluxobserver_IsFinite(Fluxobserver->x2_last))
		Fluxobserver_ResetState(Fluxobserver);

	flux_sq = fast_sq(flux);
	gamma_limit = FLUX_OBSERVER_MAX_CORRECTION_STEP / (Current_Ts * flux_sq);
	gamma = fast_min(Fluxobserver->gamma, gamma_limit);
	
	/*Use phase currents sampled in the current ADC interrupt.*/
	Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
	
	/*update input parameters*/
	Fluxobserver->Ialpha = FOC->Ialpha;
	Fluxobserver->Ibeta  = FOC->Ibeta;
	Fluxobserver->Ualpha = FOC->mod_alpha * mod_to_V;
	Fluxobserver->Ubeta  = FOC->mod_beta  * mod_to_V;
	
	/*flux observer*/
	Fluxobserver->y1_last = -Rs * Fluxobserver->Ialpha + Fluxobserver->Ualpha;
	Fluxobserver->y2_last = -Rs * Fluxobserver->Ibeta  + Fluxobserver->Ubeta;
	
	Fluxobserver->etax1 = Fluxobserver->x1_last - Ls * Fluxobserver->Ialpha;
	Fluxobserver->etax2 = Fluxobserver->x2_last - Ls * Fluxobserver->Ibeta;
	
	Fluxobserver->phi_err = flux_sq - (fast_sq(Fluxobserver->etax1) + fast_sq(Fluxobserver->etax2));
	
	Fluxobserver->x1 = Current_Ts * (Fluxobserver->y1_last + gamma * Fluxobserver->etax1 * Fluxobserver->phi_err) + Fluxobserver->x1_last;
	Fluxobserver->x2 = Current_Ts * (Fluxobserver->y2_last + gamma * Fluxobserver->etax2 * Fluxobserver->phi_err) + Fluxobserver->x2_last;

	if(!Fluxobserver_IsFinite(Fluxobserver->x1) || !Fluxobserver_IsFinite(Fluxobserver->x2))
	{
		Fluxobserver_ResetState(Fluxobserver);
		return;
	}
	
	/*iteration*/
	Fluxobserver->x1_last = Fluxobserver->x1;
	Fluxobserver->x2_last = Fluxobserver->x2;
	
	Fluxobserver->cos = (Fluxobserver->x1 - Ls * Fluxobserver->Ialpha) / flux;
	Fluxobserver->sin = (Fluxobserver->x2 - Ls * Fluxobserver->Ibeta ) / flux;
	
	/*calculate angle with atan*/
	Fluxobserver->theta_e = normalizeAngle(fast_atan2(Fluxobserver->sin, Fluxobserver->cos));
	if(!Fluxobserver_IsFinite(Fluxobserver->theta_e))
	{
		Fluxobserver_ResetState(Fluxobserver);
		return;
	}
	
	delta_theta = Fluxobserver->theta_e - Fluxobserver->theta_last;
	
	Fluxobserver->theta_last = Fluxobserver->theta_e;
	
	if(delta_theta < -4.0f)
		delta_theta += _2PI;
	else if(delta_theta > 4.0f)
		delta_theta -= _2PI;
	
	UTILS_LP_FAST(Fluxobserver->omega_e, delta_theta / Current_Ts, 0.1f);
}

/**
	* @brief  Get observer electrical phase
	* @param  *Fluxobserver: flux observer struct pointer
	* @retval observer electrical phase
 **/
float Observer_GetElePhase(Fluxobserver_TypeDef *Fluxobserver)
{
	return Fluxobserver->theta_e;
}

/**
	* @brief  Get observer electrical velocity
	* @param  *Fluxobserver: flux observer struct pointer
	* @retval observer electrical velocity
 **/
float Observer_GetEleVel(Fluxobserver_TypeDef *Fluxobserver)
{
	return Fluxobserver->omega_e;
}
