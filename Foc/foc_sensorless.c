#include "foc_sensorless.h"

#include "utils.h"
#include "hw_conf.h"

/**
	* @brief  Initialize flux observer parameters
	* @param  *Fluxobserver: flux observer struct pointer
 **/
void Fluxobserver_ParamInit(Fluxobserver_TypeDef *Fluxobserver)
{
	Fluxobserver->gamma = 1000000000.0f;
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
	float delta_theta = 0.0f;
	
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
	
	Fluxobserver->phi_err = fast_sq(flux) - (fast_sq(Fluxobserver->etax1) + fast_sq(Fluxobserver->etax2));
	
	Fluxobserver->x1 = Current_Ts * (Fluxobserver->y1_last + Fluxobserver->gamma * Fluxobserver->etax1 * Fluxobserver->phi_err) + Fluxobserver->x1_last;
	Fluxobserver->x2 = Current_Ts * (Fluxobserver->y2_last + Fluxobserver->gamma * Fluxobserver->etax2 * Fluxobserver->phi_err) + Fluxobserver->x2_last;
	
	/*iteration*/
	Fluxobserver->x1_last = Fluxobserver->x1;
	Fluxobserver->x2_last = Fluxobserver->x2;
	
	Fluxobserver->cos = (Fluxobserver->x1 - Ls * Fluxobserver->Ialpha) / flux;
	Fluxobserver->sin = (Fluxobserver->x2 - Ls * Fluxobserver->Ibeta ) / flux;
	
	/*calculate angle with atan*/
	Fluxobserver->theta_e = fast_atan2(Fluxobserver->sin,Fluxobserver->cos) + _PI;
	
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
