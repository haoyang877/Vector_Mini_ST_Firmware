#ifndef __FOC_ALGORITHM_H__
#define __FOC_ALGORITHM_H__

#include "data_type.h"
#include "foc_pid.h"

typedef struct
{
	float Vbus,Vbus_filt;
	float Ibus,Ibus_filt;
	float Power_filt;
	float Valpha,Vbeta;
	float Vd,Vq;
	float Ia,Ib,Ic;
	float Ialpha,Ibeta;
	float Ialpha_filt,Ibeta_filt;
	float Id,Iq,Id_filt,Iq_filt;
	PI_Controller_TypeDef id_pi;
	PI_Controller_TypeDef iq_pi;
	/*dq voltage of p.u.*/
	float mod_d,mod_q;
	/*alpha-beta voltage of p.u.*/
	float mod_alpha,mod_beta;
	/*duty of voltage utilization */
	float duty;
	/*duty cycle of abc*/
	float dtc_a,dtc_b,dtc_c;
	int32_t sector;
	/*temprature of power board*/
	float temp;
}FOC_TypeDef;

void Clarke_Transform(float a, float b, float c, float *alpha, float *beta);
void Inverse_Clarke_Transform(float alpha, float beta, float *a, float *b, float *c);
void Park_Transform(float alpha, float beta, float theta, float *d, float *q);
void Inverse_Park_Transform(float d, float q, float theta, float *alpha, float *beta);
void SVM_ZeroInjection(float a, float b, float c, float *tA, float *tB, float *tC);
void SVM_SectorJudge(float alpha, float beta, float *tA, float *tB, float *tC ,int32_t *sector);
void Set_A_Duty(float duty);
void Set_B_Duty(float duty);
void Set_C_Duty(float duty);

void FOC_Voltage(FOC_TypeDef *FOC, float Vd_set, float Vq_set, float phase);
void FOC_Current(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel);
void FOC_CurrentController_Reset(FOC_TypeDef *FOC);
void FOC_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel);

void PWM_TurnOnHighSides(void);
void PWM_TurnOnLowSides(void);
#endif
