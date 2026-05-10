#ifndef __FOC_SENSORLESS_H__
#define __FOC_SENSORLESS_H__

#include "data_type.h"
#include "foc_algorithm.h"

typedef struct
{
	float Ualpha,Ubeta;
	float Ialpha,Ibeta;

	float sin,cos;

	float gamma;

	float y1_last,y2_last;
	float etax1,etax2;
	float phi_err;
	float x1_last,x2_last;
	float x1,x2;
	float theta_e,omega_e;
	float theta_last,omega_last;
}Fluxobserver_TypeDef;

void Fluxobserver_ParamInit(Fluxobserver_TypeDef *Fluxobserver);
void Fluxobserver_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Fluxobserver_TypeDef *Fluxobserver);
float Observer_GetElePhase(Fluxobserver_TypeDef *Fluxobserver);
float Observer_GetEleVel(Fluxobserver_TypeDef *Fluxobserver);

#endif