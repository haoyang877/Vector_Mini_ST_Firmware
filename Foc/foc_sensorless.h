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

typedef enum
{
	SENSORLESS_STARTUP_IDLE = 0,
	SENSORLESS_STARTUP_ALIGN,
	SENSORLESS_STARTUP_OPEN_LOOP,
	SENSORLESS_STARTUP_SPEED_LOCK,
	SENSORLESS_STARTUP_HANDOFF,
	SENSORLESS_STARTUP_CLOSED_LOOP
}SensorlessStartupState_TypeDef;

typedef struct
{
	SensorlessStartupState_TypeDef state;
	float open_loop_theta;
	float open_loop_omega;
	float handoff_phase_delta;
	float direction;
	uint32_t state_ticks;
	uint32_t open_loop_ticks;
	uint32_t lock_ticks;
	uint32_t id_ramp_ticks;
	uint32_t loss_ticks;
}SensorlessStartup_TypeDef;

void Fluxobserver_ParamInit(Fluxobserver_TypeDef *Fluxobserver);
void SensorlessStartup_Reset(SensorlessStartup_TypeDef *Startup);
void Fluxobserver_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Fluxobserver_TypeDef *Fluxobserver);
float Observer_GetElePhase(Fluxobserver_TypeDef *Fluxobserver);
float Observer_GetEleVel(Fluxobserver_TypeDef *Fluxobserver);

#endif
