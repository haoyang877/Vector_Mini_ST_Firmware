#ifndef __FOC_RUN_H__
#define __FOC_RUN_H__

#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_sensorless.h"
#include "foc_pid.h"
#include "data_type.h"

void Task_Current_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver);
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PID_TypeDef *PID, Encoder_TypeDef *Encoder);
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PID_TypeDef *PID, Encoder_TypeDef *Encoder);

#endif