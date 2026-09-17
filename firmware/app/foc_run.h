#ifndef __FOC_RUN_H__
#define __FOC_RUN_H__

#include "position_cascade.h"

/** Deferred outer controller entry; called only by the lower-priority motor worker. */
void MotorOuterLoop_Service(void);
/** Fast-owner queries; no worker state is borrowed. */
bool MotorOuterLoop_IsReady(void);
bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry);

#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_sensorless.h"
#include "foc_pid.h"
#include "data_type.h"

/** Service completion, validate live inputs and release one job per ten fast ticks.
 * Must run in the fast context even when the motor is disabled, to discard stale jobs. */
void MotorOuterLoop_FastTick(MotorControl_TypeDef *motor, PI_Controller_TypeDef *pi,
    Encoder_TypeDef *encoder);

void Task_Current_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver);
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Encoder_TypeDef *Encoder);
extern const SensorlessStartupConfig_TypeDef SensorlessStartup_DefaultConfig;
extern const SensorlessStartupConfig_TypeDef SensorlessStartup_EncoderCalibConfig;

void Task_Sensorless_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Fluxobserver_TypeDef *Fluxobserver, SensorlessStartup_TypeDef *Startup, const SensorlessStartupConfig_TypeDef *Config);
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Position_Impedance_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Position_Mode_Reset(void);
void Task_Voltage_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Task_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);

#endif
