#ifndef RUNTIME_MOTOR_STATE_H
#define RUNTIME_MOTOR_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control_types.h"
#include "fault_manager.h"
#include "device_lifecycle.h"
#include "current_control_runtime.h"
#include "pi_controller.h"
#include "encoder.h"
#include "sensorless_runtime.h"
#include "control_mode_runtime.h"
#include "encoder_calibration_runtime.h"
#include "calibration_service.h"
#include "identification_service.h"
#include "monotonic_clock_port.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	MotorControlContext *motor;
	CurrentControlContext *current_control;
	PiController *speed_controller;
	EncoderContext *encoder;
	SensorlessStartupContext *sensorless_startup;
	MotionControlContext *motion_control;
	MotorCalibrationContext *motor_calibration;
	DeviceLifecycleContext *lifecycle;
	MotorStateContext *state;
	CalibrationServiceContext *calibration_service;
	IdentificationServiceContext *identification_service;
	MonotonicClockPort monotonic_clock;
} MotorFaultRuntimeBindings;

struct MotorStateContext
{
	FaultManagerContext fault_manager;
	MotorFaultRuntimeBindings bindings;
	volatile uint32_t pending_lifecycle_request;
	volatile uint32_t pending_service_result;
	uint32_t service_elapsed_ticks;
	bool service_terminal_hold;
	bool has_changed;
	DeviceState previous_device_state;
	MotorControlMode previous_control_mode;
	ServiceProcedure previous_service_procedure;
	MotorFaultCode previous_fault;
};

bool MotorState_Initialize(const MotorFaultRuntimeBindings *bindings);
MotorFaultCode MotorState_GetPrimaryFault(void);
void MotorState_RaiseFault(MotorFaultCode fault);
void MotorState_ClearFault(MotorFaultCode error_to_clear);
void MotorState_ClearAllFaults(void);
bool MotorFaults_HasActive(void);
FaultSet MotorFaults_GetActiveSet(void);
FaultSet MotorFaults_GetLatchedSet(void);
uint32_t MotorFaults_GetEventSequence(void);
bool MotorFaults_GetRecord(uint8_t fault_code, FaultRecord *record);

DeviceState MotorLifecycle_GetDeviceState(void);
MotorControlMode MotorLifecycle_GetControlMode(void);
ServiceProcedure MotorLifecycle_GetServiceProcedure(void);
ProcedureState MotorLifecycle_GetProcedureState(void);
uint8_t MotorLifecycle_GetProtocolActionCode(void);
bool MotorLifecycle_RequestControlMode(MotorControlMode mode);
bool MotorLifecycle_RequestService(ServiceProcedure procedure);
bool MotorLifecycle_RequestStandby(void);
bool MotorLifecycle_RequestClearFaults(void);
void MotorLifecycle_ReportServiceComplete(bool request_parameter_save);
void MotorLifecycle_ReportServiceFailed(void);
void MotorLifecycle_Supervise1kHz(void);

void MotorState_ResetControlState(void);
bool MotorState_HasChanged(void);
void MotorState_ClearChangeFlag(void);
void MotorState_DisablePowerStage(void);
bool MotorState_EnablePowerStage(void);

#endif
