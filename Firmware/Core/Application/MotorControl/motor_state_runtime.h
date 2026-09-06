#ifndef CORE_APPLICATION_MOTOR_CONTROL_MOTOR_STATE_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_MOTOR_STATE_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control_types.h"
#include "Core/Services/Safety/fault_manager.h"
#include "Core/Application/device_lifecycle.h"
#include "current_control_runtime.h"
#include "pi_controller.h"
#include "encoder.h"
#include "sensorless_runtime.h"
#include "control_mode_runtime.h"
#include "encoder_calibration_runtime.h"
#include "Core/Application/Commissioning/calibration_service.h"
#include "Core/Application/Commissioning/identification_service.h"
#include "bsp_system.h"
#include "Core/Application/motor_commissioning_workflow.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	MotorControlContext *motor;
	CurrentControlContext *current_control;
	PiController *speed_controller;
	EncoderContext *encoder;
	const RotorFeedbackFrame *feedback;
	SensorlessStartupContext *sensorless_startup;
	MotionControlContext *motion_control;
	MotorCalibrationContext *motor_calibration;
	DeviceLifecycleContext *lifecycle;
	CalibrationServiceContext *calibration_service;
	IdentificationServiceContext *identification_service;
	BspMonotonicClockPort monotonic_clock;
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
	MotorControlModeMask allowed_control_mode_mask;
	DeviceState previous_device_state;
	MotorControlMode previous_control_mode;
	ServiceProcedure previous_service_procedure;
	MotorFaultCode previous_fault;
	MotorCommissioningWorkflowContext commissioning;
};

bool MotorState_Initialize(MotorStateContext *context,
	const MotorFaultRuntimeBindings *bindings,
	MotorCommissioningStageMask commissioning_stage_mask,
	MotorControlModeMask allowed_control_mode_mask);
MotorFaultCode MotorState_GetPrimaryFault(const MotorStateContext *context);
void MotorState_RaiseFault(MotorStateContext *context, MotorFaultCode fault);
void MotorState_ClearFault(MotorStateContext *context,
	MotorFaultCode error_to_clear);
void MotorState_ClearAllFaults(MotorStateContext *context);
bool MotorFaults_HasActive(const MotorStateContext *context);
FaultSet MotorFaults_GetActiveSet(const MotorStateContext *context);
FaultSet MotorFaults_GetLatchedSet(const MotorStateContext *context);
uint32_t MotorFaults_GetEventSequence(const MotorStateContext *context);
bool MotorFaults_GetRecord(const MotorStateContext *context,
	uint8_t fault_code, FaultRecord *record);

DeviceState MotorLifecycle_GetDeviceState(const MotorStateContext *context);
MotorControlMode MotorLifecycle_GetControlMode(const MotorStateContext *context);
ServiceProcedure MotorLifecycle_GetServiceProcedure(
	const MotorStateContext *context);
ProcedureState MotorLifecycle_GetProcedureState(const MotorStateContext *context);
uint8_t MotorLifecycle_GetProtocolActionCode(const MotorStateContext *context);
MotorCommissioningStage MotorLifecycle_GetCommissioningStage(
	const MotorStateContext *context);
uint8_t MotorLifecycle_GetCommissioningProgressPercent(
	const MotorStateContext *context);
MotorCommissioningStage MotorLifecycle_GetCommissioningFailureStage(
	const MotorStateContext *context);
bool MotorLifecycle_RequestControlMode(MotorStateContext *context,
	MotorControlMode mode);
bool MotorLifecycle_RequestService(MotorStateContext *context,
	ServiceProcedure procedure);
bool MotorLifecycle_RequestStandby(MotorStateContext *context);
bool MotorLifecycle_RequestClearFaults(MotorStateContext *context);
void MotorLifecycle_ReportServiceComplete(MotorStateContext *context,
	bool request_parameter_save);
void MotorLifecycle_ReportServiceFailed(MotorStateContext *context);
void MotorLifecycle_Supervise1kHz(MotorStateContext *context);

void MotorState_ResetControlState(MotorStateContext *context);
bool MotorState_HasChanged(const MotorStateContext *context);
void MotorState_ClearChangeFlag(MotorStateContext *context);
void MotorState_DisableMotorDrive(MotorStateContext *context);
bool MotorState_EnableMotorDrive(MotorStateContext *context);

#endif
