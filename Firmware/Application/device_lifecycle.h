#ifndef APPLICATION_DEVICE_LIFECYCLE_H
#define APPLICATION_DEVICE_LIFECYCLE_H

#include <stdbool.h>
#include "procedure_state.h"

typedef enum
{
	DEVICE_STATE_BOOTING = 0,
	DEVICE_STATE_STANDBY,
	DEVICE_STATE_ACTIVE,
	DEVICE_STATE_SERVICING,
	DEVICE_STATE_FAULTED,
	DEVICE_STATE_UPDATING
} DeviceState;

typedef enum
{
	MOTOR_CONTROL_MODE_NONE = 0,
	MOTOR_CONTROL_MODE_CURRENT,
	MOTOR_CONTROL_MODE_SPEED,
	MOTOR_CONTROL_MODE_SENSORLESS_SPEED,
	MOTOR_CONTROL_MODE_POSITION_CASCADE,
	MOTOR_CONTROL_MODE_POSITION_IMPEDANCE,
	MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_VQ
} MotorControlMode;

typedef enum
{
	SERVICE_PROCEDURE_NONE = 0,
	SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION,
	SERVICE_PROCEDURE_ENCODER_LINEARIZATION,
	SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION,
	SERVICE_PROCEDURE_OBSERVER_CALIBRATION,
	SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION,
	SERVICE_PROCEDURE_SET_MECHANICAL_ZERO,
	SERVICE_PROCEDURE_PARAMETER_SAVE,
	SERVICE_PROCEDURE_RESTORE_DEFAULTS,
	SERVICE_PROCEDURE_COUNT
} ServiceProcedure;

typedef struct
{
	DeviceState device_state;
	MotorControlMode motor_control_mode;
	ServiceProcedure service_procedure;
	ProcedureState procedure_state;
} DeviceLifecycleContext;

void DeviceLifecycle_Initialize(DeviceLifecycleContext *context);
bool DeviceLifecycle_CompleteBoot(DeviceLifecycleContext *context);
bool DeviceLifecycle_RequestMotorControl(DeviceLifecycleContext *context,
	MotorControlMode mode);
bool DeviceLifecycle_RequestService(DeviceLifecycleContext *context,
	ServiceProcedure procedure);
bool DeviceLifecycle_RequestStandby(DeviceLifecycleContext *context);
bool DeviceLifecycle_BeginServiceRun(DeviceLifecycleContext *context);
bool DeviceLifecycle_BeginServiceVerification(DeviceLifecycleContext *context);
bool DeviceLifecycle_CompleteService(DeviceLifecycleContext *context);
void DeviceLifecycle_FailService(DeviceLifecycleContext *context);
void DeviceLifecycle_CancelService(DeviceLifecycleContext *context);
bool DeviceLifecycle_RequestUpdating(DeviceLifecycleContext *context);
bool DeviceLifecycle_AbortUpdating(DeviceLifecycleContext *context);
void DeviceLifecycle_NotifyFault(DeviceLifecycleContext *context);
bool DeviceLifecycle_ClearFault(DeviceLifecycleContext *context,
	bool all_faults_cleared);

#endif
