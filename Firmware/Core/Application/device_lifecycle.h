#ifndef CORE_APPLICATION_DEVICE_LIFECYCLE_H
#define CORE_APPLICATION_DEVICE_LIFECYCLE_H

#include <stdbool.h>
#include <stdint.h>
#include "Core/Application/procedure_state.h"

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

typedef uint8_t MotorControlModeMask;

#define MOTOR_CONTROL_MODE_MASK(mode_) \
	((MotorControlModeMask)(UINT8_C(1) << (uint8_t)(mode_)))
#define MOTOR_CONTROL_SUPPORTED_MODE_MASK ((MotorControlModeMask)UINT8_C(0xFE))

static inline bool MotorControlModeMask_Allows(MotorControlModeMask mask,
	MotorControlMode mode)
{
	return mode > MOTOR_CONTROL_MODE_NONE && mode <= MOTOR_CONTROL_MODE_VQ &&
		(mask & MOTOR_CONTROL_MODE_MASK(mode)) != 0U;
}

typedef enum
{
	SERVICE_PROCEDURE_NONE = 0,
	SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION,
	SERVICE_PROCEDURE_ENCODER_LINEARIZATION,
	SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION,
	SERVICE_PROCEDURE_OBSERVER_CALIBRATION,
	SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION,
	SERVICE_PROCEDURE_FRICTION_IDENTIFICATION,
	SERVICE_PROCEDURE_SET_MECHANICAL_ZERO,
	SERVICE_PROCEDURE_PARAMETER_SAVE,
	SERVICE_PROCEDURE_RESTORE_DEFAULTS,
	SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION,
	SERVICE_PROCEDURE_COGGING_IDENTIFICATION,
	SERVICE_PROCEDURE_FULL_COMMISSIONING,
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
