#ifndef CORE_APPLICATION_MOTOR_CONTROL_FRICTION_IDENTIFICATION_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_FRICTION_IDENTIFICATION_RUNTIME_H

#include "control_mode_runtime.h"
#include "bsp_system.h"
#include "encoder.h"
#include "friction_identification.h"
#include "Core/Application/Contracts/friction_identification_port.h"
#include "Core/Config/product_config.h"
#include "motor_service_adapter.h"

typedef struct
{
	FrictionIdentificationContext core;
	MotorControlContext *motor;
	MotorConfigurationAdapterContext *configuration_adapter;
	MotorStateContext *motor_state;
	BspCriticalSectionPort critical_section;
	bool started;
	bool port_initialized;
} FrictionIdentificationRuntimeContext;

FrictionIdentificationState FrictionIdentificationRuntime_Run(
	FrictionIdentificationRuntimeContext *context,
	MotionControlContext *motion, CurrentControlContext *current_control,
	MotorControlContext *motor, PiController *speed_controller,
	EncoderContext *encoder,
	const ProductFrictionIdentificationConfig *config);
void FrictionIdentificationRuntime_Cancel(
	FrictionIdentificationRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller);
FrictionIdentificationPort FrictionIdentificationRuntime_CreatePort(
	FrictionIdentificationRuntimeContext *context, MotorControlContext *motor,
	MotorConfigurationAdapterContext *configuration_adapter,
	const BspCriticalSectionPort *critical_section,
	MotorStateContext *motor_state);

#endif
