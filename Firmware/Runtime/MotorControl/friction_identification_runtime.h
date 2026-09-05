#ifndef RUNTIME_FRICTION_IDENTIFICATION_H
#define RUNTIME_FRICTION_IDENTIFICATION_H

#include "board_profile.h"
#include "control_mode_runtime.h"
#include "critical_section_port.h"
#include "encoder.h"
#include "friction_identification.h"
#include "friction_identification_port.h"
#include "mechanical_load_profiles.h"
#include "motor_service_adapter.h"

typedef struct
{
	FrictionIdentificationContext core;
	MotorControlContext *motor;
	MotorConfigurationAdapterContext *configuration_adapter;
	MotorStateContext *motor_state;
	CriticalSectionPort critical_section;
	bool started;
	bool port_initialized;
} FrictionIdentificationRuntimeContext;

FrictionIdentificationState FrictionIdentificationRuntime_Run(
	FrictionIdentificationRuntimeContext *context,
	MotionControlContext *motion, CurrentControlContext *current_control,
	MotorControlContext *motor, PiController *speed_controller,
	EncoderContext *encoder, const BoardProfile *board_profile,
	const MechanicalLoadProfile *load_profile);
void FrictionIdentificationRuntime_Cancel(
	FrictionIdentificationRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller);
FrictionIdentificationPort FrictionIdentificationRuntime_CreatePort(
	FrictionIdentificationRuntimeContext *context, MotorControlContext *motor,
	MotorConfigurationAdapterContext *configuration_adapter,
	const CriticalSectionPort *critical_section,
	MotorStateContext *motor_state);

#endif
