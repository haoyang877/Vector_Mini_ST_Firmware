#ifndef RUNTIME_MOTOR_SERVICE_ADAPTER_H
#define RUNTIME_MOTOR_SERVICE_ADAPTER_H

#include "motor_control_types.h"
#include "motor_command_port.h"
#include "motor_configuration_port.h"
#include "critical_section_port.h"
#include "motor_profiles.h"

typedef struct
{
	MotorControlContext *motor;
	MotorCommand pending_command;
	CriticalSectionPort critical_section;
	volatile uint32_t published_revision;
	uint32_t applied_revision;
	bool is_initialized;
} MotorCommandAdapterContext;

typedef struct
{
	MotorControlContext *motor;
	const MotorProfile *motor_profile;
	MotorConfiguration candidate;
	CriticalSectionPort critical_section;
	volatile uint32_t published_revision;
	uint32_t applied_revision;
	bool is_initialized;
} MotorConfigurationAdapterContext;

MotorCommandPort MotorServiceAdapter_CreateCommandPort(
	MotorCommandAdapterContext *context, MotorControlContext *motor,
	const CriticalSectionPort *critical_section);
bool MotorServiceAdapter_ApplyPendingCommand(
	MotorCommandAdapterContext *context);
MotorConfigurationPort MotorServiceAdapter_CreateConfigurationPort(
	MotorConfigurationAdapterContext *context, MotorControlContext *motor,
	const CriticalSectionPort *critical_section,
	const MotorProfile *motor_profile);
bool MotorServiceAdapter_ApplyPendingConfiguration(
	MotorConfigurationAdapterContext *context);
bool MotorServiceAdapter_StageCurrentOffsetResult(
	MotorConfigurationAdapterContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc);
bool MotorServiceAdapter_StagePhaseResistanceResult(
	MotorConfigurationAdapterContext *context, float resistance_ohm);

#endif
