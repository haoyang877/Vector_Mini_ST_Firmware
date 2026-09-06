#ifndef CORE_APPLICATION_MOTOR_CONTROL_MOTOR_SERVICE_ADAPTER_H
#define CORE_APPLICATION_MOTOR_CONTROL_MOTOR_SERVICE_ADAPTER_H

#include "motor_control_types.h"
#include "Core/Application/Contracts/motor_command_port.h"
#include "Core/Application/Contracts/motor_configuration_port.h"
#include "bsp_system.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	MotorControlContext *motor;
	MotorStateContext *motor_state;
	MotorCommand pending_command;
	BspCriticalSectionPort critical_section;
	volatile uint32_t published_revision;
	uint32_t applied_revision;
	bool is_initialized;
} MotorCommandAdapterContext;

typedef struct
{
	MotorControlContext *motor;
	MotorStateContext *motor_state;
	float current_loop_bandwidth_rad_s;
	MotorConfiguration candidate;
	BspCriticalSectionPort critical_section;
	volatile uint32_t published_revision;
	uint32_t applied_revision;
	bool is_initialized;
} MotorConfigurationAdapterContext;

MotorCommandPort MotorServiceAdapter_CreateCommandPort(
	MotorCommandAdapterContext *context, MotorControlContext *motor,
	const BspCriticalSectionPort *critical_section,
	MotorStateContext *motor_state);
bool MotorServiceAdapter_ApplyPendingCommand(
	MotorCommandAdapterContext *context);
MotorConfigurationPort MotorServiceAdapter_CreateConfigurationPort(
	MotorConfigurationAdapterContext *context, MotorControlContext *motor,
	const BspCriticalSectionPort *critical_section,
	float current_loop_bandwidth_rad_s, MotorStateContext *motor_state);
bool MotorServiceAdapter_ApplyPendingConfiguration(
	MotorConfigurationAdapterContext *context);
bool MotorServiceAdapter_StageCurrentOffsetResult(
	MotorConfigurationAdapterContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc);
bool MotorServiceAdapter_StageFrictionModel(
	MotorConfigurationAdapterContext *context, float coulomb_pos_a,
	float coulomb_neg_a, float viscous_pos_a_per_rad_s,
	float viscous_neg_a_per_rad_s);

#endif
