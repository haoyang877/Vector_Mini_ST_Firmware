#ifndef CORE_APPLICATION_MOTOR_CONTROL_MOTOR_DRIVE_SERVICE_H
#define CORE_APPLICATION_MOTOR_CONTROL_MOTOR_DRIVE_SERVICE_H

#include "Bsp/Api/bsp_motor_drive.h"
#include "Core/Application/Contracts/motor_output_safety_port.h"

typedef struct
{
	BspMotorDriveSample sample;
	BspCurrentSamplingPlan sampling;
} MotorDriveServiceAcquisition;

typedef struct
{
	BspMotorDrivePort port;
	BspMotorDriveConfiguration configuration;
	BspCurrentSamplingPlan queued_sampling;
	uint32_t next_cycle_sequence;
	bool is_initialized;
	bool outputs_enabled;
	bool has_pending_cycle;
	bool is_primed;
	bool has_latched_fault;
} MotorDriveServiceContext;

/* Initializes the physical endpoint with outputs disabled. Fixed-sampling
 * endpoints also receive an all-equal, zero-line-voltage PWM command before
 * this function succeeds. */
bool MotorDriveService_Initialize(MotorDriveServiceContext *context,
	const BspMotorDrivePort *port,
	const BspMotorDriveConfiguration *configuration);

bool MotorDriveService_RequestEnable(MotorDriveServiceContext *context,
	bool safety_interlock_clear);
void MotorDriveService_ForceDisable(MotorDriveServiceContext *context);

/* The service owns the monotonically increasing cycle sequence. */
bool MotorDriveService_CommitCycle(MotorDriveServiceContext *context,
	const BspMotorDriveCycleCommand *command);
bool MotorDriveService_ApplyFixedDuty(MotorDriveServiceContext *context,
	float phase_a, float phase_b, float phase_c);

/* Hardware faults and malformed acquisitions immediately de-energize the
 * bridge and latch the service. */
bool MotorDriveService_ReadSample(MotorDriveServiceContext *context,
	MotorDriveServiceAcquisition *acquisition);

bool MotorDriveService_AreOutputsEnabled(
	const MotorDriveServiceContext *context);
bool MotorDriveService_HasLatchedFault(
	const MotorDriveServiceContext *context);
void MotorDriveService_RejectOutputCommand(MotorDriveServiceContext *context);
bool MotorDriveService_ClearLatchedFault(MotorDriveServiceContext *context);
MotorOutputSafetyPort MotorDriveService_CreateOutputSafetyPort(
	MotorDriveServiceContext *context);

#endif
