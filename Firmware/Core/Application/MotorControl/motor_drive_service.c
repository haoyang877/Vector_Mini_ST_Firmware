#include "Core/Application/MotorControl/motor_drive_service.h"

#include <stddef.h>
#include <string.h>

#define MOTOR_DRIVE_SAFE_DUTY 1.0f
#define MOTOR_DRIVE_KNOWN_SAMPLE_STATUS \
	(BSP_MOTOR_DRIVE_SAMPLE_VALID | BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN)

static uint8_t MotorDriveService_CountPhases(BspMotorPhaseSet phases)
{
	uint8_t count = 0U;

	while (phases != 0U)
	{
		count = (uint8_t)(count + (phases & 1U));
		phases = (BspMotorPhaseSet)(phases >> 1U);
	}
	return count;
}

static bool MotorDriveService_PortIsValid(const BspMotorDrivePort *port)
{
	return port != NULL && port->capabilities != NULL &&
		port->initialize_safe != NULL && port->arm != NULL &&
		port->disarm != NULL && port->read_sample != NULL &&
		port->commit_cycle != NULL && port->disable_immediate != NULL &&
		port->read_faults != NULL;
}

static bool MotorDriveService_ConfigurationIsSupported(
	const BspMotorDrivePort *port,
	const BspMotorDriveConfiguration *configuration)
{
	const BspMotorDriveEndpointCapabilities *capabilities;

	if (!MotorDriveService_PortIsValid(port) || configuration == NULL ||
		configuration->current_sense_topology <= BSP_CURRENT_SENSE_NONE ||
		configuration->current_sense_topology >=
			BSP_CURRENT_SENSE_TOPOLOGY_COUNT ||
		configuration->sampling_mode <=
			BSP_CURRENT_SAMPLING_MODE_UNSPECIFIED ||
		configuration->sampling_mode >= BSP_CURRENT_SAMPLING_MODE_COUNT ||
		configuration->pwm_frequency_hz == 0U)
		return false;
	capabilities = port->capabilities;
	if (capabilities->endpoint_id == BSP_ENDPOINT_ID_NONE ||
		capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		!capabilities->supports_synchronized_sampling ||
		(capabilities->supported_current_sense_topologies &
		 BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			configuration->current_sense_topology)) == 0U ||
		(capabilities->supported_sampling_modes &
		 BSP_CURRENT_SAMPLING_MODE_BIT(configuration->sampling_mode)) == 0U)
		return false;
	if (configuration->sampling_mode == BSP_CURRENT_SAMPLING_MODE_FIXED)
	{
		return configuration->fixed_sample_count > 0U &&
			configuration->fixed_sample_count <=
				capabilities->current_sensor_capacity &&
			configuration->fixed_sample_count <=
				BSP_MOTOR_MAX_SAMPLING_POINT_COUNT &&
			(configuration->fixed_direct_phase_currents &
			 (BspMotorPhaseSet)~BSP_MOTOR_PHASE_ALL) == 0U &&
			MotorDriveService_CountPhases(
				configuration->fixed_direct_phase_currents) ==
				configuration->fixed_sample_count;
	}
	return configuration->fixed_sample_count == 0U &&
		configuration->fixed_direct_phase_currents == 0U;
}

static bool MotorDriveService_CommandIsValid(
	const MotorDriveServiceContext *context,
	const BspMotorDriveCycleCommand *command)
{
	uint8_t phase;

	if (command->sampling.mode != context->configuration.sampling_mode)
		return false;
	for (phase = 0U; phase < BSP_MOTOR_PHASE_COUNT; phase++)
	{
		if (!(command->pwm.phase_duty[phase] >= 0.0f &&
			command->pwm.phase_duty[phase] <= 1.0f))
			return false;
	}
	if (command->sampling.mode == BSP_CURRENT_SAMPLING_MODE_FIXED)
	{
		return command->sampling.modulation_sector == 0U &&
			command->sampling.sampling_point_count == 0U &&
			command->sampling.cycle_valid_phase_currents ==
				context->configuration.fixed_direct_phase_currents;
	}
	return command->sampling.modulation_sector >= 1U &&
		command->sampling.modulation_sector <= 6U &&
		command->sampling.sampling_point_count > 0U &&
		command->sampling.sampling_point_count <=
			BSP_MOTOR_MAX_SAMPLING_POINT_COUNT &&
		(command->sampling.cycle_valid_phase_currents &
		 (BspMotorPhaseSet)~BSP_MOTOR_PHASE_ALL) == 0U;
}

static void MotorDriveService_DisableUnchecked(
	MotorDriveServiceContext *context)
{
	context->port.disable_immediate(context->port.context);
	(void)context->port.disarm(context->port.context);
	context->outputs_enabled = false;
}

void MotorDriveService_ForceDisable(MotorDriveServiceContext *context)
{
	if (context == NULL || !context->is_initialized)
		return;
	MotorDriveService_DisableUnchecked(context);
}

void MotorDriveService_RejectOutputCommand(MotorDriveServiceContext *context)
{
	if (context == NULL || !context->is_initialized)
		return;
	MotorDriveService_DisableUnchecked(context);
	context->has_latched_fault = true;
}

bool MotorDriveService_CommitCycle(MotorDriveServiceContext *context,
	const BspMotorDriveCycleCommand *command)
{
	BspMotorDriveCycleCommand cycle;

	if (context == NULL || !context->is_initialized || command == NULL ||
		context->has_latched_fault || context->has_pending_cycle)
		return false;
	if (!MotorDriveService_CommandIsValid(context, command))
	{
		MotorDriveService_RejectOutputCommand(context);
		return false;
	}
	cycle = *command;
	cycle.sampling.sequence = context->next_cycle_sequence;
	if (context->port.commit_cycle(context->port.context, &cycle) !=
		BSP_RESULT_OK)
	{
		MotorDriveService_RejectOutputCommand(context);
		return false;
	}
	context->next_cycle_sequence++;
	context->queued_sampling = cycle.sampling;
	context->has_pending_cycle = true;
	return true;
}

bool MotorDriveService_ApplyFixedDuty(MotorDriveServiceContext *context,
	float phase_a, float phase_b, float phase_c)
{
	BspMotorDriveCycleCommand command;

	if (context == NULL || context->configuration.sampling_mode !=
		BSP_CURRENT_SAMPLING_MODE_FIXED)
		return false;
	(void)memset(&command, 0, sizeof(command));
	command.pwm.phase_duty[0] = phase_a;
	command.pwm.phase_duty[1] = phase_b;
	command.pwm.phase_duty[2] = phase_c;
	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	command.sampling.cycle_valid_phase_currents =
		context->configuration.fixed_direct_phase_currents;
	return MotorDriveService_CommitCycle(context, &command);
}

bool MotorDriveService_Initialize(MotorDriveServiceContext *context,
	const BspMotorDrivePort *port,
	const BspMotorDriveConfiguration *configuration)
{
	if (context == NULL)
		return false;
	(void)memset(context, 0, sizeof(*context));
	if (!MotorDriveService_ConfigurationIsSupported(port, configuration))
		return false;

	context->port = *port;
	context->configuration = *configuration;
	context->port.disable_immediate(context->port.context);
	if (context->port.initialize_safe(context->port.context, configuration) !=
		BSP_RESULT_OK)
		return false;
	context->is_initialized = true;
	if (configuration->sampling_mode == BSP_CURRENT_SAMPLING_MODE_FIXED &&
		!MotorDriveService_ApplyFixedDuty(context, MOTOR_DRIVE_SAFE_DUTY,
			MOTOR_DRIVE_SAFE_DUTY, MOTOR_DRIVE_SAFE_DUTY))
		return false;
	return true;
}

bool MotorDriveService_RequestEnable(MotorDriveServiceContext *context,
	bool safety_interlock_clear)
{
	if (context == NULL || !context->is_initialized ||
		!safety_interlock_clear || context->has_latched_fault)
		return false;
	if (context->outputs_enabled)
		return true;
	if (!context->is_primed)
		return false;
	if (context->port.read_faults(context->port.context) != 0U ||
		context->port.arm(context->port.context) != BSP_RESULT_OK)
	{
		MotorDriveService_RejectOutputCommand(context);
		return false;
	}
	context->outputs_enabled = true;
	return true;
}

bool MotorDriveService_ReadSample(MotorDriveServiceContext *context,
	MotorDriveServiceAcquisition *acquisition)
{
	BspResult result;
	BspMotorDriveSample *sample;

	if (context == NULL || !context->is_initialized || acquisition == NULL ||
		context->has_latched_fault)
		return false;
	sample = &acquisition->sample;
	if (context->port.read_faults(context->port.context) != 0U)
	{
		MotorDriveService_RejectOutputCommand(context);
		return false;
	}
	if (!context->has_pending_cycle)
		return false;
	result = context->port.read_sample(context->port.context, sample);
	if (result == BSP_RESULT_NOT_READY || result == BSP_RESULT_BUSY)
		return false;
	context->has_pending_cycle = false;
	if (result != BSP_RESULT_OK ||
		sample->sequence != context->queued_sampling.sequence ||
		sample->current_sample_count == 0U ||
		sample->current_sample_count >
			BSP_MOTOR_MAX_SAMPLING_POINT_COUNT ||
		sample->valid_phase_currents !=
			context->queued_sampling.cycle_valid_phase_currents ||
		(context->configuration.sampling_mode ==
			BSP_CURRENT_SAMPLING_MODE_FIXED &&
		 sample->current_sample_count !=
			context->configuration.fixed_sample_count) ||
		(context->configuration.sampling_mode ==
			BSP_CURRENT_SAMPLING_MODE_DYNAMIC &&
		 sample->current_sample_count !=
			context->queued_sampling.sampling_point_count) ||
		(sample->status & BSP_MOTOR_DRIVE_SAMPLE_VALID) == 0U ||
		(sample->status & ~MOTOR_DRIVE_KNOWN_SAMPLE_STATUS) != 0U ||
		(sample->status & BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN) != 0U)
	{
		MotorDriveService_RejectOutputCommand(context);
		return false;
	}
	acquisition->sampling = context->queued_sampling;
	context->is_primed = true;
	return true;
}

bool MotorDriveService_AreOutputsEnabled(
	const MotorDriveServiceContext *context)
{
	return context != NULL && context->is_initialized &&
		context->outputs_enabled;
}

bool MotorDriveService_HasLatchedFault(
	const MotorDriveServiceContext *context)
{
	return context != NULL && context->is_initialized &&
		context->has_latched_fault;
}

bool MotorDriveService_ClearLatchedFault(MotorDriveServiceContext *context)
{
	if (context == NULL || !context->is_initialized ||
		context->outputs_enabled ||
		context->port.read_faults(context->port.context) != 0U)
		return false;
	context->has_latched_fault = false;
	return true;
}

static void MotorDriveService_OutputSafetyDisable(void *context)
{
	MotorDriveService_ForceDisable((MotorDriveServiceContext *)context);
}

static bool MotorDriveService_OutputSafetyIsEnabled(void *context)
{
	return MotorDriveService_AreOutputsEnabled(
		(const MotorDriveServiceContext *)context);
}

MotorOutputSafetyPort MotorDriveService_CreateOutputSafetyPort(
	MotorDriveServiceContext *context)
{
	MotorOutputSafetyPort port;

	port.context = context;
	port.disable_immediate = MotorDriveService_OutputSafetyDisable;
	port.outputs_are_enabled = MotorDriveService_OutputSafetyIsEnabled;
	return port;
}
