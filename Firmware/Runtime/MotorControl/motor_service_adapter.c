#include "motor_service_adapter.h"

#include <math.h>

#include "motor_state_runtime.h"

static MotorPortMode MotorServiceAdapter_MapMode(MotorControlMode mode)
{
	switch (mode)
	{
		case MOTOR_CONTROL_MODE_CURRENT: return MOTOR_PORT_MODE_CURRENT;
		case MOTOR_CONTROL_MODE_SPEED: return MOTOR_PORT_MODE_SPEED;
		case MOTOR_CONTROL_MODE_SENSORLESS_SPEED: return MOTOR_PORT_MODE_SENSORLESS_SPEED;
		case MOTOR_CONTROL_MODE_POSITION_CASCADE: return MOTOR_PORT_MODE_POSITION;
		case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE: return MOTOR_PORT_MODE_POSITION_IMPEDANCE;
		case MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP: return MOTOR_PORT_MODE_VOLTAGE_OPEN_LOOP;
		case MOTOR_CONTROL_MODE_VQ: return MOTOR_PORT_MODE_VQ;
		default: return MOTOR_PORT_MODE_NONE;
	}
}

static bool MotorServiceAdapter_MapRequestedMode(MotorPortMode mode,
	MotorControlMode *control_mode)
{
	if (control_mode == 0)
		return false;
	switch (mode)
	{
		case MOTOR_PORT_MODE_CURRENT: *control_mode = MOTOR_CONTROL_MODE_CURRENT; return true;
		case MOTOR_PORT_MODE_SPEED: *control_mode = MOTOR_CONTROL_MODE_SPEED; return true;
		case MOTOR_PORT_MODE_SENSORLESS_SPEED:
			*control_mode = MOTOR_CONTROL_MODE_SENSORLESS_SPEED; return true;
		case MOTOR_PORT_MODE_POSITION: *control_mode = MOTOR_CONTROL_MODE_POSITION_CASCADE; return true;
		case MOTOR_PORT_MODE_POSITION_IMPEDANCE:
			*control_mode = MOTOR_CONTROL_MODE_POSITION_IMPEDANCE; return true;
		case MOTOR_PORT_MODE_VOLTAGE_OPEN_LOOP:
			*control_mode = MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP; return true;
		case MOTOR_PORT_MODE_VQ: *control_mode = MOTOR_CONTROL_MODE_VQ; return true;
		default: return false;
	}
}

static bool MotorServiceAdapter_MapRequestedService(MotorPortService service,
	ServiceProcedure *procedure)
{
	if (procedure == 0)
		return false;
	switch (service)
	{
		case MOTOR_PORT_SERVICE_CURRENT_OFFSET_CALIBRATION:
			*procedure = SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION; return true;
		case MOTOR_PORT_SERVICE_ENCODER_LINEARIZATION:
			*procedure = SERVICE_PROCEDURE_ENCODER_LINEARIZATION; return true;
		case MOTOR_PORT_SERVICE_ELECTRICAL_ZERO_CALIBRATION:
			*procedure = SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION; return true;
		case MOTOR_PORT_SERVICE_OBSERVER_CALIBRATION:
			*procedure = SERVICE_PROCEDURE_OBSERVER_CALIBRATION; return true;
		case MOTOR_PORT_SERVICE_PHASE_RESISTANCE_IDENTIFICATION:
			*procedure = SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION; return true;
		case MOTOR_PORT_SERVICE_FRICTION_IDENTIFICATION:
			*procedure = SERVICE_PROCEDURE_FRICTION_IDENTIFICATION; return true;
		case MOTOR_PORT_SERVICE_SET_MECHANICAL_ZERO:
			*procedure = SERVICE_PROCEDURE_SET_MECHANICAL_ZERO; return true;
		case MOTOR_PORT_SERVICE_PARAMETER_SAVE:
			*procedure = SERVICE_PROCEDURE_PARAMETER_SAVE; return true;
		case MOTOR_PORT_SERVICE_RESTORE_DEFAULTS:
			*procedure = SERVICE_PROCEDURE_RESTORE_DEFAULTS; return true;
		default: return false;
	}
}

static MotorPortMode MotorServiceAdapter_GetMode(void *context)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	return adapter == 0 || !adapter->is_initialized ? MOTOR_PORT_MODE_NONE :
		MotorServiceAdapter_MapMode(MotorLifecycle_GetControlMode());
}

static bool MotorServiceAdapter_RequestMode(void *context,
	MotorPortMode mode)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	MotorControlMode control_mode;

	if (adapter == 0 || !adapter->is_initialized ||
		!MotorServiceAdapter_MapRequestedMode(mode, &control_mode))
		return false;
	return (MotorLifecycle_GetDeviceState() == DEVICE_STATE_ACTIVE &&
		MotorLifecycle_GetControlMode() == control_mode) ||
		MotorLifecycle_RequestControlMode(control_mode);
}

static bool MotorServiceAdapter_RequestService(void *context,
	MotorPortService service)
{
	ServiceProcedure procedure;
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	if (adapter == 0 || !adapter->is_initialized ||
		!MotorServiceAdapter_MapRequestedService(service, &procedure))
		return false;
	return (MotorLifecycle_GetDeviceState() == DEVICE_STATE_SERVICING &&
		MotorLifecycle_GetServiceProcedure() == procedure) ||
		MotorLifecycle_RequestService(procedure);
}

static bool MotorServiceAdapter_RequestStandby(void *context)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	return adapter != 0 && adapter->is_initialized &&
		MotorLifecycle_RequestStandby();
}

static bool MotorServiceAdapter_RequestClearFaults(void *context)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	return adapter != 0 && adapter->is_initialized &&
		MotorLifecycle_RequestClearFaults();
}

static float MotorServiceAdapter_GetCurrentLimit(void *context)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	return adapter == 0 || !adapter->is_initialized ? 0.0f :
		adapter->motor->configuration.current_limit_a;
}

static float MotorServiceAdapter_GetSpeedLimit(void *context)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	return adapter == 0 || !adapter->is_initialized ? 0.0f :
		adapter->motor->configuration.speed_limit_rad_s;
}

static void MotorServiceAdapter_PublishCommand(
	MotorCommandAdapterContext *adapter, MotorCommand command)
{
	uint32_t interrupt_state = adapter->critical_section.enter(
		adapter->critical_section.context);
	adapter->pending_command = command;
	adapter->published_revision++;
	adapter->critical_section.exit(adapter->critical_section.context,
		interrupt_state);
}

static void MotorServiceAdapter_SetCurrentReference(void *context, float value)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	MotorCommand command;
	if (adapter == 0 || !adapter->is_initialized)
		return;
	command = adapter->pending_command;
	command.q_axis_current_reference_a = value;
	MotorServiceAdapter_PublishCommand(adapter, command);
}

static void MotorServiceAdapter_SetSpeedReference(void *context, float value)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	MotorCommand command;
	if (adapter == 0 || !adapter->is_initialized)
		return;
	command = adapter->pending_command;
	command.speed_reference_rad_s = value;
	MotorServiceAdapter_PublishCommand(adapter, command);
}

static void MotorServiceAdapter_SetPositionReference(void *context, float value)
{
	MotorCommandAdapterContext *adapter = (MotorCommandAdapterContext *)context;
	MotorCommand command;
	if (adapter == 0 || !adapter->is_initialized)
		return;
	command = adapter->pending_command;
	command.position_reference_rad = value;
	MotorServiceAdapter_PublishCommand(adapter, command);
}

static bool MotorServiceAdapter_ReadConfiguration(void *context,
	MotorParameterId parameter, float *value)
{
	MotorConfigurationAdapterContext *adapter =
		(MotorConfigurationAdapterContext *)context;
	const MotorConfiguration *configuration;

	if (adapter == 0 || !adapter->is_initialized || value == 0)
		return false;
	configuration = &adapter->candidate;
	switch (parameter)
	{
		case MOTOR_PARAMETER_POLE_PAIRS: *value = (float)configuration->pole_pairs; break;
		case MOTOR_PARAMETER_CALIBRATION_CURRENT_A: *value = configuration->calibration_current_a; break;
		case MOTOR_PARAMETER_CURRENT_LIMIT_A: *value = configuration->current_limit_a; break;
		case MOTOR_PARAMETER_SPEED_LIMIT_RAD_S: *value = configuration->speed_limit_rad_s; break;
		case MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2: *value = configuration->speed_acceleration_rad_s2; break;
		case MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2: *value = configuration->speed_deceleration_rad_s2; break;
		case MOTOR_PARAMETER_SPEED_KP: *value = configuration->speed_kp; break;
		case MOTOR_PARAMETER_SPEED_KI: *value = configuration->speed_ki; break;
		case MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2: *value = configuration->position_acceleration_rad_s2; break;
		case MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2: *value = configuration->position_deceleration_rad_s2; break;
		case MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S: *value = configuration->position_max_speed_rad_s; break;
		case MOTOR_PARAMETER_POSITION_KP_A_PER_RAD: *value = configuration->position_kp_a_per_rad; break;
		case MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S: *value = configuration->position_kd_a_per_rad_s; break;
		case MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S: *value = configuration->position_ki_a_per_rad_s; break;
		case MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A: *value = configuration->position_integral_limit_a; break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S: *value = configuration->cascade_position_kp_per_s; break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KD: *value = configuration->cascade_position_kd; break;
		case MOTOR_PARAMETER_PHASE_RESISTANCE_OHM: *value = configuration->phase_resistance_ohm; break;
		case MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H: *value = configuration->d_axis_inductance_h; break;
		case MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H: *value = configuration->q_axis_inductance_h; break;
		case MOTOR_PARAMETER_FLUX_WEBER: *value = configuration->flux_weber; break;
		default: return false;
	}
	return true;
}

static bool MotorServiceAdapter_CanStageConfiguration(void *context)
{
	MotorConfigurationAdapterContext *adapter =
		(MotorConfigurationAdapterContext *)context;
	return adapter != 0 && adapter->is_initialized &&
		MotorLifecycle_GetDeviceState() == DEVICE_STATE_STANDBY;
}

static void MotorServiceAdapter_UpdateCurrentLoopGains(
	MotorConfigurationAdapterContext *adapter, MotorParameterId parameter)
{
	MotorConfiguration *configuration = &adapter->candidate;
	float bandwidth = adapter->motor_profile->current_loop_bandwidth_rad_s;

	switch (parameter)
	{
		case MOTOR_PARAMETER_PHASE_RESISTANCE_OHM:
			configuration->d_axis_current_ki =
				configuration->phase_resistance_ohm * bandwidth;
			configuration->q_axis_current_ki =
				configuration->phase_resistance_ohm * bandwidth;
			break;
		case MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H:
			configuration->d_axis_current_kp =
				configuration->d_axis_inductance_h * bandwidth;
			break;
		case MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H:
			configuration->q_axis_current_kp =
				configuration->q_axis_inductance_h * bandwidth;
			break;
		default:
			break;
	}
}

static bool MotorServiceAdapter_StageConfiguration(void *context,
	MotorParameterId parameter, float value)
{
	MotorConfigurationAdapterContext *adapter =
		(MotorConfigurationAdapterContext *)context;
	MotorConfiguration *configuration;
	uint32_t interrupt_state;

	if (!MotorServiceAdapter_CanStageConfiguration(adapter))
		return false;
	interrupt_state = adapter->critical_section.enter(
		adapter->critical_section.context);
	if (adapter->published_revision == adapter->applied_revision)
		adapter->candidate = adapter->motor->configuration;
	configuration = &adapter->candidate;
	switch (parameter)
	{
		case MOTOR_PARAMETER_POLE_PAIRS: configuration->pole_pairs = (int32_t)value; break;
		case MOTOR_PARAMETER_CALIBRATION_CURRENT_A: configuration->calibration_current_a = value; break;
		case MOTOR_PARAMETER_CURRENT_LIMIT_A:
			configuration->current_limit_a = value;
			break;
		case MOTOR_PARAMETER_SPEED_LIMIT_RAD_S:
			configuration->speed_limit_rad_s = value;
			if (configuration->position_max_speed_rad_s > value)
				configuration->position_max_speed_rad_s = value;
			break;
		case MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2: configuration->speed_acceleration_rad_s2 = value; break;
		case MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2: configuration->speed_deceleration_rad_s2 = value; break;
		case MOTOR_PARAMETER_SPEED_KP: configuration->speed_kp = value; break;
		case MOTOR_PARAMETER_SPEED_KI: configuration->speed_ki = value; break;
		case MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2: configuration->position_acceleration_rad_s2 = value; break;
		case MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2: configuration->position_deceleration_rad_s2 = value; break;
		case MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S: configuration->position_max_speed_rad_s = value; break;
		case MOTOR_PARAMETER_POSITION_KP_A_PER_RAD: configuration->position_kp_a_per_rad = value; break;
		case MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S: configuration->position_kd_a_per_rad_s = value; break;
		case MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S: configuration->position_ki_a_per_rad_s = value; break;
		case MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A: configuration->position_integral_limit_a = value; break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S: configuration->cascade_position_kp_per_s = value; break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KD: configuration->cascade_position_kd = value; break;
		case MOTOR_PARAMETER_PHASE_RESISTANCE_OHM:
			configuration->phase_resistance_ohm = value;
			MotorServiceAdapter_UpdateCurrentLoopGains(adapter, parameter);
			break;
		case MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H:
			configuration->d_axis_inductance_h = value;
			MotorServiceAdapter_UpdateCurrentLoopGains(adapter, parameter);
			break;
		case MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H:
			configuration->q_axis_inductance_h = value;
			MotorServiceAdapter_UpdateCurrentLoopGains(adapter, parameter);
			break;
		case MOTOR_PARAMETER_FLUX_WEBER: configuration->flux_weber = value; break;
		default:
			adapter->critical_section.exit(adapter->critical_section.context,
				interrupt_state);
			return false;
	}
	adapter->published_revision++;
	adapter->critical_section.exit(adapter->critical_section.context,
		interrupt_state);
	return true;
}

bool MotorServiceAdapter_ApplyPendingConfiguration(
	MotorConfigurationAdapterContext *context)
{
	if (context == 0 || !context->is_initialized ||
		context->published_revision == context->applied_revision)
		return false;
	context->motor->configuration = context->candidate;
	if (context->motor->command.q_axis_current_reference_a >
		context->motor->configuration.current_limit_a)
		context->motor->command.q_axis_current_reference_a =
			context->motor->configuration.current_limit_a;
	else if (context->motor->command.q_axis_current_reference_a <
		-context->motor->configuration.current_limit_a)
		context->motor->command.q_axis_current_reference_a =
			-context->motor->configuration.current_limit_a;
	context->applied_revision = context->published_revision;
	return true;
}

bool MotorServiceAdapter_StageCurrentOffsetResult(
	MotorConfigurationAdapterContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc)
{
	uint32_t interrupt_state;
	if (context == 0 || !context->is_initialized ||
		MotorLifecycle_GetDeviceState() != DEVICE_STATE_SERVICING ||
		MotorLifecycle_GetServiceProcedure() !=
			SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION)
		return false;
	interrupt_state = context->critical_section.enter(
		context->critical_section.context);
	if (context->published_revision == context->applied_revision)
		context->candidate = context->motor->configuration;
	context->candidate.phase_a_current_offset_adc = phase_a_offset_adc;
	context->candidate.phase_b_current_offset_adc = phase_b_offset_adc;
	context->candidate.phase_c_current_offset_adc = phase_c_offset_adc;
	context->published_revision++;
	context->critical_section.exit(context->critical_section.context,
		interrupt_state);
	return true;
}

bool MotorServiceAdapter_StagePhaseResistanceResult(
	MotorConfigurationAdapterContext *context, float resistance_ohm)
{
	uint32_t interrupt_state;
	if (context == 0 || !context->is_initialized ||
		resistance_ohm < context->motor_profile->phase_resistance_min_ohm ||
		resistance_ohm > context->motor_profile->phase_resistance_max_ohm ||
		MotorLifecycle_GetDeviceState() != DEVICE_STATE_SERVICING ||
		MotorLifecycle_GetServiceProcedure() !=
			SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION)
		return false;
	interrupt_state = context->critical_section.enter(
		context->critical_section.context);
	if (context->published_revision == context->applied_revision)
		context->candidate = context->motor->configuration;
	context->candidate.phase_resistance_ohm = resistance_ohm;
	MotorServiceAdapter_UpdateCurrentLoopGains(context,
		MOTOR_PARAMETER_PHASE_RESISTANCE_OHM);
	context->published_revision++;
	context->critical_section.exit(context->critical_section.context,
		interrupt_state);
	return true;
}

bool MotorServiceAdapter_StageFrictionModel(
	MotorConfigurationAdapterContext *context, float coulomb_pos_a,
	float coulomb_neg_a, float viscous_pos_a_per_rad_s,
	float viscous_neg_a_per_rad_s)
{
	uint32_t interrupt_state;
	if (context == 0 || !context->is_initialized ||
		!isfinite(coulomb_pos_a) || coulomb_pos_a < 0.0f ||
		!isfinite(coulomb_neg_a) || coulomb_neg_a < 0.0f ||
		!isfinite(viscous_pos_a_per_rad_s) || viscous_pos_a_per_rad_s < 0.0f ||
		!isfinite(viscous_neg_a_per_rad_s) || viscous_neg_a_per_rad_s < 0.0f ||
		MotorLifecycle_GetDeviceState() != DEVICE_STATE_STANDBY)
		return false;
	interrupt_state = context->critical_section.enter(
		context->critical_section.context);
	if (context->published_revision == context->applied_revision)
		context->candidate = context->motor->configuration;
	context->candidate.friction_coulomb_pos_a = coulomb_pos_a;
	context->candidate.friction_coulomb_neg_a = coulomb_neg_a;
	context->candidate.friction_viscous_pos_a_per_rad_s =
		viscous_pos_a_per_rad_s;
	context->candidate.friction_viscous_neg_a_per_rad_s =
		viscous_neg_a_per_rad_s;
	context->candidate.friction_model_valid = true;
	context->published_revision++;
	context->critical_section.exit(context->critical_section.context,
		interrupt_state);
	return true;
}

MotorCommandPort MotorServiceAdapter_CreateCommandPort(
	MotorCommandAdapterContext *context, MotorControlContext *motor,
	const CriticalSectionPort *critical_section)
{
	MotorCommandPort port = {0};
	if (context == 0 || motor == 0 || critical_section == 0 ||
		critical_section->enter == 0 || critical_section->exit == 0)
		return port;
	context->motor = motor;
	context->pending_command = motor->command;
	context->critical_section = *critical_section;
	context->published_revision = 0U;
	context->applied_revision = 0U;
	context->is_initialized = true;
	port.context = context;
	port.get_mode = MotorServiceAdapter_GetMode;
	port.request_mode = MotorServiceAdapter_RequestMode;
	port.request_service = MotorServiceAdapter_RequestService;
	port.request_standby = MotorServiceAdapter_RequestStandby;
	port.request_clear_faults = MotorServiceAdapter_RequestClearFaults;
	port.get_current_limit_a = MotorServiceAdapter_GetCurrentLimit;
	port.get_speed_limit_rad_s = MotorServiceAdapter_GetSpeedLimit;
	port.set_current_reference_a = MotorServiceAdapter_SetCurrentReference;
	port.set_speed_reference_rad_s = MotorServiceAdapter_SetSpeedReference;
	port.set_position_reference_rad = MotorServiceAdapter_SetPositionReference;
	return port;
}

bool MotorServiceAdapter_ApplyPendingCommand(
	MotorCommandAdapterContext *context)
{
	MotorCommand command;
	if (context == 0 || !context->is_initialized ||
		context->published_revision == context->applied_revision)
		return false;
	command = context->pending_command;
	if (command.q_axis_current_reference_a >
		context->motor->configuration.current_limit_a)
		command.q_axis_current_reference_a =
			context->motor->configuration.current_limit_a;
	else if (command.q_axis_current_reference_a <
		-context->motor->configuration.current_limit_a)
		command.q_axis_current_reference_a =
			-context->motor->configuration.current_limit_a;
	if (command.speed_reference_rad_s > context->motor->configuration.speed_limit_rad_s)
		command.speed_reference_rad_s = context->motor->configuration.speed_limit_rad_s;
	else if (command.speed_reference_rad_s <
		-context->motor->configuration.speed_limit_rad_s)
		command.speed_reference_rad_s =
			-context->motor->configuration.speed_limit_rad_s;
	context->motor->command = command;
	context->applied_revision = context->published_revision;
	return true;
}

MotorConfigurationPort MotorServiceAdapter_CreateConfigurationPort(
	MotorConfigurationAdapterContext *context, MotorControlContext *motor,
	const CriticalSectionPort *critical_section,
	const MotorProfile *motor_profile)
{
	MotorConfigurationPort port = {0};
	if (context == 0 || motor == 0 || critical_section == 0 ||
		motor_profile == 0 ||
		motor_profile->current_loop_bandwidth_rad_s <= 0.0f ||
		critical_section->enter == 0 || critical_section->exit == 0)
		return port;
	context->motor = motor;
	context->motor_profile = motor_profile;
	context->candidate = motor->configuration;
	context->critical_section = *critical_section;
	context->published_revision = 0U;
	context->applied_revision = 0U;
	context->is_initialized = true;
	port.context = context;
	port.can_stage = MotorServiceAdapter_CanStageConfiguration;
	port.read = MotorServiceAdapter_ReadConfiguration;
	port.stage = MotorServiceAdapter_StageConfiguration;
	return port;
}
