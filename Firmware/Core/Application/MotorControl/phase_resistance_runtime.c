#include "Core/Application/MotorControl/phase_resistance_runtime.h"

#include <math.h>
#include <stddef.h>

static float PhaseResistanceRuntime_Min(float first, float second)
{
	return first < second ? first : second;
}

/** The adapter owns PWM release and current-loop reset; the core never does. */
static void PhaseResistanceRuntime_StopOutput(
	PhaseResistanceRuntimeContext *context, CurrentControlContext *current_control,
	MotorControlContext *motor)
{
	motor->targets.d_axis_current_a = 0.0f;
	motor->targets.q_axis_current_a = 0.0f;
	CurrentControlRuntime_ResetControllers(current_control);
	CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
	context->applied_mod_d = 0.0f;
	context->applied_mod_q = 0.0f;
	context->applied_voltage_valid = false;
	context->telemetry_valid = false;
}

static void PhaseResistanceRuntime_ClearResult(MotorControlContext *motor)
{
	uint8_t index;

	for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
		motor->runtime.phase_resistance_vector_ohm[index] = 0.0f;
	motor->runtime.phase_a_resistance_ohm = 0.0f;
	motor->runtime.phase_b_resistance_ohm = 0.0f;
	motor->runtime.phase_c_resistance_ohm = 0.0f;
	motor->runtime.phase_resistance_spread_pct = 0.0f;
	motor->runtime.phase_resistance_valid = false;
	motor->runtime.phase_resistance_warning = false;
	motor->runtime.phase_resistance_balanced = false;
}

static float PhaseResistanceRuntime_LimitCurrent(const MotorControlContext *motor,
	const PhaseResistanceRuntimeBoardConfig *runtime_config,
	float requested_current)
{
	float maximum_current = runtime_config->tuning.test_current_max_a;

	maximum_current = PhaseResistanceRuntime_Min(maximum_current, motor->configuration.current_limit_a);
	return PhaseResistanceRuntime_Min(requested_current, maximum_current);
}

static bool PhaseResistanceRuntime_Start(PhaseResistanceRuntimeContext *context,
	MotorControlContext *motor,
	const PhaseResistanceRuntimeBoardConfig *runtime_config)
{
	PhaseResistanceConfig config;

	config.test_current_low = PhaseResistanceRuntime_LimitCurrent(motor,
		runtime_config, runtime_config->tuning.test_current_low_a);
	config.test_current_high = PhaseResistanceRuntime_LimitCurrent(motor,
		runtime_config, runtime_config->tuning.test_current_high_a);
	config.maximum_test_current = PhaseResistanceRuntime_LimitCurrent(motor,
		runtime_config, runtime_config->tuning.test_current_max_a);
	config.minimum_test_current = runtime_config->tuning.test_current_min_a;
	config.ramp_ticks = runtime_config->control_frequency_hz *
		runtime_config->tuning.ramp_time_ms / 1000U;
	config.settle_ticks = runtime_config->control_frequency_hz *
		runtime_config->tuning.settle_time_ms / 1000U;
	config.sample_ticks = runtime_config->control_frequency_hz *
		runtime_config->tuning.sample_time_ms / 1000U;
	config.pause_ticks = runtime_config->control_frequency_hz *
		runtime_config->tuning.pause_time_ms / 1000U;
	config.timeout_ticks = runtime_config->control_frequency_hz *
		runtime_config->tuning.timeout_ms / 1000U;
	config.current_tolerance = runtime_config->tuning.current_tolerance_a;
	config.q_current_tolerance = runtime_config->tuning.q_current_tolerance_a;
	config.voltage_tolerance = runtime_config->tuning.voltage_tolerance_v;
	config.voltage_min_delta = runtime_config->tuning.voltage_min_delta_v;
	config.voltage_filter = runtime_config->tuning.voltage_filter_alpha;
	config.path_compensation_ohm =
		runtime_config->path_compensation_ohm;
	config.balance_warning_pct = runtime_config->tuning.balance_warning_pct;
	config.balance_fault_pct = runtime_config->tuning.balance_fault_pct;

	PhaseResistance_Init(&context->core);
	context->applied_mod_d = 0.0f;
	context->applied_mod_q = 0.0f;
	context->applied_voltage_valid = false;
	context->telemetry_valid = false;
	return PhaseResistance_Start(&context->core, &config);
}

static void PhaseResistanceRuntime_CopyResult(
	const PhaseResistanceRuntimeContext *context,
	MotorControlContext *motor)
{
	PhaseResistanceResult result;
	uint8_t index;

	if (!PhaseResistance_GetResult(&context->core, &result))
		return;

	for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
		motor->runtime.phase_resistance_vector_ohm[index] = result.vector_resistance[index];
	motor->runtime.phase_a_resistance_ohm = result.phase_a_resistance_ohm;
	motor->runtime.phase_b_resistance_ohm = result.phase_b_resistance_ohm;
	motor->runtime.phase_c_resistance_ohm = result.phase_c_resistance_ohm;
	motor->runtime.phase_resistance_spread_pct = result.spread_pct;
	motor->runtime.phase_resistance_valid = result.valid;
	motor->runtime.phase_resistance_warning = result.warning;
	motor->runtime.phase_resistance_balanced = result.balanced;
}

static PhaseResistanceRuntimeStatus PhaseResistanceRuntime_Fail(
	PhaseResistanceRuntimeContext *context, CurrentControlContext *current_control,
	MotorControlContext *motor, PhaseResistanceRuntimeStatus status)
{
	PhaseResistanceRuntime_StopOutput(context, current_control, motor);
	PhaseResistanceRuntime_ClearResult(motor);
	PhaseResistance_Cancel(&context->core);
	context->status = status;
	return status;
}

static PhaseResistanceRuntimeStatus PhaseResistanceRuntime_MapCoreStatus(
	PhaseResistanceCoreStatus status)
{
	if (status == PHASE_RESISTANCE_CORE_DONE)
		return PHASE_RESISTANCE_MODE_DONE;
	if (status == PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT)
		return PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT;
	return PHASE_RESISTANCE_MODE_INVALID_RESULT;
}

PhaseResistanceRuntimeStatus PhaseResistanceRuntime_Run(
	PhaseResistanceRuntimeContext *context, CurrentControlContext *current_control,
	MotorControlContext *motor,
	const PhaseResistanceRuntimeBoardConfig *runtime_config)
{
	PhaseResistanceCommand command;
	PhaseResistanceSample sample;
	PhaseResistanceCoreStatus core_status;
	PhaseResistanceRuntimeStatus mode_status;

	if (context == NULL || current_control == NULL || motor == NULL ||
		runtime_config == NULL)
		return PHASE_RESISTANCE_MODE_INVALID_RESULT;
	if (context->status != PHASE_RESISTANCE_MODE_RUNNING)
	{
		PhaseResistanceRuntime_StopOutput(context, current_control, motor);
		return context->status;
	}
	if (current_control->filtered_bus_voltage_v <
		runtime_config->undervoltage_trip_v)
		return PhaseResistanceRuntime_Fail(context, current_control, motor, PHASE_RESISTANCE_MODE_UNDER_VOLTAGE);
	if (current_control->filtered_bus_voltage_v >
		runtime_config->overvoltage_trip_v)
		return PhaseResistanceRuntime_Fail(context, current_control, motor, PHASE_RESISTANCE_MODE_OVER_VOLTAGE);

	if (!context->started)
	{
		PhaseResistanceRuntime_ClearResult(motor);
		if (!PhaseResistanceRuntime_Start(context, motor, runtime_config))
			return PhaseResistanceRuntime_Fail(context, current_control, motor, PHASE_RESISTANCE_MODE_INVALID_RESULT);
		context->started = true;
		CurrentControlRuntime_ResetControllers(current_control);
		return PHASE_RESISTANCE_MODE_RUNNING;
	}

	PhaseResistance_GetCommand(&context->core, &command);
	if (!command.inject_current)
	{
		if (command.reset_current_controller)
			PhaseResistanceRuntime_StopOutput(context, current_control, motor);
		core_status = PhaseResistance_InputSample(&context->core, NULL);
	}
	else
	{
		float applied_mod_d = context->applied_mod_d;
		float applied_mod_q = context->applied_mod_q;
		float current_magnitude;

		motor->targets.d_axis_current_a = command.id_ref;
		motor->targets.q_axis_current_a = command.iq_ref;
		CurrentControlRuntime_RunClosedLoop(current_control, motor, command.electrical_angle, 0.0f);

		sample.id = current_control->d_axis_current_a;
		sample.iq = current_control->q_axis_current_a;
		sample.id_filt = current_control->filtered_d_axis_current_a;
		sample.iq_filt = current_control->filtered_q_axis_current_a;
		if (context->applied_voltage_valid)
		{
			sample.vd = applied_mod_d * current_control->bus_voltage_v / 1.5f;
			sample.vq = applied_mod_q * current_control->bus_voltage_v / 1.5f;
		}
		else
		{
			sample.vd = 0.0f;
			sample.vq = 0.0f;
		}
		current_magnitude = sqrtf(sample.id * sample.id + sample.iq * sample.iq);
		context->telemetry.electrical_angle = command.electrical_angle;
		context->telemetry.id_ref = command.id_ref;
		context->telemetry.id = sample.id;
		context->telemetry.iq = sample.iq;
		context->telemetry.vd = sample.vd;
		context->telemetry.vq = sample.vq;
		context->telemetry.current_magnitude = current_magnitude;
		context->telemetry.parallel_voltage =
			current_magnitude > 0.0f ? (sample.vd * sample.id + sample.vq * sample.iq) /
			current_magnitude : 0.0f;
		context->telemetry.vbus = current_control->bus_voltage_v;
		context->telemetry_valid = context->applied_voltage_valid;

		context->applied_mod_d = current_control->d_axis_modulation;
		context->applied_mod_q = current_control->q_axis_modulation;
		context->applied_voltage_valid = true;
		core_status = PhaseResistance_InputSample(&context->core, &sample);
	}

	if (core_status == PHASE_RESISTANCE_CORE_RUNNING)
		return PHASE_RESISTANCE_MODE_RUNNING;

	mode_status = PhaseResistanceRuntime_MapCoreStatus(core_status);
	if (mode_status == PHASE_RESISTANCE_MODE_DONE)
	{
		PhaseResistanceRuntime_StopOutput(context, current_control, motor);
		PhaseResistanceRuntime_CopyResult(context, motor);
		context->status = mode_status;
		return mode_status;
	}
	return PhaseResistanceRuntime_Fail(context, current_control, motor, mode_status);
}

bool PhaseResistanceRuntime_GetTelemetry(
	const PhaseResistanceRuntimeContext *context,
	PhaseResistanceRuntimeTelemetry *telemetry)
{
	if (context == NULL || telemetry == NULL || !context->started ||
		!context->telemetry_valid)
		return false;

	*telemetry = context->telemetry;
	return true;
}

void PhaseResistanceRuntime_Cancel(PhaseResistanceRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor)
{
	if (context == NULL)
		return;
	if (current_control != NULL && motor != NULL &&
		(context->started || context->status != PHASE_RESISTANCE_MODE_RUNNING))
		PhaseResistanceRuntime_StopOutput(context, current_control, motor);

	PhaseResistance_Cancel(&context->core);
	context->started = false;
	context->status = PHASE_RESISTANCE_MODE_RUNNING;
}
