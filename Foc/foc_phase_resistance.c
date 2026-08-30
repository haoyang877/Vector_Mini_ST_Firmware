#include "foc_phase_resistance.h"

#include <math.h>

#include "foc_param_profile.h"
#include "hw_conf.h"

#define PHASE_RESISTANCE_RAMP_TICKS \
	((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_RAMP_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_SETTLE_TICKS \
	((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_SETTLE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_SAMPLE_TICKS \
	((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_SAMPLE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_PAUSE_TICKS \
	((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_PAUSE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_TIMEOUT_TICKS \
	((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_TIMEOUT_MS) / 1000U)
#define PHASE_RESISTANCE_TEST_CURRENT_MIN 0.5f
#define PHASE_RESISTANCE_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_Q_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_VOLTAGE_TOLERANCE 0.01f
#define PHASE_RESISTANCE_VOLTAGE_MIN_DELTA 0.005f
#define PHASE_RESISTANCE_VOLTAGE_FILTER 0.02f
#define PHASE_RESISTANCE_VBUS_MIN 10.0f
#define PHASE_RESISTANCE_VBUS_MAX 30.0f

typedef struct
{
	PhaseResistanceContext_TypeDef core;
	PhaseResistanceModeStatus_TypeDef status;
	float applied_mod_d;
	float applied_mod_q;
	PhaseResistanceModeTelemetry_TypeDef telemetry;
	bool applied_voltage_valid;
	bool telemetry_valid;
	bool started;
} PhaseResistanceModeContext_TypeDef;

static PhaseResistanceModeContext_TypeDef PhaseResistanceModeContext;

static float PhaseResistanceMode_Min(float first, float second)
{
	return first < second ? first : second;
}

/** The adapter owns PWM release and current-loop reset; the core never does. */
static void PhaseResistanceMode_StopOutput(FOC_TypeDef *foc, MotorControl_TypeDef *motor)
{
	motor->idRef = 0.0f;
	motor->iqRef = 0.0f;
	FOC_CurrentController_Reset(foc);
	PWM_TurnOnHighSides();
	PhaseResistanceModeContext.applied_mod_d = 0.0f;
	PhaseResistanceModeContext.applied_mod_q = 0.0f;
	PhaseResistanceModeContext.applied_voltage_valid = false;
	PhaseResistanceModeContext.telemetry_valid = false;
}

static void PhaseResistanceMode_ClearResult(MotorControl_TypeDef *motor)
{
	uint8_t index;

	for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
		motor->phase_resistance_vector[index] = 0.0f;
	motor->phase_resistance_a = 0.0f;
	motor->phase_resistance_b = 0.0f;
	motor->phase_resistance_c = 0.0f;
	motor->phase_resistance_spread_pct = 0.0f;
	motor->phase_resistance_valid = false;
	motor->phase_resistance_warning = false;
	motor->phase_resistance_balanced = false;
}

static float PhaseResistanceMode_LimitCurrent(const MotorControl_TypeDef *motor,
	float requested_current)
{
	float maximum_current = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A;

	maximum_current = PhaseResistanceMode_Min(maximum_current, motor->current_limit);
	return PhaseResistanceMode_Min(requested_current, maximum_current);
}

static bool PhaseResistanceMode_Start(MotorControl_TypeDef *motor)
{
	PhaseResistanceConfig_TypeDef config;

	config.test_current_low = PhaseResistanceMode_LimitCurrent(motor,
		PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_LOW_A);
	config.test_current_high = PhaseResistanceMode_LimitCurrent(motor,
		PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_HIGH_A);
	config.maximum_test_current = PhaseResistanceMode_LimitCurrent(motor,
		PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A);
	config.minimum_test_current = PHASE_RESISTANCE_TEST_CURRENT_MIN;
	config.ramp_ticks = PHASE_RESISTANCE_RAMP_TICKS;
	config.settle_ticks = PHASE_RESISTANCE_SETTLE_TICKS;
	config.sample_ticks = PHASE_RESISTANCE_SAMPLE_TICKS;
	config.pause_ticks = PHASE_RESISTANCE_PAUSE_TICKS;
	config.timeout_ticks = PHASE_RESISTANCE_TIMEOUT_TICKS;
	config.current_tolerance = PHASE_RESISTANCE_CURRENT_TOLERANCE;
	config.q_current_tolerance = PHASE_RESISTANCE_Q_CURRENT_TOLERANCE;
	config.voltage_tolerance = PHASE_RESISTANCE_VOLTAGE_TOLERANCE;
	config.voltage_min_delta = PHASE_RESISTANCE_VOLTAGE_MIN_DELTA;
	config.voltage_filter = PHASE_RESISTANCE_VOLTAGE_FILTER;
	config.path_compensation_ohm = PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM;
	config.balance_warning_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_WARNING_PCT;
	config.balance_fault_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_FAULT_PCT;

	PhaseResistance_Init(&PhaseResistanceModeContext.core);
	PhaseResistanceModeContext.applied_mod_d = 0.0f;
	PhaseResistanceModeContext.applied_mod_q = 0.0f;
	PhaseResistanceModeContext.applied_voltage_valid = false;
	PhaseResistanceModeContext.telemetry_valid = false;
	return PhaseResistance_Start(&PhaseResistanceModeContext.core, &config);
}

static void PhaseResistanceMode_CopyResult(MotorControl_TypeDef *motor)
{
	PhaseResistanceResult_TypeDef result;
	uint8_t index;

	if (!PhaseResistance_GetResult(&PhaseResistanceModeContext.core, &result))
		return;

	for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
		motor->phase_resistance_vector[index] = result.vector_resistance[index];
	motor->phase_resistance_a = result.phase_resistance_a;
	motor->phase_resistance_b = result.phase_resistance_b;
	motor->phase_resistance_c = result.phase_resistance_c;
	motor->phase_resistance_spread_pct = result.spread_pct;
	motor->phase_resistance_valid = result.valid;
	motor->phase_resistance_warning = result.warning;
	motor->phase_resistance_balanced = result.balanced;
}

static PhaseResistanceModeStatus_TypeDef PhaseResistanceMode_Fail(FOC_TypeDef *foc,
	MotorControl_TypeDef *motor, PhaseResistanceModeStatus_TypeDef status)
{
	PhaseResistanceMode_StopOutput(foc, motor);
	PhaseResistanceMode_ClearResult(motor);
	PhaseResistance_Cancel(&PhaseResistanceModeContext.core);
	PhaseResistanceModeContext.status = status;
	return status;
}

static PhaseResistanceModeStatus_TypeDef PhaseResistanceMode_MapCoreStatus(
	PhaseResistanceCoreStatus_TypeDef status)
{
	if (status == PHASE_RESISTANCE_CORE_DONE)
		return PHASE_RESISTANCE_MODE_DONE;
	if (status == PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT)
		return PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT;
	return PHASE_RESISTANCE_MODE_INVALID_RESULT;
}

PhaseResistanceModeStatus_TypeDef PhaseResistanceMode_Run(FOC_TypeDef *foc,
	MotorControl_TypeDef *motor)
{
	PhaseResistanceCommand_TypeDef command;
	PhaseResistanceSample_TypeDef sample;
	PhaseResistanceCoreStatus_TypeDef core_status;
	PhaseResistanceModeStatus_TypeDef mode_status;

	if (foc == NULL || motor == NULL)
		return PHASE_RESISTANCE_MODE_INVALID_RESULT;
	if (PhaseResistanceModeContext.status != PHASE_RESISTANCE_MODE_RUNNING)
	{
		PhaseResistanceMode_StopOutput(foc, motor);
		return PhaseResistanceModeContext.status;
	}
	if (foc->Vbus_filt < PHASE_RESISTANCE_VBUS_MIN)
		return PhaseResistanceMode_Fail(foc, motor, PHASE_RESISTANCE_MODE_UNDER_VOLTAGE);
	if (foc->Vbus_filt > PHASE_RESISTANCE_VBUS_MAX)
		return PhaseResistanceMode_Fail(foc, motor, PHASE_RESISTANCE_MODE_OVER_VOLTAGE);

	if (!PhaseResistanceModeContext.started)
	{
		PhaseResistanceMode_ClearResult(motor);
		if (!PhaseResistanceMode_Start(motor))
			return PhaseResistanceMode_Fail(foc, motor, PHASE_RESISTANCE_MODE_INVALID_RESULT);
		PhaseResistanceModeContext.started = true;
		FOC_CurrentController_Reset(foc);
		return PHASE_RESISTANCE_MODE_RUNNING;
	}

	PhaseResistance_GetCommand(&PhaseResistanceModeContext.core, &command);
	if (!command.inject_current)
	{
		if (command.reset_current_controller)
			PhaseResistanceMode_StopOutput(foc, motor);
		core_status = PhaseResistance_InputSample(&PhaseResistanceModeContext.core, NULL);
	}
	else
	{
		float applied_mod_d = PhaseResistanceModeContext.applied_mod_d;
		float applied_mod_q = PhaseResistanceModeContext.applied_mod_q;
		float current_magnitude;

		motor->idRef = command.id_ref;
		motor->iqRef = command.iq_ref;
		FOC_Current(foc, motor, command.electrical_angle, 0.0f);

		sample.id = foc->Id;
		sample.iq = foc->Iq;
		sample.id_filt = foc->Id_filt;
		sample.iq_filt = foc->Iq_filt;
		if (PhaseResistanceModeContext.applied_voltage_valid)
		{
			sample.vd = applied_mod_d * foc->Vbus / 1.5f;
			sample.vq = applied_mod_q * foc->Vbus / 1.5f;
		}
		else
		{
			sample.vd = 0.0f;
			sample.vq = 0.0f;
		}
		current_magnitude = sqrtf(sample.id * sample.id + sample.iq * sample.iq);
		PhaseResistanceModeContext.telemetry.electrical_angle = command.electrical_angle;
		PhaseResistanceModeContext.telemetry.id_ref = command.id_ref;
		PhaseResistanceModeContext.telemetry.id = sample.id;
		PhaseResistanceModeContext.telemetry.iq = sample.iq;
		PhaseResistanceModeContext.telemetry.vd = sample.vd;
		PhaseResistanceModeContext.telemetry.vq = sample.vq;
		PhaseResistanceModeContext.telemetry.current_magnitude = current_magnitude;
		PhaseResistanceModeContext.telemetry.parallel_voltage =
			current_magnitude > 0.0f ? (sample.vd * sample.id + sample.vq * sample.iq) /
			current_magnitude : 0.0f;
		PhaseResistanceModeContext.telemetry.vbus = foc->Vbus;
		PhaseResistanceModeContext.telemetry_valid =
			PhaseResistanceModeContext.applied_voltage_valid;

		PhaseResistanceModeContext.applied_mod_d = foc->mod_d;
		PhaseResistanceModeContext.applied_mod_q = foc->mod_q;
		PhaseResistanceModeContext.applied_voltage_valid = true;
		core_status = PhaseResistance_InputSample(&PhaseResistanceModeContext.core, &sample);
	}

	if (core_status == PHASE_RESISTANCE_CORE_RUNNING)
		return PHASE_RESISTANCE_MODE_RUNNING;

	mode_status = PhaseResistanceMode_MapCoreStatus(core_status);
	if (mode_status == PHASE_RESISTANCE_MODE_DONE)
	{
		PhaseResistanceMode_StopOutput(foc, motor);
		PhaseResistanceMode_CopyResult(motor);
		PhaseResistanceModeContext.status = mode_status;
		return mode_status;
	}
	return PhaseResistanceMode_Fail(foc, motor, mode_status);
}

bool PhaseResistanceMode_GetTelemetry(PhaseResistanceModeTelemetry_TypeDef *telemetry)
{
	if (telemetry == NULL || !PhaseResistanceModeContext.started ||
		!PhaseResistanceModeContext.telemetry_valid)
		return false;

	*telemetry = PhaseResistanceModeContext.telemetry;
	return true;
}

void PhaseResistanceMode_Cancel(FOC_TypeDef *foc, MotorControl_TypeDef *motor)
{
	if (foc != NULL && motor != NULL &&
		(PhaseResistanceModeContext.started ||
		PhaseResistanceModeContext.status != PHASE_RESISTANCE_MODE_RUNNING))
		PhaseResistanceMode_StopOutput(foc, motor);

	PhaseResistance_Cancel(&PhaseResistanceModeContext.core);
	PhaseResistanceModeContext.started = false;
	PhaseResistanceModeContext.status = PHASE_RESISTANCE_MODE_RUNNING;
}
