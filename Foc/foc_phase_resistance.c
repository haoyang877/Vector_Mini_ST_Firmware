#include "foc_phase_resistance.h"

#include "hw_conf.h"

#define PHASE_RESISTANCE_RAMP_TICKS (FOC_FREQ / 5U)
#define PHASE_RESISTANCE_SETTLE_TICKS (FOC_FREQ / 5U)
#define PHASE_RESISTANCE_SAMPLE_TICKS (FOC_FREQ / 10U)
#define PHASE_RESISTANCE_PAUSE_TICKS (FOC_FREQ / 10U)
#define PHASE_RESISTANCE_TIMEOUT_TICKS (FOC_FREQ * 3U)
#define PHASE_RESISTANCE_TEST_CURRENT_RATIO 0.25f
#define PHASE_RESISTANCE_TEST_CURRENT_MIN 0.5f
#define PHASE_RESISTANCE_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_Q_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_VOLTAGE_TOLERANCE 0.01f
#define PHASE_RESISTANCE_VOLTAGE_MIN_DELTA 0.005f
#define PHASE_RESISTANCE_VOLTAGE_FILTER 0.02f
#define PHASE_RESISTANCE_PATH_COMPENSATION_OHM 0.004f
#define PHASE_RESISTANCE_BALANCE_LIMIT_PCT 5.0f

typedef struct
{
	PhaseResistanceContext_TypeDef core;
	PhaseResistanceModeStatus_TypeDef status;
	bool started;
} PhaseResistanceModeContext_TypeDef;

static PhaseResistanceModeContext_TypeDef PhaseResistanceModeContext;

/** The adapter owns PWM release and current-loop reset; the core never does. */
static void PhaseResistanceMode_StopOutput(FOC_TypeDef *foc, MotorControl_TypeDef *motor)
{
	motor->idRef = 0.0f;
	motor->iqRef = 0.0f;
	FOC_CurrentController_Reset(foc);
	PWM_TurnOnHighSides();
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
	motor->phase_resistance_balanced = false;
}

static bool PhaseResistanceMode_Start(MotorControl_TypeDef *motor)
{
	PhaseResistanceConfig_TypeDef config;
	float current_limit = motor->current_limit * PHASE_RESISTANCE_TEST_CURRENT_RATIO;

	config.test_current = motor->calib_current < current_limit ?
		motor->calib_current : current_limit;
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
	config.path_compensation_ohm = PHASE_RESISTANCE_PATH_COMPENSATION_OHM;
	config.balance_limit_pct = PHASE_RESISTANCE_BALANCE_LIMIT_PCT;

	PhaseResistance_Init(&PhaseResistanceModeContext.core);
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
	if (foc->Vbus_filt < 10.0f)
		return PhaseResistanceMode_Fail(foc, motor, PHASE_RESISTANCE_MODE_UNDER_VOLTAGE);
	if (foc->Vbus_filt > 30.0f)
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
		motor->idRef = command.id_ref;
		motor->iqRef = command.iq_ref;
		FOC_Current(foc, motor, command.electrical_angle, 0.0f);

		sample.id = foc->Id;
		sample.iq = foc->Iq;
		sample.id_filt = foc->Id_filt;
		sample.iq_filt = foc->Iq_filt;
		sample.vd = foc->mod_d * foc->Vbus_filt / 1.5f;
		sample.vq = foc->mod_q * foc->Vbus_filt / 1.5f;
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
