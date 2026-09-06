#include "Core/Application/Commissioning/calibration_service.h"
#include "Core/Application/Commissioning/identification_service.h"
#include "Core/Application/motor_command_service.h"

#include <math.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	MotorPortMode mode;
	float current_limit_a;
	float speed_limit_rad_s;
	float current_reference_a;
	float speed_reference_rad_s;
} ServiceValidationMotorFake;

static MotorPortMode ServiceValidation_GetMode(void *context)
{
	return ((ServiceValidationMotorFake *)context)->mode;
}

static bool ServiceValidation_RequestMode(void *context, MotorPortMode mode)
{
	((ServiceValidationMotorFake *)context)->mode = mode;
	return true;
}

static bool ServiceValidation_RequestService(void *context,
	MotorPortService service)
{
	(void)context;
	(void)service;
	return true;
}

static bool ServiceValidation_RequestStandby(void *context)
{
	((ServiceValidationMotorFake *)context)->mode = MOTOR_PORT_MODE_NONE;
	return true;
}

static bool ServiceValidation_RequestClearFaults(void *context)
{
	(void)context;
	return true;
}

static float ServiceValidation_GetCurrentLimit(void *context)
{
	return ((ServiceValidationMotorFake *)context)->current_limit_a;
}

static float ServiceValidation_GetSpeedLimit(void *context)
{
	return ((ServiceValidationMotorFake *)context)->speed_limit_rad_s;
}

static void ServiceValidation_SetCurrent(void *context, float current_a)
{
	((ServiceValidationMotorFake *)context)->current_reference_a = current_a;
}

static void ServiceValidation_SetSpeed(void *context, float speed_rad_s)
{
	((ServiceValidationMotorFake *)context)->speed_reference_rad_s = speed_rad_s;
}

static void ServiceValidation_SetPosition(void *context, float position_rad)
{
	(void)context;
	(void)position_rad;
}

int ServiceResultValidation_RunHostTests(void)
{
	DeviceLifecycleContext lifecycle;
	CalibrationCurrentOffsetLimits offset_limits = {0};
	IdentificationServiceConfig identification_config = {0};
	CalibrationServiceContext calibration;
	IdentificationServiceContext identification;
	MotorCommandServiceContext motor_command = {0};
	ServiceValidationMotorFake motor_fake = {0};
	MotorCommandPort motor_port = {0};
	float mean_resistance_ohm = 0.0f;

	offset_limits.minimum_current_offset_adc[0] = 1948U;
	offset_limits.minimum_current_offset_adc[1] = 1947U;
	offset_limits.minimum_current_offset_adc[2] = 1946U;
	offset_limits.maximum_current_offset_adc[0] = 2148U;
	offset_limits.maximum_current_offset_adc[1] = 2149U;
	offset_limits.maximum_current_offset_adc[2] = 2150U;
	identification_config.phase_resistance_balance_fault_pct = 10.0f;
	identification_config.phase_resistance_design_ohm = 0.10f;
	identification_config.phase_resistance_min_ohm = 0.01f;
	identification_config.phase_resistance_max_ohm = 1.0f;
	identification_config.phase_resistance_design_tolerance_pct = 20.0f;
	DeviceLifecycle_Initialize(&lifecycle);
	TEST_CHECK(CalibrationService_Initialize(&calibration, &lifecycle,
		&offset_limits, 1000U));
	TEST_CHECK(CalibrationService_AcceptCurrentOffsetResult(&calibration,
		2048U, 2047U, 2049U));
	TEST_CHECK(!CalibrationService_AcceptCurrentOffsetResult(&calibration,
		1000U, 2047U, 2049U));
	TEST_CHECK(!CalibrationService_AcceptCurrentOffsetResult(&calibration,
		2048U, 1946U, 2049U));

	TEST_CHECK(IdentificationService_Initialize(&identification, &lifecycle,
		&identification_config, 1000U));
	TEST_CHECK(IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 9.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(mean_resistance_ohm > 0.099f &&
		mean_resistance_ohm < 0.101f);
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 11.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.15f, 0.15f, 0.15f, 0.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 9.0f, false,
		&mean_resistance_ohm));

	/* The public command boundary remains symmetric around zero when its
	 * implementation uses the binary32 absolute-value operation. */
	motor_fake.current_limit_a = 2.0f;
	motor_fake.speed_limit_rad_s = 12.5663706144f;
	motor_port.context = &motor_fake;
	motor_port.get_mode = ServiceValidation_GetMode;
	motor_port.request_mode = ServiceValidation_RequestMode;
	motor_port.request_service = ServiceValidation_RequestService;
	motor_port.request_standby = ServiceValidation_RequestStandby;
	motor_port.request_clear_faults = ServiceValidation_RequestClearFaults;
	motor_port.get_current_limit_a = ServiceValidation_GetCurrentLimit;
	motor_port.get_speed_limit_rad_s = ServiceValidation_GetSpeedLimit;
	motor_port.set_current_reference_a = ServiceValidation_SetCurrent;
	motor_port.set_speed_reference_rad_s = ServiceValidation_SetSpeed;
	motor_port.set_position_reference_rad = ServiceValidation_SetPosition;
	TEST_CHECK(MotorCommandService_Initialize(&motor_command, &motor_port));
	TEST_CHECK(MotorCommandService_SetCurrentReferenceA(&motor_command, 2.0f) ==
		MOTOR_COMMAND_ACCEPTED);
	TEST_CHECK(MotorCommandService_SetCurrentReferenceA(&motor_command, -2.0f) ==
		MOTOR_COMMAND_ACCEPTED);
	TEST_CHECK(motor_fake.current_reference_a == -2.0f);
	TEST_CHECK(MotorCommandService_SetCurrentReferenceA(&motor_command, 2.01f) ==
		MOTOR_COMMAND_OUT_OF_RANGE);
	TEST_CHECK(MotorCommandService_SetCurrentReferenceA(&motor_command, -2.01f) ==
		MOTOR_COMMAND_OUT_OF_RANGE);
	TEST_CHECK(MotorCommandService_SetCurrentReferenceA(&motor_command, NAN) ==
		MOTOR_COMMAND_INVALID_VALUE);
	TEST_CHECK(MotorCommandService_SetSpeedReferenceRps(&motor_command, 1.99f) ==
		MOTOR_COMMAND_ACCEPTED);
	TEST_CHECK(MotorCommandService_SetSpeedReferenceRps(&motor_command, -1.99f) ==
		MOTOR_COMMAND_ACCEPTED);
	TEST_CHECK(motor_fake.speed_reference_rad_s < -12.50f);
	TEST_CHECK(MotorCommandService_SetSpeedReferenceRps(&motor_command, 2.01f) ==
		MOTOR_COMMAND_OUT_OF_RANGE);
	TEST_CHECK(MotorCommandService_SetSpeedReferenceRps(&motor_command, -2.01f) ==
		MOTOR_COMMAND_OUT_OF_RANGE);
	TEST_CHECK(MotorCommandService_SetSpeedReferenceRps(&motor_command, INFINITY) ==
		MOTOR_COMMAND_INVALID_VALUE);
	return 0;
}
