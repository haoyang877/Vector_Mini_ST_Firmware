#include "calibration_service.h"
#include "identification_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ServiceResultValidation_RunHostTests(void)
{
	DeviceLifecycleContext lifecycle;
	BoardProfile board = {0};
	MotorProfile motor = {0};
	CalibrationServiceContext calibration;
	IdentificationServiceContext identification;
	float mean_resistance_ohm = 0.0f;

	board.minimum_current_offset_adc = 1948U;
	board.maximum_current_offset_adc = 2148U;
	motor.phase_resistance_balance_fault_pct = 10.0f;
	DeviceLifecycle_Initialize(&lifecycle);
	TEST_CHECK(CalibrationService_Initialize(&calibration, &lifecycle,
		&board, 1000U));
	TEST_CHECK(CalibrationService_AcceptCurrentOffsetResult(&calibration,
		2048U, 2047U, 2049U));
	TEST_CHECK(!CalibrationService_AcceptCurrentOffsetResult(&calibration,
		1000U, 2047U, 2049U));

	TEST_CHECK(IdentificationService_Initialize(&identification, &lifecycle,
		&motor, 1000U));
	TEST_CHECK(IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 9.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(mean_resistance_ohm > 0.099f &&
		mean_resistance_ohm < 0.101f);
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 11.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 9.0f, false,
		&mean_resistance_ohm));
	return 0;
}
