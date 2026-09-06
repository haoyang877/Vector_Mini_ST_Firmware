#include "calibration_service.h"
#include "identification_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ServiceResultValidation_RunHostTests(void)
{
	DeviceLifecycleContext lifecycle;
	CalibrationCurrentOffsetLimits offset_limits = {0};
	uint16_t minimum_offsets[3] = {1948U, 1947U, 1946U};
	uint16_t maximum_offsets[3] = {2148U, 2149U, 2150U};
	MotorProfile motor = {0};
	CalibrationServiceContext calibration;
	IdentificationServiceContext identification;
	float mean_resistance_ohm = 0.0f;

	offset_limits.minimum_current_offset_adc = minimum_offsets;
	offset_limits.maximum_current_offset_adc = maximum_offsets;
	motor.phase_resistance_balance_fault_pct = 10.0f;
	motor.phase_resistance_ohm = 0.10f;
	motor.phase_resistance_min_ohm = 0.01f;
	motor.phase_resistance_max_ohm = 1.0f;
	motor.phase_resistance_design_tolerance_pct = 20.0f;
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
		&identification, 0.15f, 0.15f, 0.15f, 0.0f, true,
		&mean_resistance_ohm));
	TEST_CHECK(!IdentificationService_AcceptPhaseResistanceResult(
		&identification, 0.10f, 0.11f, 0.09f, 9.0f, false,
		&mean_resistance_ohm));
	return 0;
}
