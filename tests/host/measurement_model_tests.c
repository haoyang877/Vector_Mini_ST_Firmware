#include "measurement_model.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static const uint16_t MinimumOffsets[3] = {0U, 0U, 0U};
static const uint16_t MaximumOffsets[3] = {4095U, 4095U, 4095U};
static const float CurrentScales[3] = {0.01f, 0.02f, 0.03f};

static MeasurementModelConfig MeasurementModelTest_CreateConfig(void)
{
	MeasurementModelConfig config = {0};
	config.minimum_valid_offset_adc = MinimumOffsets;
	config.maximum_valid_offset_adc = MaximumOffsets;
	config.current_a_per_count = CurrentScales;
	config.bus_voltage_v_per_count = 0.01f;
	config.bus_voltage_filter_alpha = 0.25f;
	config.overcurrent_trip_a = 10.0f;
	config.overvoltage_trip_v = 30.0f;
	config.undervoltage_trip_v = 10.0f;
	config.maximum_temperature_c = 100.0f;
	config.overcurrent_confirm_cycles = 1U;
	config.voltage_confirm_cycles = 1U;
	config.temperature_sample_divider = 1U;
	return config;
}

int MeasurementModel_RunHostTests(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config = MeasurementModelTest_CreateConfig();
	MeasurementModelInput input = {0};
	MeasurementModelOutput output;

	config.temperature_protection_enabled = true;
	config.temperature_invalid_is_fault = true;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) != 0U);

	input.temperature_valid = true;
	input.temperature_c = 50.0f;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) == 0U);
	TEST_CHECK(output.temperature_c == 50.0f);

	input.phase_a_adc = 100U;
	input.phase_b_adc = 100U;
	input.phase_c_adc = 100U;
	input.bus_voltage_adc = 1000U;
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK(output.phase_a_current_a == -1.0f);
	TEST_CHECK(output.phase_b_current_a == -2.0f);
	TEST_CHECK(output.phase_c_current_a == -3.0f);
	TEST_CHECK(output.bus_voltage_filtered_v == 2.5f);
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK(output.bus_voltage_filtered_v == 4.375f);

	input.temperature_c = 101.0f;
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) != 0U);

	config.temperature_protection_enabled = false;
	input.temperature_valid = false;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) == 0U);

	config.temperature_protection_enabled = true;
	config.temperature_invalid_is_fault = false;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) == 0U);
	return 0;
}
