#include "measurement_model.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static MeasurementModelConfig MeasurementModelTest_CreateConfig(void)
{
	MeasurementModelConfig config = {0};
	config.minimum_valid_offset_adc = 0;
	config.maximum_valid_offset_adc = 4095;
	config.current_a_per_count = 0.01f;
	config.bus_voltage_v_per_count = 0.01f;
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

	input.temperature_c = 101.0f;
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) != 0U);

	config.temperature_protection_enabled = false;
	input.temperature_valid = false;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_HIGH_TEMPERATURE) == 0U);
	return 0;
}
