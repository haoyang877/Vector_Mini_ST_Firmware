#include "measurement_runtime.h"
#include "current_offset_calibration_runtime.h"
#include "motor_state_runtime.h"

#include <string.h>

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)
#define TEST_EPSILON_A (0.0001f)

static MotorFaultCode MeasurementRuntimeTest_LastFault;
static uint32_t MeasurementRuntimeTest_FaultCount;

void MotorState_RaiseFault(MotorStateContext *context, MotorFaultCode fault)
{
	(void)context;
	MeasurementRuntimeTest_LastFault = fault;
	MeasurementRuntimeTest_FaultCount++;
}

static float MeasurementRuntimeTest_Abs(float value)
{
	return value >= 0.0f ? value : -value;
}

static bool MeasurementRuntimeTest_Near(float actual, float expected)
{
	return MeasurementRuntimeTest_Abs(actual - expected) <= TEST_EPSILON_A;
}

static void MeasurementRuntimeTest_ResetFault(void)
{
	MeasurementRuntimeTest_LastFault = MOTOR_FAULT_NONE;
	MeasurementRuntimeTest_FaultCount = 0U;
}

static MeasurementModelConfig MeasurementRuntimeTest_BaseConfig(void)
{
	MeasurementModelConfig config;

	memset(&config, 0, sizeof(config));
	config.bus_voltage_v_per_count = 0.01f;
	config.bus_voltage_filter_alpha = 1.0f;
	config.overcurrent_trip_a = 20.0f;
	config.overvoltage_trip_v = 60.0f;
	config.undervoltage_trip_v = 5.0f;
	config.overcurrent_confirm_cycles = 1U;
	config.voltage_confirm_cycles = 1U;
	return config;
}

static void MeasurementRuntimeTest_SetChannel(
	MeasurementCurrentChannelConfig *channel, uint8_t acquisition_index,
	MeasurementCurrentChannelRole role, uint16_t design_offset,
	float current_a_per_count)
{
	memset(channel, 0, sizeof(*channel));
	channel->acquisition_index = acquisition_index;
	channel->role = role;
	channel->offset_adc = design_offset;
	channel->minimum_valid_offset_adc = 0U;
	channel->maximum_valid_offset_adc = 4095U;
	channel->current_a_per_count = current_a_per_count;
}

static MeasurementModelConfig MeasurementRuntimeTest_ThreeShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_BaseConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT;
	config.current_sense.physical_channel_count = 3U;
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[0], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B, 2000U, 0.02f);
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[1], 0U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C, 2000U, -0.01f);
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[2], 1U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A, 2000U, 0.01f);
	return config;
}

static MeasurementModelConfig MeasurementRuntimeTest_TwoShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_BaseConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT;
	config.current_sense.physical_channel_count = 2U;
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[0], 1U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C, 2000U, 0.01f);
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[1], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A, 2000U, 0.01f);
	return config;
}

static MeasurementModelConfig MeasurementRuntimeTest_OneShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_BaseConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT;
	config.current_sense.physical_channel_count = 1U;
	MeasurementRuntimeTest_SetChannel(&config.current_sense.channels[0], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK, 2000U, 0.01f);
	return config;
}

static MotorControlContext MeasurementRuntimeTest_Motor(uint16_t phase_a,
	uint16_t phase_b, uint16_t phase_c)
{
	MotorControlContext motor;

	memset(&motor, 0, sizeof(motor));
	motor.configuration.phase_a_current_offset_adc = phase_a;
	motor.configuration.phase_b_current_offset_adc = phase_b;
	motor.configuration.phase_c_current_offset_adc = phase_c;
	return motor;
}

static int MeasurementRuntimeTest_ThreeShuntProcess(void)
{
	MeasurementModelContext model;
	MeasurementModelConfig config = MeasurementRuntimeTest_ThreeShuntConfig();
	MotorControlContext motor = MeasurementRuntimeTest_Motor(100U, 100U, 100U);
	CurrentControlContext current;
	MotorStateContext *motor_state = (MotorStateContext *)(uintptr_t)1U;

	memset(&current, 0, sizeof(current));
	MeasurementModel_Reset(&model);
	TEST_CHECK(Measurement_Configure(&model, &motor, &config));
	TEST_CHECK(model.config.current_sense.channels[0].offset_adc == 100U);
	TEST_CHECK(model.config.current_sense.channels[1].offset_adc == 100U);
	TEST_CHECK(model.config.current_sense.channels[2].offset_adc == 100U);
	current.measurement_acquisition.sample.current_raw[0] = 300U;
	current.measurement_acquisition.sample.current_raw[1] = 400U;
	current.measurement_acquisition.sample.current_raw[2] = 50U;
	current.measurement_acquisition.sample.current_sample_count = 3U;
	current.measurement_acquisition.sample.valid_phase_currents =
		BSP_MOTOR_PHASE_ALL;
	current.measurement_acquisition.sample.bus_voltage_raw = 1000U;
	current.measurement_acquisition.sample.sequence = 10U;
	current.measurement_acquisition.sampling.mode =
		BSP_CURRENT_SAMPLING_MODE_FIXED;
	MeasurementRuntimeTest_ResetFault();
	TEST_CHECK(Measurement_Process(&model, &current, false, motor_state));
	TEST_CHECK(MeasurementRuntimeTest_FaultCount == 0U);
	TEST_CHECK(current.phase_current_status == PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_a_current_a, 3.0f));
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_b_current_a, -1.0f));
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_c_current_a, -2.0f));
	TEST_CHECK(current.bus_voltage_v == 10.0f);

	/* Duplicate sample is rejected, currents are zeroed before fault capture. */
	TEST_CHECK(!Measurement_Process(&model, &current, false, motor_state));
	TEST_CHECK(current.phase_current_status ==
		PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE);
	TEST_CHECK(current.phase_a_current_a == 0.0f);
	TEST_CHECK(current.phase_b_current_a == 0.0f);
	TEST_CHECK(current.phase_c_current_a == 0.0f);
	TEST_CHECK(MeasurementRuntimeTest_LastFault == MOTOR_FAULT_POWER_STAGE);
	return 0;
}

static int MeasurementRuntimeTest_StoredOffsetRoleMapping(void)
{
	MeasurementModelContext model;
	MeasurementModelConfig config = MeasurementRuntimeTest_TwoShuntConfig();
	MotorControlContext motor = MeasurementRuntimeTest_Motor(2100U, 0U, 2200U);

	MeasurementModel_Reset(&model);
	TEST_CHECK(Measurement_Configure(&model, &motor, &config));
	TEST_CHECK(model.config.current_sense.channels[0].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C);
	TEST_CHECK(model.config.current_sense.channels[0].offset_adc == 2200U);
	TEST_CHECK(model.config.current_sense.channels[1].role ==
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A);
	TEST_CHECK(model.config.current_sense.channels[1].offset_adc == 2100U);

	config = MeasurementRuntimeTest_OneShuntConfig();
	motor = MeasurementRuntimeTest_Motor(2048U, 0U, 0U);
	TEST_CHECK(Measurement_Configure(&model, &motor, &config));
	TEST_CHECK(model.config.current_sense.channels[0].offset_adc == 2048U);
	return 0;
}

static int MeasurementRuntimeTest_OneShuntWindows(void)
{
	MeasurementModelContext model;
	MeasurementModelConfig config = MeasurementRuntimeTest_OneShuntConfig();
	MotorControlContext motor = MeasurementRuntimeTest_Motor(2048U, 0U, 0U);
	CurrentControlContext current;
	MotorStateContext *motor_state = (MotorStateContext *)(uintptr_t)1U;

	memset(&current, 0, sizeof(current));
	MeasurementModel_Reset(&model);
	TEST_CHECK(Measurement_Configure(&model, &motor, &config));
	current.measurement_acquisition.sample.current_raw[0] = 2348U;
	current.measurement_acquisition.sample.current_raw[1] = 2248U;
	current.measurement_acquisition.sample.current_sample_count = 2U;
	current.measurement_acquisition.sample.valid_phase_currents = 0U;
	current.measurement_acquisition.sample.bus_voltage_raw = 1000U;
	current.measurement_acquisition.sample.sequence = 1U;
	current.measurement_acquisition.sampling.mode =
		BSP_CURRENT_SAMPLING_MODE_DYNAMIC;
	current.measurement_acquisition.sampling.modulation_sector = 1U;
	current.measurement_acquisition.sampling.sampling_point_count = 2U;
	current.measurement_acquisition.sampling.points[0].window =
		BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST;
	current.measurement_acquisition.sampling.points[1].window =
		BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_SECOND;
	MeasurementRuntimeTest_ResetFault();
	TEST_CHECK(Measurement_Process(&model, &current, false, motor_state));
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_a_current_a, 3.0f));
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_b_current_a, -1.0f));
	TEST_CHECK(MeasurementRuntimeTest_Near(current.phase_c_current_a, -2.0f));

	current.measurement_acquisition.sample.sequence = 2U;
	current.measurement_acquisition.sampling.points[1].window =
		BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST;
	TEST_CHECK(!Measurement_Process(&model, &current, false, motor_state));
	TEST_CHECK(current.phase_current_status == PHASE_CURRENT_STATUS_INVALID_WINDOW);
	TEST_CHECK(MeasurementRuntimeTest_LastFault == MOTOR_FAULT_POWER_STAGE);
	return 0;
}

static int MeasurementRuntimeTest_TwoShuntOffsetCalibration(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_TwoShuntConfig();
	CurrentOffsetCalibrationContext calibration;
	CurrentControlContext current;
	MotorStateContext *motor_state = (MotorStateContext *)(uintptr_t)1U;
	uint16_t slot_a;
	uint16_t slot_b;
	uint16_t slot_c;

	memset(&current, 0, sizeof(current));
	CurrentOffsetCalibrationRuntime_Reset(&calibration);
	current.measurement_acquisition.sample.current_sample_count = 2U;
	current.measurement_acquisition.sample.valid_phase_currents =
		BSP_MOTOR_PHASE_A | BSP_MOTOR_PHASE_C;
	current.measurement_acquisition.sample.current_raw[1] = 100U;
	current.measurement_acquisition.sample.current_raw[2] = 200U;
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ExecuteStep(&calibration,
		&current, &config.current_sense, 2U, motor_state) ==
		CURRENT_OFFSET_CALIBRATION_RUNNING);
	current.measurement_acquisition.sample.current_raw[1] = 102U;
	current.measurement_acquisition.sample.current_raw[2] = 204U;
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ExecuteStep(&calibration,
		&current, &config.current_sense, 2U, motor_state) ==
		CURRENT_OFFSET_CALIBRATION_COMPLETE);
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ReadResult(&calibration,
		&slot_a, &slot_b, &slot_c));
	TEST_CHECK(slot_a == 202U);
	TEST_CHECK(slot_b == 0U);
	TEST_CHECK(slot_c == 101U);
	return 0;
}

static int MeasurementRuntimeTest_OneShuntOffsetCalibration(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_OneShuntConfig();
	CurrentOffsetCalibrationContext calibration;
	CurrentControlContext current;
	MotorStateContext *motor_state = (MotorStateContext *)(uintptr_t)1U;
	uint16_t slot_a;
	uint16_t slot_b;
	uint16_t slot_c;

	memset(&current, 0, sizeof(current));
	CurrentOffsetCalibrationRuntime_Reset(&calibration);
	current.measurement_acquisition.sample.current_sample_count = 2U;
	current.measurement_acquisition.sample.valid_phase_currents = 0U;
	current.measurement_acquisition.sampling.modulation_sector = 1U;
	current.measurement_acquisition.sampling.sampling_point_count = 2U;
	current.measurement_acquisition.sampling.points[0].window =
		BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST;
	current.measurement_acquisition.sampling.points[1].window =
		BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_SECOND;
	current.measurement_acquisition.sample.current_raw[0] = 100U;
	current.measurement_acquisition.sample.current_raw[1] = 102U;
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ExecuteStep(&calibration,
		&current, &config.current_sense, 2U, motor_state) ==
		CURRENT_OFFSET_CALIBRATION_RUNNING);
	current.measurement_acquisition.sample.current_raw[0] = 104U;
	current.measurement_acquisition.sample.current_raw[1] = 106U;
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ExecuteStep(&calibration,
		&current, &config.current_sense, 2U, motor_state) ==
		CURRENT_OFFSET_CALIBRATION_COMPLETE);
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ReadResult(&calibration,
		&slot_a, &slot_b, &slot_c));
	TEST_CHECK(slot_a == 103U);
	TEST_CHECK(slot_b == 0U);
	TEST_CHECK(slot_c == 0U);
	return 0;
}

static int MeasurementRuntimeTest_InvalidCalibrationFailsClosed(void)
{
	MeasurementModelConfig config = MeasurementRuntimeTest_TwoShuntConfig();
	CurrentOffsetCalibrationContext calibration;
	CurrentControlContext current;
	MotorStateContext *motor_state = (MotorStateContext *)(uintptr_t)1U;

	memset(&current, 0, sizeof(current));
	CurrentOffsetCalibrationRuntime_Reset(&calibration);
	current.measurement_acquisition.sample.current_sample_count = 2U;
	current.measurement_acquisition.sample.valid_phase_currents =
		BSP_MOTOR_PHASE_A;
	MeasurementRuntimeTest_ResetFault();
	TEST_CHECK(CurrentOffsetCalibrationRuntime_ExecuteStep(&calibration,
		&current, &config.current_sense, 2U, motor_state) ==
		CURRENT_OFFSET_CALIBRATION_RUNNING);
	TEST_CHECK(MeasurementRuntimeTest_LastFault == MOTOR_FAULT_POWER_STAGE);
	TEST_CHECK(calibration.frame_count == 0U);
	TEST_CHECK(calibration.offset_sum[0] == 0U);
	TEST_CHECK(calibration.offset_sum[1] == 0U);
	TEST_CHECK(calibration.offset_sum[2] == 0U);
	return 0;
}

int MeasurementRuntime_RunHostTests(void)
{
	int result;

	result = MeasurementRuntimeTest_ThreeShuntProcess();
	if (result != 0)
		return result;
	result = MeasurementRuntimeTest_StoredOffsetRoleMapping();
	if (result != 0)
		return result;
	result = MeasurementRuntimeTest_OneShuntWindows();
	if (result != 0)
		return result;
	result = MeasurementRuntimeTest_TwoShuntOffsetCalibration();
	if (result != 0)
		return result;
	result = MeasurementRuntimeTest_OneShuntOffsetCalibration();
	if (result != 0)
		return result;
	return MeasurementRuntimeTest_InvalidCalibrationFailsClosed();
}
