#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tle5012b.h"
#include "tle5012b_angle_sensor_adapter.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

typedef struct
{
	BspResult initialize_result;
	BspResult execute_result;
	uint32_t initialize_count;
	uint32_t execute_count;
	bool invalid_transaction;
	uint16_t angle_word;
	BspSynchronousSerialStep captured_steps[4];
	size_t captured_step_count;
} MockTransaction;

static BspResult Mock_Initialize(void *context)
{
	MockTransaction *mock = (MockTransaction *)context;

	++mock->initialize_count;
	return mock->initialize_result;
}

static BspResult Mock_Execute(void *context,
	const BspSynchronousSerialTransaction *transaction)
{
	MockTransaction *mock = (MockTransaction *)context;
	size_t index;

	++mock->execute_count;
	if (transaction == 0 || transaction->steps == 0 ||
		transaction->received_words == 0 || transaction->step_count != 2U)
	{
		mock->invalid_transaction = true;
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	mock->captured_step_count = transaction->step_count;
	for (index = 0U; index < transaction->step_count; ++index)
		mock->captured_steps[index] = transaction->steps[index];
	transaction->received_words[0] = 0xAAAAU;
	transaction->received_words[1] = mock->angle_word;
	return mock->execute_result;
}

static BspSynchronousSerialPort Mock_CreatePort(
	MockTransaction *mock)
{
	BspSynchronousSerialPort port;

	port.context = mock;
	port.initialize = Mock_Initialize;
	port.execute = Mock_Execute;
	return port;
}

static void Mock_Reset(MockTransaction *mock)
{
	(void)memset(mock, 0, sizeof(*mock));
	mock->initialize_result = BSP_RESULT_OK;
	mock->execute_result = BSP_RESULT_OK;
	mock->angle_word = 0x9234U;
}

static int Test_InitializationContract(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort port;
	Tle5012b device;

	Mock_Reset(&mock);
	port = Mock_CreatePort(&mock);
	TEST_CHECK(Tle5012b_Initialize(&device, &port));
	TEST_CHECK(device.initialized);
	TEST_CHECK(mock.initialize_count == 1U);

	port.execute = 0;
	TEST_CHECK(!Tle5012b_Initialize(&device, &port));
	TEST_CHECK(!device.initialized);
	TEST_CHECK(!Tle5012b_Initialize(0, &port));

	port = Mock_CreatePort(&mock);
	mock.initialize_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(!Tle5012b_Initialize(&device, &port));
	TEST_CHECK(!device.initialized);
	return 0;
}

static int Test_ReadTransactionAndConversion(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort port;
	Tle5012b device;
	Tle5012bSample sample;

	Mock_Reset(&mock);
	port = Mock_CreatePort(&mock);
	TEST_CHECK(Tle5012b_Initialize(&device, &port));
	TEST_CHECK(Tle5012b_ReadAngle(&device, &sample) == TLE5012B_READ_OK);
	TEST_CHECK(mock.execute_count == 1U);
	TEST_CHECK(!mock.invalid_transaction);
	TEST_CHECK(mock.captured_step_count == 2U);
	TEST_CHECK(mock.captured_steps[0].transmit_word == 0x8021U);
	TEST_CHECK(mock.captured_steps[0].controller_data_output_enabled);
	TEST_CHECK(!mock.captured_steps[0].turnaround_delay_before);
	TEST_CHECK(mock.captured_steps[1].transmit_word == 0U);
	TEST_CHECK(!mock.captured_steps[1].controller_data_output_enabled);
	TEST_CHECK(mock.captured_steps[1].turnaround_delay_before);
	TEST_CHECK(sample.angle_word == 0x9234U);
	TEST_CHECK(sample.raw_angle_q15 == 0x2468U);
	return 0;
}

static int Test_TransactionFailureIsPropagated(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort port;
	Tle5012b device;
	Tle5012bSample sample;

	Mock_Reset(&mock);
	port = Mock_CreatePort(&mock);
	TEST_CHECK(Tle5012b_Initialize(&device, &port));
	mock.execute_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(Tle5012b_ReadAngle(&device, &sample) ==
		TLE5012B_READ_TRANSACTION_ERROR);
	TEST_CHECK(mock.execute_count == 1U);
	return 0;
}

static int Test_InvalidReadArguments(void)
{
	Tle5012b device;
	Tle5012bSample sample;

	(void)memset(&device, 0, sizeof(device));
	TEST_CHECK(Tle5012b_ReadAngle(&device, &sample) ==
		TLE5012B_READ_NOT_INITIALIZED);
	TEST_CHECK(Tle5012b_ReadAngle(0, &sample) ==
		TLE5012B_READ_INVALID_ARGUMENT);
	TEST_CHECK(Tle5012b_ReadAngle(&device, 0) ==
		TLE5012B_READ_INVALID_ARGUMENT);
	return 0;
}

static int Test_BspAngleAdapterLifecycleAndAngleMapping(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort transport;
	Tle5012bAngleSensorAdapterContext adapter;
	BspAngleSensorPort port;
	BspAngleSensorSample sample;

	Mock_Reset(&mock);
	transport = Mock_CreatePort(&mock);
	TEST_CHECK(Tle5012bAngleSensorAdapter_CreatePort(&adapter, &transport, &port));
	TEST_CHECK(port.context == &adapter);
	TEST_CHECK(port.capabilities != 0);
	TEST_CHECK(port.capabilities->resolution_bits == 15U);
	TEST_CHECK(port.capabilities->is_absolute);
	TEST_CHECK(!port.capabilities->supports_multiturn);
	TEST_CHECK(!port.capabilities->provides_velocity);
	TEST_CHECK(port.initialize != 0 && port.start_acquisition != 0 &&
		port.stop_acquisition != 0 && port.request_sample != 0 &&
		port.try_read_latest != 0 && port.read_status != 0);
	TEST_CHECK(port.start_acquisition(port.context) == BSP_RESULT_NOT_READY);
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_NOT_READY);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	TEST_CHECK(port.initialize(port.context) == BSP_RESULT_OK);
	TEST_CHECK(mock.initialize_count == 1U);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	TEST_CHECK(port.start_acquisition(port.context) == BSP_RESULT_OK);
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_OK);
	TEST_CHECK(mock.execute_count == 1U);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	/* Old raw_angle_q15 0x2468 maps exactly to the generic full-turn U32. */
	TEST_CHECK(sample.single_turn_position_u32 == UINT32_C(0x24680000));
	TEST_CHECK(sample.turn_count == 0 && sample.velocity_rad_s == 0.0f);
	TEST_CHECK(sample.timestamp_us == 0U && sample.sequence == 1U);
	TEST_CHECK(sample.status == BSP_ANGLE_SAMPLE_POSITION_VALID);
	TEST_CHECK(port.read_status(port.context) == BSP_ANGLE_SAMPLE_POSITION_VALID);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	TEST_CHECK(sample.sequence == 1U);
	mock.angle_word = 0xFFFFU;
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_OK);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	TEST_CHECK(sample.single_turn_position_u32 == UINT32_C(0xFFFE0000));
	TEST_CHECK(sample.sequence == 2U);
	TEST_CHECK(port.stop_acquisition(port.context) == BSP_RESULT_OK);
	TEST_CHECK(port.read_status(port.context) == 0U);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	return 0;
}

static int Test_BspAngleAdapterFaultMappingAndRecovery(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort transport;
	Tle5012bAngleSensorAdapterContext adapter;
	BspAngleSensorPort port;
	BspAngleSensorSample sample;

	Mock_Reset(&mock);
	transport = Mock_CreatePort(&mock);
	TEST_CHECK(Tle5012bAngleSensorAdapter_CreatePort(&adapter, &transport, &port));
	mock.initialize_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(port.initialize(port.context) == BSP_RESULT_IO_ERROR);
	TEST_CHECK(port.read_status(port.context) == BSP_ANGLE_SAMPLE_SENSOR_FAULT);
	mock.initialize_result = BSP_RESULT_OK;
	TEST_CHECK(port.initialize(port.context) == BSP_RESULT_OK);
	TEST_CHECK(port.start_acquisition(port.context) == BSP_RESULT_OK);
	mock.execute_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_IO_ERROR);
	TEST_CHECK(port.read_status(port.context) == BSP_ANGLE_SAMPLE_SENSOR_FAULT);
	TEST_CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	mock.execute_result = BSP_RESULT_OK;
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_OK);
	TEST_CHECK(port.read_status(port.context) == BSP_ANGLE_SAMPLE_POSITION_VALID);
	adapter.device.initialized = false;
	TEST_CHECK(port.request_sample(port.context) == BSP_RESULT_NOT_READY);
	TEST_CHECK(port.read_status(port.context) == BSP_ANGLE_SAMPLE_SENSOR_FAULT);
	TEST_CHECK(port.try_read_latest(port.context, 0) == BSP_RESULT_INVALID_ARGUMENT);
	TEST_CHECK(port.try_read_latest(0, &sample) == BSP_RESULT_INVALID_ARGUMENT);
	TEST_CHECK(port.read_status(0) == BSP_ANGLE_SAMPLE_SENSOR_FAULT);
	return 0;
}

static int Test_BspAngleAdapterSupportsIndependentInstances(void)
{
	MockTransaction first_mock;
	MockTransaction second_mock;
	BspSynchronousSerialPort first_transport;
	BspSynchronousSerialPort second_transport;
	Tle5012bAngleSensorAdapterContext first_adapter;
	Tle5012bAngleSensorAdapterContext second_adapter;
	BspAngleSensorPort first_port;
	BspAngleSensorPort second_port;
	BspAngleSensorSample first_sample;
	BspAngleSensorSample second_sample;

	Mock_Reset(&first_mock);
	Mock_Reset(&second_mock);
	first_mock.angle_word = 0x8001U;
	second_mock.angle_word = 0x8123U;
	first_transport = Mock_CreatePort(&first_mock);
	second_transport = Mock_CreatePort(&second_mock);
	TEST_CHECK(Tle5012bAngleSensorAdapter_CreatePort(&first_adapter,
		&first_transport, &first_port));
	TEST_CHECK(Tle5012bAngleSensorAdapter_CreatePort(&second_adapter,
		&second_transport, &second_port));
	TEST_CHECK(first_port.capabilities == second_port.capabilities);
	TEST_CHECK(first_port.initialize(first_port.context) == BSP_RESULT_OK);
	TEST_CHECK(second_port.initialize(second_port.context) == BSP_RESULT_OK);
	TEST_CHECK(first_port.start_acquisition(first_port.context) == BSP_RESULT_OK);
	TEST_CHECK(second_port.start_acquisition(second_port.context) == BSP_RESULT_OK);
	TEST_CHECK(first_port.request_sample(first_port.context) == BSP_RESULT_OK);
	TEST_CHECK(second_port.request_sample(second_port.context) == BSP_RESULT_OK);
	TEST_CHECK(first_port.try_read_latest(first_port.context, &first_sample) ==
		BSP_RESULT_OK);
	TEST_CHECK(second_port.try_read_latest(second_port.context, &second_sample) ==
		BSP_RESULT_OK);
	TEST_CHECK(first_sample.single_turn_position_u32 == UINT32_C(0x00020000));
	TEST_CHECK(second_sample.single_turn_position_u32 == UINT32_C(0x02460000));
	TEST_CHECK(first_mock.execute_count == 1U && second_mock.execute_count == 1U);
	TEST_CHECK(first_sample.sequence == 1U && second_sample.sequence == 1U);
	return 0;
}

static int Test_BspAngleAdapterRejectsInvalidCreation(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort transport;
	Tle5012bAngleSensorAdapterContext adapter;
	BspAngleSensorPort port;

	Mock_Reset(&mock);
	transport = Mock_CreatePort(&mock);
	TEST_CHECK(!Tle5012bAngleSensorAdapter_CreatePort(0, &transport, &port));
	TEST_CHECK(port.context == 0 && port.initialize == 0);
	TEST_CHECK(!Tle5012bAngleSensorAdapter_CreatePort(&adapter, 0, &port));
	TEST_CHECK(port.context == 0 && port.initialize == 0);
	transport.execute = 0;
	TEST_CHECK(!Tle5012bAngleSensorAdapter_CreatePort(&adapter, &transport, &port));
	TEST_CHECK(port.context == 0 && port.initialize == 0);
	TEST_CHECK(!Tle5012bAngleSensorAdapter_CreatePort(&adapter, &transport, 0));
	return 0;
}

int Tle5012bDriver_RunHostTests(void)
{
	int result;

	result = Test_InitializationContract();
	if (result != 0)
		return result;
	result = Test_ReadTransactionAndConversion();
	if (result != 0)
		return result;
	result = Test_TransactionFailureIsPropagated();
	if (result != 0)
		return result;
	result = Test_InvalidReadArguments();
	if (result != 0)
		return result;
	result = Test_BspAngleAdapterLifecycleAndAngleMapping();
	if (result != 0)
		return result;
	result = Test_BspAngleAdapterFaultMappingAndRecovery();
	if (result != 0)
		return result;
	result = Test_BspAngleAdapterSupportsIndependentInstances();
	if (result != 0)
		return result;
	return Test_BspAngleAdapterRejectsInvalidCreation();
}

#ifdef TLE5012B_DRIVER_TEST_MAIN
int main(void)
{
	return Tle5012bDriver_RunHostTests();
}
#endif
