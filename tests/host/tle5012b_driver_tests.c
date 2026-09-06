#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tle5012b.h"
#include "tle5012b_rotor_sensor_adapter.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

typedef struct
{
	BspResult initialize_result;
	BspResult execute_result;
	uint32_t initialize_count;
	uint32_t execute_count;
	bool invalid_transaction;
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
	transaction->received_words[1] = 0x9234U;
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

static int Test_LegacyAdapterPreservesGenericSample(void)
{
	MockTransaction mock;
	BspSynchronousSerialPort transport;
	Tle5012bRotorSensorAdapterContext adapter;
	RotorSensorPort port;
	RotorSensorSample sample;

	Mock_Reset(&mock);
	transport = Mock_CreatePort(&mock);
	port = Tle5012bRotorSensorAdapter_CreatePort(&adapter, &transport);
	TEST_CHECK(port.context == &adapter);
	TEST_CHECK(port.initialize != 0);
	TEST_CHECK(port.read_sample != 0);
	TEST_CHECK(port.initialize(port.context));
	TEST_CHECK(port.read_sample(port.context, &sample) == ROTOR_SENSOR_READ_OK);
	TEST_CHECK(sample.raw_data_word == 0x9234U);
	TEST_CHECK(sample.raw_angle_q15 == 0x2468U);

	mock.execute_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(port.read_sample(port.context, &sample) ==
		ROTOR_SENSOR_READ_TRANSPORT_ERROR);
	port = Tle5012bRotorSensorAdapter_CreatePort(0, &transport);
	TEST_CHECK(port.initialize == 0);
	TEST_CHECK(port.read_sample == 0);
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
	return Test_LegacyAdapterPreservesGenericSample();
}

#ifdef TLE5012B_DRIVER_TEST_MAIN
int main(void)
{
	return Tle5012bDriver_RunHostTests();
}
#endif
