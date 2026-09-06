#ifndef DRIVERS_ANGLE_TLE5012B_TLE5012B_H
#define DRIVERS_ANGLE_TLE5012B_TLE5012B_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_synchronous_serial.h"

typedef enum
{
	TLE5012B_READ_OK = 0,
	TLE5012B_READ_INVALID_ARGUMENT,
	TLE5012B_READ_NOT_INITIALIZED,
	TLE5012B_READ_TRANSACTION_ERROR
} Tle5012bReadStatus;

typedef struct
{
	uint16_t angle_word;
	uint16_t raw_angle_q15;
} Tle5012bSample;

typedef struct
{
	BspSynchronousSerialPort transaction;
	bool initialized;
} Tle5012b;

#if defined(__CC_ARM)
#define TLE5012B_INLINE static __forceinline
#else
#define TLE5012B_INLINE static inline
#endif

TLE5012B_INLINE bool Tle5012b_Initialize(Tle5012b *device,
	const BspSynchronousSerialPort *transaction)
{
	if (device == 0)
		return false;
	device->initialized = false;
	if (transaction == 0 || transaction->initialize == 0 ||
		transaction->execute == 0)
	{
		return false;
	}

	device->transaction = *transaction;
	if (device->transaction.initialize(device->transaction.context) !=
		BSP_RESULT_OK)
	{
		return false;
	}

	device->initialized = true;
	return true;
}

TLE5012B_INLINE Tle5012bReadStatus Tle5012b_ReadAngle(Tle5012b *device,
	Tle5012bSample *sample)
{
	static const BspSynchronousSerialStep steps[] =
	{
		{ 0x8021U, true, false },
		{ 0U, false, true }
	};
	uint16_t received_words[sizeof(steps) / sizeof(steps[0])];
	BspSynchronousSerialTransaction transaction;

	if (device == 0 || sample == 0)
		return TLE5012B_READ_INVALID_ARGUMENT;
	if (!device->initialized)
		return TLE5012B_READ_NOT_INITIALIZED;

	transaction.steps = steps;
	transaction.received_words = received_words;
	transaction.step_count = sizeof(steps) / sizeof(steps[0]);
	if (device->transaction.execute(device->transaction.context,
		&transaction) != BSP_RESULT_OK)
	{
		return TLE5012B_READ_TRANSACTION_ERROR;
	}

	sample->angle_word = received_words[1];
	sample->raw_angle_q15 =
		(uint16_t)((received_words[1] & 0x7FFFU) << 1U);
	return TLE5012B_READ_OK;
}

#undef TLE5012B_INLINE

#endif
