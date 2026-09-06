#ifndef FIRMWARE_BSP_API_BSP_SYNCHRONOUS_SERIAL_H
#define FIRMWARE_BSP_API_BSP_SYNCHRONOUS_SERIAL_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * All words in one transaction execute under a single select assertion.
 * Per-step direction metadata supports a bidirectional data wire without
 * exposing MCU pins, registers, or vendor SDK types to a device driver.
 */
typedef struct
{
	uint16_t transmit_word;
	bool controller_data_output_enabled;
	bool turnaround_delay_before;
} BspSynchronousSerialStep;

typedef struct
{
	const BspSynchronousSerialStep *steps;
	uint16_t *received_words;
	size_t step_count;
} BspSynchronousSerialTransaction;

/*
 * execute() is synchronous but must have a finite implementation-defined
 * timeout. It must not retain transaction pointers. Before returning,
 * including on failure, it restores controller data output, waits for the bus
 * to become idle, and deasserts select.
 */
typedef struct
{
	void *context;
	BspResult (*initialize)(void *context);
	BspResult (*execute)(void *context,
		const BspSynchronousSerialTransaction *transaction);
} BspSynchronousSerialPort;

#ifdef __cplusplus
}
#endif

#endif
