#ifndef PLATFORM_STM32G431_ANGLE_SERIAL_STM32G431_H
#define PLATFORM_STM32G431_ANGLE_SERIAL_STM32G431_H

#include "bsp_synchronous_serial.h"

#define ANGLE_SERIAL_STM32G431_GPIO_PIN_COUNT 16U
#define ANGLE_SERIAL_STM32G431_GPIO_AF_COUNT  16U
#define ANGLE_SERIAL_STM32G431_MAX_STEP_COUNT 8U

/*
 * The board owns this resource map. Opaque pointers keep vendor types out of
 * the public header while the STM32 implementation performs the checked casts.
 */
typedef struct
{
	void *spi_handle;
	void *select_gpio_port;
	void *data_gpio_port;
	uint16_t transfer_spin_limit;
	uint16_t select_pin_mask;
	uint8_t data_pin_index;
	uint8_t data_alternate_function;
	uint8_t maximum_step_count;
	bool select_active_low;
} AngleSerialStm32g431ResourceConfig;

/* The composition root owns one context for each configured serial endpoint. */
typedef struct
{
	const AngleSerialStm32g431ResourceConfig *resources;
} AngleSerialStm32g431Context;

bool AngleSerialStm32g431_ResourceConfigIsValid(
	const AngleSerialStm32g431ResourceConfig *resources);

/* On failure, port is cleared and context remains unbound. */
bool AngleSerialStm32g431_CreatePort(AngleSerialStm32g431Context *context,
	const AngleSerialStm32g431ResourceConfig *resources,
	BspSynchronousSerialPort *port);

#endif
