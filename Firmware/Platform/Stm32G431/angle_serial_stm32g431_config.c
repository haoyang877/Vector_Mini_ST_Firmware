#include "angle_serial_stm32g431.h"

bool AngleSerialStm32g431_ResourceConfigIsValid(
	const AngleSerialStm32g431ResourceConfig *resources)
{
	uint16_t data_pin_mask;

	if (resources == 0 || resources->spi_handle == 0 ||
		resources->select_gpio_port == 0 || resources->data_gpio_port == 0 ||
		resources->select_pin_mask == 0U ||
		(resources->select_pin_mask &
			(uint16_t)(resources->select_pin_mask - 1U)) != 0U ||
		resources->data_pin_index >= ANGLE_SERIAL_STM32G431_GPIO_PIN_COUNT ||
		resources->data_alternate_function >=
			ANGLE_SERIAL_STM32G431_GPIO_AF_COUNT ||
		resources->transfer_spin_limit == 0U ||
		resources->maximum_step_count == 0U ||
		resources->maximum_step_count > ANGLE_SERIAL_STM32G431_MAX_STEP_COUNT)
	{
		return false;
	}

	data_pin_mask = (uint16_t)(UINT16_C(1) << resources->data_pin_index);
	return resources->select_gpio_port != resources->data_gpio_port ||
		resources->select_pin_mask != data_pin_mask;
}
