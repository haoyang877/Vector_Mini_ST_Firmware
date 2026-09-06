#include "angle_serial_stm32g431.h"

#include "stm32g4xx_hal.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static bool AngleSerial_WaitFlag(SPI_TypeDef *spi, uint32_t flag,
	bool expected_set, uint32_t spin_limit)
{
	uint32_t spin = 0U;

	while ((((spi->SR & flag) != 0U) ? true : false) != expected_set)
	{
		if (++spin > spin_limit)
			return false;
	}
	return true;
}

static void AngleSerial_SetSelect(
	const AngleSerialStm32g431ResourceConfig *resources, bool selected)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)resources->select_gpio_port;
	const bool drive_high = selected ? !resources->select_active_low :
		resources->select_active_low;

	gpio->BSRR = drive_high ? (uint32_t)resources->select_pin_mask :
		((uint32_t)resources->select_pin_mask << 16U);
}

static void AngleSerial_SetDataOutputEnabled(
	const AngleSerialStm32g431ResourceConfig *resources, bool enabled)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)resources->data_gpio_port;
	const uint32_t mode_shift = (uint32_t)resources->data_pin_index * 2U;
	const uint32_t mode_mask = UINT32_C(3) << mode_shift;
	const uint32_t alternate_index =
		(uint32_t)resources->data_pin_index >> 3U;
	const uint32_t alternate_shift =
		((uint32_t)resources->data_pin_index & UINT32_C(7)) * 4U;
	const uint32_t alternate_mask = UINT32_C(15) << alternate_shift;

	if (!enabled)
	{
		gpio->MODER &= ~mode_mask;
		return;
	}

	gpio->MODER = (gpio->MODER & ~mode_mask) |
		(UINT32_C(2) << mode_shift);
	gpio->AFR[alternate_index] =
		(gpio->AFR[alternate_index] & ~alternate_mask) |
		((uint32_t)resources->data_alternate_function << alternate_shift);
}

static BspResult AngleSerial_Initialize(void *context)
{
	AngleSerialStm32g431Context *adapter =
		(AngleSerialStm32g431Context *)context;
	SPI_HandleTypeDef *spi;

	if (adapter == 0 || adapter->resources == 0 ||
		adapter->resources->spi_handle == 0)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	spi = (SPI_HandleTypeDef *)adapter->resources->spi_handle;
	if (spi->Instance == 0)
		return BSP_RESULT_INVALID_ARGUMENT;

	spi->Init.DataSize = SPI_DATASIZE_16BIT;
	spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spi->Init.CLKPhase = SPI_PHASE_2EDGE;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	return HAL_SPI_Init(spi) == HAL_OK ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

static bool AngleSerial_TransferWord(SPI_TypeDef *spi, uint16_t transmit,
	uint16_t *receive, uint32_t spin_limit)
{
	if ((spi->CR1 & SPI_CR1_SPE) == 0U)
		spi->CR1 |= SPI_CR1_SPE;
	if ((spi->SR & SPI_FLAG_OVR) != 0U)
	{
		(void)*(__IO uint16_t *)&spi->DR;
		(void)spi->SR;
	}
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_TXE, true, spin_limit))
		return false;
	*(__IO uint16_t *)&spi->DR = transmit;
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_RXNE, true, spin_limit))
		return false;
	*receive = *(__IO uint16_t *)&spi->DR;
	return AngleSerial_WaitFlag(spi, SPI_FLAG_BSY, false, spin_limit);
}

static BspResult AngleSerial_Execute(void *context,
	const BspSynchronousSerialTransaction *transaction)
{
	AngleSerialStm32g431Context *adapter =
		(AngleSerialStm32g431Context *)context;
	const AngleSerialStm32g431ResourceConfig *resources;
	SPI_HandleTypeDef *handle;
	SPI_TypeDef *spi;
	size_t index;
	bool data_output_enabled = true;
	bool result = true;

	if (adapter == 0 || adapter->resources == 0 || transaction == 0 ||
		transaction->steps == 0 || transaction->received_words == 0 ||
		transaction->step_count == 0U ||
		transaction->step_count > adapter->resources->maximum_step_count)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	resources = adapter->resources;
	handle = (SPI_HandleTypeDef *)resources->spi_handle;
	if (handle == 0 || handle->Instance == 0)
		return BSP_RESULT_INVALID_ARGUMENT;
	spi = handle->Instance;

	AngleSerial_SetSelect(resources, true);
	for (index = 0U; index < transaction->step_count; ++index)
	{
		const BspSynchronousSerialStep *step = &transaction->steps[index];

		if (step->controller_data_output_enabled != data_output_enabled)
		{
			AngleSerial_SetDataOutputEnabled(resources,
				step->controller_data_output_enabled);
			data_output_enabled = step->controller_data_output_enabled;
		}
		if (step->turnaround_delay_before)
		{
			__NOP();
			__NOP();
		}
		if (!AngleSerial_TransferWord(spi, step->transmit_word,
			&transaction->received_words[index],
			resources->transfer_spin_limit))
		{
			result = false;
			break;
		}
	}

	if (!data_output_enabled)
		AngleSerial_SetDataOutputEnabled(resources, true);
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_BSY, false,
		resources->transfer_spin_limit))
	{
		result = false;
	}
	AngleSerial_SetSelect(resources, false);
	return result ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

bool AngleSerialStm32g431_CreatePort(AngleSerialStm32g431Context *context,
	const AngleSerialStm32g431ResourceConfig *resources,
	BspSynchronousSerialPort *port)
{
	if (port == 0)
		return false;
	port->context = 0;
	port->initialize = 0;
	port->execute = 0;
	if (context == 0)
		return false;
	context->resources = 0;
	if (!AngleSerialStm32g431_ResourceConfigIsValid(resources))
		return false;

	context->resources = resources;
	port->context = context;
	port->initialize = AngleSerial_Initialize;
	port->execute = AngleSerial_Execute;
	return true;
}
