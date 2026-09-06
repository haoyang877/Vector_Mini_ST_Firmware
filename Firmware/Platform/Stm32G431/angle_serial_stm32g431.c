#include "angle_serial_stm32g431.h"

#include "main.h"
#include "spi.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#ifndef ANGLE_SERIAL_SPI_XFER_SPIN_MAX
#define ANGLE_SERIAL_SPI_XFER_SPIN_MAX 340U
#endif

static bool AngleSerial_WaitFlag(SPI_TypeDef *spi, uint32_t flag,
	bool expected_set)
{
	uint32_t spin = 0U;

	while ((((spi->SR & flag) != 0U) ? true : false) != expected_set)
	{
		if (++spin > ANGLE_SERIAL_SPI_XFER_SPIN_MAX)
			return false;
	}
	return true;
}

static BspResult AngleSerial_Initialize(void *context)
{
	SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)context;

	if (spi == 0)
		return BSP_RESULT_INVALID_ARGUMENT;
	spi->Init.DataSize = SPI_DATASIZE_16BIT;
	spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spi->Init.CLKPhase = SPI_PHASE_2EDGE;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	return HAL_SPI_Init(spi) == HAL_OK ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

static void AngleSerial_SetSelect(bool selected)
{
	BRD_ENC_CS_GPIO_Port->BSRR = selected ?
		((uint32_t)BRD_ENC_CS_Pin << 16U) : (uint32_t)BRD_ENC_CS_Pin;
}

static void AngleSerial_SetDataOutputEnabled(bool enabled)
{
	if (!enabled)
	{
		GPIOB->MODER &= ~(0x3UL << 30U);
		return;
	}

	GPIOB->MODER = (GPIOB->MODER & ~(0x3UL << 30U)) | (0x2UL << 30U);
	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFUL << 28U)) | (0x5UL << 28U);
}

static bool AngleSerial_TransferWord(SPI_TypeDef *spi, uint16_t transmit,
	uint16_t *receive)
{
	if ((spi->CR1 & SPI_CR1_SPE) == 0U)
		spi->CR1 |= SPI_CR1_SPE;
	if ((spi->SR & SPI_FLAG_OVR) != 0U)
	{
		(void)*(__IO uint16_t *)&spi->DR;
		(void)spi->SR;
	}
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_TXE, true))
		return false;
	*(__IO uint16_t *)&spi->DR = transmit;
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_RXNE, true))
		return false;
	*receive = *(__IO uint16_t *)&spi->DR;
	return AngleSerial_WaitFlag(spi, SPI_FLAG_BSY, false);
}

static BspResult AngleSerial_Execute(void *context,
	const BspSynchronousSerialTransaction *transaction)
{
	SPI_HandleTypeDef *handle = (SPI_HandleTypeDef *)context;
	SPI_TypeDef *spi;
	size_t index;
	bool data_output_enabled = true;
	bool result = true;

	if (handle == 0 || transaction == 0 ||
		transaction->steps == 0 || transaction->received_words == 0 ||
		transaction->step_count == 0U)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	spi = handle->Instance;
	AngleSerial_SetSelect(true);
	for (index = 0U; index < transaction->step_count; ++index)
	{
		const BspSynchronousSerialStep *step = &transaction->steps[index];

		if (step->controller_data_output_enabled != data_output_enabled)
		{
			AngleSerial_SetDataOutputEnabled(
				step->controller_data_output_enabled);
			data_output_enabled = step->controller_data_output_enabled;
		}
		if (step->turnaround_delay_before)
		{
			__NOP();
			__NOP();
		}
		if (!AngleSerial_TransferWord(spi, step->transmit_word,
			&transaction->received_words[index]))
		{
			result = false;
			break;
		}
	}

	if (!data_output_enabled)
		AngleSerial_SetDataOutputEnabled(true);
	if (!AngleSerial_WaitFlag(spi, SPI_FLAG_BSY, false))
		result = false;
	AngleSerial_SetSelect(false);
	return result ? BSP_RESULT_OK : BSP_RESULT_IO_ERROR;
}

BspSynchronousSerialPort AngleSerialStm32g431_CreatePort(void)
{
	BspSynchronousSerialPort port;

	port.context = &hspi2;
	port.initialize = AngleSerial_Initialize;
	port.execute = AngleSerial_Execute;
	return port;
}
