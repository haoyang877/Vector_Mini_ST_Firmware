#include "rotor_sensor_tle5012b.h"

#include "main.h"
#include "spi.h"

#ifndef TLE5012B_SPI_XFER_SPIN_MAX
#define TLE5012B_SPI_XFER_SPIN_MAX 340U
#endif

static bool RotorSensorTle5012b_WaitFlag(SPI_TypeDef *spi,
	uint32_t flag, bool expected_set)
{
	uint32_t spin = 0U;

	while ((((spi->SR & flag) != 0U) ? true : false) != expected_set)
	{
		if (++spin > TLE5012B_SPI_XFER_SPIN_MAX)
			return false;
	}
	return true;
}

static bool RotorSensorTle5012b_Transfer16(SPI_TypeDef *spi,
	uint16_t transmit, uint16_t *receive)
{
	if ((spi->CR1 & SPI_CR1_SPE) == 0U)
		spi->CR1 |= SPI_CR1_SPE;
	if ((spi->SR & SPI_FLAG_OVR) != 0U)
	{
		(void)*(__IO uint16_t *)&spi->DR;
		(void)spi->SR;
	}
	if (!RotorSensorTle5012b_WaitFlag(spi, SPI_FLAG_TXE, true))
		return false;
	*(__IO uint16_t *)&spi->DR = transmit;
	if (!RotorSensorTle5012b_WaitFlag(spi, SPI_FLAG_RXNE, true))
		return false;
	*receive = *(__IO uint16_t *)&spi->DR;
	return RotorSensorTle5012b_WaitFlag(spi, SPI_FLAG_BSY, false);
}

static void RotorSensorTle5012b_MosiHighImpedance(void)
{
	GPIOB->MODER &= ~(0x3UL << 30U);
}

static void RotorSensorTle5012b_MosiRestoreAlternateFunction(void)
{
	GPIOB->MODER = (GPIOB->MODER & ~(0x3UL << 30U)) | (0x2UL << 30U);
	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFUL << 28U)) | (0x5UL << 28U);
}

static bool RotorSensorTle5012b_Initialize(void *context)
{
	SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)context;

	spi->Init.DataSize = SPI_DATASIZE_16BIT;
	spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spi->Init.CLKPhase = SPI_PHASE_2EDGE;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	return HAL_SPI_Init(spi) == HAL_OK;
}

static bool RotorSensorTle5012b_ReadSample(void *context,
	RotorSensorRawSample *sample)
{
	SPI_HandleTypeDef *handle = (SPI_HandleTypeDef *)context;
	SPI_TypeDef *spi = handle->Instance;
	uint16_t discard;
	uint16_t angle_word = 0U;
	bool result;

	if (sample == 0)
		return false;

	BRD_ENC_CS_GPIO_Port->BSRR = (uint32_t)BRD_ENC_CS_Pin << 16U;
	result = RotorSensorTle5012b_Transfer16(spi, 0x8021U, &discard);
	if (result)
	{
		RotorSensorTle5012b_MosiHighImpedance();
		__NOP();
		__NOP();
		result = RotorSensorTle5012b_Transfer16(spi, 0U, &angle_word);
		RotorSensorTle5012b_MosiRestoreAlternateFunction();
	}
	(void)RotorSensorTle5012b_WaitFlag(spi, SPI_FLAG_BSY, false);
	BRD_ENC_CS_GPIO_Port->BSRR = BRD_ENC_CS_Pin;

	if (!result)
		return false;
	sample->angle_word = angle_word;
	sample->raw_angle_q15 = (uint16_t)((angle_word & 0x7FFFU) << 1U);
	return true;
}

RotorSensorPort RotorSensorTle5012b_CreatePort(void)
{
	RotorSensorPort port;

	port.context = &hspi2;
	port.initialize = RotorSensorTle5012b_Initialize;
	port.read_sample = RotorSensorTle5012b_ReadSample;
	return port;
}
