#include "encoder.h"

#include <limits.h>
#include <string.h>
#include "spi.h"
#include "hw_conf.h"
#include "utils.h"

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX 340U
#endif

#define ENCODER_VELOCITY_ZERO_THRESHOLD_Q15 8

static void Encoder_MarkReadStatus(Encoder_TypeDef *encoder, Encoder_ReadStatus status)
{
	encoder->read_status = status;
	if (status == ENCODER_READ_OK)
	{
		encoder->bad_frame_streak = 0U;
		return;
	}

	encoder->read_status_latched = status;
	encoder->read_error_count++;
	if (encoder->bad_frame_streak < UINT16_MAX)
		encoder->bad_frame_streak++;
}

static bool SPI_WaitFlag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t timeout_spin)
{
	uint32_t spin = 0U;
	while ((SPIx->SR & flag) == 0U)
	{
		if (++spin > timeout_spin)
			return false;
	}
	return true;
}

static bool SPI_WaitBSYClear(SPI_TypeDef *SPIx, uint32_t timeout_spin)
{
	uint32_t spin = 0U;
	while ((SPIx->SR & SPI_FLAG_BSY) != 0U)
	{
		if (++spin > timeout_spin)
			return false;
	}
	return true;
}

static uint8_t SPI_Reg_TxRx8(SPI_TypeDef *SPIx, uint8_t tx_data, bool *ok)
{
	if ((SPIx->CR1 & SPI_CR1_SPE) == 0U)
		SPIx->CR1 |= SPI_CR1_SPE;

	if ((SPIx->SR & SPI_FLAG_OVR) != 0U)
	{
		(void)*(__IO uint8_t *)&SPIx->DR;
		(void)SPIx->SR;
	}

	if (!SPI_WaitFlag(SPIx, SPI_FLAG_TXE, ENC_SPI_XFER_SPIN_MAX))
	{
		*ok = false;
		return 0U;
	}
	*(__IO uint8_t *)&SPIx->DR = tx_data;

	if (!SPI_WaitFlag(SPIx, SPI_FLAG_RXNE, ENC_SPI_XFER_SPIN_MAX))
	{
		*ok = false;
		return 0U;
	}

	{
		uint8_t rx_data = *(__IO uint8_t *)&SPIx->DR;
		if (!SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX))
		{
			*ok = false;
			return 0U;
		}
		*ok = true;
		return rx_data;
	}
}

static uint8_t MT6701_CRC6(uint32_t payload18)
{
	uint8_t crc = 0U;
	int bit_index;

	for (bit_index = 17; bit_index >= 0; --bit_index)
	{
		uint8_t input_bit = (uint8_t)((payload18 >> bit_index) & 1U);
		uint8_t feedback = (uint8_t)(((crc >> 5) & 1U) ^ input_bit);
		crc = (uint8_t)((crc << 1) & 0x3FU);
		if (feedback != 0U)
			crc ^= 0x03U;
	}
	return (uint8_t)(crc & 0x3FU);
}

static bool Encoder_ReadMt6701Frame(Encoder_TypeDef *encoder, uint16_t *raw_q15)
{
	SPI_TypeDef *SPIx = ext_enc_spi.Instance;
	uint8_t rx_bytes[3] = {0U, 0U, 0U};
	bool transfer_ok = true;
	uint8_t byte_index;
	uint32_t frame;
	uint16_t raw14;
	uint8_t status;
	uint8_t crc_received;
	uint8_t crc_calculated;

	EXT_ENC_CS_ENABLE;
	for (byte_index = 0U; byte_index < 3U; ++byte_index)
	{
		rx_bytes[byte_index] = SPI_Reg_TxRx8(SPIx, 0x00U, &transfer_ok);
		if (!transfer_ok)
			break;
	}
	(void)SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX);
	EXT_ENC_CS_DISABLE;

	if (!transfer_ok)
	{
		Encoder_MarkReadStatus(encoder, ENCODER_READ_SPI_TIMEOUT);
		return false;
	}

	frame = ((uint32_t)rx_bytes[0] << 16) | ((uint32_t)rx_bytes[1] << 8) | rx_bytes[2];
	raw14 = (uint16_t)((frame >> 10) & 0x3FFFU);
	status = (uint8_t)((frame >> 6) & 0x0FU);
	crc_received = (uint8_t)(frame & 0x3FU);
	crc_calculated = MT6701_CRC6((frame >> 6) & 0x3FFFFU);

	encoder->mt6701_status_bits = status;
	encoder->mt6701_crc_received = crc_received;
	encoder->mt6701_crc_calculated = crc_calculated;

	if (crc_received != crc_calculated)
	{
		encoder->mt6701_crc_error_count++;
		Encoder_MarkReadStatus(encoder, ENCODER_READ_CRC_MISMATCH);
		return false;
	}
	if ((status & 0x08U) != 0U)
	{
		Encoder_MarkReadStatus(encoder, ENCODER_READ_OVERSPEED);
		return false;
	}

	switch (status & 0x03U)
	{
		case 1U:
			Encoder_MarkReadStatus(encoder, ENCODER_READ_MAGNET_TOO_STRONG);
			return false;
		case 2U:
			Encoder_MarkReadStatus(encoder, ENCODER_READ_MAGNET_TOO_WEAK);
			return false;
		case 3U:
			Encoder_MarkReadStatus(encoder, ENCODER_READ_MAGNET_INVALID);
			return false;
		default:
			break;
	}

	*raw_q15 = (uint16_t)(raw14 << 2);
	Encoder_MarkReadStatus(encoder, ENCODER_READ_OK);
	return true;
}

static uint16_t Encoder_ApplyDirectionQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
	if (encoder->reverse == 0U)
		return raw_q15;

	return (uint16_t)(0U - raw_q15);
}

static uint16_t Encoder_ApplyLinearizationQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
	uint16_t lut_index = raw_q15 >> 6;
	uint16_t fraction = raw_q15 & 0x003FU;
	int32_t correction_a = encoder->linearization_lut_q15[lut_index];
	int32_t correction_b = encoder->linearization_lut_q15[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
	int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

	return (uint16_t)((int32_t)raw_q15 - correction);
}

void Encoder_ResetVelocity(Encoder_TypeDef *encoder)
{
	memset(encoder->velocity_delta_history, 0, sizeof(encoder->velocity_delta_history));
	encoder->velocity_divider = 0U;
	encoder->velocity_history_index = 0U;
	encoder->velocity_sample_count = 0U;
	encoder->velocity_delta_sum = 0;
	encoder->velocity_ready = false;
	encoder->velocity_shadow_q15 = encoder->shadow_q15;
	encoder->vel_mech = 0.0f;
	encoder->vel_elec = 0.0f;
}

void Encoder_SetReverse(Encoder_TypeDef *encoder, bool reverse)
{
	uint8_t reverse_value = reverse ? 1U : 0U;
	uint32_t primask;

	if (encoder->reverse == reverse_value)
		return;

	primask = __get_PRIMASK();
	__disable_irq();
	encoder->reverse = reverse_value;
	encoder->electrical_zero_q15 = 0U;
	encoder->mechanical_zero_q15 = 0U;
	encoder->calib_flag = 0U;
	memset(encoder->linearization_lut_q15, 0, sizeof(encoder->linearization_lut_q15));
	encoder->raw_q15 = 0U;
	encoder->directed_q15 = 0U;
	encoder->linearized_q15 = 0U;
	encoder->previous_linearized_q15 = 0U;
	encoder->shadow_q15 = 0;
	encoder->mechanical_zero_shadow_q15 = 0;
	encoder->has_valid_sample = false;
	encoder->theta_elec = 0.0f;
	encoder->theta_mech = 0.0f;
	Encoder_ResetVelocity(encoder);
	__set_PRIMASK(primask);
}

static void Encoder_UpdateVelocity2kHz(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
	int64_t delta64;
	int32_t delta_q15;
	int32_t sum_abs;
	float velocity_scale;

	if (++encoder->velocity_divider < SPEED_LOOP_DIVIDER)
		return;
	encoder->velocity_divider = 0U;

	delta64 = encoder->shadow_q15 - encoder->velocity_shadow_q15;
	encoder->velocity_shadow_q15 = encoder->shadow_q15;
	if (delta64 > INT32_MAX)
		delta_q15 = INT32_MAX;
	else if (delta64 < INT32_MIN)
		delta_q15 = INT32_MIN;
	else
		delta_q15 = (int32_t)delta64;

	encoder->velocity_delta_sum -= encoder->velocity_delta_history[encoder->velocity_history_index];
	encoder->velocity_delta_history[encoder->velocity_history_index] = delta_q15;
	encoder->velocity_delta_sum += delta_q15;
	encoder->velocity_history_index = (uint8_t)((encoder->velocity_history_index + 1U) % ENCODER_VELOCITY_WINDOW);
	if (encoder->velocity_sample_count < ENCODER_VELOCITY_WINDOW)
		encoder->velocity_sample_count++;
	encoder->velocity_ready = encoder->velocity_sample_count == ENCODER_VELOCITY_WINDOW;

	sum_abs = encoder->velocity_delta_sum;
	if (sum_abs < 0)
		sum_abs = -sum_abs;
	if (!encoder->velocity_ready || sum_abs <= ENCODER_VELOCITY_ZERO_THRESHOLD_Q15)
	{
		encoder->vel_mech = 0.0f;
	}
	else
	{
		velocity_scale = _2PI / ((float)ENCODER_Q15_CPR * (float)ENCODER_VELOCITY_WINDOW * Speed_Ts);
		encoder->vel_mech = (float)encoder->velocity_delta_sum * velocity_scale;
	}
	encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

static void Encoder_UpdateAngles(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
	uint16_t electrical_q15;

	electrical_q15 = (uint16_t)((uint32_t)(uint16_t)(encoder->linearized_q15 - encoder->electrical_zero_q15) * pole_pairs);
	encoder->theta_elec = (float)electrical_q15 * (_2PI / (float)ENCODER_Q15_CPR);
	encoder->theta_mech = (float)(encoder->shadow_q15 - encoder->mechanical_zero_shadow_q15) * (_2PI / (float)ENCODER_Q15_CPR);
}

void Encoder_ParamInit(Encoder_TypeDef *encoder)
{
	SPI_HandleTypeDef *encoder_spi = &ext_enc_spi;

	encoder->reverse = encoder->reverse != 0U ? 1U : 0U;
	encoder->raw_q15 = 0U;
	encoder->directed_q15 = 0U;
	encoder->linearized_q15 = 0U;
	encoder->previous_linearized_q15 = 0U;
	encoder->shadow_q15 = 0;
	encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
	encoder->velocity_shadow_q15 = 0;
	encoder->has_valid_sample = false;
	encoder->theta_elec = 0.0f;
	encoder->theta_mech = 0.0f;
	encoder->read_status = ENCODER_READ_OK;
	encoder->read_status_latched = ENCODER_READ_OK;
	encoder->mt6701_status_bits = 0U;
	encoder->mt6701_crc_received = 0U;
	encoder->mt6701_crc_calculated = 0U;
	encoder->mt6701_crc_error_count = 0U;
	encoder->read_error_count = 0U;
	encoder->bad_frame_streak = 0U;
	Encoder_ResetVelocity(encoder);

	encoder_spi->Init.DataSize = SPI_DATASIZE_8BIT;
	encoder_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	encoder_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
	encoder_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	if (HAL_SPI_Init(encoder_spi) != HAL_OK)
		Error_Handler();
}

bool Encoder_IsOnline(const Encoder_TypeDef *encoder)
{
	return encoder->has_valid_sample &&
	       encoder->bad_frame_streak < ENCODER_BAD_FRAME_OFFLINE_COUNT;
}

bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *encoder, uint16_t electrical_zero_q15)
{
	if (!Encoder_IsOnline(encoder))
		return false;

	encoder->electrical_zero_q15 = electrical_zero_q15;
	encoder->calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
	return true;
}

bool Encoder_SetElectricalZero(Encoder_TypeDef *encoder)
{
	return Encoder_SetElectricalZeroQ15(encoder, encoder->linearized_q15);
}

bool Encoder_SetMechanicalZero(Encoder_TypeDef *encoder)
{
	if (!Encoder_IsOnline(encoder))
		return false;

	encoder->mechanical_zero_q15 = encoder->linearized_q15;
	encoder->mechanical_zero_shadow_q15 = encoder->shadow_q15;
	encoder->calib_flag |= ENC_CALIB_MECHANICAL_ZERO;
	encoder->theta_mech = 0.0f;
	return true;
}

void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder)
{
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	int32_t delta_q15;
	uint32_t pole_pairs;

	if (!Encoder_ReadMt6701Frame(encoder, &raw_q15))
		return;

	directed_q15 = Encoder_ApplyDirectionQ15(encoder, raw_q15);
	linearized_q15 = Encoder_ApplyLinearizationQ15(encoder, directed_q15);
	encoder->raw_q15 = raw_q15;
	encoder->directed_q15 = directed_q15;
	encoder->linearized_q15 = linearized_q15;
	pole_pairs = MotorControl->motor_pole_pairs > 0 ? (uint32_t)MotorControl->motor_pole_pairs : 1U;

	if (!encoder->has_valid_sample)
	{
		encoder->previous_linearized_q15 = linearized_q15;
		encoder->shadow_q15 = linearized_q15;
		if ((encoder->calib_flag & ENC_CALIB_MECHANICAL_ZERO) == 0U)
			encoder->mechanical_zero_shadow_q15 = 0;
		else
			encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
		encoder->has_valid_sample = true;
		Encoder_ResetVelocity(encoder);
		Encoder_UpdateAngles(encoder, pole_pairs);
		return;
	}

	delta_q15 = (int32_t)linearized_q15 - (int32_t)encoder->previous_linearized_q15;
	if (delta_q15 > ENCODER_Q15_HALF_TURN)
		delta_q15 -= (int32_t)ENCODER_Q15_CPR;
	else if (delta_q15 < -ENCODER_Q15_HALF_TURN)
		delta_q15 += (int32_t)ENCODER_Q15_CPR;

	encoder->previous_linearized_q15 = linearized_q15;
	encoder->shadow_q15 += delta_q15;
	Encoder_UpdateVelocity2kHz(encoder, pole_pairs);
	Encoder_UpdateAngles(encoder, pole_pairs);
}

float Encoder_GetElePhase(const Encoder_TypeDef *encoder)
{
	return encoder->theta_elec;
}

float Encoder_GetMecPos(const Encoder_TypeDef *encoder)
{
	return encoder->theta_mech;
}

float Encoder_GetEleVel(const Encoder_TypeDef *encoder)
{
	return encoder->vel_elec;
}

float Encoder_GetMecVel(const Encoder_TypeDef *encoder)
{
	return encoder->vel_mech;
}

float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *encoder)
{
	return (float)encoder->linearized_q15 / (float)ENCODER_Q15_CPR;
}
