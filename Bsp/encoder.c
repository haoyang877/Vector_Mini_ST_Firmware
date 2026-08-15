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

static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *SPIx, uint16_t tx_data, bool *ok)
{
	uint16_t rx_data;

	if ((SPIx->CR1 & SPI_CR1_SPE) == 0U)
		SPIx->CR1 |= SPI_CR1_SPE;

	if ((SPIx->SR & SPI_FLAG_OVR) != 0U)
	{
		(void)*(__IO uint16_t *)&SPIx->DR;
		(void)SPIx->SR;
	}

	if (!SPI_WaitFlag(SPIx, SPI_FLAG_TXE, ENC_SPI_XFER_SPIN_MAX))
	{
		*ok = false;
		return 0U;
	}
	*(__IO uint16_t *)&SPIx->DR = tx_data;

	if (!SPI_WaitFlag(SPIx, SPI_FLAG_RXNE, ENC_SPI_XFER_SPIN_MAX))
	{
		*ok = false;
		return 0U;
	}

	rx_data = *(__IO uint16_t *)&SPIx->DR;
	if (!SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX))
	{
		*ok = false;
		return 0U;
	}

	*ok = true;
	return rx_data;
}

static void SPI2_MOSI_HiZ(void)
{
	GPIOB->MODER &= ~(0x3UL << 30U);
}

static void SPI2_MOSI_RestoreAF(void)
{
	GPIOB->MODER = (GPIOB->MODER & ~(0x3UL << 30U)) | (0x2UL << 30U);
	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFUL << 28U)) | (0x5UL << 28U);
}

static bool Encoder_ReadTle5012BFrame(Encoder_TypeDef *encoder, uint16_t *raw_q15)
{
	SPI_TypeDef *SPIx = brd_enc_spi.Instance;
	uint16_t angle_word = 0U;
	bool transfer_ok = true;

	BRD_ENC_CS_ENABLE;
	(void)SPI_Reg_TxRx16(SPIx, 0x8021U, &transfer_ok);
	if (transfer_ok)
	{
		SPI2_MOSI_HiZ();
		__NOP();
		__NOP();
		angle_word = SPI_Reg_TxRx16(SPIx, 0U, &transfer_ok);
		SPI2_MOSI_RestoreAF();
	}
	(void)SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX);
	BRD_ENC_CS_DISABLE;

	if (!transfer_ok)
	{
		Encoder_MarkReadStatus(encoder, ENCODER_READ_SPI_TIMEOUT);
		return false;
	}

	encoder->tle5012_angle_word = angle_word;
	encoder->tle5012_safety_word = 0U;
	encoder->tle5012_crc_received = 0U;
	encoder->tle5012_crc_calculated = 0U;
	*raw_q15 = (uint16_t)((angle_word & 0x7FFFU) << 1U);
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
	SPI_HandleTypeDef *encoder_spi = &brd_enc_spi;

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
	encoder->tle5012_angle_word = 0U;
	encoder->tle5012_safety_word = 0U;
	encoder->tle5012_crc_received = 0U;
	encoder->tle5012_crc_calculated = 0U;
	encoder->tle5012_crc_error_count = 0U;
	encoder->read_error_count = 0U;
	encoder->bad_frame_streak = 0U;
	Encoder_ResetVelocity(encoder);

	encoder_spi->Init.DataSize = SPI_DATASIZE_16BIT;
	encoder_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	encoder_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
	encoder_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

	if (!Encoder_ReadTle5012BFrame(encoder, &raw_q15))
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