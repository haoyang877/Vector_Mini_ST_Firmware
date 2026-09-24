#include "encoder.h"

#include <limits.h>
#include <string.h>
#include "spi.h"
#include "hw_conf.h"
#include "utils.h"

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX 340U
#endif

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

static uint16_t SPI_Reg_ReadRx16(SPI_TypeDef *SPIx, bool *ok)
{
	uint16_t rx_data;
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

static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *SPIx, uint16_t tx_data, bool *ok)
{
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
	return SPI_Reg_ReadRx16(SPIx, ok);
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

bool Encoder_BeginSample(void)
{
	SPI_TypeDef *SPIx = brd_enc_spi.Instance;
	/* Never wait ahead of current protection. Unexpected bus state retains
	 * the original synchronous recovery path in Encoder_CompleteSample. */
	if ((SPIx->CR1 & SPI_CR1_SPE) == 0U ||
		(SPIx->SR & (SPI_FLAG_TXE | SPI_FLAG_OVR | SPI_FLAG_BSY | SPI_FLAG_RXNE)) != SPI_FLAG_TXE)
		return false;
	BRD_ENC_CS_ENABLE;
	*(__IO uint16_t *)&SPIx->DR = 0x8021U;
	return true;
}

static bool Encoder_ReadTle5012BFrame(Encoder_TypeDef *encoder, uint16_t *raw_q15,
	bool sample_started)
{
	SPI_TypeDef *SPIx = brd_enc_spi.Instance;
	uint16_t angle_word = 0U;
	bool transfer_ok = true;

	if (sample_started)
		(void)SPI_Reg_ReadRx16(SPIx, &transfer_ok);
	else
	{
		BRD_ENC_CS_ENABLE;
		(void)SPI_Reg_TxRx16(SPIx, 0x8021U, &transfer_ok);
	}
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
	encoder->pll_theta_rad = 0.0f;
	encoder->pll_omega_rad_s = 0.0f;
	encoder->velocity_ready = false;
	encoder->velocity_divider = 0U;
	encoder->vel_mech = 0.0f;
	encoder->vel_mech_continuous = 0.0f;
	encoder->vel_mech_fast = 0.0f;
	encoder->vel_elec = 0.0f;

	memset(encoder->velocity_win4_delta_history, 0,
		sizeof(encoder->velocity_win4_delta_history));
	encoder->velocity_win4_index = 0U;
	encoder->velocity_win4_sample_count = 0U;
	encoder->velocity_win4_delta_sum = 0;
	encoder->velocity_win4_ready = false;
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

/* 20 kHz 快路径二阶角度跟踪观测器（PLL）：由线性化单圈角估计机械角速度。
 * 输入：encoder（读 linearized_q15）、pole_pairs 极对数（调用方保证 >= 1）；仅有效帧调用。
 * 输出：pll_theta_rad/pll_omega_rad_s/velocity_ready、vel_mech_continuous 与 vel_mech_fast
 *       （PLL 下两者同值）、vel_mech（含软零速死区）、vel_elec。
 * 首帧/重启（velocity_ready == false）以当前角初始化观测器，不产生速度冲击；之后在 [0,2π)
 * 内部回绕估计角，误差不足半圈时单次 wrap 归位，角速度按上限限幅。
 * 不写 theta_elec/theta_mech，也不触及多圈与标定量。
 * 结束时维护快拍分频计数，供 Encoder_DidUpdateVelocity() 报告 2 kHz 慢拍边界。 */
static void Encoder_UpdateVelocityPll(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
	float theta_meas = (float)encoder->linearized_q15 * (_2PI / (float)ENCODER_Q15_CPR);
	float abs_omega = encoder->pll_omega_rad_s;
	float blend;
	float omega_n;
	float kp;
	float ki;
	float err;
	float vel;

	/* 按速调度观测器带宽：低速 600 Hz（相位余量优先，允许更硬的转速环增益），
	 * 高速 300 Hz（滤掉随速增长的与角度相关的周期误差），中间线性过渡。
	 * 调度量取上一拍估计速度：自洽、不依赖外部反馈，且转速连续时无跳变。 */
	if (abs_omega < 0.0f)
		abs_omega = -abs_omega;
	if (abs_omega <= ENCODER_PLL_SCHED_LOW_RAD_S)
		blend = 0.0f;
	else if (abs_omega >= ENCODER_PLL_SCHED_HIGH_RAD_S)
		blend = 1.0f;
	else
		blend = (abs_omega - ENCODER_PLL_SCHED_LOW_RAD_S) /
			(ENCODER_PLL_SCHED_HIGH_RAD_S - ENCODER_PLL_SCHED_LOW_RAD_S);
	omega_n = _2PI * (ENCODER_PLL_OMEGA_N_LOW_SPEED_HZ +
		(ENCODER_PLL_OMEGA_N_HIGH_SPEED_HZ - ENCODER_PLL_OMEGA_N_LOW_SPEED_HZ) * blend);
	kp = 2.0f * ENCODER_PLL_ZETA * omega_n * Current_Ts;
	ki = omega_n * omega_n * Current_Ts;

	if (++encoder->velocity_divider >= SPEED_LOOP_DIVIDER)
		encoder->velocity_divider = 0U;

	if (!encoder->velocity_ready)
	{
		encoder->pll_theta_rad = theta_meas;
		encoder->pll_omega_rad_s = 0.0f;
		encoder->velocity_ready = true;
		encoder->vel_mech = 0.0f;
		encoder->vel_mech_continuous = 0.0f;
		encoder->vel_mech_fast = 0.0f;
		encoder->vel_elec = 0.0f;
		return;
	}

	err = theta_meas - encoder->pll_theta_rad;

	if (err > _PI)
		err -= _2PI;
	else if (err < -_PI)
		err += _2PI;

	encoder->pll_theta_rad += Current_Ts * encoder->pll_omega_rad_s + kp * err;
	encoder->pll_omega_rad_s += ki * err;

	if (encoder->pll_theta_rad > _2PI)
		encoder->pll_theta_rad -= _2PI;
	else if (encoder->pll_theta_rad < 0.0f)
		encoder->pll_theta_rad += _2PI;

	if (encoder->pll_omega_rad_s > ENCODER_PLL_OMEGA_MAX_RAD_S)
		encoder->pll_omega_rad_s = ENCODER_PLL_OMEGA_MAX_RAD_S;
	else if (encoder->pll_omega_rad_s < -ENCODER_PLL_OMEGA_MAX_RAD_S)
		encoder->pll_omega_rad_s = -ENCODER_PLL_OMEGA_MAX_RAD_S;

	vel = encoder->pll_omega_rad_s;
	encoder->vel_mech_continuous = vel;
	encoder->vel_mech_fast = vel;
	if (vel < ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S && vel > -ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S)
		encoder->vel_mech = 0.0f;
	else
		encoder->vel_mech = vel;
	encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

/* A/B 对照用：4 抽头/2 ms 短窗测速（旧短窗方案 A2）。
 * 输入：encoder、pole_pairs、delta_q15 本拍位移增量（0.5 ms）；仅有效帧调用。
 * 输出：覆盖 vel_mech_continuous/vel_mech（8 计数门限）/vel_mech_fast（1 计数门限）/vel_elec，
 *       口径与旧短窗实现一致；仅在 ENCODER_VELOCITY_SOURCE 选中 WINDOW4 时被调用。
 * 无模型、无预测：群延时 (TAPS-1)/2*Speed_Ts + 半拍 ≈ 0.75 ms。 */
#if ENCODER_VELOCITY_SOURCE == ENCODER_VELOCITY_SOURCE_WINDOW4
static void Encoder_UpdateVelocityWindow4(Encoder_TypeDef *encoder, uint32_t pole_pairs,
	int32_t delta_q15)
{
	int32_t sum_abs;
	float velocity_scale;

	encoder->velocity_win4_delta_sum -=
		encoder->velocity_win4_delta_history[encoder->velocity_win4_index];
	encoder->velocity_win4_delta_history[encoder->velocity_win4_index] = delta_q15;
	encoder->velocity_win4_delta_sum += delta_q15;
	encoder->velocity_win4_index =
		(uint8_t)((encoder->velocity_win4_index + 1U) % ENCODER_VELOCITY_WINDOW4_TAPS);
	if (encoder->velocity_win4_sample_count < ENCODER_VELOCITY_WINDOW4_TAPS)
		encoder->velocity_win4_sample_count++;
	encoder->velocity_win4_ready =
		encoder->velocity_win4_sample_count == ENCODER_VELOCITY_WINDOW4_TAPS;

	/* 4 个增量是连续 4 个 20 kHz 快拍（窗口 4*Current_Ts=200 µs），
	 * 必须用 Current_Ts 归一化；用 Speed_Ts 会让分母大 10 倍、速度报小 10 倍。 */
	velocity_scale = _2PI / ((float)ENCODER_Q15_CPR *
		(float)ENCODER_VELOCITY_WINDOW4_TAPS * Current_Ts);
	encoder->vel_mech_continuous = encoder->velocity_win4_ready ?
		(float)encoder->velocity_win4_delta_sum * velocity_scale : 0.0f;

	sum_abs = encoder->velocity_win4_delta_sum;
	if (sum_abs < 0)
		sum_abs = -sum_abs;
	if (!encoder->velocity_win4_ready ||
		sum_abs <= ENCODER_VELOCITY_WINDOW4_ZERO_THRESHOLD_Q15)
		encoder->vel_mech = 0.0f;
	else
		encoder->vel_mech = encoder->vel_mech_continuous;

	if (!encoder->velocity_win4_ready ||
		sum_abs <= ENCODER_VELOCITY_WINDOW4_FAST_ZERO_THRESHOLD_Q15)
		encoder->vel_mech_fast = 0.0f;
	else
		encoder->vel_mech_fast = encoder->vel_mech_continuous;

	encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}
#endif

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

void Encoder_CompleteSample(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder,
	bool sample_started)
{
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	int32_t delta_q15;
	uint32_t pole_pairs;

	if (!Encoder_ReadTle5012BFrame(encoder, &raw_q15, sample_started))
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
		{
			encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
			/* Resolve only the first sample's turn; the portable axis policy
			 * preserves the calibrated zero and leaves travel checks active. */
			encoder->shadow_q15 = encoder->mechanical_zero_shadow_q15 +
				MotorAxisProfile_InitialEncoderOffsetQ15(&MotorControl->axis_profile,
					MotorControl->axis_profile_valid,
					(int32_t)linearized_q15 - (int32_t)encoder->mechanical_zero_q15);
		}
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
	Encoder_UpdateVelocityPll(encoder, pole_pairs);
#if ENCODER_VELOCITY_SOURCE == ENCODER_VELOCITY_SOURCE_WINDOW4
	Encoder_UpdateVelocityWindow4(encoder, pole_pairs, delta_q15);
#endif
	Encoder_UpdateAngles(encoder, pole_pairs);
}

void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder)
{
	Encoder_CompleteSample(MotorControl, encoder, false);
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

float Encoder_GetMecVelFast(const Encoder_TypeDef *encoder)
{
	return encoder->vel_mech_fast;
}

float Encoder_GetMecVelContinuous(const Encoder_TypeDef *encoder)
{
	return encoder->vel_mech_continuous;
}

bool Encoder_DidUpdateVelocity(const Encoder_TypeDef *encoder)
{
	if (encoder->bad_frame_streak == 0U && encoder->velocity_ready && encoder->velocity_divider == 0U)
		return true;

	return false;
}

float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *encoder)
{
	return (float)encoder->linearized_q15 / (float)ENCODER_Q15_CPR;
}
