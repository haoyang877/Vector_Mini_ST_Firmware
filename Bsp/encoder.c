#include "encoder.h"

#include <stdbool.h>
#include <string.h>
#include "spi.h"
#include "hw_conf.h"
#include "utils.h"
#include "foc_errhandle.h"

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX (340U)
#endif

static void Encoder_MarkReadStatus(Encoder_TypeDef *Encoder, Encoder_ReadStatus status)
{
	Encoder->read_status = status;
	if (status != ENCODER_READ_OK)
	{
		Encoder->read_status_latched = status;
		Encoder->read_error_count++;
	}
}

static bool SPI_WaitFlag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t timeout_spin)
{
	uint32_t spin = 0U;
	while (!(SPIx->SR & flag))
	{
		if (++spin > timeout_spin)
		{
			return false;
		}
	}
	return true;
}

static bool SPI_WaitBSYClear(SPI_TypeDef *SPIx, uint32_t timeout_spin)
{
	uint32_t spin = 0U;
	while (SPIx->SR & SPI_FLAG_BSY)
	{
		if (++spin > timeout_spin)
		{
			return false;
		}
	}
	return true;
}

static uint8_t SPI_Reg_TxRx8(SPI_TypeDef *SPIx, uint8_t tx_data, bool *ok)
{
	if (!(SPIx->CR1 & SPI_CR1_SPE))
	{
		SPIx->CR1 |= SPI_CR1_SPE;
	}
	if (SPIx->SR & SPI_FLAG_OVR)
	{
		(void)*(__IO uint8_t *)&SPIx->DR;
		(void)SPIx->SR;
	}
	if (!SPI_WaitFlag(SPIx, SPI_FLAG_TXE, ENC_SPI_XFER_SPIN_MAX)) { *ok = false; return 0U; }
	*(__IO uint8_t *)&SPIx->DR = tx_data;
	if (!SPI_WaitFlag(SPIx, SPI_FLAG_RXNE, ENC_SPI_XFER_SPIN_MAX)) { *ok = false; return 0U; }
	uint8_t rx = *(__IO uint8_t *)&SPIx->DR;
	if (!SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX)) { *ok = false; return 0U; }
	*ok = true;
	return rx;
}


/**
	* @brief  Init encoder parameters
 **/
void Encoder_ParamInit(Encoder_TypeDef *Encoder)
{
    Encoder->raw                 = 0;
    Encoder->count_in_cpr        = 0;
    Encoder->count_in_cpr_prev   = 0;
    Encoder->shadow_count        = 0;
    Encoder->pos_cpr_counts      = 0;
    Encoder->vel_estimate_counts = 0;
    Encoder->pos                 = 0;
    Encoder->vel                 = 0;
    Encoder->theta_elec          = 0;
    Encoder->vel_elec            = 0;
    Encoder->interpolation       = 0;
    Encoder->read_status         = ENCODER_READ_OK;
    Encoder->read_status_latched  = ENCODER_READ_OK;
    Encoder->mt6701_status_bits   = 0;
    Encoder->mt6701_crc_received  = 0;
    Encoder->mt6701_crc_calculated = 0;
    Encoder->mt6701_crc_error_count = 0;
    Encoder->read_error_count     = 0;
    Encoder->read_error_streak    = 0;

    int   encoder_pll_bw   	   		= 2000;
    float bandwidth            		= fast_min(encoder_pll_bw, 0.25f * PWM_TIM_FREQ);
    Encoder->pll_kp         		= 2.0f * bandwidth;           
    Encoder->pll_ki         		= 0.25f * fast_sq(Encoder->pll_kp); 
    Encoder->snap_threshold 		= 0.5f * Current_Ts * Encoder->pll_ki;
	
	SPI_HandleTypeDef *enc_spi = &ext_enc_spi;
	
	switch(Encoder->type)
	{
		case TLE5012B:
			enc_spi->Init.DataSize = SPI_DATASIZE_16BIT;
			enc_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
			enc_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
			enc_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
		
			Encoder->resolution = 15;
			Encoder->cpr = 32768;
			Encoder->one_by_cpr = 1.0f / 32768.0f;
		break;
		
		case MT6816:
			enc_spi->Init.DataSize = SPI_DATASIZE_16BIT;
			enc_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
			enc_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
			enc_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
		
			Encoder->resolution = 14;
			Encoder->cpr = 16384;
			Encoder->one_by_cpr = 1.0f / 16384.0f;
		break;
		
		case MT6701:
			enc_spi->Init.DataSize = SPI_DATASIZE_8BIT;
			enc_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
			enc_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
			enc_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

			Encoder->resolution = 14;
			Encoder->cpr = 16384;
			Encoder->one_by_cpr = 1.0f / 16384.0f;
		break;
		
		default:break;
	}
	
	if (HAL_SPI_Init(enc_spi) != HAL_OK)
	{
		Error_Handler();
	}
	

}

/**
  * @brief  SPI register-level transmit/receive one 16-bit word
  * @param  SPIx: SPI peripheral (SPI1)
  * @param  tx_data: 16-bit data to transmit
  * @retval 16-bit received data
  */
static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *SPIx, uint16_t tx_data)
{
	uint32_t spin;

	/* Enable SPI if disabled */
	if (!(SPIx->CR1 & SPI_CR1_SPE))
	{
		SPIx->CR1 |= SPI_CR1_SPE;
	}

	/* Clear pending OVR by reading DR then SR */
	if (SPIx->SR & SPI_FLAG_OVR)
	{
		(void)*(__IO uint16_t *)&SPIx->DR;
		(void)SPIx->SR;
	}

	/* Wait for TXE flag with timeout */
	spin = 0U;
	while (!(SPIx->SR & SPI_FLAG_TXE))
	{
		if (++spin > ENC_SPI_XFER_SPIN_MAX) break;
	}

	/* Write tx data to DR to start transfer */
	*(__IO uint16_t *)&SPIx->DR = tx_data;

	/* Wait for RXNE flag with timeout */
	spin = 0U;
	while (!(SPIx->SR & SPI_FLAG_RXNE))
	{
		if (++spin > ENC_SPI_XFER_SPIN_MAX) break;
	}

	/* Read rx data from DR */
	return *(__IO uint16_t *)&SPIx->DR;
}

/**
	* @brief  Read raw data of TLE5012B encoder
    * @retval Raw data of mechanical angle (0-32767)
 **/
uint16_t ReadTLE5012B_Raw(void)
{
	uint16_t data_t[2] = {0x8021, 0x0000};
	uint16_t data_r[2];

	EXT_ENC_CS_ENABLE;
	data_r[0] = SPI_Reg_TxRx16(EXT_ENC_SPI, data_t[0]);
	data_r[1] = SPI_Reg_TxRx16(EXT_ENC_SPI, data_t[1]);
	EXT_ENC_CS_DISABLE;

	return data_r[1] & 0x7FFF;
}

/**
	* @brief  Read raw data of MT6816 encoder
    * @retval Raw data of mechanical angle (0-16383)
 **/
uint16_t ReadMT6816_Raw(void)
{
	uint16_t sample_data = 0U;
	uint8_t parity_ok = 0U;
	uint16_t angle = 0U;
	uint16_t data_t[2];
	uint16_t data_r[2] = {0U, 0U};

	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;

	for (uint8_t i = 0U; i < 3U; ++i)
	{
		EXT_ENC_CS_ENABLE;
		data_r[0] = SPI_Reg_TxRx16(EXT_ENC_SPI, data_t[0]);
		EXT_ENC_CS_DISABLE;
		EXT_ENC_CS_ENABLE;
		data_r[1] = SPI_Reg_TxRx16(EXT_ENC_SPI, data_t[1]);
		EXT_ENC_CS_DISABLE;

		sample_data = ((data_r[0] & 0x00FFU) << 8) | (data_r[1] & 0x00FFU);
		uint8_t high_count = 0U;
		for (uint8_t bit = 0U; bit < 16U; ++bit)
		{
			if ((sample_data & (1U << bit)) != 0U)
				high_count++;
		}
		if ((high_count & 1U) == 0U)
		{
			parity_ok = 1U;
			break;
		}
	}

	if (parity_ok != 0U)
		angle = sample_data >> 2;

	return angle;
}

/**
	* @brief  Read raw data of MT6701 encoder
    * @param  Reg: Register address 
    * @retval Raw data of mechanical angle (0-16383)
 **/
static uint8_t MT6701_CRC6(uint32_t payload18)
{
	uint8_t crc = 0U;
	for (int i = 17; i >= 0; --i)
	{
		uint8_t bit = (uint8_t)((payload18 >> i) & 1U);
		uint8_t fb = (uint8_t)(((crc >> 5) & 1U) ^ bit);
		crc = (uint8_t)((crc << 1) & 0x3FU);
		if (fb)
		{
			crc ^= 0x03U;
		}
	}
	return crc & 0x3FU;
}

uint16_t ReadMT6701_Raw(Encoder_TypeDef *Encoder)
{
	SPI_HandleTypeDef *enc_spi = &ext_enc_spi;
	SPI_TypeDef *SPIx = enc_spi->Instance;
	uint8_t rx_bytes[3] = {0U, 0U, 0U};
	bool ok = true;
	uint16_t raw = Encoder->raw;

	EXT_ENC_CS_ENABLE;
	for (uint8_t i = 0; i < 3U; ++i)
	{
		rx_bytes[i] = SPI_Reg_TxRx8(SPIx, 0x00U, &ok);
		if (!ok)
		{
			break;
		}
	}
	(void)SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX);
	EXT_ENC_CS_DISABLE;

	if (!ok)
	{
		Encoder_MarkReadStatus(Encoder, ENCODER_READ_SPI_TIMEOUT);
		return raw;
	}

	uint32_t frame = ((uint32_t)rx_bytes[0] << 16) | ((uint32_t)rx_bytes[1] << 8) | rx_bytes[2];
	uint16_t angle = (uint16_t)((frame >> 10) & 0x3FFFU);
	uint8_t status = (uint8_t)((frame >> 6) & 0x0FU);
	uint8_t crc_rx = (uint8_t)(frame & 0x3FU);
	uint8_t crc_calc = MT6701_CRC6((frame >> 6) & 0x3FFFFU);

	Encoder->mt6701_status_bits = status;
	Encoder->mt6701_crc_received = crc_rx;
	Encoder->mt6701_crc_calculated = crc_calc;

	if (crc_rx != crc_calc)
	{
		Encoder->mt6701_crc_error_count++;
		Encoder_MarkReadStatus(Encoder, ENCODER_READ_CRC_MISMATCH);
		return raw;
	}
	if ((status & 0x08U) != 0U)
	{
		Encoder_MarkReadStatus(Encoder, ENCODER_READ_OVERSPEED);
		return raw;
	}
	switch (status & 0x03U)
	{
		case 1U:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_MAGNET_TOO_STRONG);
			return raw;
		case 2U:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_MAGNET_TOO_WEAK);
			return raw;
		case 3U:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_MAGNET_INVALID);
		return raw;
		default:
		break;
	}

	Encoder_MarkReadStatus(Encoder, ENCODER_READ_OK);
	return angle;
}

uint16_t ReadSPIEncoder_Raw(Encoder_TypeDef *Encoder)
{
	switch(Encoder->type)
	{
		case TLE5012B:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_OK);
			return ReadTLE5012B_Raw();
		case MT6816:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_OK);
			return ReadMT6816_Raw();
		case MT6701:
			return ReadMT6701_Raw(Encoder);
		default:
			Encoder_MarkReadStatus(Encoder, ENCODER_READ_UNSUPPORTED);
			return (uint16_t)Encoder->raw;
	}
}

/**
	* @brief  Update encoder parameters
			  get encoder count considering offset and linearization
			  apply PLL to get a smooth speed profile
    * @param  *MotorControl: MotorControl struct pointer
	  @param  *Encoder: Encoder struct pointer
 **/
void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	if(Encoder->enable == ENCODER_DISABLE)
		return;
	
	int raw = ReadSPIEncoder_Raw(Encoder);
	if (Encoder->read_status != ENCODER_READ_OK)
	{
		if (Encoder->read_error_streak < UINT16_MAX)
			Encoder->read_error_streak++;
		if (Encoder->read_error_streak >= 500U &&
		   (MotorControl->ModeNow == Calib_EncoderOffset ||
		    MotorControl->ModeNow == Current_Mode ||
		    MotorControl->ModeNow == Speed_Mode ||
		    MotorControl->ModeNow == Position_Mode ||
		    MotorControl->ModeNow == Vq_Mode))
		{
			Set_ErrorNow(Encoder_Error);
		}
		return;
	}
	Encoder->read_error_streak = 0U;
	if(Encoder->dir == -1)
		raw = Encoder->cpr - raw;
	if(raw >= Encoder->cpr)
		raw -= Encoder->cpr;
	Encoder->raw = raw;
	
	if(Encoder->raw == 0 || Encoder->raw == 1 || Encoder->raw == Encoder->cpr || Encoder->raw == Encoder->cpr - 1)
	{
		if(++ Encoder->disconnect_count >= 500)
		{
			if(MotorControl->ModeNow == Calib_EncoderOffset ||  
			   MotorControl->ModeNow == Current_Mode ||
			   MotorControl->ModeNow == Speed_Mode ||
			   MotorControl->ModeNow == Vq_Mode)
			{
				Set_ErrorNow(Encoder_Error);
			}
			Encoder->disconnect_count = 0;
		}
	}
	else
	{
		Encoder->disconnect_count = 0;
	}
    /* Linearization LUT with interpolation between adjacent entries. */
	int off_bit = Encoder->resolution - ENCODER_OFFSET_LUT_BITS;
	int lut_index = Encoder->raw >> off_bit;
	int off_1 = Encoder->offset_lut[lut_index];
	int off_2 = Encoder->offset_lut[(lut_index + 1) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
	int off_interp = off_1
				 + ((off_2 - off_1) * (Encoder->raw - (lut_index << off_bit)) >> off_bit);
    int count = Encoder->raw - off_interp;
	
    /*  Wrap in ENCODER_CPR */
    while (count >= Encoder->cpr)
        count -= Encoder->cpr;
    while (count < 0)
        count += Encoder->cpr;
    Encoder->count_in_cpr = count;
	
    /* Delta count */
    int delta_count           = Encoder->count_in_cpr - Encoder->count_in_cpr_prev;
    Encoder->count_in_cpr_prev = Encoder->count_in_cpr;
    while (delta_count > +(Encoder->cpr >> 1))
        delta_count -= Encoder->cpr;
    while (delta_count < -(Encoder->cpr >> 1))
        delta_count += Encoder->cpr;
	
    /* Add measured delta to encoder count */
    Encoder->shadow_count += delta_count;
	
    /* Run vel PLL */
    Encoder->pos_cpr_counts += Current_Ts * Encoder->vel_estimate_counts;
    float delta_pos_cpr_counts = (float) (Encoder->count_in_cpr - (int) Encoder->pos_cpr_counts);
    while (delta_pos_cpr_counts > +(Encoder->cpr >> 1))
        delta_pos_cpr_counts -= (float)Encoder->cpr;
    while (delta_pos_cpr_counts < -(Encoder->cpr >> 1))
        delta_pos_cpr_counts += (float)Encoder->cpr;
    Encoder->pos_cpr_counts += Current_Ts * Encoder->pll_kp * delta_pos_cpr_counts;
    while (Encoder->pos_cpr_counts >= Encoder->cpr)
        Encoder->pos_cpr_counts -= (float)Encoder->cpr;
    while (Encoder->pos_cpr_counts < 0)
        Encoder->pos_cpr_counts += (float)Encoder->cpr;
    Encoder->vel_estimate_counts += Current_Ts * Encoder->pll_ki * delta_pos_cpr_counts;

    /* align delta-sigma on zero to prevent jitter */
    bool snap_to_zero_vel = false;
    if (fast_abs(Encoder->vel_estimate_counts) < Encoder->snap_threshold) 
	{
        Encoder->vel_estimate_counts = 0.0f;
        snap_to_zero_vel            = true;
    }

    /* run encoder count interpolation */
    /* if we are stopped, make sure we don't randomly drift */
    if (snap_to_zero_vel) 
	{
        Encoder->interpolation = 0.5f;
        /*reset interpolation if encoder edge comes */
    } 
	else if (delta_count > 0) 
	{
        Encoder->interpolation = 0.0f;
    } 
	else if (delta_count < 0) 
	{
        Encoder->interpolation = 1.0f;
    } 
	else 
	{
        /* Interpolate (predict) between encoder counts using vel_estimate */
        Encoder->interpolation += Current_Ts * Encoder->vel_estimate_counts;
        /* don't allow interpolation indicated position outside of [enc, enc+1) */
        if (Encoder->interpolation > 1.0f)
            Encoder->interpolation = 1.0f;
        if (Encoder->interpolation < 0.0f)
            Encoder->interpolation = 0.0f;
    }
    float interpolated_enc = Encoder->count_in_cpr - Encoder->offset + Encoder->interpolation;
    while (interpolated_enc >= Encoder->cpr)
        interpolated_enc -= Encoder->cpr;
    while (interpolated_enc < 0)
        interpolated_enc += Encoder->cpr;

    float shadow_count_f = (float)Encoder->shadow_count;
    Encoder->turns          = shadow_count_f * Encoder->one_by_cpr;
    float residual       = shadow_count_f - Encoder->turns * (float)Encoder->cpr - Encoder->zero_count;
	
    /*outputs from encoder for controller*/
    Encoder->pos = Encoder->turns + residual * Encoder->one_by_cpr;
    UTILS_LP_MOVING_AVG_APPROX(Encoder->vel, (Encoder->vel_estimate_counts * Encoder->one_by_cpr), 5);
	
	if(MotorControl->motor_pole_pairs <= 0 || MotorControl->motor_pole_pairs > 30)
		Set_ErrorNow(PolePairs_Error);

    Encoder->theta_elec  = normalizeAngle((interpolated_enc * _2PI * MotorControl->motor_pole_pairs) * Encoder->one_by_cpr);
    Encoder->vel_elec 	= Encoder->vel * _2PI * MotorControl->motor_pole_pairs;
	
	Encoder->theta_mech  = Encoder->pos * _2PI;
	Encoder->vel_mech  	= Encoder->vel * _2PI;
	
}

/**
	* @brief  Get encoder electrical phase
	* @param  *Encoder: encoder struct pointer
	* @retval electrical phase
 **/
float Encoder_GetElePhase(Encoder_TypeDef *Encoder)
{
	return Encoder->theta_elec;
}

/**
	* @brief  Get encoder mechanical position
	* @param  *Encoder: encoder struct pointer
	* @retval mechanical position
 **/
float Encoder_GetMecPos(Encoder_TypeDef *Encoder)
{
	return Encoder->theta_mech;
}

/**
	* @brief  Get encoder electrical velocity
	* @param  *Encoder: encoder struct pointer
	* @retval electrical velocity
 **/
float Encoder_GetEleVel(Encoder_TypeDef *Encoder)
{
	return Encoder->vel_elec;
}

/**
	* @brief  Get encoder mechanical velocity
	* @param  *Encoder: encoder struct pointer
	* @retval mechanical velocity
 **/
float Encoder_GetMecVel(Encoder_TypeDef *Encoder)
{
	return Encoder->vel_mech;
}

/**
	* @brief  Get encoder count ratio in one CPR
	* @param  *Encoder: encoder struct pointer
	* @retval count ratio in one CPR
 **/
float Encoder_GetCountInCPR_Ratio(Encoder_TypeDef *Encoder)
{
	return Encoder->count_in_cpr * Encoder->one_by_cpr;
}

/**
	* @brief  Detect external encoder type changes and reinitialize parameters
	* @param  *Encoder: external encoder struct pointer
 **/
void Encoder_ChangeDetect(Encoder_TypeDef *Encoder)
{
	static Encoder_Type type_last;
	static bool is_first_call = true;

	if (is_first_call)
	{
		type_last = Encoder->type;
		is_first_call = false;
	}

	if (Encoder->type != type_last)
	{
		Encoder->calib_flag = 0U;
		Encoder->offset = 0;
		Encoder->zero_count = 0;
		memset(Encoder->offset_lut, 0, sizeof(Encoder->offset_lut));
		Encoder_ParamInit(Encoder);
	}

	type_last = Encoder->type;
}

/**
	* @brief  Set current encoder count as zero position
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Set_ZeroPosition(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	Encoder->zero_count = Encoder->count_in_cpr;
	Encoder->shadow_count = Encoder->zero_count;
	
	/*electrical angle zero position calibrated*/
	Encoder->calib_flag |= ENC_CALIB_ZERO_POS;
	
	MotorControl->posTrajUpdated = true;
	
	MotorControl->ModeNow = Save_Param;
}


