#include "encoder.h"

#include <stdbool.h>
#include "spi.h"
#include "hw_conf.h"
#include "utils.h"
#include "foc_errhandle.h"

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX (340U)
#endif

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

    int   encoder_pll_bw   	   		= 2000;
    float bandwidth            		= fast_min(encoder_pll_bw, 0.25f * PWM_TIM_FREQ);
    Encoder->pll_kp         		= 2.0f * bandwidth;           
    Encoder->pll_ki         		= 0.25f * fast_sq(Encoder->pll_kp); 
    Encoder->snap_threshold 		= 0.5f * Current_Ts * Encoder->pll_ki;
	
	SPI_HandleTypeDef *enc_spi;
	
	if(Encoder->source == ON_BOARD)
		enc_spi = &brd_enc_spi;
	else if(Encoder->source == EXTERNAL)
		enc_spi = &ext_enc_spi;
	
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
			enc_spi->Init.DataSize = SPI_DATASIZE_16BIT;
			enc_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
			enc_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
			enc_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
		
			Encoder->resolution = 14;
			Encoder->cpr = 16384;
			Encoder->one_by_cpr = 1.0f / 16384.0f;
		break;
		
		default:break;
	}
	
	/*enable SPI*/
//	if(Encoder->source == ON_BOARD)
//		BRD_ENC_SPI->CR1 |= SPI_CR1_SPE;
//	else if(Encoder->source == EXTERNAL)
//		EXT_ENC_SPI->CR1 |= SPI_CR1_SPE;
	
	if (HAL_SPI_Init(enc_spi) != HAL_OK)
	{
		Error_Handler();
	}
	

}

static int ENC_SPI_Xfer16(Encoder_Source source, uint16_t tx, uint16_t *prx)
{
	uint32_t timeout = ENC_SPI_XFER_SPIN_MAX;

	if(source == ON_BOARD)
	{
		while ((BRD_ENC_SPI->SR & SPI_SR_TXE) == 0U)
		{
			if (--timeout == 0U)
				return -1;
		}

		BRD_ENC_SPI->DR = tx;

		timeout = ENC_SPI_XFER_SPIN_MAX;
		while ((BRD_ENC_SPI->SR & SPI_SR_RXNE) == 0U)
		{
			if (--timeout == 0U)
				return -2;
		}

		*prx = (uint16_t)BRD_ENC_SPI->DR;
	}
	else if(source == EXTERNAL)
	{
		while ((EXT_ENC_SPI->SR & SPI_SR_TXE) == 0U)
		{
			if (--timeout == 0U)
				return -1;
		}

		EXT_ENC_SPI->DR = tx;

		timeout = ENC_SPI_XFER_SPIN_MAX;
		while ((EXT_ENC_SPI->SR & SPI_SR_RXNE) == 0U)
		{
			if (--timeout == 0U)
				return -2;
		}

		*prx = (uint16_t)EXT_ENC_SPI->DR;
	}
	
	return 0;
}

/** 超时后如 RXNE 仍置位则读走数据；先读 SR 便于释放部分错误标志。 */
static void ENC_SPI_DrainIfRxPending(Encoder_Source source)
{
	if(source == ON_BOARD)
	{
		volatile uint32_t sr = BRD_ENC_SPI->SR;
		(void)sr;
		if ((BRD_ENC_SPI->SR & SPI_SR_RXNE) != 0)
			(void)BRD_ENC_SPI->DR;
	}
	else if(source == EXTERNAL)
	{
		volatile uint32_t sr = EXT_ENC_SPI->SR;
		(void)sr;
		if ((EXT_ENC_SPI->SR & SPI_SR_RXNE) != 0)
			(void)EXT_ENC_SPI->DR;	
	}
}

static void SPI2_MOSI_HiZ(void)
{
	GPIO_InitTypeDef g = {0};
	g.Pin = GPIO_PIN_15;
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &g);
}

static void SPI2_MOSI_Restore_AF(void)
{
	GPIO_InitTypeDef g = {0};
	g.Pin = GPIO_PIN_15;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &g);
}

/**
	* @brief  Read raw data of TLE5012B encoder
    * @retval Raw data of mechanical angle (0-32767)
 **/
uint16_t ReadTLE5012B_Raw(Encoder_Source source)
{
	uint16_t data_t[2] = {0x8021, 0x0000};
	uint16_t data_r[2];
	
	/*Register method, more efficient*/
//	if(source == ON_BOARD)
//	{	
//		BRD_ENC_CS_ENABLE;
//		state = ENC_SPI_Xfer16(source, data_t, &dummy);
//		if (state == 0)
//			state = ENC_SPI_Xfer16(source, 0x0000, &data_r);
//		BRD_ENC_CS_DISABLE;

//		if (state != 0)
//		{
//			ENC_SPI_DrainIfRxPending(source);
//			return 0;
//		}
//	}
//	else if(source == EXTERNAL)
//	{
//		EXT_ENC_CS_ENABLE;
//		state = ENC_SPI_Xfer16(source, data_t, &dummy);
//		if (state == 0)
//			state = ENC_SPI_Xfer16(source, 0x0000, &data_r);
//		EXT_ENC_CS_DISABLE;

//		if (state != 0)
//		{
//			ENC_SPI_DrainIfRxPending(source);
//			return 0;
//		}
//	}
	
	/*HAL method, less efficient*/
	if(source == ON_BOARD)
	{
		BRD_ENC_CS_ENABLE;
		HAL_SPI_Transmit(&brd_enc_spi, (uint8_t *)&data_t[0], 1, 10);
		SPI2_MOSI_HiZ();
		HAL_SPI_Receive(&brd_enc_spi, (uint8_t *)&data_r[1], 1, 10);
		SPI2_MOSI_Restore_AF();
		BRD_ENC_CS_DISABLE;
	}
	else if(source == EXTERNAL)
	{
		EXT_ENC_CS_ENABLE;
		HAL_SPI_TransmitReceive(&ext_enc_spi, (uint8_t *)data_t, (uint8_t *)data_r, 2, 10);
		EXT_ENC_CS_DISABLE;	
	}
	
	return (data_r[1] & 0x7FFF);
}

/**
	* @brief  Read raw data of MT6816 encoder
    * @retval Raw data of mechanical angle (0-16383)
 **/
uint16_t ReadMT6816_Raw(Encoder_Source source)
{
	uint16_t sample_data;		

	uint8_t pc_flag;	
	uint16_t angle = 0;
	
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	
	for(uint8_t i = 0; i < 3; i++)
	{
		/*Register method, more efficient*/
//		if(source == ON_BOARD)
//		{
//			BRD_ENC_CS_ENABLE;
//			while((BRD_ENC_SPI->SR & 1 << 1) == 0);
//			BRD_ENC_SPI->DR = data_t[0];
//			while((BRD_ENC_SPI->SR & 1 << 0) == 0);
//			data_r[0] = BRD_ENC_SPI->DR;
//			BRD_ENC_CS_DISABLE;
//			
//			BRD_ENC_CS_ENABLE;
//			while((BRD_ENC_SPI->SR & 1 << 1) == 0);
//			BRD_ENC_SPI->DR = data_t[1];
//			while((BRD_ENC_SPI->SR & 1 << 0) == 0);
//			data_r[1] = BRD_ENC_SPI->DR;		
//			BRD_ENC_CS_DISABLE;		
//		}
//		else if(source == EXTERNAL)
//		{
//			EXT_ENC_CS_ENABLE;
//			while((EXT_ENC_SPI->SR & 1 << 1) == 0);
//			EXT_ENC_SPI->DR = data_t[0];
//			while((EXT_ENC_SPI->SR & 1 << 0) == 0);
//			data_r[0] = EXT_ENC_SPI->DR;
//			EXT_ENC_CS_DISABLE;
//			
//			EXT_ENC_CS_ENABLE;
//			while((EXT_ENC_SPI->SR & 1 << 1) == 0);
//			EXT_ENC_SPI->DR = data_t[1];
//			while((EXT_ENC_SPI->SR & 1 << 0) == 0);
//			data_r[1] = EXT_ENC_SPI->DR;		
//			EXT_ENC_CS_DISABLE;		
//		}

		/*HAL method, less efficient*/
		if(source == EXTERNAL)
		{
			EXT_ENC_CS_ENABLE;
			HAL_SPI_TransmitReceive(&ext_enc_spi, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, 0x10);
			EXT_ENC_CS_DISABLE;
			EXT_ENC_CS_ENABLE;
			HAL_SPI_TransmitReceive(&ext_enc_spi, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, 0x10);
			EXT_ENC_CS_DISABLE;		
		}
		
		sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		h_count = 0;
		
		for(int j = 0; j < 16; j++)
		{
			if(sample_data & (0x0001 << j))
				h_count++;
		}
		if(h_count & 0x01){
			pc_flag = 0;
		}
		else{
			pc_flag = 1;
			break;
		}
	}
	if(pc_flag)
	{
		angle = sample_data >> 2;
	}
	
	return angle;
}

/**
	* @brief  Read raw data of MT6701 encoder
    * @param  Reg: Register address 
    * @retval Raw data of mechanical angle (0-16383)
 **/
uint16_t ReadMT6701_Raw(Encoder_Source source)
{
	uint16_t data_t;
	uint16_t data_r;
	
	data_t = 0x0000;
	
	/*Register method, more efficient*/
	if(source == EXTERNAL)
	{
		BRD_ENC_CS_ENABLE;
		while((BRD_ENC_SPI->SR & 1 << 1) == 0);
		BRD_ENC_SPI->DR = data_t;
		while((BRD_ENC_SPI->SR & 1 << 0) == 0);
		data_r = BRD_ENC_SPI->DR;
		BRD_ENC_CS_DISABLE;		
	}
	
	return (data_r & 0xFFFC) >> 2;
}

/**
	* @brief  Read raw data of encoder
    * @retval Raw data of mechanical angle
 **/
uint16_t ReadSPIEncoder_Raw(Encoder_TypeDef *Encoder)
{
	uint16_t encoder_raw;
	switch(Encoder->type)
	{
		case TLE5012B:
			encoder_raw = ReadTLE5012B_Raw(Encoder->source);
		break;
		
		case MT6816:
			encoder_raw = ReadMT6816_Raw(Encoder->source);
		break;
		
		case MT6701:
			encoder_raw = ReadMT6701_Raw(Encoder->source);
		break;
		
		default:break;
	}
	return encoder_raw;
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
	
	if(Encoder->dir == +1)
		Encoder->raw = ReadSPIEncoder_Raw(Encoder);
	else if(Encoder->dir == -1)
		Encoder->raw = Encoder->cpr - ReadSPIEncoder_Raw(Encoder);
	
	if(Encoder->raw == 0 || Encoder->raw == 1 || Encoder->raw == Encoder->cpr || Encoder->raw == Encoder->cpr - 1)
	{
		if(++ Encoder->disconnect_count >= 500)
		{
			if(MotorControl->ModeNow == Calib_EncoderOffset ||  
			   MotorControl->ModeNow == Current_Mode ||
  			   MotorControl->ModeNow == Speed_Mode)
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
	
    /* Linearization */
	int off_bit     = Encoder->resolution - 7;
    int off_1       = Encoder->offset_lut[(Encoder->raw) >> off_bit];             // lookup table lower entry
    int off_2       = Encoder->offset_lut[((Encoder->raw >> off_bit) + 1) % 128]; // lookup table higher entry
    int off_interp  = off_1
                       + ((off_2 - off_1) * (Encoder->raw - ((Encoder->raw >> off_bit) << off_bit))
                        >> off_bit); // Interpolate between lookup table entries
    int count = Encoder->raw - off_interp;
	
    /*  Wrap in ENCODER_CPR */
    while (count > Encoder->cpr)
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
    while (Encoder->pos_cpr_counts > Encoder->cpr)
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
    while (interpolated_enc > Encoder->cpr)
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

float Encoder_GetElePhase(Encoder_TypeDef *Encoder)
{
	return Encoder->theta_elec;
}

float Encoder_GetMecPos(Encoder_TypeDef *Encoder)
{
	return Encoder->theta_mech;
}

float Encoder_GetEleVel(Encoder_TypeDef *Encoder)
{
	return Encoder->vel_elec;
}

float Encoder_GetMecVel(Encoder_TypeDef *Encoder)
{
	return Encoder->vel_mech;
}

float Encoder_GetCountInCPR_Ratio(Encoder_TypeDef *Encoder)
{
	return Encoder->count_in_cpr * Encoder->one_by_cpr;
}

void Encoder_ChangeDetect(Encoder_TypeDef *Encoder1, Encoder_TypeDef *Encoder2)
{
	static Encoder_Type Encoder1_type_last;
	static Encoder_Type Encoder2_type_last;
	
	if(Encoder1->type != Encoder1_type_last)
	{
		Encoder_ParamInit(Encoder1);
	}
	if(Encoder2->type != Encoder2_type_last)
	{
		Encoder_ParamInit(Encoder2);
	}
	
	Encoder1_type_last = Encoder1->type;
	Encoder2_type_last = Encoder2->type;
}

void Task_Set_ZeroPosition(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	Encoder->zero_count = Encoder->count_in_cpr;
	Encoder->shadow_count = Encoder->zero_count;
	
	MotorControl->posTrajUpdated = true;
	
	MotorControl->ModeNow = Save_Param;
}


