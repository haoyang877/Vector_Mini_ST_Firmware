#include "foc_sensing.h"

#include "adc.h"
#include <math.h>
#include "hw_conf.h"
#include "utils.h"
#include "foc_errhandle.h"

#define OVERCURRENT_CONFIRM_CYCLES 5U

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;

//#pragma arm section code = "CCMRAMCODE"

/**
	* @brief  Bus voltage sensing and calculation
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	static int overvoltage_count,undervoltage_count;
	
	FOC->Vbus = (float)(VBUS_ADC->VBUS_ADC_CHANNEL) * SENSING_VBUS_FACTOR;
	
	UTILS_LP_FAST(FOC->Vbus_filt, FOC->Vbus, 0.05f);
	
	if(MotorControl->ModeNow == Current_Mode || 
	   MotorControl->ModeNow == Speed_Mode ||
	   MotorControl->ModeNow == Position_Mode ||
	   MotorControl->ModeNow == Calib_Motor_R_L_Flux ||
	   MotorControl->ModeNow == Calib_EncoderOffset ||
	   MotorControl->ModeNow == Vq_Mode ||
	   MotorControl->ModeNow == Sensorless_Speed_Mode)
	{
		/*over voltage protect*/
		if(FOC->Vbus_filt > 30.0f)
		{
			if(++overvoltage_count >= 10000)
			{
				Set_ErrorNow(Over_Voltage);
			}
			overvoltage_count = 0;
		}
		else
		{
			overvoltage_count = 0;
		}
			
		/*under voltage protect*/
		if(FOC->Vbus_filt < 10.0f)
		{
			if(++undervoltage_count >= 10000)
			{
				Set_ErrorNow(Under_Voltage);
				undervoltage_count = 0;
			}
		}
		else
		{
			undervoltage_count = 0;
		}
	}
}

/**
	* @brief  Three phase current sensing and calculating
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void Current_Cal(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	static uint8_t overcurrent_count;

	/*when actual current is near zero, adc offset is outght to be around 2048*/
	if(MotorControl->A_Offset < 1948 || MotorControl->A_Offset > 2148 ||
	   MotorControl->B_Offset < 1948 || MotorControl->B_Offset > 2148 ||
	   MotorControl->C_Offset < 1948 || MotorControl->C_Offset > 2148	)
	{
		Set_ErrorNow(CurrentOffset_Error);
	}
	
	else
	{	
		FOC->Ia = -((float)((int16_t)CURRENT_ADC->IA_ADC_CHANNEL - MotorControl->A_Offset)) * SENSING_CURR_FACTOR;
		FOC->Ib = -((float)((int16_t)CURRENT_ADC->IB_ADC_CHANNEL - MotorControl->B_Offset)) * SENSING_CURR_FACTOR;
		FOC->Ic = -((float)((int16_t)CURRENT_ADC->IC_ADC_CHANNEL - MotorControl->C_Offset)) * SENSING_CURR_FACTOR;
	}
	
	float i_limit = MotorControl->current_limit + 10.0f;
	
	if(fast_abs(FOC->Ia) > i_limit || fast_abs(FOC->Ib) > i_limit || fast_abs(FOC->Ic) > i_limit)
	{
		if(overcurrent_count < OVERCURRENT_CONFIRM_CYCLES)
			overcurrent_count++;
		if(overcurrent_count >= OVERCURRENT_CONFIRM_CYCLES)
			Set_ErrorNow(Over_Current);
	}
	else
	{
		overcurrent_count = 0U;
	}
}

/**
	* @brief  Temperature sensing and protection
	* @param  *FOC: FOC struct pointer
 **/
void Temperature_Update(FOC_TypeDef *FOC)
{
	static float count;
	
	/*from NTC datasheet*/
	const float B = 3455.0f;
	float R2 = 10.0f;
	float T2 = 25.0f;
	
	uint32_t adc_val;
	float R1;

	if(++count >= 20)
	{
		adc_val = TEMP_ADC->TEMP_ADC_CHANNEL;
		
		R1 = (4095.0f / (float)adc_val - 1.0f) * TEMP_R2;
		
		FOC->temp = (1.0f / ((1.0f / B) * logf(R1 / R2) + (1.0f / (T2 + 273.15f))) - 273.15f);
		
		count = 0;
	}
	
	if(FOC->temp >= 100.0f)
	{
		Set_ErrorNow(High_Temprature);
	}
}
//#pragma arm section
