#include "board_config.h"

/**
	* @brief  Initialize board peripherals and application modules
 **/
void Board_Init(void)
{
	/*read parameters and calibration data from flash*/
	/*if magic word invalid or not calibrated, fall back to code defaults*/
	flash_read_param();
	
	/*motor control related parameters initialize*/
	MotorControl_Init();
	
	/*delay function init*/
	delay_init(170);
	
	/*ADC inner calibration*/
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	
	/*enable three phase PWM output*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);

	/*enable channel 4 PWM to trigger ADC conversion*/
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	/*enable ADC1 ADC 2 injection mode sampling*/
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	
	/*enable ADC 2 injection mode sampling*/
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
		
	/*enable TIM7 interrupt*/
	HAL_TIM_Base_Start_IT(&htim7);
	
	/*CAN1 filter init*/
	FDCAN1_Param_Init();
}
