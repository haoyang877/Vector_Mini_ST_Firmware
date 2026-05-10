#ifndef __HW_CONF_H__
#define __HW_CONF_H__

#include "main.h"

#define PWM_TIM_CLOCK			170000000
#define PWM_TIM_FREQ			20000 /*Hz*/
#define PWM_TIM_PERIOD			(PWM_TIM_CLOCK / PWM_TIM_FREQ / 2)	
#define PWM_PERIOD				5e-5f

#define FOC_FREQ				PWM_TIM_FREQ
#define FOC_PERIOD				1.0f / (float)FOC_FREQ

/*shunt resistor (ohm)*/
#define SENSING_RES				0.002f
/*current amplify gain (V/A)*/
#define CURRENT_AMP_GAIN		10.0f
/*current sensing factor (adc value/A)*/
#define SENSING_CURR_FACTOR		(float)(3.3f / 4095.0f / CURRENT_AMP_GAIN / SENSING_RES)

/*bus voltagge R1 R2 (kohm)*/
#define VBUS_R1					10.0f
#define VBUS_R2					1.0f
/*bus voltage sensing factor (adc value/V)*/
#define SENSING_VBUS_FACTOR    	(float)(3.3f / 4095.0f * (VBUS_R1 + VBUS_R2) / VBUS_R2)

#define TEMP_R2					3.3f

/*MOSFET approximate deadtime (s)*/
#define MOS_DEADTIME			2.1e-7f
/*voltage of body diode (V)*/
#define MOS_VDIODE				0.5f

#define DBC_FACTOR				MOS_DEADTIME / PWM_PERIOD / 1000000000.0f

#define CURRENT_ADC				ADC2
#define IA_ADC_CHANNEL			JDR1
#define IB_ADC_CHANNEL			JDR2
#define IC_ADC_CHANNEL			JDR3

#define VBUS_ADC				ADC2
#define VBUS_ADC_CHANNEL		JDR4

#define TEMP_ADC				ADC1
#define TEMP_ADC_CHANNEL		JDR1

/*current loop period (s)*/
#define Current_Ts	0.00005f
/*speed loop period (s)*/
#define Speed_Ts	0.0001f
/*position loop period (s)*/
#define Position_Ts 0.0002f

#endif
