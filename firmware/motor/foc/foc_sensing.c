#include "foc_sensing.h"

#include "adc.h"
#include <math.h>
#include "hw_conf.h"
#include "utils.h"
#include "foc_errhandle.h"
#include "bus_voltage_profile.h"
#include "mcu_temperature.h"
#include "stm32g4xx_ll_adc.h"

#define OVERCURRENT_CONFIRM_CYCLES 5U

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
volatile McuTemperatureTelemetry McuTemperature;

//#pragma arm section code = "CCMRAMCODE"

/**
	* @brief  Bus voltage sensing and calculation
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	static uint32_t overvoltage_count, undervoltage_count, hard_overvoltage_count;
	
	FOC->Vbus = (float)(VBUS_ADC->VBUS_ADC_CHANNEL) * ADC2_SUM_TO_COUNTS * SENSING_VBUS_FACTOR;
	
	UTILS_LP_FAST(FOC->Vbus_filt, FOC->Vbus, 0.05f);
	
	if(MotorControl->ModeNow == Current_Mode || 
	   MotorControl->ModeNow == Speed_Mode ||
	   MotorControl->ModeNow == Position_Mode ||
	   MotorControl->ModeNow == Position_Impedance_Mode ||
	   MotorControl->ModeNow == Calib_Motor_R_L_Flux ||
	   MotorControl->ModeNow == Calib_PhaseResistance ||
	   MotorControl->ModeNow == Calib_EncoderOffset ||
	   MotorControl->ModeNow == Calib_EncoderObserver ||
	   MotorControl->ModeNow == Calib_EleAngelOffset ||
	   MotorControl->ModeNow == Vq_Mode ||
	   MotorControl->ModeNow == Voltage_OpenLoop ||
	   MotorControl->ModeNow == Sensorless_Speed_Mode ||
	   MotorControl->ModeNow == Calib_Anticogging ||
	   MotorControl->ModeNow == Calib_Friction)
	{
		/* Keep the first fault latched. Raw samples bypass the LPF near the
		 * hardware voltage ceiling; filtered thresholds reject short dips. */
		if (MotorControl->ErrorNow != No_Error)
			return;
		if (!isfinite(FOC->Vbus) || !isfinite(FOC->Vbus_filt))
		{
			Set_ErrorNow(Over_Voltage);
			return;
		}
		if (FOC->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
		{
			if (hard_overvoltage_count < BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
				++hard_overvoltage_count;
			if (hard_overvoltage_count >= BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
			{
				Set_ErrorNow(Over_Voltage);
				return;
			}
		}
		else
			hard_overvoltage_count = 0U;
		if(FOC->Vbus_filt >= BUS_VOLTAGE_OVERVOLTAGE_V)
		{
			if (overvoltage_count < (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
				++overvoltage_count;
			if(overvoltage_count >= (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
			{
				Set_ErrorNow(Over_Voltage);
			}
		}
		else
		{
			overvoltage_count = 0;
		}
			
		/*under voltage protect*/
		if(FOC->Vbus_filt <= BUS_VOLTAGE_UNDERVOLTAGE_V)
		{
			if (undervoltage_count < (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
				++undervoltage_count;
			if(undervoltage_count >= (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
			{
				Set_ErrorNow(Under_Voltage);
			}
		}
		else
		{
			undervoltage_count = 0;
		}
	}
	else
	{
		overvoltage_count = 0U;
		undervoltage_count = 0U;
		hard_overvoltage_count = 0U;
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
	if(!isfinite(MotorControl->A_Offset) || !isfinite(MotorControl->B_Offset) || !isfinite(MotorControl->C_Offset) ||
	   MotorControl->A_Offset < 1948 || MotorControl->A_Offset > 2148 ||
	   MotorControl->B_Offset < 1948 || MotorControl->B_Offset > 2148 ||
	   MotorControl->C_Offset < 1948 || MotorControl->C_Offset > 2148	)
	{
		Set_ErrorNow(CurrentOffset_Error);
	}
	
	else
	{	
		FOC->Ia = -((float)CURRENT_ADC->IA_ADC_CHANNEL * ADC2_SUM_TO_COUNTS - MotorControl->A_Offset) * SENSING_CURR_FACTOR;
		FOC->Ib = -((float)CURRENT_ADC->IB_ADC_CHANNEL * ADC2_SUM_TO_COUNTS - MotorControl->B_Offset) * SENSING_CURR_FACTOR;
		FOC->Ic = -((float)CURRENT_ADC->IC_ADC_CHANNEL * ADC2_SUM_TO_COUNTS - MotorControl->C_Offset) * SENSING_CURR_FACTOR;
	}
	
	if(fast_abs(FOC->Ia) > CURRENT_OVERCURRENT_TRIP_A ||
	   fast_abs(FOC->Ib) > CURRENT_OVERCURRENT_TRIP_A ||
	   fast_abs(FOC->Ic) > CURRENT_OVERCURRENT_TRIP_A)
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
    float celsius, vdda;
    bool good = false;
    /* Software-triggered ADC1 sequence completes between 1 kHz supervisor
     * calls. Read both ranks only on JEOS, then request the next sequence.
     * ADC2/PWM current sampling and the 20 kHz IRQ are unaffected. */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS)) {
        McuTemperature.raw_ts = ADC1->JDR1;
        McuTemperature.raw_vref = ADC1->JDR2;
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
        good = McuTemperature_Convert(McuTemperature.raw_ts, McuTemperature.raw_vref,
            *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR, *VREFINT_CAL_ADDR, &celsius, &vdda);
        if (good) {
            FOC->temp = McuTemperature.valid ? FOC->temp + 0.02f * (celsius - FOC->temp) : celsius;
            McuTemperature.raw_celsius = celsius;
            McuTemperature.vdda_mv = vdda;
            McuTemperature.valid = 1U;
            McuTemperature.missed_ms = 0U;
            ++McuTemperature.sample_count;
            /* 90 C leaves 15 C below even suffix-6's 105 C junction limit,
             * allowing calibration/sensing error. No winding/MOSFET coverage. */
            if (celsius >= MCU_TEMPERATURE_TRIP_C && MotorControl.ErrorNow == No_Error)
                Set_ErrorNow(High_Temprature);
        } else {
            McuTemperature.valid = 0U;
            FOC->temp = NAN;
        }
    }
    if (!good) {
        if (McuTemperature.missed_ms < MCU_TEMPERATURE_TIMEOUT_MS) ++McuTemperature.missed_ms;
        if (!McuTemperature.valid || McuTemperature.missed_ms >= MCU_TEMPERATURE_TIMEOUT_MS) {
            McuTemperature.valid = 0U;
            FOC->temp = NAN;
        }
        if (McuTemperature.missed_ms >= MCU_TEMPERATURE_TIMEOUT_MS && MotorControl.ErrorNow == No_Error)
            Set_ErrorNow(TemperatureSensor_Error);
    }
    if (!LL_ADC_INJ_IsConversionOngoing(ADC1)) LL_ADC_INJ_StartConversion(ADC1);
}
//#pragma arm section
