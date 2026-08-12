#ifndef __HW_CONF_H__
#define __HW_CONF_H__

#include "main.h"

#define PWM_TIM_CLOCK			170000000
#define PWM_TIM_FREQ			20000 /*Hz*/
#define PWM_TIM_PERIOD			(PWM_TIM_CLOCK / PWM_TIM_FREQ / 2)	
#define PWM_PERIOD				5e-5f

#define FOC_FREQ				PWM_TIM_FREQ
#define FOC_PERIOD				(1.0f / (float)FOC_FREQ)

/*RTT output sampling frequency; must divide FOC_FREQ exactly*/
#define RTT_SAMPLE_RATE_HZ		2000U

#if RTT_SAMPLE_RATE_HZ == 0U
#error "RTT_SAMPLE_RATE_HZ must be greater than zero"
#elif RTT_SAMPLE_RATE_HZ > FOC_FREQ
#error "RTT_SAMPLE_RATE_HZ must not exceed FOC_FREQ"
#elif (FOC_FREQ % RTT_SAMPLE_RATE_HZ) != 0U
#error "RTT_SAMPLE_RATE_HZ must divide FOC_FREQ exactly"
#endif

#define RTT_SAMPLE_DIVIDER		(FOC_FREQ / RTT_SAMPLE_RATE_HZ)

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

/* Current loop executes at the PWM/FOC rate. */
#define Current_Ts                  (FOC_PERIOD)

/* All speed PI controllers and encoder velocity estimation run at 2 kHz. */
#define SPEED_LOOP_FREQ             2000U
#if SPEED_LOOP_FREQ == 0U
#error "SPEED_LOOP_FREQ must be greater than zero"
#elif SPEED_LOOP_FREQ > FOC_FREQ
#error "SPEED_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % SPEED_LOOP_FREQ) != 0U
#error "SPEED_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define SPEED_LOOP_DIVIDER          (FOC_FREQ / SPEED_LOOP_FREQ)
#define Speed_Ts                    (1.0f / (float)SPEED_LOOP_FREQ)

/* Position trajectory update period (s). */
#define Position_Ts                 0.0002f

/* Sensorless speed-mode startup and observer handoff. */
#define SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S       0.50f
#define SENSORLESS_ALIGN_HOLD_TIME_S               0.30f
#define SENSORLESS_ALIGN_TIME_S                    (SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S + SENSORLESS_ALIGN_HOLD_TIME_S)
#define SENSORLESS_ALIGN_CURRENT_A                  2.0f
#define SENSORLESS_STARTUP_IQ_INITIAL_A             0.2f
#define SENSORLESS_STARTUP_IQ_A                     1.2f
#define SENSORLESS_STARTUP_IQ_RAMP_TIME_S           1.00f
#define SENSORLESS_STARTUP_ID_A                     0.5f
#define SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S      250.0f
#define SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S   250.0f
#define SENSORLESS_STARTUP_RAMP_TIME_S             1.00f
#define SENSORLESS_STARTUP_ELEC_ACCEL_RAD_S2       (SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S / SENSORLESS_STARTUP_RAMP_TIME_S)
#define SENSORLESS_STARTUP_SPEED_LOCK_TIME_S       0.20f
#define SENSORLESS_OBSERVER_LOCK_RATIO             0.25f
#define SENSORLESS_ANGLE_HANDOFF_TIME_S            0.10f
#define SENSORLESS_STARTUP_LOCK_TIMEOUT_S          3.00f
#define SENSORLESS_ID_RAMP_DOWN_TIME_S             0.50f
#define SENSORLESS_OBSERVER_LOSS_TIME_S            0.20f
#define SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S     5000.0f
#define SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA         0.1042f

#endif
