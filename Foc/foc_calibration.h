#ifndef __FOC_CALIBRATION_H__
#define __FOC_CALIBRATION_H__

#include "main.h"

#include "foc_algorithm.h"
#include "data_type.h"
#include "encoder.h"
#include "foc_sensorless.h"

#define OFFSET_LUT_NUM   		ENCODER_OFFSET_LUT_SIZE

#define MAX_MOTOR_POLE_PAIRS 	20U
#define SAMPLES_PER_PPAIR    	128U

#define COGGING_MAP_NUM		 	5000U

typedef enum 
{
	CS_NULL = 0,
	
	CS_ADC_OFFSET_START,
	CS_ADC_OFFSET_LOOP,
	CS_ADC_OFFSET_END,
	
	CS_MOTOR_R_START,
	CS_MOTOR_RA_LOOP,
	CS_MOTOR_RB_LOOP,
	CS_MOTOR_RC_LOOP,
	CS_MOTOR_R_END,
	
	CS_MOTOR_L_START,
	CS_MOTOR_LD_LOOP,
	CS_MOTOR_LQ_LOOP,
	CS_MOTOR_L_END,
	
	CS_MOTOR_FLUX_START,
	CS_MOTOR_FLUX_LOOP,
	CS_MOTOR_FLUX_END,
	
	CS_DIR_START,
	CS_DIR_LOOP,
	CS_DIR_END,
	
	CS_ENCODER_START,
	CS_ENCODER_CW_LOOP,
	CS_ENCODER_CCW_LOOP,
	CS_ENCODER_END,
	
	CS_ANTICOGGING_START,
	CS_ANTICOGGING_CW_TEMP,
	CS_ANTICOGGING_CW_SAMPLE,
	CS_ANTICOGGING_CCW_TEMP,
	CS_ANTICOGGING_CCW_SAMPLE,
	CS_ANTICOGGING_END,
	
	/*observer-based encoder linearization calibration*/
	CS_OBS_ALIGN,
	CS_OBS_ALIGN_LOOP,
	CS_OBS_RAMP_CW,
	CS_OBS_SAMPLE_CW,
	CS_OBS_RAMP_CCW,
	CS_OBS_SAMPLE_CCW,
	CS_OBS_END,
	
}CalibStep_TyepeDef;

void Task_Calib_R_L_Flux(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Task_Calib_Encoder(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Calib_EncoderObserver(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver);
void Task_Calib_EleAngelOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Calib_CurrentOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);

#endif
