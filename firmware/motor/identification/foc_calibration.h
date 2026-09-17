#ifndef __FOC_CALIBRATION_H__
#define __FOC_CALIBRATION_H__

#include "main.h"
#include "foc_algorithm.h"
#include "data_type.h"
#include "encoder.h"
#include "foc_sensorless.h"

#define OFFSET_LUT_NUM              ENCODER_OFFSET_LUT_SIZE
#define MAX_MOTOR_POLE_PAIRS        20U
#define COGGING_MAP_NUM             5000U

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
	CS_ANTICOGGING_START,
	CS_ANTICOGGING_CW_TEMP,
	CS_ANTICOGGING_CW_SAMPLE,
	CS_ANTICOGGING_CCW_TEMP,
	CS_ANTICOGGING_CCW_SAMPLE,
	CS_ANTICOGGING_END,
	CS_OBS_ALIGN_ORIGIN,
	CS_OBS_WAIT_CLOSED_LOOP,
	CS_OBS_SPEED_STABLE,
	CS_OBS_FIND_ORIGIN,
	CS_OBS_SAMPLE_CW,
	CS_OBS_BUILD_LUT,
	CS_OBS_VERIFY_CW,
	CS_OBS_STOP_DECEL,
	CS_OBS_STOP_CURRENT,
	CS_ENC_OFFSET_ALIGN,
	CS_ENC_OFFSET_ALIGN_LOOP,
	CS_ENC_OFFSET_RAMP_CW,
	CS_ENC_OFFSET_SAMPLE_CW,
	CS_ENC_OFFSET_END
} CalibStep_TyepeDef;

void Task_Calib_R_L_Flux(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Task_Calib_EncoderOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
	Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver);
void Task_Calib_EncoderObserver(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
	PI_Controller_TypeDef *SpeedController, Encoder_TypeDef *Encoder,
	Fluxobserver_TypeDef *Fluxobserver, SensorlessStartup_TypeDef *Startup);
void Task_Calib_EleAngelOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Calib_CurrentOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);

#endif
