#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

#include "main.h"
#include <stdbool.h>

typedef enum
{
	Motor_Disable = 0,
	Current_Mode,
	Speed_Mode,
	Position_Mode,
	Calib_Motor_R_L_Flux,
	Calib_EncoderOffset,
	Calib_Anticogging,
	Set_ZeroPosition,
	Default_Param,
	Save_Param,
	Clear_Error,
	Calib_CurrentOffset,
	Voltage_OpenLoop,
	Calib_EncoderObserver,
	Calib_EleAngelOffset = 15,
	MODE_NUM
}ModeNow_TypeDef; 

typedef enum
{
	No_Error = 0,
	CurrentOffset_Error,
	Encoder_Error,
	PolePairs_Error,
	CAN_DisConnect,
	Large_Phase_Resistance,
	Large_Phase_Inductance,
	Over_Current,
	Over_Voltage,
	Under_Voltage,
	High_Temprature,
	Encoder_NotCalibrated,
	MotorParam_Error,
}ErrorNow_TypeDef;

typedef struct
{
	ModeNow_TypeDef  ModeNow;
	ErrorNow_TypeDef ErrorNow;
	
	float ModeNow_f;
	float ErrorNow_f;
	
	uint16_t A_Offset;
	uint16_t B_Offset;
	uint16_t C_Offset;
	
	int32_t motor_pole_pairs;
	float motor_phase_resistance;
	float motor_d_inductance;
	float motor_q_inductance;
	float motor_flux;
	
	bool isUseSensorless;
	
	float calib_current;
	float current_limit;
	float speed_limit;
	
	float idRef;
	float id_Kp;
	float id_Ki;
	float iqRef;
	float iq_Kp;
	float iq_Ki;
	
	bool isUseSpeedRamp;
	float speedAcc;
	float speedDec;
	float speedRef;
	float speedShadow;
	float speed_Kp;
	float speed_Ki;
	
	bool posTrajUpdated;
	bool isReachTargetPos;
	float pos_error_window;
	float posAcc;
	float posDec;
	float pos_maxspeed;
	float posRef;
	float posShadow;
	float pos_Kp;
	float pos_Kd;
	
	/*voltage open-loop mode*/
	float ol_voltage;	/*open-loop voltage amplitude (V, d-axis)*/
	float ol_elec_vel;	/*open-loop electrical angular velocity (rad/s)*/
	float ol_theta;		/*open-loop electrical angle (rad)*/
	
}MotorControl_TypeDef;

#endif
