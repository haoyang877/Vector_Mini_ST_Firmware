#include "foc_errhandle.h"

#include "tim.h"
#include "foc_algorithm.h"
#include "foc_pid.h"

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
extern PID_TypeDef PID_Speed;
extern PID_TypeDef PID_Position;
extern ModeNow_TypeDef  ModeLast;
extern ErrorNow_TypeDef ErrorLast;

bool is_Mode_Error_Change;

ModeNow_TypeDef Get_ModeNow(void)
{
	return MotorControl.ModeNow;
}

void Set_ModeNow(ModeNow_TypeDef tModeNow)
{
	MotorControl.ModeNow = tModeNow;
}

ErrorNow_TypeDef Get_ErroNow(void)
{
	return MotorControl.ErrorNow;
}

void Set_ErrorNow(ErrorNow_TypeDef tErrorNow)
{
	MotorControl.ErrorNow = tErrorNow;
}

void Clear_RunningData(void)
{
	MotorControl.idRef		 = 0.0f;
	MotorControl.iqRef		 = 0.0f;
	MotorControl.speedRef    = 0.0f;
	MotorControl.speedShadow = 0.0f;
	MotorControl.posShadow   = 0.0f;
	
	FOC.Id     = 0.0f;
	FOC.Iq     = 0.0f;
	FOC.Vd_int = 0.0f;
	FOC.Vq_int = 0.0f;
	
	PID_Speed.error		   = 0.0f;
	PID_Speed.error_sum    = 0.0f;
}

void ModeSwitch_Handle(ModeNow_TypeDef mode_set)
{
	if(MotorControl.ModeNow == Motor_Disable && MotorControl.ErrorNow == No_Error)
	{
		MotorControl.ModeNow = mode_set;
	}
	
	if((MotorControl.ModeNow  == Current_Mode || 
	    MotorControl.ModeNow  == Speed_Mode   || 
	    MotorControl.ModeNow  == Position_Mode ||
	    MotorControl.ModeNow == Calib_Motor_R_L_Flux ||
	    MotorControl.ModeNow == Calib_EncoderOffset) &&
	    MotorControl.ErrorNow == No_Error)
	{
		if(mode_set == Motor_Disable)
			MotorControl.ModeNow = mode_set;
	}
	
	if(mode_set == Clear_Error)
	{
		if(MotorControl.ErrorNow != No_Error)
		{
			MotorControl.ErrorNow = No_Error;
			MotorControl.ModeNow = Motor_Disable;
		}
	}
}

void Detect_Mode_Error_Change(void)
{
	if(ModeLast != MotorControl.ModeNow || ErrorLast != MotorControl.ErrorNow)
		is_Mode_Error_Change = true;
}

bool Return_Mode_Error_Change(void)
{
	return is_Mode_Error_Change;
}

void Clear_Mode_Error_Change(void)
{
	is_Mode_Error_Change = false;
}

void Stop_PWM_Generate(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	
	HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_3);
	
}

void Start_PWM_Generate(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);
}
