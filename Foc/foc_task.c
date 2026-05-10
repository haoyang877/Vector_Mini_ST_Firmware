#include "foc_task.h"

#include "common_inc.h"

MotorControl_TypeDef MotorControl;
PID_TypeDef PID_Speed;
Encoder_TypeDef OnBoard_Encoder,External_Encoder;
Fluxobserver_TypeDef Fluxobserver;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

void MotorControl_Init(void)
{
	External_Encoder.source = EXTERNAL;
	Encoder_ParamInit(&External_Encoder);
	OnBoard_Encoder.source = ON_BOARD;
	Encoder_ParamInit(&OnBoard_Encoder);
	
	Fluxobserver_ParamInit(&Fluxobserver);
	
	MotorControl.posTrajUpdated = true;
	MotorControl.pos_error_window = 0.001f;
	MotorControl.pos_Kp = 0.5f;
	MotorControl.pos_Kd = 0.1f;
}

/**
	* @brief  FOC task, motor control related
			  use finite state machine
 **/
void FOC20kHzIRQHandler(void)
{
	Vbus_Update(&FOC, &MotorControl);
	
	Current_Cal(&FOC, &MotorControl);
	
	Temperature_Update(&FOC);
	
	if(MotorControl.isUseSensorless == true)
	{
		Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);
	}
	else
	{
		Encoder_Update(&MotorControl, &External_Encoder);
		Encoder_Update(&MotorControl, &OnBoard_Encoder);	
	}

	switch(MotorControl.ModeNow)
	{
		case Motor_Disable:
			PWM_TurnOnHighSides();
		break;
		
		case Current_Mode:
			Task_Current_Mode(&FOC, &MotorControl, &External_Encoder, &Fluxobserver);
		break;
		
		case Speed_Mode:
			Task_Speed_Mode(&FOC, &MotorControl, &PID_Speed, &External_Encoder);
		break;
		
		case Position_Mode:
			Task_Position_Mode(&FOC, &MotorControl, &PID_Speed, &External_Encoder);
		break;
		
		case Calib_Motor_R_L_Flux:
			Task_Calib_R_L_Flux(&FOC, &MotorControl);
		break;
		
		case Calib_EncoderOffset:
			Task_Calib_Encoder(&FOC, &MotorControl, &External_Encoder);
		break;
		
		case Set_ZeroPosition:
			Task_Set_ZeroPosition(&MotorControl, &External_Encoder);
		break;
		
		case Default_Param:
			Param_Return_Default();
		
		case Clear_Error:
			Set_ErrorNow(No_Error);
		default:break;
	}
	
	/*no error*/
	if(MotorControl.ErrorNow == No_Error)
	{
		LED_SetState(0, (uint8_t)MotorControl.ModeNow);
	}
	/*error*/
	else
	{
		/*tolerant when writing paramters to flash*/
		if(MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param)
			MotorControl.ModeNow = Motor_Disable;
		
		LED_SetState(1, (uint8_t)MotorControl.ErrorNow);
	}
	
	if(ModeLast != Motor_Disable && MotorControl.ModeNow == Motor_Disable) 
	{
		Clear_RunningData();
		Stop_PWM_Generate();
	}
	
	if(ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable)
	{
		Start_PWM_Generate();
	}
	
	Detect_Mode_Error_Change();
	
	ModeLast  = MotorControl.ModeNow;
	ErrorLast = MotorControl.ErrorNow;
	
	MotorControl.ModeNow_f = MotorControl.ModeNow;
	MotorControl.ErrorNow_f = MotorControl.ErrorNow;
}