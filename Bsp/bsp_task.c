#include "bsp_task.h"

uint16_t Led_Cnt;
uint16_t RGB_Cnt;
uint16_t EncoderChange_Cnt;
uint16_t CANBRSwitching_Cnt;

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder,External_Encoder;

void BSP1kHzIRQHandler(void)
{
	USB_PrintProfile();
	
	if(++Led_Cnt >= 200)
	{
		LED_Task();
		Led_Cnt=0;
	}
		
	if(++RGB_Cnt >= 50)
	{
		switch(MotorControl.ModeNow)
		{
			case Current_Mode:
				Set_RGB_BreathingColor(GREEN);
			break;
			
			case Speed_Mode:
				Set_RGB_BreathingColor(CYAN);
			break;
			
			case Position_Mode:
				Set_RGB_BreathingColor(BLUE);
			break;
			
			case Calib_Motor_R_L_Flux:
				Set_RGB_BreathingColor(PURPLE);
			break;
			
			case Calib_EncoderOffset:
				Set_RGB_BreathingColor(PURPLE);
			break;
			
			default:
				Set_RGB_BreathingColor(COLOR_NULL);
			break;
		}
		
		RGB_Cnt = 0;
	}
	
	if(++EncoderChange_Cnt >= 100)
	{
		Encoder_ChangeDetect(&OnBoard_Encoder, &External_Encoder);
		EncoderChange_Cnt = 0;
	}
	
	if(++CANBRSwitching_Cnt >= 100)
	{
		CAN_BaudRateSwitching();
		CANBRSwitching_Cnt = 0;
	}
	
	CAN_DisConnect_Handle();
}