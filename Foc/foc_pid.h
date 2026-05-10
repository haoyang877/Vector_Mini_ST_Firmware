#ifndef __FOC_PID_H__
#define __FOC_PID_H__

typedef struct
{
	/*reference value,feedback value*/
	float ref_value,fbk_value;
	/*Kp Ki*/
	float Kp,Ki;
	/*error now,error last,error_summation*/
	float error,error_last,error_sum;
	/*the maximum value of PID output(absolute)*/
	float output_max;
	/*PID output*/
	float output;
}PID_TypeDef;

float Speed_PI_Ctrl(PID_TypeDef *PID);
float Position_P_Ctrl(PID_TypeDef *PID);
void Clear_PID_Param(PID_TypeDef *PID);

#endif