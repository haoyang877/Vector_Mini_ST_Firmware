#include "foc_pid.h"

#include "utils.h"
#include "hw_conf.h"

/**
	* @brief  Speed loop PI control 
    * @param  *PID: PID struct pointer
	* @retval output value of PI calculation
 **/
float Speed_PI_Ctrl(PID_TypeDef *PID)
{
	PID->error = PID->ref_value - PID->fbk_value;
	
	PID->error_sum += PID->error * PID->Ki * Speed_Ts;
	
	PID->error_sum = constrain(PID->error_sum, -1.0f, 1.0f);
	
	/*disable integral once error is too large to reduce overshot*/
	if(PID->error > 0.7f * PID->ref_value || PID->error < -0.7f * PID->ref_value)
		PID->error_sum = 0.0f;
	
	PID->output = PID->Kp * PID->error + PID->error_sum;
	
	PID->output = constrain(PID->output, -1.0f, 1.0f);
	
	return PID->output;
}

/**
	* @brief  Position loop P control 
    * @param  *PID: PID struct pointer
	* @retval output value of PI calculation
 **/
float Position_P_Ctrl(PID_TypeDef *PID)
{
	PID->error = PID->ref_value - PID->fbk_value;
	
	PID->error_sum += PID->error * PID->Ki * Position_Ts;
	
	PID->error_sum = constrain(PID->error_sum, -1.0f, 1.0f);
	
	PID->output = PID->Kp * PID->error;
	
	PID->output= constrain(PID->output, -1.0f, 1.0f);

	return PID->output * PID->output_max;
}

/**
	* @brief  Clear running PID parameter
    * @param  *PID: PID struct pointer
 **/
void Clear_PID_Param(PID_TypeDef *PID)
{
	PID->ref_value = 0.0f;
	PID->error = 0.0f;
	PID->output = 0.0f;
	PID->error_sum = 0.0f;
}