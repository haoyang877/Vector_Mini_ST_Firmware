#include "foc_algorithm.h"

#include <stdbool.h>
#include "utils.h"
#include "hw_conf.h"

#define FOC_MAX_MODULATION (0.95f * SQRT_3_BY_2)

static bool FOC_SetVoltageModulation(FOC_TypeDef *FOC, float voltage_d, float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (FOC->Vbus_filt <= 0.0f)
    {
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / FOC->Vbus_filt;
    FOC->mod_d = voltage_to_modulation * voltage_d;
    FOC->mod_q = voltage_to_modulation * voltage_q;
    modulation_magnitude = fast_sqrt(FOC->mod_d * FOC->mod_d + FOC->mod_q * FOC->mod_q);

    if (modulation_magnitude > FOC_MAX_MODULATION)
    {
        float modulation_scale = FOC_MAX_MODULATION / modulation_magnitude;
        FOC->mod_d *= modulation_scale;
        FOC->mod_q *= modulation_scale;
        modulation_magnitude = FOC_MAX_MODULATION;
    }

    if (modulation_magnitude > 0.0f)
    {
        FOC->duty = sign_hard(FOC->mod_q) * modulation_magnitude / FOC_MAX_MODULATION;
    }
    else
    {
        FOC->duty = 0.0f;
    }

    return true;
}

//#pragma arm section code = "CCMRAMCODE"
/**
	* @brief  Clarke transform
			  abc coordinate to alpha-beta coordinate
	* @param  a
	* @param  b
	* @param  c
	* @param  *alpha
	* @param  *beta
 **/
void Clarke_Transform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta  = (b - c) * ONE_BY_SQRT_3;
}

/**
	* @brief  Inverse clarke transform
			  alpha-beta coordinate to abc coordinate
	* @param  alpha
	* @param  beta
	* @param  *a
	* @param  *b
	* @param  *c
 **/
void Inverse_Clarke_Transform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + SQRT_3_BY_2 * beta;
    *c = -0.5f * alpha - SQRT_3_BY_2 * beta;
}

/**
	* @brief  Park transform  
	          alpha-beta coordinate to d-q coordinate 
	* @param  alpha
	* @param  beta
	* @param  theta
	* @param  *d
	* @param  *q
 **/
void Park_Transform(float alpha, float beta, float theta, float *d, float *q)
{
	float sin = fast_sin(theta);
	float cos = fast_cos(theta);
	
 	*d =  alpha * cos + beta * sin;
	*q = -alpha * sin + beta * cos;
}

/**
	* @brief  Inverse park transform
			  d-q coordinate to alpha-beta coordinate 
	* @param  d
	* @param  q
	* @param  theta
	* @param  *alpha
	* @param  *beta
 **/
void Inverse_Park_Transform(float d, float q, float theta, float *alpha, float *beta)
{
	float sin = fast_sin(theta);
	float cos = fast_cos(theta);
	
	*alpha = d * cos - q * sin;
	*beta  = d * sin + q * cos;
}

/**
	* @brief  Space vector modulation 
			  sector judgement method
	* @param  a
	* @param  b
	* @param  c
	* @param  *alpha
	* @param  *beta
 **/
void SVM_SectorJudge(float alpha, float beta, float *tA, float *tB, float *tC ,int32_t *sector)
{
    if (beta >= 0.0f) 
	{
        if (alpha >= 0.0f) 
		{
            //quadrant I
            if (ONE_BY_SQRT_3 * beta > alpha)
                *sector = 2; //*sector v2-v3
            else
                *sector = 1; //*sector v1-v2
        } 
		else 
		{
            //quadrant II
            if (-ONE_BY_SQRT_3 * beta > alpha)
                *sector = 3; //*sector v3-v4
            else
                *sector = 2; //*sector v2-v3
        }
    } 
	else 
	{
        if (alpha >= 0.0f) 
		{
            //quadrant IV
            if (-ONE_BY_SQRT_3 * beta > alpha)
                *sector = 5; //*sector v5-v6
            else
                *sector = 6; //*sector v6-v1
        } 
		else 
		{
            //quadrant III
            if (ONE_BY_SQRT_3 * beta > alpha)
                *sector = 4; //*sector v4-v5
            else
                *sector = 5; //*sector v5-v6
        }
    }

    switch (*sector) 
	{
		// *sector v1-v2
		case 1: 
		{
			// Vector on-times
			float t1 = alpha - ONE_BY_SQRT_3 * beta;
			float t2 = TWO_BY_SQRT_3 * beta;

			// PWM timings
			*tA = (1.0f - t1 - t2) * 0.5f;
			*tB = *tA + t1;
			*tC = *tB + t2;
		} 
		break;

		// *sector v2-v3
		case 2: 
		{
			// Vector on-times
			float t2 =  alpha + ONE_BY_SQRT_3 * beta;
			float t3 = -alpha + ONE_BY_SQRT_3 * beta;

			// PWM timings
			*tB = (1.0f - t2 - t3) * 0.5f;
			*tA = *tB + t3;
			*tC = *tA + t2;
		} 
		break;

		// *sector v3-v4
		case 3: 
		{
			// Vector on-times
			float t3 = TWO_BY_SQRT_3 * beta;
			float t4 = -alpha - ONE_BY_SQRT_3 * beta;

			// PWM timings
			*tB = (1.0f - t3 - t4) * 0.5f;
			*tC = *tB + t3;
			*tA = *tC + t4;
		} 
		break;

		// *sector v4-v5
		case 4: 
		{
			// Vector on-times
			float t4 = -alpha + ONE_BY_SQRT_3 * beta;
			float t5 = -TWO_BY_SQRT_3 * beta;

			// PWM timings
			*tC = (1.0f - t4 - t5) * 0.5f;
			*tB = *tC + t5;
			*tA = *tB + t4;
		} 
		break;

		// *sector v5-v6
		case 5: 
		{
			// Vector on-times
			float t5 = -alpha - ONE_BY_SQRT_3 * beta;
			float t6 =  alpha - ONE_BY_SQRT_3 * beta;

			// PWM timings
			*tC = (1.0f - t5 - t6) * 0.5f;
			*tA = *tC + t5;
			*tB = *tA + t6;
		} 
		break;

		// *sector v6-v1
		case 6: 
		{
			// Vector on-times
			float t6 = -TWO_BY_SQRT_3 * beta;
			float t1 = alpha + ONE_BY_SQRT_3 * beta;

			// PWM timings
			*tA = (1.0f - t6 - t1) * 0.5f;
			*tC = *tA + t1;
			*tB = *tC + t6;
		} 
		break;
    }
}

/**
	* @brief  Set duty cycle of phase A
	* @param  duty: duty cycle of phase A 0-1
 **/
void Set_A_Duty(float duty)
{
	TIM1->CCR1 = (uint16_t)(duty * PWM_TIM_PERIOD);
}

/**
	* @brief  Set duty cycle of phase B
	* @param  duty: duty cycle of phase B 0-1
 **/
void Set_B_Duty(float duty)
{
	TIM1->CCR2 = (uint16_t)(duty * PWM_TIM_PERIOD);
}

/**
	* @brief  Set duty cycle of phase C
	* @param  duty: duty cycle of phase C 0-1
 **/
void Set_C_Duty(float duty)
{
	TIM1->CCR3 = (uint16_t)(duty * PWM_TIM_PERIOD);
}

/**
	* @brief  Voltage loop 
    * @param  *FOC: FOC struct pointer
	* @param  Vd_set: voltage set in d axis 
	* @param  Vq_set: voltage set in q axis
	* @param  phase: electrical angle
 **/
void FOC_Voltage(FOC_TypeDef *FOC, float Vd_set, float Vq_set, float phase)
{
    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC_SetVoltageModulation(FOC, Vd_set, Vq_set);

    Inverse_Park_Transform(FOC->mod_d, FOC->mod_q, phase, &FOC->mod_alpha, &FOC->mod_beta);
    SVM_SectorJudge(FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    Set_A_Duty(FOC->dtc_a);
    Set_B_Duty(FOC->dtc_b);
    Set_C_Duty(FOC->dtc_c);
}

/**
	* @brief  Current loop 
    * @param  *FOC: FOC struct pointer
    * @param  *MotorControl: MotorControl struct pointer
	* @param  phase: electrical angle
	* @param  phase_vel: electrical angular velocity 
 **/
void FOC_Current(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    if (FOC->Vbus_filt > 0.0f)
    {
        max_voltage = FOC_MAX_MODULATION * FOC->Vbus_filt / 1.5f;
        PI_Controller_Configure(&FOC->id_pi, MotorControl->id_Kp, MotorControl->id_Ki, Current_Ts, -max_voltage, max_voltage);
        PI_Controller_Configure(&FOC->iq_pi, MotorControl->iq_Kp, MotorControl->iq_Ki, Current_Ts, -max_voltage, max_voltage);

        voltage_d = PI_Controller_Run(&FOC->id_pi, MotorControl->idRef, FOC->Id);
        voltage_q = PI_Controller_Run(&FOC->iq_pi, MotorControl->iqRef, FOC->Iq);
        FOC_SetVoltageModulation(FOC, voltage_d, voltage_q);

        PI_Controller_TrackOutput(&FOC->id_pi, FOC->mod_d * FOC->Vbus_filt / 1.5f);
        PI_Controller_TrackOutput(&FOC->iq_pi, FOC->mod_q * FOC->Vbus_filt / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(FOC);
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
    }

    Inverse_Park_Transform(FOC->mod_d, FOC->mod_q, phase + phase_vel * Current_Ts, &FOC->mod_alpha, &FOC->mod_beta);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC->Ibus = FOC->mod_d * FOC->Id + FOC->mod_q * FOC->Iq;
    UTILS_LP_FAST(FOC->Ibus_filt, FOC->Ibus, 0.01f);
    FOC->Power_filt = FOC->Vbus_filt * FOC->Ibus_filt;

    SVM_SectorJudge(FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    Set_A_Duty(FOC->dtc_a);
    Set_B_Duty(FOC->dtc_b);
    Set_C_Duty(FOC->dtc_c);
}

void FOC_CurrentController_Reset(FOC_TypeDef *FOC)
{
    PI_Controller_Reset(&FOC->id_pi);
    PI_Controller_Reset(&FOC->iq_pi);
}

/**
	* @brief  Q-axis voltage control with closed-loop zero d-axis current
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  phase: encoder electrical angle
	* @param  phase_vel: encoder electrical angular velocity
	**/
void FOC_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    if (FOC->Vbus_filt > 0.0f)
    {
        max_voltage = FOC_MAX_MODULATION * FOC->Vbus_filt / 1.5f;
        PI_Controller_Configure(&FOC->id_pi, MotorControl->id_Kp, MotorControl->id_Ki, Current_Ts, -max_voltage, max_voltage);

        voltage_d = PI_Controller_Run(&FOC->id_pi, 0.0f, FOC->Id);
        voltage_q = constrain(MotorControl->vqRef, -max_voltage, max_voltage);
        FOC_SetVoltageModulation(FOC, voltage_d, voltage_q);
        PI_Controller_TrackOutput(&FOC->id_pi, FOC->mod_d * FOC->Vbus_filt / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(FOC);
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
    }

    Inverse_Park_Transform(FOC->mod_d, FOC->mod_q, phase + phase_vel * Current_Ts, &FOC->mod_alpha, &FOC->mod_beta);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC->Ibus = FOC->mod_d * FOC->Id + FOC->mod_q * FOC->Iq;
    UTILS_LP_FAST(FOC->Ibus_filt, FOC->Ibus, 0.01f);
    FOC->Power_filt = FOC->Vbus_filt * FOC->Ibus_filt;

    SVM_SectorJudge(FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    Set_A_Duty(FOC->dtc_a);
    Set_B_Duty(FOC->dtc_b);
    Set_C_Duty(FOC->dtc_c);
}

/**
	* @brief  Turn on all high side mosfets
 **/
void PWM_TurnOnHighSides(void)
{
	TIM1->CCR1 = (uint16_t)(PWM_TIM_PERIOD);
	TIM1->CCR2 = (uint16_t)(PWM_TIM_PERIOD);
	TIM1->CCR3 = (uint16_t)(PWM_TIM_PERIOD);
} 

/**
	* @brief  Turn on all low side mosfets
 **/
void PWM_TurnOnLowSides(void)
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
} 

//#pragma arm section
