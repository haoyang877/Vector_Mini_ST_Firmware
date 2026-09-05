#include "current_control_runtime.h"

#include <stdbool.h>
#include <stddef.h>
#include "fast_math.h"
#include "control_loop_config.h"
#include "svpwm.h"
#include "current_control_math.h"

#define CURRENT_CONTROL_MAX_MODULATION (0.95f * MATH_SQRT_3_BY_2)

static void CurrentControlRuntime_Park(float alpha, float beta, float angle,
	float *direct, float *quadrature)
{
	CurrentControl_Park(alpha, beta, FastMath_Sin(angle), FastMath_Cos(angle),
		direct, quadrature);
}

static void CurrentControlRuntime_InversePark(float direct, float quadrature, float angle,
	float *alpha, float *beta)
{
	CurrentControl_InversePark(direct, quadrature, FastMath_Sin(angle),
		FastMath_Cos(angle), alpha, beta);
}

static void CurrentControlRuntime_ApplyModulation(CurrentControlContext *CurrentControl)
{
	SvpwmOutput output;

	if (!Svpwm_Calculate(CurrentControl->alpha_modulation, CurrentControl->beta_modulation, &output))
	{
		CurrentControl->phase_a_duty = 1.0f;
		CurrentControl->phase_b_duty = 1.0f;
		CurrentControl->phase_c_duty = 1.0f;
		CurrentControl->sector = 0;
		PowerStage_RejectOutputCommand(CurrentControl->power_stage);
		return;
	}

	CurrentControl->phase_a_duty = output.phase_a_duty;
	CurrentControl->phase_b_duty = output.phase_b_duty;
	CurrentControl->phase_c_duty = output.phase_c_duty;
	CurrentControl->sector = output.sector;
	(void)PowerStage_ApplyDutyCycles(CurrentControl->power_stage,
		CurrentControl->phase_a_duty, CurrentControl->phase_b_duty, CurrentControl->phase_c_duty);
}

static bool CurrentControlRuntime_SetVoltageModulation(CurrentControlContext *CurrentControl, float voltage_d, float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (CurrentControl->filtered_bus_voltage_v <= 0.0f)
    {
        CurrentControl->d_axis_modulation = 0.0f;
        CurrentControl->q_axis_modulation = 0.0f;
        CurrentControl->modulation_utilization = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / CurrentControl->filtered_bus_voltage_v;
    CurrentControl->d_axis_modulation = voltage_to_modulation * voltage_d;
    CurrentControl->q_axis_modulation = voltage_to_modulation * voltage_q;
    modulation_magnitude = FastMath_Sqrt(CurrentControl->d_axis_modulation * CurrentControl->d_axis_modulation + CurrentControl->q_axis_modulation * CurrentControl->q_axis_modulation);

    if (modulation_magnitude > CURRENT_CONTROL_MAX_MODULATION)
    {
        float modulation_scale = CURRENT_CONTROL_MAX_MODULATION / modulation_magnitude;
        CurrentControl->d_axis_modulation *= modulation_scale;
        CurrentControl->q_axis_modulation *= modulation_scale;
        modulation_magnitude = CURRENT_CONTROL_MAX_MODULATION;
    }

    if (modulation_magnitude > 0.0f)
    {
        CurrentControl->modulation_utilization = FastMath_Sign(CurrentControl->q_axis_modulation) * modulation_magnitude / CURRENT_CONTROL_MAX_MODULATION;
    }
    else
    {
        CurrentControl->modulation_utilization = 0.0f;
    }

    return true;
}

/**
	* @brief  Voltage loop 
    * @param  *CurrentControl: CurrentControl struct pointer
	* @param  Vd_set: voltage set in d axis 
	* @param  Vq_set: voltage set in q axis
	* @param  phase: electrical angle
 **/
void CurrentControlRuntime_RunVoltage(CurrentControlContext *CurrentControl, float Vd_set, float Vq_set, float phase)
{
    CurrentControl_Clarke(CurrentControl->phase_a_current_a, CurrentControl->phase_b_current_a, CurrentControl->phase_c_current_a, &CurrentControl->alpha_current_a, &CurrentControl->beta_current_a);
    CurrentControlRuntime_Park(CurrentControl->alpha_current_a, CurrentControl->beta_current_a, phase, &CurrentControl->d_axis_current_a, &CurrentControl->q_axis_current_a);

    FAST_MATH_LOW_PASS(CurrentControl->filtered_d_axis_current_a, CurrentControl->d_axis_current_a, 0.01f);
    FAST_MATH_LOW_PASS(CurrentControl->filtered_q_axis_current_a, CurrentControl->q_axis_current_a, 0.01f);

    CurrentControlRuntime_SetVoltageModulation(CurrentControl, Vd_set, Vq_set);

    CurrentControlRuntime_InversePark(CurrentControl->d_axis_modulation, CurrentControl->q_axis_modulation, phase,
        &CurrentControl->alpha_modulation, &CurrentControl->beta_modulation);
	CurrentControlRuntime_ApplyModulation(CurrentControl);
}

/**
	* @brief  Current loop 
    * @param  *CurrentControl: CurrentControl struct pointer
    * @param  *MotorControl: MotorControl struct pointer
	* @param  phase: electrical angle
	* @param  phase_vel: electrical angular velocity 
 **/
void CurrentControlRuntime_RunClosedLoop(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, float phase, float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    CurrentControl_Clarke(CurrentControl->phase_a_current_a, CurrentControl->phase_b_current_a, CurrentControl->phase_c_current_a, &CurrentControl->alpha_current_a, &CurrentControl->beta_current_a);
    CurrentControlRuntime_Park(CurrentControl->alpha_current_a, CurrentControl->beta_current_a, phase, &CurrentControl->d_axis_current_a, &CurrentControl->q_axis_current_a);

    if (CurrentControl->filtered_bus_voltage_v > 0.0f)
    {
        max_voltage = CURRENT_CONTROL_MAX_MODULATION * CurrentControl->filtered_bus_voltage_v / 1.5f;
        PI_Controller_Configure(&CurrentControl->id_pi, MotorControl->configuration.d_axis_current_kp, MotorControl->configuration.d_axis_current_ki, CURRENT_LOOP_PERIOD_S, -max_voltage, max_voltage);
        PI_Controller_Configure(&CurrentControl->iq_pi, MotorControl->configuration.q_axis_current_kp, MotorControl->configuration.q_axis_current_ki, CURRENT_LOOP_PERIOD_S, -max_voltage, max_voltage);

        voltage_d = PI_Controller_Run(&CurrentControl->id_pi, MotorControl->targets.d_axis_current_a, CurrentControl->d_axis_current_a);
        voltage_q = PI_Controller_Run(&CurrentControl->iq_pi, MotorControl->targets.q_axis_current_a, CurrentControl->q_axis_current_a);
        CurrentControlRuntime_SetVoltageModulation(CurrentControl, voltage_d, voltage_q);

        PI_Controller_TrackOutput(&CurrentControl->id_pi, CurrentControl->d_axis_modulation * CurrentControl->filtered_bus_voltage_v / 1.5f);
        PI_Controller_TrackOutput(&CurrentControl->iq_pi, CurrentControl->q_axis_modulation * CurrentControl->filtered_bus_voltage_v / 1.5f);
    }
    else
    {
        CurrentControlRuntime_ResetControllers(CurrentControl);
        CurrentControl->d_axis_modulation = 0.0f;
        CurrentControl->q_axis_modulation = 0.0f;
        CurrentControl->modulation_utilization = 0.0f;
    }

    CurrentControlRuntime_InversePark(CurrentControl->d_axis_modulation, CurrentControl->q_axis_modulation,
        phase + phase_vel * CURRENT_LOOP_PERIOD_S, &CurrentControl->alpha_modulation, &CurrentControl->beta_modulation);

    FAST_MATH_LOW_PASS(CurrentControl->filtered_d_axis_current_a, CurrentControl->d_axis_current_a, 0.01f);
    FAST_MATH_LOW_PASS(CurrentControl->filtered_q_axis_current_a, CurrentControl->q_axis_current_a, 0.01f);

    CurrentControl->bus_current_a = CurrentControl->d_axis_modulation * CurrentControl->d_axis_current_a + CurrentControl->q_axis_modulation * CurrentControl->q_axis_current_a;
    FAST_MATH_LOW_PASS(CurrentControl->filtered_bus_current_a, CurrentControl->bus_current_a, 0.01f);
    CurrentControl->filtered_power_w = CurrentControl->filtered_bus_voltage_v * CurrentControl->filtered_bus_current_a;

	CurrentControlRuntime_ApplyModulation(CurrentControl);
}

void CurrentControlRuntime_ResetControllers(CurrentControlContext *CurrentControl)
{
    PI_Controller_Reset(&CurrentControl->id_pi);
    PI_Controller_Reset(&CurrentControl->iq_pi);
}

/**
	* @brief  Q-axis voltage control with closed-loop zero d-axis current
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  phase: encoder electrical angle
	* @param  phase_vel: encoder electrical angular velocity
	**/
void CurrentControlRuntime_RunQVoltage(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, float phase, float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    CurrentControl_Clarke(CurrentControl->phase_a_current_a, CurrentControl->phase_b_current_a, CurrentControl->phase_c_current_a, &CurrentControl->alpha_current_a, &CurrentControl->beta_current_a);
    CurrentControlRuntime_Park(CurrentControl->alpha_current_a, CurrentControl->beta_current_a, phase, &CurrentControl->d_axis_current_a, &CurrentControl->q_axis_current_a);

    if (CurrentControl->filtered_bus_voltage_v > 0.0f)
    {
        max_voltage = CURRENT_CONTROL_MAX_MODULATION * CurrentControl->filtered_bus_voltage_v / 1.5f;
        PI_Controller_Configure(&CurrentControl->id_pi, MotorControl->configuration.d_axis_current_kp, MotorControl->configuration.d_axis_current_ki, CURRENT_LOOP_PERIOD_S, -max_voltage, max_voltage);

        voltage_d = PI_Controller_Run(&CurrentControl->id_pi, 0.0f, CurrentControl->d_axis_current_a);
        voltage_q = FastMath_Clamp(MotorControl->targets.q_axis_voltage_v, -max_voltage, max_voltage);
        CurrentControlRuntime_SetVoltageModulation(CurrentControl, voltage_d, voltage_q);
        PI_Controller_TrackOutput(&CurrentControl->id_pi, CurrentControl->d_axis_modulation * CurrentControl->filtered_bus_voltage_v / 1.5f);
    }
    else
    {
        CurrentControlRuntime_ResetControllers(CurrentControl);
        CurrentControl->d_axis_modulation = 0.0f;
        CurrentControl->q_axis_modulation = 0.0f;
        CurrentControl->modulation_utilization = 0.0f;
    }

    CurrentControlRuntime_InversePark(CurrentControl->d_axis_modulation, CurrentControl->q_axis_modulation,
        phase + phase_vel * CURRENT_LOOP_PERIOD_S, &CurrentControl->alpha_modulation, &CurrentControl->beta_modulation);

    FAST_MATH_LOW_PASS(CurrentControl->filtered_d_axis_current_a, CurrentControl->d_axis_current_a, 0.01f);
    FAST_MATH_LOW_PASS(CurrentControl->filtered_q_axis_current_a, CurrentControl->q_axis_current_a, 0.01f);

    CurrentControl->bus_current_a = CurrentControl->d_axis_modulation * CurrentControl->d_axis_current_a + CurrentControl->q_axis_modulation * CurrentControl->q_axis_current_a;
    FAST_MATH_LOW_PASS(CurrentControl->filtered_bus_current_a, CurrentControl->bus_current_a, 0.01f);
    CurrentControl->filtered_power_w = CurrentControl->filtered_bus_voltage_v * CurrentControl->filtered_bus_current_a;

	CurrentControlRuntime_ApplyModulation(CurrentControl);
}

/**
	* @brief  Turn on all high side mosfets
 **/
void CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControlContext *CurrentControl)
{
	if (CurrentControl != NULL)
		(void)PowerStage_ApplyDutyCycles(CurrentControl->power_stage, 1.0f, 1.0f, 1.0f);
} 

/**
	* @brief  Turn on all low side mosfets
 **/
void CurrentControlRuntime_ApplyLowSideZeroVector(CurrentControlContext *CurrentControl)
{
	if (CurrentControl != NULL)
		(void)PowerStage_ApplyDutyCycles(CurrentControl->power_stage, 0.0f, 0.0f, 0.0f);
} 

//#pragma arm section
