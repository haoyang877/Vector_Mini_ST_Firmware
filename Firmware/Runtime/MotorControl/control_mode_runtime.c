#include "control_mode_runtime.h"

#include <math.h>

#include "motor_state_runtime.h"
#include "control_loop_config.h"
#include "control_tuning_access.h"
#include "fast_math.h"
#include "position_cascade.h"
#include "position_impedance.h"

static float ControlModeRuntime_AddCoggingCompensation(
	const MotorControlContext *motor, const EncoderContext *encoder,
	float current_reference_a)
{
	uint16_t position;
	uint16_t index;
	uint16_t fraction;
	int32_t current_a_ma;
	int32_t current_b_ma;
	float compensation_a;

	if (motor == 0 || encoder == 0 ||
		(encoder->calib_flag & ENC_CALIB_COGGING) == 0U)
		return current_reference_a;
	/* Cogging is fixed to the rotor/absolute encoder, not the user-selected
	 * mechanical coordinate zero. Keep lookup in the calibrated absolute frame. */
	position = encoder->linearized_q15;
	index = position >> 9U;
	fraction = position & 0x01FFU;
	current_a_ma = encoder->cogging_compensation_map_ma[index];
	current_b_ma = encoder->cogging_compensation_map_ma[(index + 1U) & 0x7FU];
	compensation_a = (float)(current_a_ma +
		(((current_b_ma - current_a_ma) * (int32_t)fraction) >> 9U)) * 0.001f;
	return FastMath_Clamp(current_reference_a + compensation_a,
		-motor->configuration.current_limit_a,
		motor->configuration.current_limit_a);
}

/**
	* @brief  Current mode control task
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
	* @param  *Fluxobserver: flux observer struct pointer
 **/
void ControlModeRuntime_RunCurrent(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, EncoderContext *Encoder, FluxObserverContext *Fluxobserver)
{
	float theta_elec;
	float vel_elec;

	if(MotorControl->configuration.use_sensorless_feedback == true)
	{
		theta_elec = FluxObserver_GetElectricalAngle(Fluxobserver);
		vel_elec = FluxObserver_GetElectricalVelocity(Fluxobserver);
	}
	else
	{
		theta_elec = Encoder_GetElePhase(Encoder);
		vel_elec = Encoder_GetEleVel(Encoder);		
	}

	CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Speed mode control task
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void ControlModeRuntime_RunSpeed(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *controller,
	EncoderContext *Encoder)
{
    float theta_elec;
    float vel_elec;
    float vel_mech;
	bool use_speed_ramp;

    theta_elec = Encoder_GetElePhase(Encoder);
    vel_elec = Encoder_GetEleVel(Encoder);
    vel_mech = Encoder_GetMecVel(Encoder);

    if (motion == 0)
        return;

    if (++motion->speed_loop_count >= SPEED_LOOP_DIVIDER)
    {
        use_speed_ramp = MotorControl->configuration.speed_acceleration_rad_s2 > 0.0f && MotorControl->configuration.speed_deceleration_rad_s2 > 0.0f;

        if (use_speed_ramp)
        {
            if (MotorControl->targets.speed_rad_s > MotorControl->runtime.speed_command_ramp_rad_s)
            {
                MotorControl->runtime.speed_command_ramp_rad_s += MotorControl->configuration.speed_acceleration_rad_s2 * SPEED_LOOP_PERIOD_S;
                if (MotorControl->runtime.speed_command_ramp_rad_s > MotorControl->targets.speed_rad_s)
                {
                    MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
                }
            }
            else if (MotorControl->targets.speed_rad_s < MotorControl->runtime.speed_command_ramp_rad_s)
            {
                MotorControl->runtime.speed_command_ramp_rad_s -= MotorControl->configuration.speed_deceleration_rad_s2 * SPEED_LOOP_PERIOD_S;
                if (MotorControl->runtime.speed_command_ramp_rad_s < MotorControl->targets.speed_rad_s)
                {
                    MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
                }
            }
        }
        else
        {
            MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
        }

        PI_Controller_Configure(controller, MotorControl->configuration.speed_kp, MotorControl->configuration.speed_ki, SPEED_LOOP_PERIOD_S, -1.0f, 1.0f);
        MotorControl->targets.d_axis_current_a = 0.0f;
        MotorControl->targets.q_axis_current_a =
			ControlModeRuntime_AddCoggingCompensation(MotorControl, Encoder,
				PI_Controller_Run(controller,
					MotorControl->runtime.speed_command_ramp_rad_s, vel_mech) *
					MotorControl->configuration.current_limit_a);
        motion->speed_loop_count = 0U;
    }

    CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, theta_elec, vel_elec);
}

static float Sensorless_AngleDifference(float target, float source)
{
	float difference = target - source;

	if (difference > MATH_PI)
		difference -= MATH_TWO_PI;
	else if (difference < -MATH_PI)
		difference += MATH_TWO_PI;

	return difference;
}

static bool Sensorless_ObserverIsUsable(const MotorControlContext *MotorControl,
	const FluxObserverContext *Fluxobserver)
{
	return Fluxobserver->theta_e == Fluxobserver->theta_e &&
	       Fluxobserver->omega_e == Fluxobserver->omega_e &&
	       FastMath_Abs(Fluxobserver->omega_e) <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

static bool Sensorless_StartupTuningIsValid(
	const SensorlessStartupTuning *tuning)
{
	return tuning != 0 && tuning->align_current_ramp_time_s > 0.0f &&
		tuning->align_hold_time_s >= 0.0f && tuning->align_current_a > 0.0f &&
		tuning->startup_iq_initial_a >= 0.0f &&
		tuning->startup_iq_a >= tuning->startup_iq_initial_a &&
		tuning->startup_iq_ramp_time_s > 0.0f && tuning->startup_id_a >= 0.0f &&
		tuning->minimum_current_limit_a >= 0.0f &&
		tuning->minimum_electrical_velocity_rad_s > 0.0f &&
		tuning->target_electrical_velocity_rad_s >=
			tuning->minimum_electrical_velocity_rad_s &&
		tuning->startup_ramp_time_s > 0.0f && tuning->speed_lock_time_s > 0.0f &&
		tuning->speed_lock_filter_alpha > 0.0f &&
		tuning->speed_lock_filter_alpha <= 1.0f &&
		tuning->observer_lock_ratio > 0.0f && tuning->angle_handoff_time_s > 0.0f &&
		tuning->lock_timeout_s >= tuning->speed_lock_time_s &&
		tuning->id_ramp_down_time_s > 0.0f && tuning->observer_loss_time_s > 0.0f;
}

static bool Sensorless_StartupCurrentsAreValid(
	const MotorControlContext *MotorControl,
	const SensorlessStartupTuning *tuning)
{
	float limit_squared = MotorControl->configuration.current_limit_a *
		MotorControl->configuration.current_limit_a;
	float startup_squared = tuning->startup_iq_a * tuning->startup_iq_a +
		tuning->startup_id_a * tuning->startup_id_a;

	return MotorControl->configuration.current_limit_a >=
			tuning->minimum_current_limit_a &&
		MotorControl->configuration.current_limit_a >= tuning->align_current_a &&
		startup_squared <= limit_squared;
}

static void Sensorless_UpdateSpeedReference(MotorControlContext *MotorControl)
{
	bool use_speed_ramp = MotorControl->configuration.speed_acceleration_rad_s2 > 0.0f && MotorControl->configuration.speed_deceleration_rad_s2 > 0.0f;

	if (use_speed_ramp)
	{
		if (MotorControl->targets.speed_rad_s > MotorControl->runtime.speed_command_ramp_rad_s)
		{
			MotorControl->runtime.speed_command_ramp_rad_s += MotorControl->configuration.speed_acceleration_rad_s2 * SPEED_LOOP_PERIOD_S;
			if (MotorControl->runtime.speed_command_ramp_rad_s > MotorControl->targets.speed_rad_s)
				MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
		}
		else if (MotorControl->targets.speed_rad_s < MotorControl->runtime.speed_command_ramp_rad_s)
		{
			MotorControl->runtime.speed_command_ramp_rad_s -= MotorControl->configuration.speed_deceleration_rad_s2 * SPEED_LOOP_PERIOD_S;
			if (MotorControl->runtime.speed_command_ramp_rad_s < MotorControl->targets.speed_rad_s)
				MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
		}
	}
	else
	{
		MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
	}
}

/**
	* @brief  Sensorless speed control with align, open-loop startup and observer handoff
 **/
void ControlModeRuntime_RunSensorlessSpeed(CurrentControlContext *CurrentControl,
						MotorControlContext *MotorControl,
						PiController *controller,
						FluxObserverContext *Fluxobserver,
						SensorlessStartupContext *Startup,
						const SensorlessStartupTuning *tuning,
						MotorStateContext *motor_state)
{
	float pole_pairs = (float)MotorControl->configuration.pole_pairs;
	float min_mech_vel;
	float requested_direction;

	if (!Sensorless_StartupTuningIsValid(tuning) || pole_pairs <= 0.0f ||
		MotorControl->configuration.phase_resistance_ohm <= 0.0f ||
		MotorControl->configuration.d_axis_inductance_h <= 0.0f || MotorControl->configuration.q_axis_inductance_h <= 0.0f ||
		MotorControl->configuration.flux_weber <= 0.0f || MotorControl->configuration.current_limit_a <= 0.0f)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}

	if (FastMath_Abs(MotorControl->targets.speed_rad_s) <= 1e-4f)
	{
		SensorlessStartup_Reset(Startup);
		PI_Controller_Reset(controller);
		MotorControl->runtime.speed_command_ramp_rad_s = 0.0f;
		MotorControl->targets.d_axis_current_a = 0.0f;
		MotorControl->targets.q_axis_current_a = 0.0f;
		CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, 0.0f, 0.0f);
		return;
	}

	min_mech_vel = tuning->minimum_electrical_velocity_rad_s / pole_pairs;
	if (FastMath_Abs(MotorControl->targets.speed_rad_s) < min_mech_vel)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
		return;
	}

	if (!Sensorless_StartupCurrentsAreValid(MotorControl, tuning))
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
		return;
	}

	requested_direction = MotorControl->targets.speed_rad_s >= 0.0f ? 1.0f : -1.0f;

	if (Startup->state == SENSORLESS_STARTUP_IDLE)
	{
		FluxObserver_Initialize(Fluxobserver, MotorControl->tuning_profile,
			MotorControl);
		CurrentControlRuntime_ResetControllers(CurrentControl);
		PI_Controller_Reset(controller);
		Startup->state = SENSORLESS_STARTUP_ALIGN;
		Startup->state_ticks = 0U;
		Startup->direction = requested_direction;
	}

	switch (Startup->state)
	{
		case SENSORLESS_STARTUP_ALIGN:
			MotorControl->targets.d_axis_current_a = tuning->align_current_a *
				FastMath_Clamp(((float)Startup->state_ticks + 1.0f) * CURRENT_LOOP_PERIOD_S /
					tuning->align_current_ramp_time_s, 0.0f, 1.0f);
			MotorControl->targets.q_axis_current_a = 0.0f;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, 0.0f, 0.0f);
			if (++Startup->state_ticks >= (uint32_t)((tuning->align_current_ramp_time_s +
				tuning->align_hold_time_s) / CURRENT_LOOP_PERIOD_S))
			{
				Startup->state = SENSORLESS_STARTUP_OPEN_LOOP;
				Startup->open_loop_theta = 0.0f;
				Startup->open_loop_omega = 0.0f;
				Startup->direction = requested_direction;
				Startup->state_ticks = 0U;
				Startup->lock_ticks = 0U;
			}
		break;

		case SENSORLESS_STARTUP_OPEN_LOOP:
		{
			float iq_ramp_ratio;

			if (requested_direction != Startup->direction)
			{
				SensorlessStartup_Reset(Startup);
				break;
			}

			if (FastMath_Abs(Startup->open_loop_omega) <
				tuning->target_electrical_velocity_rad_s)
			{
				Startup->open_loop_omega += Startup->direction *
					(tuning->target_electrical_velocity_rad_s /
					 tuning->startup_ramp_time_s) * CURRENT_LOOP_PERIOD_S;
				if (FastMath_Abs(Startup->open_loop_omega) >=
					tuning->target_electrical_velocity_rad_s)
					Startup->open_loop_omega = Startup->direction *
						tuning->target_electrical_velocity_rad_s;
			}

			Startup->open_loop_ticks++;
			iq_ramp_ratio = FastMath_Clamp((float)Startup->open_loop_ticks *
				CURRENT_LOOP_PERIOD_S / tuning->startup_iq_ramp_time_s, 0.0f, 1.0f);

			Startup->open_loop_theta = FastMath_NormalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * CURRENT_LOOP_PERIOD_S);
			MotorControl->targets.d_axis_current_a = tuning->startup_id_a;
			MotorControl->targets.q_axis_current_a = Startup->direction *
				(tuning->startup_iq_initial_a +
				 (tuning->startup_iq_a - tuning->startup_iq_initial_a) * iq_ramp_ratio);
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			if (FastMath_Abs(Startup->open_loop_omega) >=
				tuning->target_electrical_velocity_rad_s)
			{
				Startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
				Startup->state_ticks = 0U;
				Startup->lock_ticks = 0U;
			}
		}
		break;
		case SENSORLESS_STARTUP_SPEED_LOCK:
		{
			float observer_velocity;
			float speed_error;
			bool is_observer_locked;

		if (requested_direction != Startup->direction ||
			!Sensorless_ObserverIsUsable(MotorControl, Fluxobserver))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				return;
			}

			Startup->open_loop_theta = FastMath_NormalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * CURRENT_LOOP_PERIOD_S);
			Startup->state_ticks++;

			MotorControl->targets.d_axis_current_a = tuning->startup_id_a;
			MotorControl->targets.q_axis_current_a = Startup->direction *
				tuning->startup_iq_a;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			observer_velocity = FluxObserver_GetElectricalVelocity(Fluxobserver);
			if (Startup->state_ticks == 1U)
				Startup->lock_speed_feedback = observer_velocity;
			else
				Startup->lock_speed_feedback += tuning->speed_lock_filter_alpha *
					(observer_velocity - Startup->lock_speed_feedback);
			speed_error = FastMath_Abs(Startup->lock_speed_feedback -
				Startup->open_loop_omega);
			is_observer_locked = Startup->lock_speed_feedback *
				Startup->open_loop_omega > 0.0f &&
				speed_error <= FastMath_Abs(Startup->open_loop_omega) *
					tuning->observer_lock_ratio;

			if (is_observer_locked)
				Startup->lock_ticks++;
			else
				Startup->lock_ticks = 0U;

			if (Startup->lock_ticks >= (uint32_t)(tuning->speed_lock_time_s /
				CURRENT_LOOP_PERIOD_S))
			{
				PI_Controller_Reset(controller);
				PI_Controller_Configure(controller, MotorControl->configuration.speed_kp, MotorControl->configuration.speed_ki,
					SPEED_LOOP_PERIOD_S, -1.0f, Startup->speed_pi_output_max);
				PI_Controller_TrackOutput(controller, MotorControl->targets.q_axis_current_a / MotorControl->configuration.current_limit_a);
				Startup->handoff_phase_delta = Sensorless_AngleDifference(Startup->open_loop_theta,
					FluxObserver_GetElectricalAngle(Fluxobserver));
				Startup->state = SENSORLESS_STARTUP_HANDOFF;
				Startup->state_ticks = 0U;
			}

			if (Startup->state_ticks >= (uint32_t)(tuning->lock_timeout_s /
				CURRENT_LOOP_PERIOD_S))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				return;
			}
		}
		break;
		case SENSORLESS_STARTUP_HANDOFF:
		{
			float blend;
			float phase;
			float phase_vel;

			if (requested_direction != Startup->direction ||
				!Sensorless_ObserverIsUsable(MotorControl, Fluxobserver))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				return;
			}

			Startup->open_loop_theta = FastMath_NormalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * CURRENT_LOOP_PERIOD_S);
			blend = FastMath_Clamp((float)(++Startup->state_ticks) *
				CURRENT_LOOP_PERIOD_S / tuning->angle_handoff_time_s, 0.0f, 1.0f);
			phase = FastMath_NormalizeAngle(FluxObserver_GetElectricalAngle(Fluxobserver) +
				(1.0f - blend) * Startup->handoff_phase_delta);
			phase_vel = Startup->open_loop_omega + blend *
				(FluxObserver_GetElectricalVelocity(Fluxobserver) - Startup->open_loop_omega);

			MotorControl->targets.d_axis_current_a = tuning->startup_id_a;
			MotorControl->targets.q_axis_current_a = Startup->direction *
				tuning->startup_iq_a;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, phase, phase_vel);

			if (blend >= 1.0f)
			{
				Startup->state = SENSORLESS_STARTUP_CLOSED_LOOP;
				Startup->state_ticks = 0U;
				Startup->id_ramp_ticks = 0U;
				Startup->loss_ticks = 0U;
				Startup->speed_feedback = FluxObserver_GetElectricalVelocity(Fluxobserver) / pole_pairs;
				MotorControl->runtime.speed_command_ramp_rad_s = Startup->speed_feedback;
			}
		}
		break;

		case SENSORLESS_STARTUP_CLOSED_LOOP:
		{
			float observer_vel = FluxObserver_GetElectricalVelocity(Fluxobserver);
			float observer_mech_vel = observer_vel / pole_pairs;

			if (!Sensorless_ObserverIsUsable(MotorControl, Fluxobserver))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				return;
			}

			if (requested_direction * observer_vel < 0.0f &&
				FastMath_Abs(observer_vel) < tuning->minimum_electrical_velocity_rad_s)
			{
				SensorlessStartup_Reset(Startup);
				CurrentControlRuntime_ResetControllers(CurrentControl);
				PI_Controller_Reset(controller);
				return;
			}

			MotorControl->targets.d_axis_current_a = tuning->startup_id_a *
				(1.0f - FastMath_Clamp((float)Startup->id_ramp_ticks *
				CURRENT_LOOP_PERIOD_S / tuning->id_ramp_down_time_s, 0.0f, 1.0f));
			if (Startup->id_ramp_ticks < (uint32_t)(tuning->id_ramp_down_time_s /
				CURRENT_LOOP_PERIOD_S))
				Startup->id_ramp_ticks++;

			if (++Startup->speed_loop_ticks >= SPEED_LOOP_DIVIDER)
			{
				Startup->speed_feedback += SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA *
					(observer_mech_vel - Startup->speed_feedback);
				Sensorless_UpdateSpeedReference(MotorControl);
				PI_Controller_Configure(controller, MotorControl->configuration.speed_kp, MotorControl->configuration.speed_ki, SPEED_LOOP_PERIOD_S, -1.0f, 1.0f);
				MotorControl->targets.q_axis_current_a = PI_Controller_Run(controller, MotorControl->runtime.speed_command_ramp_rad_s, Startup->speed_feedback) * MotorControl->configuration.current_limit_a;
				Startup->speed_loop_ticks = 0U;
			}

			if (FastMath_Abs(observer_vel) <
				tuning->minimum_electrical_velocity_rad_s * 0.5f)
				Startup->loss_ticks++;
			else
				Startup->loss_ticks = 0U;

			if (Startup->loss_ticks >= (uint32_t)(tuning->observer_loss_time_s /
				CURRENT_LOOP_PERIOD_S))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				return;
			}

			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, FluxObserver_GetElectricalAngle(Fluxobserver), observer_vel);
		}
		break;

		default:
			SensorlessStartup_Reset(Startup);
		break;
	}
}
/**
	* @brief  Cascade position-speed-current cascade control task
 **/
void ControlModeRuntime_RunPositionCascade(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const MotorProfile *motor_profile, MotorStateContext *motor_state)
{
	PositionCascadeConfig config;
	PositionCascadeOutput output;
	float theta_elec;
	float theta_mech;
	float vel_elec;
	float vel_mech;

	if (motion == 0 || CurrentControl == 0 || MotorControl == 0 || Encoder == 0 ||
		motor_profile == 0)
		return;
	theta_elec = Encoder_GetElePhase(Encoder);
	theta_mech = Encoder_GetMecPos(Encoder);
	vel_elec = Encoder_GetEleVel(Encoder);
	vel_mech = Encoder_GetMecVel(Encoder);

	config.target_position = MotorControl->targets.position_rad;
	config.position_error_window = MotorControl->configuration.position_error_window_rad;
	config.acceleration = MotorControl->configuration.position_acceleration_rad_s2;
	config.deceleration = MotorControl->configuration.position_deceleration_rad_s2;
	config.maximum_speed = MotorControl->configuration.position_max_speed_rad_s;
	config.position_kp = MotorControl->configuration.cascade_position_kp_per_s;
	config.position_kd = MotorControl->configuration.cascade_position_kd;
	config.speed_kp = MotorControl->configuration.speed_kp;
	config.speed_ki = MotorControl->configuration.speed_ki;
	config.current_limit_a = MotorControl->configuration.current_limit_a;
	config.position_kp_limit = motor_profile->cascade_position_kp_limit_per_s;
	config.position_kd_limit = motor_profile->cascade_position_kd_limit;
	config.position_sample_period_s = CASCADE_POSITION_LOOP_PERIOD_S;
	config.speed_sample_period_s = SPEED_LOOP_PERIOD_S;
	config.position_loop_divider = CASCADE_POSITION_LOOP_DIVIDER;
	config.speed_loop_divider = SPEED_LOOP_DIVIDER;

	if (!PositionCascade_Update(&motion->cascade, &config,
		theta_mech, vel_mech, &output))
	{
		MotorControl->targets.d_axis_current_a = 0.0f;
		MotorControl->targets.q_axis_current_a = 0.0f;
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}

	MotorControl->runtime.position_command_ramp_rad = output.position_reference;
	MotorControl->runtime.speed_command_ramp_rad_s = output.speed_reference;
	MotorControl->runtime.position_velocity_filtered_rad_s = output.speed_feedback;
	MotorControl->runtime.has_reached_position = output.target_reached;
	MotorControl->targets.d_axis_current_a = 0.0f;
	MotorControl->targets.q_axis_current_a =
		ControlModeRuntime_AddCoggingCompensation(MotorControl, Encoder,
			output.iq_reference);
	CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Position impedance control task
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void ControlModeRuntime_RunPositionImpedance(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const MotorProfile *motor_profile, float maximum_current_limit_a,
	MotorStateContext *motor_state)
{
	PositionImpedanceConfig config;
	PositionImpedanceOutput output;
	float theta_elec;
	float theta_mech;
	float vel_elec;

	if (motion == 0 || CurrentControl == 0 || MotorControl == 0 || Encoder == 0 ||
		motor_profile == 0 || !isfinite(maximum_current_limit_a) ||
		maximum_current_limit_a <= 0.0f ||
		MotorControl->mechanical_load_profile == 0)
		return;
	theta_elec = Encoder_GetElePhase(Encoder);
	theta_mech = Encoder_GetMecPos(Encoder);
	vel_elec = Encoder_GetEleVel(Encoder);

	config.target_position = MotorControl->targets.position_rad;
	config.position_error_window = MotorControl->configuration.position_error_window_rad;
	config.acceleration = MotorControl->configuration.position_acceleration_rad_s2;
	config.deceleration = MotorControl->configuration.position_deceleration_rad_s2;
	config.maximum_speed = MotorControl->configuration.position_max_speed_rad_s;
	config.speed_limit_rad_s = MotorControl->configuration.speed_limit_rad_s;
	config.kp = MotorControl->configuration.position_kp_a_per_rad;
	config.kd = MotorControl->configuration.position_kd_a_per_rad_s;
	config.ki = MotorControl->configuration.position_ki_a_per_rad_s;
	config.integral_limit = MotorControl->configuration.position_integral_limit_a;
	config.output_limit = MotorControl->configuration.current_limit_a;
	config.kp_limit = motor_profile->position_kp_limit_a_per_rad;
	config.kd_limit = motor_profile->position_kd_limit_a_per_rad_s;
	config.ki_limit = motor_profile->position_ki_limit_a_per_rad_s;
	config.maximum_speed_limit_rad_s = motor_profile->position_speed_limit_rps * MATH_TWO_PI;
	config.maximum_current_limit_a = maximum_current_limit_a;
	config.friction_feedforward_enabled = MotorControl->configuration.
		friction_model_valid || MotorControl->mechanical_load_profile->
		position_friction_feedforward_enabled;
	config.friction_positive_current = MotorControl->configuration.
		friction_model_valid ? MotorControl->configuration.friction_coulomb_pos_a :
		MotorControl->mechanical_load_profile->friction_positive_current_a;
	config.friction_negative_current = MotorControl->configuration.
		friction_model_valid ? MotorControl->configuration.friction_coulomb_neg_a :
		MotorControl->mechanical_load_profile->friction_negative_current_a;
	config.breakaway_positive_current = FastMath_Max(
		MotorControl->mechanical_load_profile->breakaway_positive_current_a,
		config.friction_positive_current);
	config.breakaway_negative_current = FastMath_Max(
		MotorControl->mechanical_load_profile->breakaway_negative_current_a,
		config.friction_negative_current);
	config.friction_current_slew_rate = MotorControl->mechanical_load_profile->
		friction_current_slew_rate_a_per_s;
	config.friction_position_enter = MotorControl->mechanical_load_profile->
		friction_position_enter_rad;
	config.friction_position_exit = MotorControl->mechanical_load_profile->
		friction_position_exit_rad;
	config.friction_reference_speed = MotorControl->mechanical_load_profile->
		friction_reference_speed_rad_s;
	config.friction_stop_speed = MotorControl->mechanical_load_profile->
		friction_stop_speed_rad_s;
	config.friction_move_speed = MotorControl->mechanical_load_profile->
		friction_move_speed_rad_s;
	config.friction_stuck_time = MotorControl->mechanical_load_profile->
		friction_stuck_time_s;
	config.friction_landing_position = MotorControl->mechanical_load_profile->
		friction_landing_position_rad;
	config.friction_landing_speed = MotorControl->mechanical_load_profile->
		friction_landing_speed_rad_s;
	config.friction_recovery_delay = MotorControl->mechanical_load_profile->
		friction_recovery_delay_s;
	config.friction_recovery_pulse_time = MotorControl->mechanical_load_profile->
		friction_recovery_pulse_time_s;
	config.friction_recovery_cooldown = MotorControl->mechanical_load_profile->
		friction_recovery_cooldown_s;
	config.sample_period_s = POSITION_LOOP_PERIOD_S;
	config.loop_divider = POSITION_LOOP_DIVIDER;

	if (!PositionImpedance_Update(&motion->impedance, &config,
		theta_mech, &output))
	{
		MotorControl->targets.d_axis_current_a = 0.0f;
		MotorControl->targets.q_axis_current_a = 0.0f;
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}

	MotorControl->runtime.position_command_ramp_rad = output.position_reference;
	MotorControl->runtime.speed_command_ramp_rad_s = output.speed_reference;
	MotorControl->runtime.position_velocity_filtered_rad_s = output.velocity_feedback;
	MotorControl->runtime.has_reached_position = output.target_reached;
	MotorControl->targets.d_axis_current_a = 0.0f;
	MotorControl->targets.q_axis_current_a =
		ControlModeRuntime_AddCoggingCompensation(MotorControl, Encoder,
			output.iq_reference);
	CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, theta_elec, vel_elec);
}

void ControlModeRuntime_ResetPosition(MotionControlContext *motion)
{
	if (motion == 0)
		return;
	PositionCascade_Reset(&motion->cascade);
	PositionImpedance_Reset(&motion->impedance);
	motion->speed_loop_count = 0U;
}

/**
	* @brief  Voltage open-loop mode
	*         rotate electrical angle by open-loop velocity and
	*         apply open-loop voltage on d-axis
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void ControlModeRuntime_RunVoltageOpenLoop(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl)
{
	/*integrate electrical angle from open-loop velocity*/
	MotorControl->runtime.open_loop_electrical_angle_rad = FastMath_NormalizeAngle(MotorControl->runtime.open_loop_electrical_angle_rad + MotorControl->configuration.open_loop_electrical_velocity_rad_s * CURRENT_LOOP_PERIOD_S);
	
	/*open-loop voltage drive on d-axis*/
	CurrentControlRuntime_RunVoltage(CurrentControl, MotorControl->configuration.open_loop_voltage_v, 0.0f, MotorControl->runtime.open_loop_electrical_angle_rad);
}

/**
	* @brief  Q-axis voltage mode using encoder electrical angle
	*         regulate d-axis current to zero and directly command Vq
	* @param  *CurrentControl: CurrentControl struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
	**/
void ControlModeRuntime_RunQVoltage(CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	MotorStateContext *motor_state)
{
	if(!Encoder_IsOnline(Encoder))
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
		return;
	}
	if((Encoder->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
		return;
	}

	MotorControl->targets.d_axis_current_a = 0.0f;
	MotorControl->targets.q_axis_current_a = 0.0f;

	CurrentControlRuntime_RunQVoltage(CurrentControl,
				MotorControl,
				Encoder_GetElePhase(Encoder),
				Encoder_GetEleVel(Encoder));
}
