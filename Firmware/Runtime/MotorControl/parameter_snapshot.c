#include "parameter_snapshot.h"

#include <math.h>
#include <string.h>

#include "fast_math.h"

#define ParameterMotor (context->motor)
#define ParameterEncoder (context->encoder)
#define ParameterBoardProfile (context->board_profile)
#define ParameterMotorProfile (context->motor_profile)
#define ParameterEncoderProfile (context->encoder_profile)
#define ParameterMechanicalLoadProfile (context->mechanical_load_profile)
#define CanConfiguration (context->can_configuration)
#define MotorControl (*ParameterMotor)
#define OnBoard_Encoder (*ParameterEncoder)

bool ParameterSnapshot_Initialize(ParameterSnapshotContext *context,
	MotorControlContext *motor, EncoderContext *encoder,
	const BoardProfile *board_profile, const MotorProfile *motor_profile,
	const EncoderProfile *encoder_profile,
	const MechanicalLoadProfile *mechanical_load_profile,
	CanConfigurationServiceContext *can_configuration)
{
	if (context == 0 || motor == 0 || encoder == 0 || board_profile == 0 ||
		motor_profile == 0 || encoder_profile == 0 ||
		mechanical_load_profile == 0 || can_configuration == 0)
		return false;
	context->motor = motor;
	context->encoder = encoder;
	context->board_profile = board_profile;
	context->motor_profile = motor_profile;
	context->encoder_profile = encoder_profile;
	context->mechanical_load_profile = mechanical_load_profile;
	context->can_configuration = can_configuration;
	return true;
}

void ParameterSnapshot_LoadDefaults(ParameterSnapshotContext *context)
{
	MotorControl.configuration.use_sensorless_feedback = false;
	MotorControl.configuration.open_loop_voltage_v = ParameterMotorProfile->open_loop_voltage_v;
	MotorControl.configuration.open_loop_electrical_velocity_rad_s = ParameterMotorProfile->open_loop_electrical_velocity_rad_s;
	MotorControl.configuration.position_error_window_rad = ParameterMotorProfile->position_error_window_rad;
	MotorControl.command.q_axis_voltage_reference_v = 0.0f;
	MotorControl.runtime.open_loop_electrical_angle_rad = ParameterMotorProfile->open_loop_initial_theta_rad;
	(void)CanConfigurationService_SetNodeId(CanConfiguration,
		ParameterBoardProfile->default_can_node_id);
	(void)CanConfigurationService_SetHeartbeatMs(CanConfiguration,
		ParameterBoardProfile->default_can_heartbeat_ms);

	MotorControl.configuration.phase_a_current_offset_adc = ParameterBoardProfile->default_phase_a_current_offset_adc;
	MotorControl.configuration.phase_b_current_offset_adc = ParameterBoardProfile->default_phase_b_current_offset_adc;
	MotorControl.configuration.phase_c_current_offset_adc = ParameterBoardProfile->default_phase_c_current_offset_adc;
	MotorControl.configuration.pole_pairs = ParameterMotorProfile->pole_pairs;
	MotorControl.configuration.phase_resistance_ohm = ParameterMotorProfile->phase_resistance_ohm;
	MotorControl.configuration.d_axis_inductance_h = ParameterMotorProfile->d_axis_inductance_h;
	MotorControl.configuration.q_axis_inductance_h = ParameterMotorProfile->q_axis_inductance_h;
	MotorControl.configuration.flux_weber = ParameterMotorProfile->flux_weber;
	MotorControl.configuration.d_axis_current_kp = ParameterMotorProfile->d_axis_inductance_h * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_kp = ParameterMotorProfile->q_axis_inductance_h * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.d_axis_current_ki = ParameterMotorProfile->phase_resistance_ohm * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_ki = MotorControl.configuration.d_axis_current_ki;
	MotorControl.configuration.calibration_current_a = ParameterMotorProfile->calibration_current_a;
	MotorControl.configuration.current_limit_a = ParameterMotorProfile->current_limit_a;
	MotorControl.configuration.speed_limit_rad_s = ParameterMechanicalLoadProfile->default_speed_limit_rps * MATH_TWO_PI;
	MotorControl.configuration.speed_acceleration_rad_s2 = ParameterMotorProfile->speed_acceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.speed_deceleration_rad_s2 = ParameterMotorProfile->speed_deceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.speed_kp = ParameterMotorProfile->speed_kp;
	MotorControl.configuration.speed_ki = ParameterMotorProfile->speed_ki;
	MotorControl.configuration.position_acceleration_rad_s2 = ParameterMotorProfile->position_acceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.position_deceleration_rad_s2 = ParameterMotorProfile->position_deceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.position_max_speed_rad_s = ParameterMechanicalLoadProfile->default_position_max_speed_rps * MATH_TWO_PI;
	MotorControl.configuration.position_kp_a_per_rad = ParameterMotorProfile->position_kp_a_per_rad;
	MotorControl.configuration.position_kd_a_per_rad_s = ParameterMotorProfile->position_kd_a_per_rad_s;
	MotorControl.configuration.position_ki_a_per_rad_s = ParameterMotorProfile->position_ki_a_per_rad_s;
	MotorControl.configuration.position_integral_limit_a = ParameterMotorProfile->position_integral_limit_a;
	MotorControl.configuration.cascade_position_kp_per_s = ParameterMotorProfile->cascade_position_kp_per_s;
	MotorControl.configuration.cascade_position_kd = ParameterMotorProfile->cascade_position_kd;
	MotorControl.configuration.friction_coulomb_pos_a = 0.0f;
	MotorControl.configuration.friction_coulomb_neg_a = 0.0f;
	MotorControl.configuration.friction_viscous_pos_a_per_rad_s = 0.0f;
	MotorControl.configuration.friction_viscous_neg_a_per_rad_s = 0.0f;
	MotorControl.configuration.friction_model_valid = false;

	OnBoard_Encoder.electrical_zero_q15 = ParameterEncoderProfile->default_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = ParameterEncoderProfile->default_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = ParameterEncoderProfile->default_calibration_flag;
	OnBoard_Encoder.reverse = ParameterEncoderProfile->default_reverse;
	memset(OnBoard_Encoder.linearization_lut_q15, 0,
		sizeof(OnBoard_Encoder.linearization_lut_q15));
	memset(OnBoard_Encoder.cogging_compensation_map_ma, 0,
		sizeof(OnBoard_Encoder.cogging_compensation_map_ma));
}

void ParameterSnapshot_Capture(const ParameterSnapshotContext *context,
	ParameterSnapshot *param)
{
	uint32_t index;
	memset(param, 0, sizeof(*param));
	/* Only module-specific calibration is persisted. All design values are
	 * deliberately reloaded from typed profiles on each boot. */
	param->phase_a_current_offset_adc = (float)MotorControl.configuration.phase_a_current_offset_adc;
	param->phase_b_current_offset_adc = (float)MotorControl.configuration.phase_b_current_offset_adc;
	param->phase_c_current_offset_adc = (float)MotorControl.configuration.phase_c_current_offset_adc;
	param->encoder_electrical_zero_q15 = OnBoard_Encoder.electrical_zero_q15;
	param->encoder_mechanical_zero_q15 = OnBoard_Encoder.mechanical_zero_q15;
	param->encoder_calib_flag = OnBoard_Encoder.calib_flag;
	param->encoder_reverse = OnBoard_Encoder.reverse;
	for (index = 0U; index < ENCODER_OFFSET_LUT_SIZE; ++index)
		param->encoder_linearization_lut_q15[index] = OnBoard_Encoder.linearization_lut_q15[index];
	param->current_sense_shunt_milliohm = ParameterBoardProfile->current_sense_shunt_milliohm;
	param->friction_coulomb_pos_a = MotorControl.configuration.friction_coulomb_pos_a;
	param->friction_coulomb_neg_a = MotorControl.configuration.friction_coulomb_neg_a;
	param->friction_viscous_pos_a_per_rad_s = MotorControl.configuration.friction_viscous_pos_a_per_rad_s;
	param->friction_viscous_neg_a_per_rad_s = MotorControl.configuration.friction_viscous_neg_a_per_rad_s;
	param->friction_model_valid = MotorControl.configuration.friction_model_valid ? 1U : 0U;
	for (index = 0U; index < ENCODER_COGGING_MAP_SIZE; ++index)
		param->cogging_compensation_map_ma[index] = OnBoard_Encoder.cogging_compensation_map_ma[index];
	param->schema_version = PARAM_SCHEMA_VERSION;
}

void ParameterSnapshot_Apply(ParameterSnapshotContext *context,
	const ParameterSnapshot *param)
{
	uint32_t index;
	uint8_t allowed_flags = ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO |
		ENC_CALIB_MECHANICAL_ZERO | ENC_CALIB_COGGING;
	bool offsets_valid;
	bool friction_valid;

	ParameterSnapshot_LoadDefaults(context);
	if (param == 0 || param->magic_word != PARAMETER_SNAPSHOT_MAGIC ||
		param->schema_version < PARAM_SCHEMA_VERSION_PREVIOUS_CASCADE ||
		param->schema_version > PARAM_SCHEMA_VERSION || param->encoder_reverse > 1U)
		return;
	offsets_valid = param->phase_a_current_offset_adc >= ParameterBoardProfile->minimum_current_offset_adc &&
		param->phase_a_current_offset_adc <= ParameterBoardProfile->maximum_current_offset_adc &&
		param->phase_b_current_offset_adc >= ParameterBoardProfile->minimum_current_offset_adc &&
		param->phase_b_current_offset_adc <= ParameterBoardProfile->maximum_current_offset_adc &&
		param->phase_c_current_offset_adc >= ParameterBoardProfile->minimum_current_offset_adc &&
		param->phase_c_current_offset_adc <= ParameterBoardProfile->maximum_current_offset_adc;
	if (offsets_valid)
	{
		MotorControl.configuration.phase_a_current_offset_adc = (uint16_t)param->phase_a_current_offset_adc;
		MotorControl.configuration.phase_b_current_offset_adc = (uint16_t)param->phase_b_current_offset_adc;
		MotorControl.configuration.phase_c_current_offset_adc = (uint16_t)param->phase_c_current_offset_adc;
	}
	OnBoard_Encoder.reverse = param->encoder_reverse;
	OnBoard_Encoder.electrical_zero_q15 = param->encoder_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = param->encoder_calib_flag & allowed_flags;
	for (index = 0U; index < ENCODER_OFFSET_LUT_SIZE; ++index)
		OnBoard_Encoder.linearization_lut_q15[index] = param->encoder_linearization_lut_q15[index];
	friction_valid = param->schema_version >= PARAM_SCHEMA_VERSION_PREVIOUS_COGGING &&
		param->friction_model_valid == 1U && isfinite(param->friction_coulomb_pos_a) &&
		param->friction_coulomb_pos_a >= 0.0f && isfinite(param->friction_coulomb_neg_a) &&
		param->friction_coulomb_neg_a >= 0.0f && isfinite(param->friction_viscous_pos_a_per_rad_s) &&
		param->friction_viscous_pos_a_per_rad_s >= 0.0f &&
		isfinite(param->friction_viscous_neg_a_per_rad_s) &&
		param->friction_viscous_neg_a_per_rad_s >= 0.0f;
	if (friction_valid)
	{
		MotorControl.configuration.friction_coulomb_pos_a = param->friction_coulomb_pos_a;
		MotorControl.configuration.friction_coulomb_neg_a = param->friction_coulomb_neg_a;
		MotorControl.configuration.friction_viscous_pos_a_per_rad_s = param->friction_viscous_pos_a_per_rad_s;
		MotorControl.configuration.friction_viscous_neg_a_per_rad_s = param->friction_viscous_neg_a_per_rad_s;
		MotorControl.configuration.friction_model_valid = true;
	}
	if (param->schema_version == PARAM_SCHEMA_VERSION &&
		(OnBoard_Encoder.calib_flag & ENC_CALIB_COGGING) != 0U)
	{
		for (index = 0U; index < ENCODER_COGGING_MAP_SIZE; ++index)
			OnBoard_Encoder.cogging_compensation_map_ma[index] = param->cogging_compensation_map_ma[index];
	}
	else
		OnBoard_Encoder.calib_flag &= (uint8_t)~ENC_CALIB_COGGING;
}
