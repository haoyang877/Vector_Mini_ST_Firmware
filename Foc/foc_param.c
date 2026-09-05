#include "foc_param.h"

#include <string.h>
#include "common_inc.h"
#include "foc_param_profile.h"

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder;
extern CANMsg_TypeDef CANMsg;

void Param_Return_Default(void)
{
	CANMsg.node_id = PARAM_HW_CAN_NODE_ID;

	MotorControl.A_Offset = PARAM_HW_CURRENT_OFFSET_A_COUNTS;
	MotorControl.B_Offset = PARAM_HW_CURRENT_OFFSET_B_COUNTS;
	MotorControl.C_Offset = PARAM_HW_CURRENT_OFFSET_C_COUNTS;

	MotorControl.motor_pole_pairs = PARAM_MOTOR_POLE_PAIRS;
	MotorControl.motor_phase_resistance = PARAM_MOTOR_PHASE_RESISTANCE_OHM;
	MotorControl.motor_d_inductance = PARAM_MOTOR_D_INDUCTANCE_H;
	MotorControl.motor_q_inductance = PARAM_MOTOR_Q_INDUCTANCE_H;
	MotorControl.motor_flux = PARAM_MOTOR_FLUX_WB;
	MotorControl.id_Kp = MotorControl.motor_d_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Kp = MotorControl.motor_q_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.id_Ki = MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Ki = MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.ol_voltage = PARAM_APP_OPEN_LOOP_VOLTAGE_V;
	MotorControl.ol_elec_vel = PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S;
	MotorControl.ol_theta = PARAM_APP_OPEN_LOOP_THETA_RAD;

	OnBoard_Encoder.electrical_zero_q15 = PARAM_APP_ENCODER_ELECTRICAL_ZERO_Q15;
	OnBoard_Encoder.mechanical_zero_q15 = PARAM_APP_ENCODER_MECHANICAL_ZERO_Q15;
	OnBoard_Encoder.calib_flag = PARAM_APP_ENCODER_CALIB_FLAG;
	OnBoard_Encoder.reverse = PARAM_APP_ENCODER_REVERSE;
	memset(OnBoard_Encoder.linearization_lut_q15, 0, sizeof(OnBoard_Encoder.linearization_lut_q15));

	MotorControl.calib_current = PARAM_MOTOR_CALIB_CURRENT_A;
	MotorControl.current_limit = PARAM_MOTOR_CURRENT_LIMIT_A;
	MotorControl.vqRef = 0.0f;
	MotorControl.speed_limit = PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI;
	MotorControl.speedAcc = PARAM_APP_SPEED_ACCEL_RPS2 * _2PI;
	MotorControl.speedDec = PARAM_APP_SPEED_DECEL_RPS2 * _2PI;
	MotorControl.speed_Kp = PARAM_APP_SPEED_KP;
	MotorControl.speed_Ki = PARAM_APP_SPEED_KI;
	MotorControl.posAcc = PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
	MotorControl.posDec = PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
	MotorControl.pos_maxspeed = PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI;
	MotorControl.pos_Kp = PARAM_APP_POSITION_KP;
	MotorControl.pos_Kd = PARAM_APP_POSITION_KD;
	MotorControl.pos_Ki = PARAM_APP_POSITION_KI;
	MotorControl.pos_integral_limit = PARAM_APP_POSITION_INTEGRAL_LIMIT_A;
	MotorControl.cascade_pos_Kp = PARAM_APP_CASCADE_POSITION_KP;
	MotorControl.cascade_pos_Kd = PARAM_APP_CASCADE_POSITION_KD;
	MotorControl.friction_coulomb_pos_a = 0.0f;
	MotorControl.friction_coulomb_neg_a = 0.0f;
	MotorControl.friction_viscous_pos_a_per_rad_s = 0.0f;
	MotorControl.friction_viscous_neg_a_per_rad_s = 0.0f;
	MotorControl.friction_model_valid = false;
	CANMsg.can_hb_set = PARAM_HW_CAN_HEARTBEAT_MS;

	MotorControl.ModeNow = Save_Param;
}

void Param_Upload(InterfaceParam_TypeDef *param)
{
	uint32_t lut_index;

	memset(param, 0, sizeof(*param));
	param->node_id = (float)CANMsg.node_id;
	param->currentoffset_a = (float)MotorControl.A_Offset;
	param->currentoffset_b = (float)MotorControl.B_Offset;
	param->currentoffset_c = (float)MotorControl.C_Offset;
	param->motor_pole_pairs = (float)MotorControl.motor_pole_pairs;
	param->motor_phase_resistance = MotorControl.motor_phase_resistance;
	param->motor_d_inductance = MotorControl.motor_d_inductance;
	param->motor_q_inductance = MotorControl.motor_q_inductance;
	param->motor_flux = MotorControl.motor_flux;
	param->encoder_electrical_zero_q15 = OnBoard_Encoder.electrical_zero_q15;
	param->encoder_mechanical_zero_q15 = OnBoard_Encoder.mechanical_zero_q15;
	param->encoder_calib_flag = OnBoard_Encoder.calib_flag;
	param->encoder_reverse = OnBoard_Encoder.reverse;
	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
		param->encoder_linearization_lut_q15[lut_index] = OnBoard_Encoder.linearization_lut_q15[lut_index];
	param->calib_current = MotorControl.calib_current;
	param->current_limit = MotorControl.current_limit;
	param->id_kp = MotorControl.id_Kp;
	param->id_ki = MotorControl.id_Ki;
	param->iq_kp = MotorControl.iq_Kp;
	param->iq_ki = MotorControl.iq_Ki;
	param->speed_limit = MotorControl.speed_limit;
	param->speedAcc = MotorControl.speedAcc;
	param->speedDec = MotorControl.speedDec;
	param->speed_kp = MotorControl.speed_Kp;
	param->speed_ki = MotorControl.speed_Ki;
	param->posAcc = MotorControl.posAcc;
	param->posDec = MotorControl.posDec;
	param->pos_maxspeed = MotorControl.pos_maxspeed;
	param->pos_kp = MotorControl.pos_Kp;
	param->pos_kd = MotorControl.pos_Kd;
	param->pos_ki = MotorControl.pos_Ki;
	param->current_sense_shunt_milliohm = CURRENT_SENSE_SHUNT_MILLIOHM;
	param->pos_integral_limit = MotorControl.pos_integral_limit;
	param->cascade_pos_kp = MotorControl.cascade_pos_Kp;
	param->cascade_pos_kd = MotorControl.cascade_pos_Kd;
	param->friction_coulomb_pos_a = MotorControl.friction_coulomb_pos_a;
	param->friction_coulomb_neg_a = MotorControl.friction_coulomb_neg_a;
	param->friction_viscous_pos_a_per_rad_s =
		MotorControl.friction_viscous_pos_a_per_rad_s;
	param->friction_viscous_neg_a_per_rad_s =
		MotorControl.friction_viscous_neg_a_per_rad_s;
	param->friction_model_valid = MotorControl.friction_model_valid ? 1U : 0U;
	param->can_hb = (float)CANMsg.can_hb_set;
	param->schema_version = PARAM_SCHEMA_VERSION;
}

void Param_Download(const InterfaceParam_TypeDef *param)
{
	uint32_t lut_index;
	uint32_t stored_shunt_milliohm;
	bool legacy_cascade_parameters;
	bool current_sense_profile_changed;
	float position_speed_limit;

	if (param->magic_word != MAGIC_WORD ||
		(param->schema_version != PARAM_SCHEMA_VERSION &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_FRICTION &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_INTEGRAL_LIMIT &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_CURRENT_SENSE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_CASCADE))
	{
		Param_Return_Default();
		return;
	}
	legacy_cascade_parameters = param->schema_version == PARAM_SCHEMA_VERSION_LEGACY_CASCADE;
	if (param->schema_version == PARAM_SCHEMA_VERSION_LEGACY_CASCADE)
		stored_shunt_milliohm = CURRENT_SENSE_SHUNT_2_MILLIOHM;
	else if (param->schema_version == PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE)
		stored_shunt_milliohm = CURRENT_SENSE_SHUNT_6_MILLIOHM;
	else
		stored_shunt_milliohm = param->current_sense_shunt_milliohm;
	current_sense_profile_changed = stored_shunt_milliohm != CURRENT_SENSE_SHUNT_MILLIOHM;
	if (param->encoder_reverse > 1U)
	{
		Param_Return_Default();
		return;
	}

	CANMsg.node_id = (uint8_t)param->node_id;
	MotorControl.A_Offset = (uint16_t)param->currentoffset_a;
	MotorControl.B_Offset = (uint16_t)param->currentoffset_b;
	MotorControl.C_Offset = (uint16_t)param->currentoffset_c;
	MotorControl.motor_pole_pairs = (int32_t)param->motor_pole_pairs;
	MotorControl.motor_phase_resistance = param->motor_phase_resistance;
	MotorControl.motor_d_inductance = param->motor_d_inductance;
	MotorControl.motor_q_inductance = param->motor_q_inductance;
	MotorControl.motor_flux = param->motor_flux;
	OnBoard_Encoder.electrical_zero_q15 = param->encoder_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = param->encoder_calib_flag;
	OnBoard_Encoder.reverse = param->encoder_reverse;
	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
		OnBoard_Encoder.linearization_lut_q15[lut_index] = param->encoder_linearization_lut_q15[lut_index];
	/* A scaling change invalidates saved current-domain defaults, but not calibration data. */
	if (current_sense_profile_changed ||
		!isfinite(param->calib_current) || param->calib_current < 0.0f)
		MotorControl.calib_current = PARAM_MOTOR_CALIB_CURRENT_A;
	else
		MotorControl.calib_current = constrain(param->calib_current, 0.0f, CURRENT_CALIB_LIMIT_MAX_A);

	if (current_sense_profile_changed ||
		!isfinite(param->current_limit) || param->current_limit <= 0.0f)
		MotorControl.current_limit = PARAM_MOTOR_CURRENT_LIMIT_A;
	else
		MotorControl.current_limit = constrain(param->current_limit, 0.0f, CURRENT_COMMAND_LIMIT_MAX_A);
	MotorControl.id_Kp = param->id_kp;
	MotorControl.id_Ki = param->id_ki;
	MotorControl.iq_Kp = param->iq_kp;
	MotorControl.iq_Ki = param->iq_ki;
	MotorControl.speed_limit = isfinite(param->speed_limit) && param->speed_limit > 0.0f ?
		constrain(param->speed_limit, 0.0f, PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI) :
		PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI;
	position_speed_limit = fast_min(MotorControl.speed_limit,
		POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI);
	MotorControl.speedAcc = param->speedAcc;
	MotorControl.speedDec = param->speedDec;
	MotorControl.speed_Kp = param->speed_kp;
	MotorControl.speed_Ki = param->speed_ki;
	/*
	 * Schema v4 gains drove the speed PI and have incompatible units. Preserve
	 * motor/encoder calibration, but migrate the position settings to the safe
	 * low-speed impedance defaults.
	 */
	if (legacy_cascade_parameters)
	{
		MotorControl.posAcc = PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
		MotorControl.posDec = PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
		MotorControl.pos_maxspeed = fast_min(PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI,
			position_speed_limit);
		MotorControl.pos_Kp = PARAM_APP_POSITION_KP;
		MotorControl.pos_Kd = PARAM_APP_POSITION_KD;
		MotorControl.pos_Ki = PARAM_APP_POSITION_KI;
		MotorControl.pos_integral_limit = PARAM_APP_POSITION_INTEGRAL_LIMIT_A;
		MotorControl.cascade_pos_Kp =
			isfinite(param->pos_kp) && param->pos_kp >= 0.0f ?
			constrain(param->pos_kp, 0.0f, CASCADE_POSITION_KP_MAX_PER_S) :
			PARAM_APP_CASCADE_POSITION_KP;
		MotorControl.cascade_pos_Kd =
			isfinite(param->pos_kd) && param->pos_kd >= 0.0f ?
			constrain(param->pos_kd, 0.0f, CASCADE_POSITION_KD_MAX) :
			PARAM_APP_CASCADE_POSITION_KD;
	}
	else
	{
		MotorControl.posAcc = isfinite(param->posAcc) && param->posAcc > 0.0f ?
			constrain(param->posAcc, 0.0f, 200.0f * _2PI) :
			PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
		MotorControl.posDec = isfinite(param->posDec) && param->posDec > 0.0f ?
			constrain(param->posDec, 0.0f, 200.0f * _2PI) :
			PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
		MotorControl.pos_maxspeed = isfinite(param->pos_maxspeed) && param->pos_maxspeed > 0.0f ?
			constrain(param->pos_maxspeed, 0.0f, position_speed_limit) :
			fast_min(PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI, position_speed_limit);
		MotorControl.pos_Kp = isfinite(param->pos_kp) && param->pos_kp >= 0.0f ?
			constrain(param->pos_kp, 0.0f, POSITION_IMPEDANCE_KP_MAX_A_PER_RAD) :
			PARAM_APP_POSITION_KP;
		MotorControl.pos_Kd = isfinite(param->pos_kd) && param->pos_kd >= 0.0f ?
			constrain(param->pos_kd, 0.0f, POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S) :
			PARAM_APP_POSITION_KD;
		MotorControl.pos_Ki = isfinite(param->pos_ki) && param->pos_ki >= 0.0f ?
			constrain(param->pos_ki, 0.0f, POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S) :
			PARAM_APP_POSITION_KI;
		MotorControl.pos_integral_limit =
			param->schema_version >= PARAM_SCHEMA_VERSION_LEGACY_INTEGRAL_LIMIT &&
			isfinite(param->pos_integral_limit) && param->pos_integral_limit >= 0.0f ?
			constrain(param->pos_integral_limit, 0.0f, CURRENT_COMMAND_LIMIT_MAX_A) :
			PARAM_APP_POSITION_INTEGRAL_LIMIT_A;
		MotorControl.cascade_pos_Kp =
			param->schema_version >= PARAM_SCHEMA_VERSION_LEGACY_FRICTION &&
			isfinite(param->cascade_pos_kp) && param->cascade_pos_kp >= 0.0f ?
			constrain(param->cascade_pos_kp, 0.0f, CASCADE_POSITION_KP_MAX_PER_S) :
			PARAM_APP_CASCADE_POSITION_KP;
		MotorControl.cascade_pos_Kd =
			param->schema_version >= PARAM_SCHEMA_VERSION_LEGACY_FRICTION &&
			isfinite(param->cascade_pos_kd) && param->cascade_pos_kd >= 0.0f ?
			constrain(param->cascade_pos_kd, 0.0f, CASCADE_POSITION_KD_MAX) :
			PARAM_APP_CASCADE_POSITION_KD;
	}
	if (param->schema_version == PARAM_SCHEMA_VERSION &&
		!current_sense_profile_changed && param->friction_model_valid == 1U &&
		isfinite(param->friction_coulomb_pos_a) && param->friction_coulomb_pos_a >= 0.0f &&
		isfinite(param->friction_coulomb_neg_a) && param->friction_coulomb_neg_a >= 0.0f &&
		isfinite(param->friction_viscous_pos_a_per_rad_s) &&
		param->friction_viscous_pos_a_per_rad_s >= 0.0f &&
		isfinite(param->friction_viscous_neg_a_per_rad_s) &&
		param->friction_viscous_neg_a_per_rad_s >= 0.0f)
	{
		MotorControl.friction_coulomb_pos_a = param->friction_coulomb_pos_a;
		MotorControl.friction_coulomb_neg_a = param->friction_coulomb_neg_a;
		MotorControl.friction_viscous_pos_a_per_rad_s =
			param->friction_viscous_pos_a_per_rad_s;
		MotorControl.friction_viscous_neg_a_per_rad_s =
			param->friction_viscous_neg_a_per_rad_s;
		MotorControl.friction_model_valid = true;
	}
	else
	{
		MotorControl.friction_coulomb_pos_a = 0.0f;
		MotorControl.friction_coulomb_neg_a = 0.0f;
		MotorControl.friction_viscous_pos_a_per_rad_s = 0.0f;
		MotorControl.friction_viscous_neg_a_per_rad_s = 0.0f;
		MotorControl.friction_model_valid = false;
	}
	CANMsg.can_hb_set = (uint32_t)param->can_hb;
}
