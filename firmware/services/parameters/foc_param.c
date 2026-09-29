#include "foc_param.h"

/* 当前级联位置速度上限；存储布局中的退役字段不再装载。 */
#define PARAM_POSITION_MAX_SPEED_RPS 0.50f

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "crc32.h"
#include "parameter_value_service.h"
#include "foc_sensing.h"

/* 参数读写与默认值：限值取自平台感测契约，数值换算不访问硬件。 */
#include "current_sense_profile.h"
#include "motor_types.h"
#include "foc_param_profile.h"
#include "param_comm_bridge.h"
#include "utils.h"

#include "foc_cogging_calibration.h"

uint32_t Param_RecordCrc(const InterfaceParam *param)
{
    uint32_t offset = (uint32_t)offsetof(InterfaceParam, record_crc32);
    uint32_t crc = Crc32_Compute(param, offset, 0U);
    uint32_t tail = offset + sizeof(param->record_crc32);
    return Crc32_Compute((const uint8_t *)param + tail, sizeof(*param) - tail, crc);
}

/* startup adapter only: the portable module owns joint IDs and baselines.
 * Individual motor/current calibration and stored axis bounds are untouched. */
static bool param_apply_joint_profile(MotorControlState *motor)
{
    MotorJointControlConfig config;
    if (motor == NULL || !motor->axis_profile_valid)
    {
        return false;
    }
    if (motor->axis_profile.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC)
    {
        return true;
    }
    if (!isfinite(motor->speed_limit_rad_s) || motor->speed_limit_rad_s <= 0.0f ||
        !isfinite(motor->current_limit_a) || motor->current_limit_a <= 0.0f)
    {
        return false;
    }
    if (!MotorAxisProfile_Resolve(&motor->axis_profile, &config))
    {
        return false;
    }
    motor->cascade_position_kp = config.position_kp;
    motor->cascade_position_kd = config.position_kd;
    motor->speed_kp = config.speed_kp;
    motor->speed_ki = config.speed_ki;
    motor->position_acceleration_rad_s2 = config.acceleration_rad_s2;
    motor->position_deceleration_rad_s2 = config.deceleration_rad_s2;
    motor->position_maximum_speed_rad_s =
        fminf(motor->axis_profile.maximum_speed_rad_s, motor->speed_limit_rad_s);
    motor->current_limit_a = fminf(motor->current_limit_a, config.maximum_current_a);
    return true;
}

void Param_Return_Default(MotorControlState *motor_state, EncoderState *rotor_encoder)
{
    FocCogging_ResetMap();
    memset(&motor_state->axis_profile, 0, sizeof(motor_state->axis_profile));
    motor_state->axis_profile_valid = true;
    motor_state->configuration_confirmed = false;
    motor_state->configuration_source = MOTOR_CONFIGURATION_DEFAULTS;
    Protocol_NodeId_Set((uint8_t)PARAM_HW_CAN_NODE_ID);

    motor_state->phase_a_offset_counts = PARAM_HW_CURRENT_OFFSET_A_COUNTS;
    motor_state->phase_b_offset_counts = PARAM_HW_CURRENT_OFFSET_B_COUNTS;
    motor_state->phase_c_offset_counts = PARAM_HW_CURRENT_OFFSET_C_COUNTS;

    motor_state->motor_pole_pairs = PARAM_MOTOR_POLE_PAIRS;
    motor_state->motor_phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM;
    motor_state->motor_d_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H;
    motor_state->motor_q_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H;
    motor_state->motor_flux_wb = PARAM_MOTOR_FLUX_WB;
    motor_state->id_kp =
        motor_state->motor_d_inductance_h * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
    motor_state->iq_kp =
        motor_state->motor_q_inductance_h * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
    motor_state->id_ki =
        motor_state->motor_phase_resistance_ohm * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
    motor_state->iq_ki =
        motor_state->motor_phase_resistance_ohm * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;

    rotor_encoder->electrical_zero_q15 = PARAM_APP_ENCODER_ELECTRICAL_ZERO_Q15;
    rotor_encoder->mechanical_zero_q15 = PARAM_APP_ENCODER_MECHANICAL_ZERO_Q15;
    rotor_encoder->calib_flag = PARAM_APP_ENCODER_CALIB_FLAG;
    rotor_encoder->reverse = PARAM_APP_ENCODER_REVERSE;
    memset(rotor_encoder->linearization_lut_q15, 0, sizeof(rotor_encoder->linearization_lut_q15));

    motor_state->calibration_current_a = PARAM_MOTOR_CALIB_CURRENT_A;
    motor_state->current_limit_a = PARAM_MOTOR_CURRENT_LIMIT_A;
    motor_state->vq_target_v = 0.0f;
    motor_state->speed_limit_rad_s = Param_SpeedLimitRadS(Protocol_NodeId_Get());
    motor_state->speed_acceleration_rad_s2 = PARAM_APP_SPEED_ACCEL_RPS2 * _2PI;
    motor_state->speed_deceleration_rad_s2 = PARAM_APP_SPEED_DECEL_RPS2 * _2PI;
    motor_state->speed_kp = PARAM_APP_SPEED_KP;
    motor_state->speed_ki = PARAM_APP_SPEED_KI;
    motor_state->position_acceleration_rad_s2 = PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
    motor_state->position_deceleration_rad_s2 = PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
    motor_state->position_maximum_speed_rad_s = PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI;
    motor_state->cascade_position_kp = PARAM_APP_CASCADE_POSITION_KP;
    motor_state->cascade_position_kd = PARAM_APP_CASCADE_POSITION_KD;
    motor_state->friction_coulomb_pos_a = 0.0f;
    motor_state->friction_coulomb_neg_a = 0.0f;
    motor_state->friction_viscous_pos_a_per_rad_s = 0.0f;
    motor_state->friction_viscous_neg_a_per_rad_s = 0.0f;
    motor_state->friction_model_valid = false;
}

void Param_Upload(InterfaceParam *param,
                  const MotorControlState *motor_state,
                  const EncoderState *rotor_encoder)
{
    uint32_t lut_index;

    memset(param, 0, sizeof(*param));
    param->node_id = (float)Protocol_NodeId_Get();
    param->currentoffset_a = (float)motor_state->phase_a_offset_counts;
    param->currentoffset_b = (float)motor_state->phase_b_offset_counts;
    param->currentoffset_c = (float)motor_state->phase_c_offset_counts;
    param->motor_pole_pairs = (float)motor_state->motor_pole_pairs;
    param->motor_phase_resistance_ohm = motor_state->motor_phase_resistance_ohm;
    param->motor_d_inductance_h = motor_state->motor_d_inductance_h;
    param->motor_q_inductance_h = motor_state->motor_q_inductance_h;
    param->motor_flux_wb = motor_state->motor_flux_wb;
    param->encoder_electrical_zero_q15 = rotor_encoder->electrical_zero_q15;
    param->encoder_mechanical_zero_q15 = rotor_encoder->mechanical_zero_q15;
    param->encoder_calib_flag = rotor_encoder->calib_flag;
    param->encoder_reverse = rotor_encoder->reverse;
    for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
    {
        param->encoder_linearization_lut_q15[lut_index] =
            rotor_encoder->linearization_lut_q15[lut_index];
    }
    param->calibration_current_a = motor_state->calibration_current_a;
    param->current_limit_a = motor_state->current_limit_a;
    param->id_kp = motor_state->id_kp;
    param->id_ki = motor_state->id_ki;
    param->iq_kp = motor_state->iq_kp;
    param->iq_ki = motor_state->iq_ki;
    param->speed_limit_rad_s = motor_state->speed_limit_rad_s;
    param->speed_acceleration_rad_s2 = motor_state->speed_acceleration_rad_s2;
    param->speed_deceleration_rad_s2 = motor_state->speed_deceleration_rad_s2;
    param->speed_kp = motor_state->speed_kp;
    param->speed_ki = motor_state->speed_ki;
    param->position_acceleration_rad_s2 = motor_state->position_acceleration_rad_s2;
    param->position_deceleration_rad_s2 = motor_state->position_deceleration_rad_s2;
    param->position_maximum_speed_rad_s = motor_state->position_maximum_speed_rad_s;
    param->current_sense_shunt_milliohm = CURRENT_SENSE_SHUNT_MILLIOHM;
    param->cascade_pos_kp = motor_state->cascade_position_kp;
    param->cascade_pos_kd = motor_state->cascade_position_kd;
    param->friction_coulomb_pos_a = motor_state->friction_coulomb_pos_a;
    param->friction_coulomb_neg_a = motor_state->friction_coulomb_neg_a;
    param->friction_viscous_pos_a_per_rad_s = motor_state->friction_viscous_pos_a_per_rad_s;
    param->friction_viscous_neg_a_per_rad_s = motor_state->friction_viscous_neg_a_per_rad_s;
    param->friction_model_valid = motor_state->friction_model_valid ? 1U : 0U;
    param->schema_version = PARAM_SCHEMA_VERSION;
    param->magic_word = MAGIC_WORD;
    param->configuration_flags = motor_state->configuration_confirmed ? 1U : 0U;
    param->axis_profile = motor_state->axis_profile;
    FocCogging_ExportMap(&param->cogging, motor_state, rotor_encoder);
    param->record_crc32 = Param_RecordCrc(param);
}

void Param_Download(const InterfaceParam *param,
                    MotorControlState *motor_state,
                    EncoderState *rotor_encoder)
{
    uint32_t lut_index;
    bool current_sense_profile_changed;
    float position_speed_limit;

    if (param->magic_word != MAGIC_WORD || param->schema_version != PARAM_SCHEMA_VERSION)
    {
        Param_Return_Default(motor_state, rotor_encoder);
        motor_state->configuration_source =
            param->magic_word == UINT32_MAX && param->schema_version == UINT32_MAX
                ? MOTOR_CONFIGURATION_DEFAULTS
                : MOTOR_CONFIGURATION_UNSUPPORTED;
        return;
    }
    if (param->record_crc32 != Param_RecordCrc(param) || param->configuration_flags > 1U ||
        !isfinite(param->motor_pole_pairs) || param->motor_pole_pairs < 1.0f ||
        param->motor_pole_pairs >= 2147483648.0f || !isfinite(param->node_id) ||
        param->node_id < 0.0f || param->node_id > 255.0f)
    {
        Param_Return_Default(motor_state, rotor_encoder);
        motor_state->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }
    current_sense_profile_changed =
        param->current_sense_shunt_milliohm != CURRENT_SENSE_SHUNT_MILLIOHM;
    if (param->encoder_reverse > 1U || (param->encoder_calib_flag & ~ENC_CALIB_ALL) != 0U)
    {
        Param_Return_Default(motor_state, rotor_encoder);
        motor_state->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }

    /* Resolve axis identity before selecting the node-specific speed ceiling. */
    Protocol_NodeId_Set(MotorAxisProfile_CanNodeId(&param->axis_profile, (uint8_t)param->node_id));
    motor_state->phase_a_offset_counts = param->currentoffset_a;
    motor_state->phase_b_offset_counts = param->currentoffset_b;
    motor_state->phase_c_offset_counts = param->currentoffset_c;
    motor_state->motor_pole_pairs = (int32_t)param->motor_pole_pairs;
    motor_state->motor_phase_resistance_ohm = param->motor_phase_resistance_ohm;
    motor_state->motor_d_inductance_h = param->motor_d_inductance_h;
    motor_state->motor_q_inductance_h = param->motor_q_inductance_h;
    motor_state->motor_flux_wb = param->motor_flux_wb;
    rotor_encoder->electrical_zero_q15 = param->encoder_electrical_zero_q15;
    rotor_encoder->mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
    rotor_encoder->calib_flag = param->encoder_calib_flag;
    rotor_encoder->reverse = param->encoder_reverse;
    for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
    {
        rotor_encoder->linearization_lut_q15[lut_index] =
            param->encoder_linearization_lut_q15[lut_index];
    }
    /* A scaling change invalidates saved current-domain defaults, but not calibration data. */
    if (current_sense_profile_changed || !isfinite(param->calibration_current_a) ||
        param->calibration_current_a < 0.0f)
    {
        motor_state->calibration_current_a = PARAM_MOTOR_CALIB_CURRENT_A;
    }
    else
    {
        motor_state->calibration_current_a = Math_Constrain(
            param->calibration_current_a, 0.0f, CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A);
    }

    if (current_sense_profile_changed || !isfinite(param->current_limit_a) ||
        param->current_limit_a <= 0.0f)
    {
        motor_state->current_limit_a = PARAM_MOTOR_CURRENT_LIMIT_A;
    }
    else
    {
        motor_state->current_limit_a =
            Math_Constrain(param->current_limit_a, 0.0f, CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A);
    }
    motor_state->id_kp = param->id_kp;
    motor_state->id_ki = param->id_ki;
    motor_state->iq_kp = param->iq_kp;
    motor_state->iq_ki = param->iq_ki;
    motor_state->speed_limit_rad_s =
        isfinite(param->speed_limit_rad_s) && param->speed_limit_rad_s > 0.0f
            ? Math_Constrain(
                  param->speed_limit_rad_s, 0.0f, Param_SpeedLimitRadS(Protocol_NodeId_Get()))
            : Param_SpeedLimitRadS(Protocol_NodeId_Get());
    position_speed_limit =
        Math_Min(motor_state->speed_limit_rad_s, PARAM_POSITION_MAX_SPEED_RPS * _2PI);
    motor_state->speed_acceleration_rad_s2 = param->speed_acceleration_rad_s2;
    motor_state->speed_deceleration_rad_s2 = param->speed_deceleration_rad_s2;
    motor_state->speed_kp = param->speed_kp;
    motor_state->speed_ki = param->speed_ki;
    motor_state->position_acceleration_rad_s2 =
        isfinite(param->position_acceleration_rad_s2) && param->position_acceleration_rad_s2 > 0.0f
            ? Math_Constrain(param->position_acceleration_rad_s2, 0.0f, 200.0f * _2PI)
            : PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
    motor_state->position_deceleration_rad_s2 =
        isfinite(param->position_deceleration_rad_s2) && param->position_deceleration_rad_s2 > 0.0f
            ? Math_Constrain(param->position_deceleration_rad_s2, 0.0f, 200.0f * _2PI)
            : PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
    motor_state->position_maximum_speed_rad_s =
        isfinite(param->position_maximum_speed_rad_s) && param->position_maximum_speed_rad_s > 0.0f
            ? Math_Constrain(param->position_maximum_speed_rad_s, 0.0f, position_speed_limit)
            : Math_Min(PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI, position_speed_limit);
    motor_state->cascade_position_kp =
        isfinite(param->cascade_pos_kp) && param->cascade_pos_kp >= 0.0f
            ? Math_Constrain(param->cascade_pos_kp, 0.0f, CASCADE_POSITION_KP_MAX_PER_S)
            : PARAM_APP_CASCADE_POSITION_KP;
    motor_state->cascade_position_kd =
        isfinite(param->cascade_pos_kd) && param->cascade_pos_kd >= 0.0f
            ? Math_Constrain(param->cascade_pos_kd, 0.0f, CASCADE_POSITION_KD_MAX)
            : PARAM_APP_CASCADE_POSITION_KD;
    if (!current_sense_profile_changed && param->friction_model_valid == 1U &&
        isfinite(param->friction_coulomb_pos_a) && param->friction_coulomb_pos_a >= 0.0f &&
        isfinite(param->friction_coulomb_neg_a) && param->friction_coulomb_neg_a >= 0.0f &&
        isfinite(param->friction_viscous_pos_a_per_rad_s) &&
        param->friction_viscous_pos_a_per_rad_s >= 0.0f &&
        isfinite(param->friction_viscous_neg_a_per_rad_s) &&
        param->friction_viscous_neg_a_per_rad_s >= 0.0f)
    {
        motor_state->friction_coulomb_pos_a = param->friction_coulomb_pos_a;
        motor_state->friction_coulomb_neg_a = param->friction_coulomb_neg_a;
        motor_state->friction_viscous_pos_a_per_rad_s = param->friction_viscous_pos_a_per_rad_s;
        motor_state->friction_viscous_neg_a_per_rad_s = param->friction_viscous_neg_a_per_rad_s;
        motor_state->friction_model_valid = true;
    }
    else
    {
        motor_state->friction_coulomb_pos_a = 0.0f;
        motor_state->friction_coulomb_neg_a = 0.0f;
        motor_state->friction_viscous_pos_a_per_rad_s = 0.0f;
        motor_state->friction_viscous_neg_a_per_rad_s = 0.0f;
        motor_state->friction_model_valid = false;
    }
    motor_state->axis_profile_valid =
        MotorAxisProfile_Load(&param->axis_profile, &motor_state->axis_profile);
    motor_state->axis_profile_valid = param_apply_joint_profile(motor_state);
    FocCogging_ResetMap();
    if (!current_sense_profile_changed &&
        param->cogging.full_scale_a == CURRENT_SENSE_PROFILE_FULL_SCALE_A &&
        (rotor_encoder->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
        CoggingMap_Valid(&param->cogging,
                         Cogging_EncoderSignature(rotor_encoder->reverse,
                                                  rotor_encoder->electrical_zero_q15,
                                                  motor_state->motor_pole_pairs,
                                                  rotor_encoder->linearization_lut_q15)))
    {
        FocCogging_ImportMap(&param->cogging);
    }
    motor_state->configuration_source = MOTOR_CONFIGURATION_STORED;
    motor_state->configuration_confirmed =
        param->configuration_flags == 1U && !current_sense_profile_changed;
    if (!ParameterValue_ConfigurationValid(motor_state) || !MotorCurrentOffsets_Valid(motor_state))
    {
        Param_Return_Default(motor_state, rotor_encoder);
        motor_state->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
    }
}
