#include "parameter_record.h"

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "crc32.h"
#include "parameter_values.h"
#include "foc_sensing.h"

/* 参数读写与默认值：限值取自平台感测契约，数值换算不访问硬件。 */
#include "current_sense_profile.h"
#include "motor_axis_profile.h"
#include "position_cascade_config.h"
#include "motor_profile_defaults.h"
#include "board_profile_defaults.h"
#include "application_profile_defaults.h"
#include "device_profile_defaults.h"
#include "control_math.h"

static uint32_t record_crc(const void *record, uint32_t bytes)
{
    uint32_t offset = (uint32_t)offsetof(InterfaceParam, record_crc32);
    uint32_t crc = Crc32_Compute(record, offset, 0U);
    uint32_t tail = offset + sizeof(uint32_t);
    return Crc32_Compute((const uint8_t *)record + tail, bytes - tail, crc);
}

uint32_t Param_RecordCrc(const InterfaceParam *param)
{
    return record_crc(param, sizeof(*param));
}

typedef char Schema14Layout
    [(offsetof(InterfaceParam, thermal) == 2240U && sizeof(InterfaceParam) == 2288U) ? 1 : -1];

/* 应用包络来自编译配置；有效轴记录只收窄，通信地址不参与控制限值。 */
static float application_speed_ceiling_rad_s(const MotorAxisProfile *profile, bool valid)
{
    float limit = PARAM_APP_SPEED_CEILING_RAD_S;
    if (valid && profile->magic != 0U && isfinite(profile->maximum_speed_rad_s) &&
        profile->maximum_speed_rad_s > 0.0f)
    {
        limit = fminf(limit, profile->maximum_speed_rad_s);
    }
    return limit;
}

/* 启动装配关节外环基线；不覆盖单台模型、电流标定或已保存的轴包络。 */
static bool param_apply_joint_profile(MotorConfiguration *configuration)
{
    MotorJointControlConfig config;
    if (configuration == NULL || !configuration->application.axis_profile_valid)
    {
        return false;
    }
    if (configuration->application.axis_profile.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC)
    {
        return true;
    }
    if (!isfinite(configuration->application.speed_limit_rad_s) ||
        configuration->application.speed_limit_rad_s <= 0.0f ||
        !isfinite(configuration->application.current_limit_a) ||
        configuration->application.current_limit_a <= 0.0f)
    {
        return false;
    }
    if (!MotorAxisProfile_Resolve(&configuration->application.axis_profile, &config))
    {
        return false;
    }
    configuration->application.cascade_position_kp = config.position_kp;
    configuration->application.cascade_position_kd = config.position_kd;
    configuration->application.speed_kp = config.speed_kp;
    configuration->application.speed_ki = config.speed_ki;
    configuration->application.position_acceleration_rad_s2 = config.acceleration_rad_s2;
    configuration->application.position_deceleration_rad_s2 = config.deceleration_rad_s2;
    configuration->application.position_maximum_speed_rad_s =
        fminf(configuration->application.axis_profile.maximum_speed_rad_s,
              configuration->application.speed_limit_rad_s);
    configuration->application.current_limit_a =
        fminf(configuration->application.current_limit_a, config.maximum_current_a);
    return true;
}

void Param_Return_Default(MotorConfiguration *configuration)
{
    MotorWindingThermal_Default(&configuration->model.thermal);
    configuration->application.speed_feedforward =
        (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    memset(&configuration->application.axis_profile,
           0,
           sizeof(configuration->application.axis_profile));
    configuration->application.axis_profile_valid = true;
    configuration->configuration_confirmed = false;
    configuration->configuration_source = MOTOR_CONFIGURATION_DEFAULTS;
    configuration->device.node_id = PARAM_DEVICE_CAN_NODE_ID;
    configuration->application.speed_ceiling_rad_s = PARAM_APP_SPEED_CEILING_RAD_S;

    configuration->device.phase_a_offset_counts = PARAM_HW_CURRENT_OFFSET_A_COUNTS;
    configuration->device.phase_b_offset_counts = PARAM_HW_CURRENT_OFFSET_B_COUNTS;
    configuration->device.phase_c_offset_counts = PARAM_HW_CURRENT_OFFSET_C_COUNTS;

    configuration->model.motor_pole_pairs = PARAM_MOTOR_POLE_PAIRS;
    configuration->model.motor_phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM;
    configuration->model.motor_d_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H;
    configuration->model.motor_q_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H;
    configuration->model.motor_flux_wb = PARAM_MOTOR_FLUX_WB;
    /* 仅首次/回退默认配置使用编译期派生 PI，已保存整定仍直接装载。 */
    configuration->model.id_kp = PARAM_MOTOR_DEFAULT_ID_KP;
    configuration->model.iq_kp = PARAM_MOTOR_DEFAULT_IQ_KP;
    configuration->model.id_ki = PARAM_MOTOR_DEFAULT_ID_KI;
    configuration->model.iq_ki = PARAM_MOTOR_DEFAULT_IQ_KI;

    configuration->device.encoder.electrical_zero_q15 = PARAM_DEVICE_ENCODER_ELECTRICAL_ZERO_Q15;
    configuration->device.encoder.mechanical_zero_q15 = PARAM_DEVICE_ENCODER_MECHANICAL_ZERO_Q15;
    configuration->device.encoder.calib_flag = PARAM_DEVICE_ENCODER_CALIB_FLAG;
    configuration->device.encoder.reverse = PARAM_DEVICE_ENCODER_REVERSE;
    memset(configuration->device.encoder.linearization_lut_q15,
           0,
           sizeof(configuration->device.encoder.linearization_lut_q15));

    configuration->model.calibration_current_a = PARAM_MOTOR_CALIB_CURRENT_A;
    configuration->application.current_limit_a = PARAM_APP_DEFAULT_CURRENT_LIMIT_A;
    /* 默认应用保持低速；统一上限不代表首次配置的活动速度。 */
    configuration->application.speed_limit_rad_s = Math_Min(
        configuration->application.speed_ceiling_rad_s, PARAM_APP_DEFAULT_SPEED_LIMIT_RAD_S);
    configuration->application.speed_acceleration_rad_s2 = PARAM_APP_SPEED_ACCEL_RAD_S2;
    configuration->application.speed_deceleration_rad_s2 = PARAM_APP_SPEED_DECEL_RAD_S2;
    configuration->application.speed_kp = PARAM_APP_SPEED_KP;
    configuration->application.speed_ki = PARAM_APP_SPEED_KI;
    configuration->application.position_acceleration_rad_s2 = PARAM_APP_POSITION_ACCEL_RAD_S2;
    configuration->application.position_deceleration_rad_s2 = PARAM_APP_POSITION_DECEL_RAD_S2;
    configuration->application.position_maximum_speed_rad_s = PARAM_APP_POSITION_MAX_SPEED_RAD_S;
    configuration->application.cascade_position_kp = PARAM_APP_CASCADE_POSITION_KP;
    configuration->application.cascade_position_kd = PARAM_APP_CASCADE_POSITION_KD;
}

void Param_Upload(InterfaceParam *param, const MotorConfiguration *configuration)
{
    uint32_t lut_index;

    memset(param, 0, sizeof(*param));
    param->node_id = (float)configuration->device.node_id;
    param->currentoffset_a = (float)configuration->device.phase_a_offset_counts;
    param->currentoffset_b = (float)configuration->device.phase_b_offset_counts;
    param->currentoffset_c = (float)configuration->device.phase_c_offset_counts;
    param->motor_pole_pairs = (float)configuration->model.motor_pole_pairs;
    param->motor_phase_resistance_ohm = configuration->model.motor_phase_resistance_ohm;
    param->motor_d_inductance_h = configuration->model.motor_d_inductance_h;
    param->motor_q_inductance_h = configuration->model.motor_q_inductance_h;
    param->motor_flux_wb = configuration->model.motor_flux_wb;
    param->encoder_electrical_zero_q15 = configuration->device.encoder.electrical_zero_q15;
    param->encoder_mechanical_zero_q15 = configuration->device.encoder.mechanical_zero_q15;
    param->encoder_calib_flag = configuration->device.encoder.calib_flag;
    param->encoder_reverse = configuration->device.encoder.reverse;
    for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
    {
        param->encoder_linearization_lut_q15[lut_index] =
            configuration->device.encoder.linearization_lut_q15[lut_index];
    }
    param->calibration_current_a = configuration->model.calibration_current_a;
    param->current_limit_a = configuration->application.current_limit_a;
    param->id_kp = configuration->model.id_kp;
    param->id_ki = configuration->model.id_ki;
    param->iq_kp = configuration->model.iq_kp;
    param->iq_ki = configuration->model.iq_ki;
    param->speed_limit_rad_s = configuration->application.speed_limit_rad_s;
    param->speed_acceleration_rad_s2 = configuration->application.speed_acceleration_rad_s2;
    param->speed_deceleration_rad_s2 = configuration->application.speed_deceleration_rad_s2;
    param->speed_kp = configuration->application.speed_kp;
    param->speed_ki = configuration->application.speed_ki;
    param->position_acceleration_rad_s2 = configuration->application.position_acceleration_rad_s2;
    param->position_deceleration_rad_s2 = configuration->application.position_deceleration_rad_s2;
    param->position_maximum_speed_rad_s = configuration->application.position_maximum_speed_rad_s;
    param->current_sense_shunt_milliohm = CURRENT_SENSE_SHUNT_MILLIOHM;
    param->cascade_pos_kp = configuration->application.cascade_position_kp;
    param->cascade_pos_kd = configuration->application.cascade_position_kd;
    param->friction_coulomb_pos_a = 0.0f;
    param->friction_coulomb_neg_a = 0.0f;
    param->friction_viscous_pos_a_per_rad_s = 0.0f;
    param->friction_viscous_neg_a_per_rad_s = 0.0f;
    param->friction_model_valid = 0U;
    param->schema_version = PARAM_SCHEMA_VERSION;
    param->magic_word = MAGIC_WORD;
    param->configuration_flags = configuration->configuration_confirmed ? 1U : 0U;
    param->axis_profile = configuration->application.axis_profile;
    param->thermal = configuration->model.thermal;
    param->speed_feedforward_enabled =
        configuration->application.speed_feedforward.enabled ? 1U : 0U;
    param->speed_feedforward_gain_a_per_rad_s2 =
        configuration->application.speed_feedforward.gain_a_per_rad_s2;
    param->speed_feedforward_acceleration_rad_s2 =
        configuration->application.speed_feedforward.acceleration_limit_rad_s2;
    param->speed_feedforward_jerk_rad_s3 =
        configuration->application.speed_feedforward.jerk_limit_rad_s3;
    param->speed_feedforward_current_limit_a =
        configuration->application.speed_feedforward.current_limit_a;
    param->record_crc32 = Param_RecordCrc(param);
}

void Param_Download(const void *record, uint32_t size_bytes, MotorConfiguration *configuration)
{
    const InterfaceParam *param = record;
    uint32_t lut_index;
    uint32_t record_bytes;
    bool current_sense_profile_changed;
    float position_speed_limit;

    if (record == NULL || size_bytes < offsetof(InterfaceParam, magic_word) + sizeof(uint32_t))
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }
    if (param->magic_word != MAGIC_WORD ||
        (param->schema_version != PARAM_SCHEMA_VERSION && param->schema_version != 12U &&
         param->schema_version != 13U))
    {
        Param_Return_Default(configuration);
        configuration->configuration_source =
            param->magic_word == UINT32_MAX && param->schema_version == UINT32_MAX
                ? MOTOR_CONFIGURATION_DEFAULTS
                : MOTOR_CONFIGURATION_UNSUPPORTED;
        return;
    }
    record_bytes = param->schema_version == 12U   ? PARAM_SCHEMA12_BYTES
                   : param->schema_version == 13U ? PARAM_SCHEMA13_BYTES
                                                  : sizeof(*param);
    if (size_bytes < record_bytes || param->record_crc32 != record_crc(record, record_bytes) ||
        param->configuration_flags > 1U || !isfinite(param->motor_pole_pairs) ||
        param->motor_pole_pairs < 1.0f || param->motor_pole_pairs >= 2147483648.0f ||
        !isfinite(param->node_id) || param->node_id < 1.0f || param->node_id >= 255.0f ||
        param->node_id != (float)(uint8_t)param->node_id)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }
    current_sense_profile_changed =
        param->current_sense_shunt_milliohm != CURRENT_SENSE_SHUNT_MILLIOHM;
    if (param->encoder_reverse > 1U || (param->encoder_calib_flag & ~ENC_CALIB_ALL) != 0U)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }

    /* 先给旧记录完整新增默认值，绝不读取schema12末尾之后的Flash垃圾。 */
    MotorWindingThermal_Default(&configuration->model.thermal);
    configuration->application.speed_feedforward =
        (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    if (param->schema_version != 12U)
    {
        /* 用小型对齐副本读取不同版本尾部，避免创建第二份完整参数或越界访问。 */
        struct
        {
            MotorWindingThermalConfig thermal;
            uint32_t speed_feedforward_enabled;
            float speed_feedforward_gain_a_per_rad_s2;
            float speed_feedforward_acceleration_rad_s2;
            float speed_feedforward_jerk_rad_s3;
            float speed_feedforward_current_limit_a;
        } extension;
        const uint32_t tail = offsetof(InterfaceParam, thermal);
        const uint32_t source_tail = param->schema_version == 13U ? PARAM_SCHEMA12_BYTES : tail;
        memcpy(&extension, (const uint8_t *)record + source_tail, sizeof(extension));
        SpeedFeedforwardConfig feedforward = {extension.speed_feedforward_enabled == 1U,
                                              extension.speed_feedforward_gain_a_per_rad_s2,
                                              extension.speed_feedforward_acceleration_rad_s2,
                                              extension.speed_feedforward_jerk_rad_s3,
                                              extension.speed_feedforward_current_limit_a};
        if (extension.speed_feedforward_enabled > 1U ||
            !MotorWindingThermal_Valid(&extension.thermal) ||
            !SpeedFeedforward_IsValid(&feedforward))
        {
            Param_Return_Default(configuration);
            configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
            return;
        }
        configuration->model.thermal = extension.thermal;
        configuration->application.speed_feedforward = feedforward;
    }

    /* 与 Loader 共用 node_id 字段；应用轴身份不能改写设备地址。 */
    configuration->device.node_id = (uint8_t)param->node_id;
    configuration->application.axis_profile_valid =
        MotorAxisProfile_Load(&param->axis_profile, &configuration->application.axis_profile);
    configuration->application.speed_ceiling_rad_s = application_speed_ceiling_rad_s(
        &configuration->application.axis_profile, configuration->application.axis_profile_valid);
    configuration->device.phase_a_offset_counts = param->currentoffset_a;
    configuration->device.phase_b_offset_counts = param->currentoffset_b;
    configuration->device.phase_c_offset_counts = param->currentoffset_c;
    configuration->model.motor_pole_pairs = (int32_t)param->motor_pole_pairs;
    configuration->model.motor_phase_resistance_ohm = param->motor_phase_resistance_ohm;
    configuration->model.motor_d_inductance_h = param->motor_d_inductance_h;
    configuration->model.motor_q_inductance_h = param->motor_q_inductance_h;
    configuration->model.motor_flux_wb = param->motor_flux_wb;
    configuration->device.encoder.electrical_zero_q15 = param->encoder_electrical_zero_q15;
    configuration->device.encoder.mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
    configuration->device.encoder.calib_flag = param->encoder_calib_flag;
    configuration->device.encoder.reverse = param->encoder_reverse;
    for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
    {
        configuration->device.encoder.linearization_lut_q15[lut_index] =
            param->encoder_linearization_lut_q15[lut_index];
    }
    /* A scaling change invalidates saved current-domain defaults, but not calibration data. */
    if (current_sense_profile_changed || !isfinite(param->calibration_current_a) ||
        param->calibration_current_a < 0.0f)
    {
        configuration->model.calibration_current_a = PARAM_MOTOR_CALIB_CURRENT_A;
    }
    else
    {
        configuration->model.calibration_current_a = Math_Constrain(
            param->calibration_current_a, 0.0f, CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A);
    }

    if (current_sense_profile_changed || !isfinite(param->current_limit_a) ||
        param->current_limit_a <= 0.0f)
    {
        configuration->application.current_limit_a = PARAM_APP_DEFAULT_CURRENT_LIMIT_A;
    }
    else
    {
        configuration->application.current_limit_a =
            Math_Constrain(param->current_limit_a, 0.0f, CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A);
    }
    configuration->model.id_kp = param->id_kp;
    configuration->model.id_ki = param->id_ki;
    configuration->model.iq_kp = param->iq_kp;
    configuration->model.iq_ki = param->iq_ki;
    configuration->application.speed_limit_rad_s =
        isfinite(param->speed_limit_rad_s) && param->speed_limit_rad_s > 0.0f &&
                param->speed_limit_rad_s <= PARAM_APP_SPEED_CEILING_RAD_S
            ? Math_Constrain(
                  param->speed_limit_rad_s, 0.0f, configuration->application.speed_ceiling_rad_s)
            : Math_Min(configuration->application.speed_ceiling_rad_s,
                       PARAM_APP_DEFAULT_SPEED_LIMIT_RAD_S);
    position_speed_limit = Math_Min(configuration->application.speed_limit_rad_s,
                                    PARAM_APP_POSITION_RESTORE_SPEED_MAX_RAD_S);
    configuration->application.speed_acceleration_rad_s2 = param->speed_acceleration_rad_s2;
    configuration->application.speed_deceleration_rad_s2 = param->speed_deceleration_rad_s2;
    configuration->application.speed_kp = param->speed_kp;
    configuration->application.speed_ki = param->speed_ki;
    configuration->application.position_acceleration_rad_s2 =
        isfinite(param->position_acceleration_rad_s2) && param->position_acceleration_rad_s2 > 0.0f
            ? Math_Constrain(
                  param->position_acceleration_rad_s2, 0.0f, PARAM_APP_POSITION_ACCEL_MAX_RAD_S2)
            : PARAM_APP_POSITION_ACCEL_RAD_S2;
    configuration->application.position_deceleration_rad_s2 =
        isfinite(param->position_deceleration_rad_s2) && param->position_deceleration_rad_s2 > 0.0f
            ? Math_Constrain(
                  param->position_deceleration_rad_s2, 0.0f, PARAM_APP_POSITION_ACCEL_MAX_RAD_S2)
            : PARAM_APP_POSITION_DECEL_RAD_S2;
    configuration->application.position_maximum_speed_rad_s =
        isfinite(param->position_maximum_speed_rad_s) && param->position_maximum_speed_rad_s > 0.0f
            ? Math_Constrain(param->position_maximum_speed_rad_s, 0.0f, position_speed_limit)
            : Math_Min(PARAM_APP_POSITION_MAX_SPEED_RAD_S, position_speed_limit);
    configuration->application.cascade_position_kp =
        isfinite(param->cascade_pos_kp) && param->cascade_pos_kp >= 0.0f
            ? Math_Constrain(param->cascade_pos_kp, 0.0f, CASCADE_POSITION_KP_MAX_PER_S)
            : PARAM_APP_CASCADE_POSITION_KP;
    configuration->application.cascade_position_kd =
        isfinite(param->cascade_pos_kd) && param->cascade_pos_kd >= 0.0f
            ? Math_Constrain(param->cascade_pos_kd, 0.0f, CASCADE_POSITION_KD_MAX)
            : PARAM_APP_CASCADE_POSITION_KD;
    configuration->application.axis_profile_valid = param_apply_joint_profile(configuration);
    configuration->configuration_source = MOTOR_CONFIGURATION_STORED;
    configuration->configuration_confirmed =
        param->configuration_flags == 1U && !current_sense_profile_changed;
    if (!ParameterValue_ConfigurationValid(configuration) ||
        !MotorCurrentOffsets_Valid(configuration))
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
    }
}
