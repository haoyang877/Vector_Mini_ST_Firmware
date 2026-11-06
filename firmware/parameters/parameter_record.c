#include "parameter_record.h"

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "crc32.h"
#include "parameter_values.h"
#include "motor_sensing.h"

/* 参数读写与默认值：限值取自平台感测契约，数值换算不访问硬件。 */
#include "motor_axis_profile.h"
#include "application_config.h"
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

typedef char
    Schema14Layout[(offsetof(InterfaceParam, hall_sin_off_counts) == 2188U &&
                    offsetof(InterfaceParam, hall_tag_status) == 2204U &&
                    offsetof(InterfaceParam, thermal) == 2240U && sizeof(InterfaceParam) == 2288U)
                       ? 1
                       : -1];

/* 高24位识别 Hall v1，低字节 bit7 成功、bit0..6 为标定故障。旧槽为0/1。 */
#define PARAM_HALL_V1 0x48410100U
#define PARAM_HALL_SUCCESS 0x80U

void Param_StoreLinearHall(InterfaceParam *param, const LinearHall *hall)
{
    if (!hall->calibration_success && hall->calibration_faults == 0U)
    {
        return;
    }
    param->hall_sin_off_counts = hall->sin_off_counts;
    param->hall_cos_off_counts = hall->cos_off_counts;
    param->hall_sin_amp = hall->sin_amp;
    param->hall_cos_amp = hall->cos_amp;
    param->hall_tag_status = PARAM_HALL_V1 | hall->calibration_faults |
                             (hall->calibration_success ? PARAM_HALL_SUCCESS : 0U);
    param->record_crc32 = Param_RecordCrc(param);
}

void Param_RestoreLinearHall(const InterfaceParam *param, LinearHall *hall)
{
    uint32_t status = param->hall_tag_status;
    bool success = (status & PARAM_HALL_SUCCESS) != 0U;
    uint8_t faults = (uint8_t)(status & 0x7fU);
    if ((status & 0xffffff00U) != PARAM_HALL_V1 || success != (faults == 0U) ||
        !LinearHall_CoefficientsInRange(param->hall_sin_off_counts, param->hall_sin_amp) ||
        !LinearHall_CoefficientsInRange(param->hall_cos_off_counts, param->hall_cos_amp))
    {
        return;
    }
    hall->sin_off_counts = param->hall_sin_off_counts;
    hall->cos_off_counts = param->hall_cos_off_counts;
    hall->sin_amp = param->hall_sin_amp;
    hall->cos_amp = param->hall_cos_amp;
    hall->calibration_success = success;
    hall->calibration_faults = faults;
}

/* OEN1 兼容读取；OEN2 为128点，OEN3 将1024点表与输出相位整体保存。 */
#define PARAM_OUTPUT_ENCODER_V1 0x314e454fU
#define PARAM_OUTPUT_ENCODER_V2 0x324e454fU
#define PARAM_OUTPUT_ENCODER_V3 0x334e454fU
typedef char OutputEncoderLayout[(sizeof(OutputEncoderRecord) == 2084U &&
                                  offsetof(OutputEncoderRecord, hall_lut_status) == 28U &&
                                  offsetof(OutputEncoderRecord, crc32) == 2080U &&
                                  offsetof(MotorAxisRecord, output) == 2288U &&
                                  sizeof(MotorAxisRecord) == 4372U)
                                     ? 1
                                     : -1];

void Param_StoreOutputEncoder(MotorAxisRecord *record,
                              const OutputEncoder *encoder,
                              const LinearHall *hall)
{
    record->output = (OutputEncoderRecord){.magic = PARAM_OUTPUT_ENCODER_V3,
                                           .motor_crc32 = record->motor.record_crc32,
                                           .params = encoder->params};
    if (hall != NULL)
    {
        record->output.hall_lut_status = hall->lut_status;
        memcpy(record->output.hall_lut_q15, hall->lut_q15, sizeof(hall->lut_q15));
    }
    record->output.crc32 = Crc32_Compute(&record->output, offsetof(OutputEncoderRecord, crc32), 0U);
}

bool Param_RestoreOutputEncoder(const MotorAxisRecord *record,
                                OutputEncoder *encoder,
                                LinearHall *hall)
{
    const OutputEncoderRecord *r = &record->output;
    const OutputEncoderParams *p = &r->params;
    bool legacy = r->magic == PARAM_OUTPUT_ENCODER_V1;
    bool small = r->magic == PARAM_OUTPUT_ENCODER_V2;
    uint32_t bytes = legacy ? 28U : small ? 288U : (uint32_t)offsetof(OutputEncoderRecord, crc32);
    uint32_t crc;
    memcpy(&crc, (const uint8_t *)r + bytes, sizeof(crc));
    if ((!legacy && !small && r->magic != PARAM_OUTPUT_ENCODER_V3) ||
        r->motor_crc32 != record->motor.record_crc32 || crc != Crc32_Compute(r, bytes, 0U) ||
        p->gear_ratio != encoder->params.gear_ratio || p->gear_ratio < 1.0f ||
        p->gear_ratio != floorf(p->gear_ratio) || !isfinite(p->phase_rad) ||
        fabsf(p->phase_rad) > _PI || !isfinite(p->zero_rad) || p->zero_rad < 0.0f ||
        p->zero_rad >= _2PI || (p->direction != 1 && p->direction != -1) ||
        ((p->status & 0x7fU) > OUTPUT_ENCODER_CAL_RELATION) || (p->status & ~0x183U) != 0U ||
        ((p->status & OUTPUT_ENCODER_CAL_OK) != 0U && (p->status & 0x7fU) != 0U))
    {
        return false;
    }
    if (!legacy && (r->hall_lut_status != 0U && r->hall_lut_status != LINEAR_HALL_LUT_OK &&
                    r->hall_lut_status != LINEAR_HALL_LUT_COVERAGE &&
                    r->hall_lut_status != LINEAR_HALL_LUT_MAPPING))
    {
        return false;
    }
    if (!legacy && r->hall_lut_status == LINEAR_HALL_LUT_OK)
    {
        if (hall == NULL || !hall->calibration_success)
        {
            return false;
        }
        if (small)
        {
            for (unsigned i = 0U; i < 128U; ++i)
            {
                /* 展开后的八个子段仍须各至少一个Q15计数。 */
                if (512 + r->hall_lut_q15[(i + 1U) & 127U] - r->hall_lut_q15[i] < 8)
                {
                    return false;
                }
            }
        }
        else if (!LinearHall_LutIsMonotonic(r->hall_lut_q15))
        {
            return false;
        }
    }
    if (hall != NULL)
    {
        hall->lut_status = legacy ? 0U : r->hall_lut_status;
        memset(hall->lut_q15, 0, sizeof(hall->lut_q15));
        if (hall->lut_status == LINEAR_HALL_LUT_OK)
        {
            if (small)
            {
                for (unsigned i = 0U; i < LINEAR_HALL_LUT_SIZE; ++i)
                {
                    int32_t a = r->hall_lut_q15[i >> 3U];
                    int32_t b = r->hall_lut_q15[((i >> 3U) + 1U) & 127U];
                    hall->lut_q15[i] = (int16_t)(a + (b - a) * (int32_t)(i & 7U) / 8);
                }
            }
            else
            {
                memcpy(hall->lut_q15, r->hall_lut_q15, sizeof(hall->lut_q15));
            }
        }
    }
    encoder->params = *p;
    return true;
}

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

/* 配置确认绑定节点、AXS 修订和电气模型。型号错配不能沿用原编码器标定。 */
static void bind_preset(MotorConfiguration *configuration, const MotorAxisPreset *preset)
{
    const MotorElectricalPreset *m = preset->electrical;
    MotorModelConfig *model = &configuration->model;
    configuration->preset = preset;
    model->motor_pole_pairs = m->pole_pairs;
    model->motor_phase_resistance_ohm = m->resistance_ohm;
    model->motor_d_inductance_h = model->motor_q_inductance_h = m->inductance_h;
    model->motor_flux_wb = m->flux_wb;
    model->id_kp = model->iq_kp = m->id_kp;
    model->id_ki = model->iq_ki = m->id_ki;
}

void Param_Return_Default(MotorConfiguration *configuration)
{
    configuration->preset = NULL;
    MotorWindingThermal_Default(&configuration->model.thermal);
    configuration->application.speed_feedforward =
        (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    memset(&configuration->application.axis_profile,
           0,
           sizeof(configuration->application.axis_profile));
    configuration->application.axis_profile_valid = true;
    configuration->configuration_confirmed = false;
    configuration->configuration_source = MOTOR_CONFIGURATION_DEFAULTS;
    /* 未装配的产品板不注册整车 CAN 地址。 */
    configuration->device.node_id =
        BSP_BOARD == BSP_BOARD_AT32F456 ? UINT8_MAX : PARAM_DEVICE_CAN_NODE_ID;
    configuration->application.axis_profile_valid = BSP_BOARD != BSP_BOARD_AT32F456;
    configuration->application.speed_ceiling_rad_s = PARAM_APP_SPEED_CEILING_RAD_S;

    configuration->device.phase_a_offset_counts = PARAM_HW_CURRENT_OFFSET_A_COUNTS;
    configuration->device.phase_b_offset_counts = PARAM_HW_CURRENT_OFFSET_B_COUNTS;
    configuration->device.phase_c_offset_counts = PARAM_HW_CURRENT_OFFSET_C_COUNTS;

    configuration->model.motor_pole_pairs = PARAM_MOTOR_POLE_PAIRS;
    configuration->model.motor_phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM;
    configuration->model.motor_d_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H;
    configuration->model.motor_q_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H;
    configuration->model.motor_flux_wb = PARAM_MOTOR_FLUX_WB;
    /* 默认模型和已保存模型均遵守当前电流环整定。 */
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

bool Param_CreateAxis(uint8_t node_id,
                      float minimum_rad,
                      float maximum_rad,
                      float speed_rad_s,
                      const MotorWindingThermalConfig *thermal,
                      MotorConfiguration *configuration)
{
    if (configuration == NULL)
    {
        return false;
    }
    Param_Return_Default(configuration);
    const MotorAxisPreset *p = MotorAxisProfile_ForNode(node_id);
    MotorAxisProfile axis;
    if (p == NULL ||
        !MotorAxisProfile_CreateJoint(
            &axis, p->joint, MOTOR_PRODUCT_AXIS_REVISION, minimum_rad, maximum_rad, speed_rad_s))
    {
        return false;
    }
    /* HT 采用型号已有热输入；YQ 必须显式提供已核对的热输入。 */
    MotorWindingThermalConfig heat = HT8115_HEAT_INITIALIZER;
    if (thermal != NULL)
    {
        heat = *thermal;
    }
    else if (p->electrical->gear_ratio != 1.0f)
    {
        return false;
    }
    if (!MotorWindingThermal_Valid(&heat))
    {
        return false;
    }
    bind_preset(configuration, p);
    configuration->model.thermal = heat;
    configuration->model.calibration_current_a = p->calibration_limit_a;
    configuration->device.node_id = node_id;
    MotorApplicationConfig *app = &configuration->application;
    app->axis_profile = axis;
    app->axis_profile_valid = true;
    app->current_limit_a = fminf(p->control.maximum_current_a, heat.continuous_vector_a);
    app->speed_ceiling_rad_s = app->speed_limit_rad_s = speed_rad_s;
    app->speed_feedforward.current_limit_a =
        fminf(app->current_limit_a, app->speed_feedforward.current_limit_a);
    app->speed_feedforward.acceleration_limit_rad_s2 = p->control.acceleration_rad_s2;
    app->position_maximum_speed_rad_s = speed_rad_s;
    app->speed_kp = p->control.speed_kp;
    app->speed_ki = p->control.speed_ki;
    app->cascade_position_kp = p->control.position_kp;
    app->cascade_position_kd = p->control.position_kd;
    app->position_acceleration_rad_s2 = app->speed_acceleration_rad_s2 =
        p->control.acceleration_rad_s2;
    app->position_deceleration_rad_s2 = app->speed_deceleration_rad_s2 =
        p->control.deceleration_rad_s2;
    return true;
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
    bool current_sense_profile_changed;
    float position_speed_limit;

    if (record == NULL || size_bytes < offsetof(InterfaceParam, magic_word) + sizeof(uint32_t))
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }
    if (param->magic_word != MAGIC_WORD || param->schema_version != PARAM_SCHEMA_VERSION)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source =
            param->magic_word == UINT32_MAX && param->schema_version == UINT32_MAX
                ? MOTOR_CONFIGURATION_DEFAULTS
                : MOTOR_CONFIGURATION_UNSUPPORTED;
        return;
    }
    if (size_bytes < sizeof(*param) || param->record_crc32 != Param_RecordCrc(param) ||
        param->configuration_flags > 1U || !isfinite(param->motor_pole_pairs) ||
        param->motor_pole_pairs < 1.0f || param->motor_pole_pairs >= 2147483648.0f ||
        !isfinite(param->node_id) || param->node_id < 1.0f || param->node_id >= 255.0f ||
        param->node_id != (float)(uint8_t)param->node_id)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }
    configuration->preset = NULL;
    const MotorAxisPreset *preset = NULL;
    if (BSP_BOARD == BSP_BOARD_AT32F456)
    {
        preset = MotorAxisProfile_ForNode((uint8_t)param->node_id);
        MotorAxisProfile axis;
        if (preset == NULL || !MotorAxisProfile_Load(&param->axis_profile, &axis) ||
            axis.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC ||
            axis.identity.numeric.config_revision != MOTOR_PRODUCT_AXIS_REVISION ||
            axis.identity.numeric.joint_type != preset->joint ||
            axis.maximum_speed_rad_s > preset->speed_ceiling_rad_s ||
            param->motor_pole_pairs != (float)preset->electrical->pole_pairs ||
            param->motor_phase_resistance_ohm != preset->electrical->resistance_ohm ||
            param->motor_d_inductance_h != preset->electrical->inductance_h ||
            param->motor_q_inductance_h != preset->electrical->inductance_h ||
            param->motor_flux_wb != preset->electrical->flux_wb)
        {
            Param_Return_Default(configuration);
            configuration->configuration_source = MOTOR_CONFIGURATION_UNSUPPORTED;
            return;
        }
    }
    else if (param->axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
             param->axis_profile.identity.numeric.config_revision == MOTOR_PRODUCT_AXIS_REVISION)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_UNSUPPORTED;
        return;
    }
    current_sense_profile_changed =
        param->current_sense_shunt_milliohm != CURRENT_SENSE_SHUNT_MILLIOHM;
    if (param->encoder_reverse > 1U ||
        (param->encoder_calib_flag & ~(ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO)) != 0U)
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
        return;
    }

    /* 当前记录完整校验后，直接读取热模型和速度前馈。 */
    {
        SpeedFeedforwardConfig feedforward = {param->speed_feedforward_enabled == 1U,
                                              param->speed_feedforward_gain_a_per_rad_s2,
                                              param->speed_feedforward_acceleration_rad_s2,
                                              param->speed_feedforward_jerk_rad_s3,
                                              param->speed_feedforward_current_limit_a};
        if (param->speed_feedforward_enabled > 1U || !MotorWindingThermal_Valid(&param->thermal) ||
            !SpeedFeedforward_IsValid(&feedforward))
        {
            Param_Return_Default(configuration);
            configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
            return;
        }
        configuration->model.thermal = param->thermal;
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
    /* 电流PI整定独立于观测器模型；原记录的R/L与标定仍原样装载。 */
    if (preset != NULL)
    {
        bind_preset(configuration, preset);
        configuration->model.calibration_current_a =
            fminf(configuration->model.calibration_current_a, preset->calibration_limit_a);
        configuration->application.current_limit_a =
            fminf(configuration->application.current_limit_a, preset->control.maximum_current_a);
        configuration->application.speed_ceiling_rad_s = param->axis_profile.maximum_speed_rad_s;
    }
    else if (CURRENT_LOOP_USE_IDENTIFIED_GAINS)
    {
        configuration->model.id_kp = PARAM_MOTOR_DEFAULT_ID_KP;
        configuration->model.iq_kp = PARAM_MOTOR_DEFAULT_IQ_KP;
        configuration->model.id_ki = PARAM_MOTOR_DEFAULT_ID_KI;
        configuration->model.iq_ki = PARAM_MOTOR_DEFAULT_IQ_KI;
    }
    else
    {
        configuration->model.id_kp =
            configuration->model.motor_d_inductance_h * CURRENT_LOOP_BANDWIDTH_RAD_S;
        configuration->model.id_ki =
            configuration->model.motor_phase_resistance_ohm * CURRENT_LOOP_BANDWIDTH_RAD_S;
        configuration->model.iq_kp =
            configuration->model.motor_q_inductance_h * CURRENT_LOOP_BANDWIDTH_RAD_S;
        configuration->model.iq_ki = configuration->model.id_ki;
    }
    configuration->application.speed_limit_rad_s =
        isfinite(param->speed_limit_rad_s) && param->speed_limit_rad_s > 0.0f &&
                param->speed_limit_rad_s <=
                    (preset == NULL ? PARAM_APP_SPEED_CEILING_RAD_S : preset->speed_ceiling_rad_s)
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
    if (preset == NULL)
    {
        configuration->application.axis_profile_valid = param_apply_joint_profile(configuration);
    }
    else
    {
        MotorApplicationConfig *app = &configuration->application;
        app->speed_acceleration_rad_s2 =
            fminf(app->speed_acceleration_rad_s2, preset->control.acceleration_rad_s2);
        app->speed_deceleration_rad_s2 =
            fminf(app->speed_deceleration_rad_s2, preset->control.deceleration_rad_s2);
        app->position_acceleration_rad_s2 =
            fminf(app->position_acceleration_rad_s2, preset->control.acceleration_rad_s2);
        app->position_deceleration_rad_s2 =
            fminf(app->position_deceleration_rad_s2, preset->control.deceleration_rad_s2);
        if (isfinite(param->position_maximum_speed_rad_s) &&
            param->position_maximum_speed_rad_s > 0.0f)
        {
            app->position_maximum_speed_rad_s =
                fminf(app->speed_limit_rad_s, param->position_maximum_speed_rad_s);
        }
        app->current_limit_a =
            fminf(app->current_limit_a, configuration->model.thermal.continuous_vector_a);
        app->speed_feedforward.current_limit_a =
            fminf(app->current_limit_a, app->speed_feedforward.current_limit_a);
        app->speed_feedforward.acceleration_limit_rad_s2 =
            fminf(app->speed_feedforward.acceleration_limit_rad_s2,
                  fminf(app->speed_acceleration_rad_s2, app->speed_deceleration_rad_s2));
    }
    configuration->configuration_source = MOTOR_CONFIGURATION_STORED;
    configuration->configuration_confirmed =
        param->configuration_flags == 1U && !current_sense_profile_changed;
    if (!ParameterValue_ConfigurationValid(configuration) ||
        !MotorDeviceOffsets_InRange(&configuration->device))
    {
        Param_Return_Default(configuration);
        configuration->configuration_source = MOTOR_CONFIGURATION_CORRUPT;
    }
}
