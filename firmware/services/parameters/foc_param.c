#include "foc_param.h"

#include <string.h>
#include "common_inc.h"
#include "foc_param_profile.h"
#include "motor_thermal_control.h"

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder;
extern CANMsg_TypeDef CANMsg;

/* FSL1旧空记录继续采用已发布默认值，不随历史控制内核切换而改变。 */
#define PARAM_FRICTION_ATTACK_DEFAULT_A_PER_S 2.0f

/* 参数尾记录共用同一小端字CRC契约；只在启动/保存路径执行。 */
static uint32_t parameter_words_crc(const uint32_t *words, uint32_t word_count)
{
    uint32_t crc = UINT32_MAX;
    uint32_t index;
    for (index = 0U; index < word_count * 4U; ++index)
    {
        uint32_t bit;
        crc ^= (words[index / 4U] >> ((index % 4U) * 8U)) & 0xFFU;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

/* 私有参数存储计算；与Flash I/O、控制时序分离，固定little-endian CRC契约。 */
static uint32_t position_filter_crc(const PositionFilterRecord *record)
{
    uint32_t words[3];
    words[0] = record->magic;
    words[1] = record->version;
    words[2] = record->half_cutoff;
    return parameter_words_crc(words, 3U);
}

/* 缺省旧尾部兼容10 Hz；非空损坏记录拒绝准入，不悄悄降低反馈延迟。 */
static bool position_filter_load(const PositionFilterRecord *record, bool *half_cutoff)
{
    bool blank_zero = record->magic == 0U && record->version == 0U &&
        record->half_cutoff == 0U && record->crc32 == 0U;
    bool blank_erased = record->magic == UINT32_MAX && record->version == UINT32_MAX &&
        record->half_cutoff == UINT32_MAX && record->crc32 == UINT32_MAX;
    if (blank_zero || blank_erased)
    {
        *half_cutoff = false;
        return true;
    }
    if (record->magic != PARAM_POSITION_FILTER_MAGIC ||
        record->version != PARAM_POSITION_FILTER_VERSION || record->half_cutoff > 1U ||
        record->crc32 != position_filter_crc(record))
    {
        return false;
    }
    *half_cutoff = record->half_cutoff == 1U;
    return true;
}

/* 参数只在启动/保存时处理；显式按小端字计算 CRC，不在控制 ISR 中运行。 */
static uint32_t position_speed_pi_crc(const PositionSpeedPiRecord *record)
{
    uint32_t words[4];
    words[0] = record->magic;
    words[1] = record->version;
    memcpy(&words[2], &record->kp_s_per_rad, sizeof(uint32_t));
    memcpy(&words[3], &record->ki_per_rad, sizeof(uint32_t));
    return parameter_words_crc(words, 4U);
}

static uint32_t motor_thermal_record_crc(const MotorThermalRecord *record)
{
    uint32_t words[9];
    memcpy(words, record, sizeof(words));
    return parameter_words_crc(words, 9U);
}

static bool motor_thermal_record_load(const MotorThermalRecord *record)
{
    uint32_t words[10];
    uint32_t i;
    bool blank_zero = true;
    bool blank_erased = true;
    memcpy(words, record, sizeof(words));
    for (i = 0U; i < 10U; ++i)
    {
        blank_zero = blank_zero && words[i] == 0U;
        blank_erased = blank_erased && words[i] == UINT32_MAX;
    }
    if (blank_zero || blank_erased)
    {
        return MotorThermalControl_LoadParameters(NULL);
    }
    return record->magic == PARAM_MOTOR_THERMAL_MAGIC &&
           record->version == PARAM_MOTOR_THERMAL_VERSION &&
           record->crc32 == motor_thermal_record_crc(record) &&
           MotorThermalControl_LoadParameters(record->values);
}

static uint32_t friction_slew_crc(const FrictionSlewRecord *record)
{
    uint32_t words[4];
    words[0] = record->magic;
    words[1] = record->version;
    memcpy(&words[2], &record->attack_a_per_s, sizeof(uint32_t));
    memcpy(&words[3], &record->release_a_per_s, sizeof(uint32_t));
    return parameter_words_crc(words, 4U);
}

/* 旧空尾记录使用新默认值且不自动保存；非空错误拒绝参数准入。 */
static bool friction_slew_load(const FrictionSlewRecord *record)
{
    uint32_t words[5];
    bool zero = true, erased = true;
    uint32_t i;
    memcpy(words, record, sizeof(words));
    for (i = 0U; i < 5U; ++i)
    {
        zero = zero && words[i] == 0U;
        erased = erased && words[i] == UINT32_MAX;
    }
    MotorControl.friction_attack_slew_a_per_s = PARAM_FRICTION_ATTACK_DEFAULT_A_PER_S;
    MotorControl.friction_release_slew_a_per_s = POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S;
    if (zero || erased) { return true; }
    if (record->magic != PARAM_FRICTION_SLEW_MAGIC || record->version != PARAM_FRICTION_SLEW_VERSION ||
        !isfinite(record->attack_a_per_s) || record->attack_a_per_s <= 0.0f || record->attack_a_per_s > 1000.0f ||
        !isfinite(record->release_a_per_s) || record->release_a_per_s <= 0.0f || record->release_a_per_s > 1000.0f ||
        record->crc32 != friction_slew_crc(record))
    {
        return false;
    }
    MotorControl.friction_attack_slew_a_per_s = record->attack_a_per_s;
    MotorControl.friction_release_slew_a_per_s = record->release_a_per_s;
    return true;
}

/* 空尾部保留调用方给出的旧实际增益；非空非法尾部不能悄悄复用速度模式参数。 */
static bool position_speed_pi_load(const PositionSpeedPiRecord *record,
    float *kp_s_per_rad, float *ki_per_rad, bool *missing)
{
    uint32_t words[5];
    bool blank_zero = true;
    bool blank_erased = true;
    uint32_t index;
    memcpy(words, record, sizeof(words));
    for (index = 0U; index < 5U; ++index)
    {
        blank_zero = blank_zero && words[index] == 0U;
        blank_erased = blank_erased && words[index] == UINT32_MAX;
    }
    *missing = blank_zero || blank_erased;
    if (*missing)
    {
        return isfinite(*kp_s_per_rad) && *kp_s_per_rad >= 0.0f &&
            isfinite(*ki_per_rad) && *ki_per_rad >= 0.0f;
    }
    if (record->magic != PARAM_POSITION_SPEED_PI_MAGIC ||
        record->version != PARAM_POSITION_SPEED_PI_VERSION ||
        !isfinite(record->kp_s_per_rad) || record->kp_s_per_rad < 0.0f ||
        !isfinite(record->ki_per_rad) || record->ki_per_rad < 0.0f ||
        record->crc32 != position_speed_pi_crc(record))
    {
        return false;
    }
    *kp_s_per_rad = record->kp_s_per_rad;
    *ki_per_rad = record->ki_per_rad;
    return true;
}

/* 参数记录按字的小端顺序校验；仅启动/保存使用，不进入速度控制计算。 */
static uint32_t speed_feedforward_crc(const SpeedFeedforwardRecord *record)
{
    uint32_t words[7];
    memcpy(words, record, sizeof(words));
    return parameter_words_crc(words, 7U);
}

static bool speed_feedforward_load(const SpeedFeedforwardRecord *record,
    SpeedFeedforwardConfig *config)
{
    uint32_t words[8], index;
    bool zero = true, erased = true;
    SpeedFeedforward_Default(config);
    memcpy(words, record, sizeof(words));
    for (index = 0U; index < 8U; ++index)
    {
        zero = zero && words[index] == 0U;
        erased = erased && words[index] == UINT32_MAX;
    }
    if (zero || erased)
    {
        return true;
    }
    if (record->magic != PARAM_SPEED_FEEDFORWARD_MAGIC ||
        record->version != PARAM_SPEED_FEEDFORWARD_VERSION || record->enabled > 1U ||
        record->crc32 != speed_feedforward_crc(record))
    {
        return false;
    }
    config->enabled = record->enabled == 1U;
    config->gain_a_per_rad_s2 = record->gain_a_per_rad_s2;
    config->acceleration_limit_rad_s2 = record->acceleration_limit_rad_s2;
    config->jerk_limit_rad_s3 = record->jerk_limit_rad_s3;
    config->current_limit_a = record->current_limit_a;
    return SpeedFeedforward_IsValid(config);
}

bool Param_SetSpeedLimit(float limit_rad_s)
{
    if (!isfinite(limit_rad_s) || limit_rad_s <= 0.0f ||
        limit_rad_s > Param_SpeedLimitRadS(CANMsg.node_id)) return false;
    MotorControl.speed_limit = limit_rad_s;
    /* A reduced limit also reduces an existing command; the speed ramp still
     * uses the configured acceleration/deceleration. */
    MotorControl.speedRef = constrain(MotorControl.speedRef, -limit_rad_s, limit_rad_s);
    MotorControl.pos_maxspeed = fminf(MotorControl.pos_maxspeed, limit_rad_s);
    return true;
}

/* Startup adapter only: the portable module owns joint IDs and baselines.
 * Individual motor/current calibration and stored axis bounds are untouched. */
static bool Param_ApplyJointProfile(MotorControl_TypeDef *motor)
{
    MotorJointControlConfig config;
    if (motor == NULL || !motor->axis_profile_valid) return false;
    if (motor->axis_profile.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC) return true;
    if (!isfinite(motor->speed_limit) || motor->speed_limit <= 0.0f ||
        !isfinite(motor->current_limit) || motor->current_limit <= 0.0f) return false;
    if (!MotorAxisProfile_Resolve(&motor->axis_profile, &config)) return false;
    motor->cascade_pos_Kp = config.position_kp;
    motor->cascade_pos_Kd = config.position_kd;
    motor->speed_Kp = config.speed_kp;
    motor->speed_Ki = config.speed_ki;
    motor->posAcc = config.acceleration_rad_s2;
    motor->posDec = config.deceleration_rad_s2;
    motor->pos_maxspeed = fminf(motor->axis_profile.maximum_speed_rad_s, motor->speed_limit);
    motor->current_limit = fminf(motor->current_limit, config.maximum_current_a);
    return true;
}

void Param_Return_Default(void)
{
    SpeedFeedforward_Default(&MotorControl.speed_ff_config);
    SpeedFeedforward_Reset(&MotorControl.speed_ff_state);
    /* 启动缺省采用热默认值；运行中独立热标定配置与历史不被恢复默认覆盖。 */
    (void)MotorThermalControl_LoadParameters(NULL);
    MotorControl.position_accel_ff_gain_a_per_rad_s2 = 0.0f;
    MotorControl.position_support_limit_a = POSITION_SERVO_SUPPORT_LIMIT_A;
    MotorControl.position_support_adjust_a_per_s = POSITION_SERVO_SUPPORT_ADJUST_A_PER_S;
    MotorControl.position_support_release_a_per_s = POSITION_SERVO_SUPPORT_RELEASE_A_PER_S;
    MotorControl.position_velocity_filter_half_cutoff = false;
	memset(&MotorControl.axis_profile, 0, sizeof(MotorControl.axis_profile));
	MotorControl.axis_profile_valid = true;
    MotorControl.load_profile_flags = 0U;
    MotorControl.load_profile_valid = true;
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
	MotorControl.speed_limit = Param_SpeedLimitRadS(CANMsg.node_id);
	MotorControl.speedAcc = PARAM_APP_SPEED_ACCEL_RPS2 * _2PI;
	MotorControl.speedDec = PARAM_APP_SPEED_DECEL_RPS2 * _2PI;
	MotorControl.speed_Kp = PARAM_APP_SPEED_KP;
	MotorControl.speed_Ki = PARAM_APP_SPEED_KI;
    MotorControl.position_speed_Kp = PARAM_APP_SPEED_KP;
    MotorControl.position_speed_Ki = PARAM_APP_SPEED_KI;
    MotorControl.friction_attack_slew_a_per_s = PARAM_FRICTION_ATTACK_DEFAULT_A_PER_S;
    MotorControl.friction_release_slew_a_per_s = POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S;
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
	param->axis_profile = MotorControl.axis_profile;
    param->position_filter.magic = PARAM_POSITION_FILTER_MAGIC;
    param->position_filter.version = PARAM_POSITION_FILTER_VERSION;
    param->position_filter.half_cutoff = MotorControl.position_velocity_filter_half_cutoff ? 1U : 0U;
    param->position_filter.crc32 = position_filter_crc(&param->position_filter);
    param->position_speed_pi.magic = PARAM_POSITION_SPEED_PI_MAGIC;
    param->position_speed_pi.version = PARAM_POSITION_SPEED_PI_VERSION;
    param->position_speed_pi.kp_s_per_rad = MotorControl.position_speed_Kp;
    param->position_speed_pi.ki_per_rad = MotorControl.position_speed_Ki;
    param->position_speed_pi.crc32 = position_speed_pi_crc(&param->position_speed_pi);
    param->friction_slew.magic = PARAM_FRICTION_SLEW_MAGIC;
    param->friction_slew.version = PARAM_FRICTION_SLEW_VERSION;
    param->friction_slew.attack_a_per_s = MotorControl.friction_attack_slew_a_per_s;
    param->friction_slew.release_a_per_s = MotorControl.friction_release_slew_a_per_s;
    param->friction_slew.crc32 = friction_slew_crc(&param->friction_slew);
    param->speed_feedforward.magic = PARAM_SPEED_FEEDFORWARD_MAGIC;
    param->speed_feedforward.version = PARAM_SPEED_FEEDFORWARD_VERSION;
    param->speed_feedforward.enabled = MotorControl.speed_ff_config.enabled ? 1U : 0U;
    param->speed_feedforward.gain_a_per_rad_s2 = MotorControl.speed_ff_config.gain_a_per_rad_s2;
    param->speed_feedforward.acceleration_limit_rad_s2 = MotorControl.speed_ff_config.acceleration_limit_rad_s2;
    param->speed_feedforward.jerk_limit_rad_s3 = MotorControl.speed_ff_config.jerk_limit_rad_s3;
    param->speed_feedforward.current_limit_a = MotorControl.speed_ff_config.current_limit_a;
    param->speed_feedforward.crc32 = speed_feedforward_crc(&param->speed_feedforward);
    param->motor_thermal.magic = PARAM_MOTOR_THERMAL_MAGIC;
    param->motor_thermal.version = PARAM_MOTOR_THERMAL_VERSION;
    for (lut_index = 0U; lut_index < MOTOR_THERMAL_PARAMETER_COUNT; ++lut_index)
    {
        (void)MotorThermalControl_GetParameter((MotorThermalParameter)lut_index,
                                               &param->motor_thermal.values[lut_index]);
    }
    param->motor_thermal.crc32 = motor_thermal_record_crc(&param->motor_thermal);
    if (!MotorControl.load_profile_valid ||
        !MotorLoadRecord_Create(MotorControl.load_profile_flags, &param->load_profile))
    {
        /* 不把损坏运行配置保存成合法的全零默认记录。 */
        param->load_profile.magic = MOTOR_LOAD_RECORD_MAGIC;
    }
}

bool Param_Download(const InterfaceParam_TypeDef *param)
{
	uint32_t lut_index;
	uint32_t stored_shunt_milliohm;
	bool legacy_cascade_parameters;
	bool position_tuning_requires_migration;
	bool current_sense_profile_changed;
	float position_speed_limit;
    float speed_mode_kp;
    float speed_mode_ki;
    bool position_speed_pi_missing;

    /* 新RAM候选不借用旧Flash尾部，避免把擦除值当成支撑配置。 */
    MotorControl.position_support_limit_a = POSITION_SERVO_SUPPORT_LIMIT_A;
    MotorControl.position_support_adjust_a_per_s = POSITION_SERVO_SUPPORT_ADJUST_A_PER_S;
    MotorControl.position_support_release_a_per_s = POSITION_SERVO_SUPPORT_RELEASE_A_PER_S;

	if (param->magic_word != MAGIC_WORD ||
		(param->schema_version != PARAM_SCHEMA_VERSION &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_POSITION_TUNING &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_FRICTION &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_INTEGRAL_LIMIT &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_CURRENT_SENSE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_LEGACY_CASCADE))
	{
		Param_Return_Default();
		return false;
	}
	legacy_cascade_parameters = param->schema_version == PARAM_SCHEMA_VERSION_LEGACY_CASCADE;
	position_tuning_requires_migration = param->schema_version != PARAM_SCHEMA_VERSION;
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
		return false;
	}

	/* ponytail: 临时优先使用 Flash 节点号；多机联调须保证 ID 唯一，结束后恢复轴绑定。 */
	if (isfinite(param->node_id) && param->node_id >= 0.0f &&
		param->node_id <= 7.0f &&
		param->node_id == (float)(uint8_t)param->node_id)
	{
		CANMsg.node_id = (uint8_t)param->node_id;
	}
	else
	{
		CANMsg.node_id = MotorAxisProfile_CanNodeId(&param->axis_profile,
			PARAM_HW_CAN_NODE_ID);
	}
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
		constrain(param->speed_limit, 0.0f, Param_SpeedLimitRadS(CANMsg.node_id)) :
		Param_SpeedLimitRadS(CANMsg.node_id);
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
		if (position_tuning_requires_migration)
		{
			/*
			 * Schema v10 accompanies the revised 2 kHz landing controller. Do not
			 * carry legacy impedance tuning into the updated control law.
			 */
			MotorControl.pos_Kp = PARAM_APP_POSITION_KP;
			MotorControl.pos_Kd = PARAM_APP_POSITION_KD;
			MotorControl.pos_Ki = PARAM_APP_POSITION_KI;
			MotorControl.pos_integral_limit = PARAM_APP_POSITION_INTEGRAL_LIMIT_A;
		}
		else
		{
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
				isfinite(param->pos_integral_limit) && param->pos_integral_limit >= 0.0f ?
				constrain(param->pos_integral_limit, 0.0f, CURRENT_COMMAND_LIMIT_MAX_A) :
				PARAM_APP_POSITION_INTEGRAL_LIMIT_A;
		}
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
	if (param->schema_version >= PARAM_SCHEMA_VERSION_LEGACY_POSITION_TUNING &&
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
	MotorControl.axis_profile_valid = MotorAxisProfile_Load(&param->axis_profile,
		&MotorControl.axis_profile);
    speed_mode_kp = MotorControl.speed_Kp;
    speed_mode_ki = MotorControl.speed_Ki;
	MotorControl.axis_profile_valid = Param_ApplyJointProfile(&MotorControl);
    MotorControl.load_profile_flags = 0U;
    MotorControl.load_profile_valid = MotorLoadRecord_Load(&param->load_profile,
        &MotorControl.load_profile_flags);
    if (!MotorControl.load_profile_valid)
    {
        return false;
    }
	/* Unsupported identity/configuration must not trigger a migration rewrite. */
	if (!MotorControl.axis_profile_valid) return false;
    if (!position_filter_load(&param->position_filter, &MotorControl.position_velocity_filter_half_cutoff))
    {
        /* 复用参数准入闸；坏尾记录不能启用位置输出或触发迁移保存。 */
        MotorControl.axis_profile_valid = false;
        return false;
    }
    /* 旧 AXS2 的启动增益也是旧实际值；新记录恢复后不能覆盖任何用户 PI。 */
    MotorControl.position_speed_Kp = MotorControl.speed_Kp;
    MotorControl.position_speed_Ki = MotorControl.speed_Ki;
    if (!position_speed_pi_load(&param->position_speed_pi,
        &MotorControl.position_speed_Kp, &MotorControl.position_speed_Ki,
        &position_speed_pi_missing))
    {
        MotorControl.axis_profile_valid = false;
        return false;
    }
    if (!position_speed_pi_missing)
    {
        MotorControl.speed_Kp = speed_mode_kp;
        MotorControl.speed_Ki = speed_mode_ki;
    }

    SpeedFeedforward_Reset(&MotorControl.speed_ff_state);
    if (!friction_slew_load(&param->friction_slew))
    {
        MotorControl.axis_profile_valid = false;
        return false;
    }
    if (!speed_feedforward_load(&param->speed_feedforward, &MotorControl.speed_ff_config))
    {
        MotorControl.axis_profile_valid = false;
        return false;
    }
    if (!motor_thermal_record_load(&param->motor_thermal))
    {
        MotorControl.axis_profile_valid = false;
        return false;
    }
    return position_tuning_requires_migration || position_speed_pi_missing;
}
