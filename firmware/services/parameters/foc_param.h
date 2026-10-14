#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__
#include <stddef.h>
#include "motor_thermal.h"

#include "main.h"
#include "encoder.h"
#include "position_impedance_config.h"
#include "position_cascade_config.h"
#include "motor_axis_profile.h"
#include "motor_load_profile.h"

#define PARAM_SCHEMA_VERSION 10U
#define PARAM_SCHEMA_VERSION_LEGACY_POSITION_TUNING 9U
#define PARAM_SCHEMA_VERSION_LEGACY_FRICTION 8U
#define PARAM_SCHEMA_VERSION_LEGACY_INTEGRAL_LIMIT 7U
#define PARAM_SCHEMA_VERSION_LEGACY_CURRENT_SENSE 6U
#define PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE 5U
#define PARAM_SCHEMA_VERSION_LEGACY_CASCADE 4U

/** @brief PVF1/v1独立尾记录；标志0/1对应速度低通10/5 Hz，CRC覆盖前12字节。 */
#define PARAM_POSITION_FILTER_MAGIC 0x31465650U
#define PARAM_POSITION_FILTER_VERSION 1U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t half_cutoff;
    uint32_t crc32;
} PositionFilterRecord;

/** @brief PSP1/v1位置级联速度PI尾记录；CRC覆盖前16字节，旧字段偏移保持。 */
#define PARAM_POSITION_SPEED_PI_MAGIC 0x31505350U
#define PARAM_POSITION_SPEED_PI_VERSION 1U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float kp_s_per_rad;
    float ki_per_rad;
    uint32_t crc32;
} PositionSpeedPiRecord;

/** @brief FSL1/v1摩擦电流斜率，A/s；CRC覆盖前16字节，旧字段偏移保持。 */
#define PARAM_FRICTION_SLEW_MAGIC 0x314C5346U
#define PARAM_FRICTION_SLEW_VERSION 1U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float attack_a_per_s;
    float release_a_per_s;
    uint32_t crc32;
} FrictionSlewRecord;

/** @brief SFF1/v1速度前馈尾记录；小端，CRC覆盖前28字节，旧字段不移动。 */
#define PARAM_SPEED_FEEDFORWARD_MAGIC 0x31464653U
#define PARAM_SPEED_FEEDFORWARD_VERSION 1U
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t enabled;
    float gain_a_per_rad_s2;
    float acceleration_limit_rad_s2;
    float jerk_limit_rad_s3;
    float current_limit_a;
    uint32_t crc32;
} SpeedFeedforwardRecord;

/** @brief MTH1/v1独立热配置；七个float按MotorThermalParameter顺序，CRC覆盖前36字节。 */
#define PARAM_MOTOR_THERMAL_MAGIC 0x3148544DU
#define PARAM_MOTOR_THERMAL_VERSION 1U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float values[MOTOR_THERMAL_PARAMETER_COUNT];
    uint32_t crc32;
} MotorThermalRecord;

typedef struct
{
	float node_id;
	float currentoffset_a;
	float currentoffset_b;
	float currentoffset_c;
	float motor_pole_pairs;
	float motor_phase_resistance;
	float motor_d_inductance;
	float motor_q_inductance;
	float motor_flux;
	uint16_t encoder_electrical_zero_q15;
	uint16_t encoder_mechanical_zero_q15;
	uint8_t encoder_calib_flag;
	uint8_t encoder_reverse;
	uint8_t encoder_reserved[2];
	int16_t encoder_linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
	float id_kp;
	float id_ki;
	float iq_kp;
	float iq_ki;
	float speedAcc;
	float speedDec;
	float speed_kp;
	float speed_ki;
	float posAcc;
	float posDec;
	float pos_maxspeed;
	float pos_kp;
	float pos_kd;
	float calib_current;
	float current_limit;
	float speed_limit;
	float can_hb;
	uint32_t schema_version;
	uint32_t magic_word;
	/* Appended in schema v5 so the v4 schema/magic offsets remain readable. */
	float pos_ki;
	/* Appended in schema v6; identifies the current-sense scaling in Flash. */
	uint32_t current_sense_shunt_milliohm;
	/* Appended in schema v7; position-integrator output limit in amperes. */
	float pos_integral_limit;
	/* Appended in schema v8; legacy cascade outer-loop gains. */
	float cascade_pos_kp;
	float cascade_pos_kd;
	/* Appended in schema v9; current-domain model, compensation remains disabled. */
	float friction_coulomb_pos_a;
	float friction_coulomb_neg_a;
	float friction_viscous_pos_a_per_rad_s;
	float friction_viscous_neg_a_per_rad_s;
	uint32_t friction_model_valid;
	/* Optional AXS1 extension; legacy schema/calibration offsets are unchanged. */
	MotorAxisProfile axis_profile;
    /* DMP1 v1 独立扩展；schema10 既有字段偏移保持不变。 */
    MotorLoadRecord load_profile;
    /* schema10既有字段偏移保持；旧固件忽略PVF1扩展。 */
    PositionFilterRecord position_filter;
    PositionSpeedPiRecord position_speed_pi;
    FrictionSlewRecord friction_slew;
    SpeedFeedforwardRecord speed_feedforward;
    MotorThermalRecord motor_thermal;
} InterfaceParam_TypeDef;

typedef char MotorThermalRecordSize[(sizeof(MotorThermalRecord) == 40U) ? 1 : -1];
typedef char MotorThermalRecordOffset[(offsetof(InterfaceParam_TypeDef, motor_thermal) == 2344U) ? 1 : -1];

void Param_Return_Default(void);
void Param_Upload(InterfaceParam_TypeDef *param);
/** Set the runtime limit in rad/s, bounded by the current node's ceiling. */
bool Param_SetSpeedLimit(float limit_rad_s);
/** Load parameters and report whether a valid legacy record must be rewritten. */
bool Param_Download(const InterfaceParam_TypeDef *param);

#endif
