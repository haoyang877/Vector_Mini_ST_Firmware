#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__

#include "main.h"
#include "angle_feedback.h"
#include "position_impedance_config.h"
#include "position_cascade_config.h"
#include "motor_axis_profile.h"
#include "cogging_calibration.h"

/* 参数持久化契约：InterfaceParam_TypeDef 是 Flash 参数区的存储布局，
 * schema 版本与字段追加规则见各字段注释；本头文件只声明布局与读写接口。 */

#define PARAM_SCHEMA_VERSION 11U
#define PARAM_SCHEMA_VERSION_LEGACY_COGGING 10U
#define PARAM_SCHEMA_VERSION_LEGACY_POSITION_TUNING 9U
#define PARAM_SCHEMA_VERSION_LEGACY_FRICTION 8U
#define PARAM_SCHEMA_VERSION_LEGACY_INTEGRAL_LIMIT 7U
#define PARAM_SCHEMA_VERSION_LEGACY_CURRENT_SENSE 6U
#define PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE 5U
#define PARAM_SCHEMA_VERSION_LEGACY_CASCADE 4U

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
    /* Schema v11: complete, independently CRC-checked 1024-point Q15 Iq map. */
    CoggingMapRecord cogging;
} InterfaceParam_TypeDef;

/**
 * @brief 将参数恢复为本固件的编译期默认值。
 * @note 只改运行态与持久化候选值，不写 Flash；保存由显式保存流程执行。
 */
void Param_Return_Default(void);

/**
 * @brief 把当前运行态参数导出到接口结构。
 * @param param 输出结构，缓冲区归调用方所有。
 * @note 导出的是当前生效值，不触发 Flash 访问。
 */
void Param_Upload(InterfaceParam_TypeDef *param);

/**
 * @brief 设置运行速度上限，按当前节点的上限截断。
 * @param limit_rad_s 速度上限，单位 rad/s。
 * @return 数值有限且已写入返回 true；非法值返回 false 且不改写。
 */
bool Param_SetSpeedLimit(float limit_rad_s);

/**
 * @brief 载入参数并报告是否需要重写旧的兼容记录。
 * @param param 待载入的结构，只读。
 * @return 载入成功且需要写回升级后的记录时返回 true。
 * @note 校验失败整条拒绝，不部分覆盖运行参数。
 */
bool Param_Download(const InterfaceParam_TypeDef *param);

#endif
