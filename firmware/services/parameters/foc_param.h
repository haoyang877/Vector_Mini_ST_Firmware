#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__

#include <stdbool.h>
#include <stdint.h>

#include "angle_feedback.h"
#include "position_cascade_config.h"
#include "motor_axis_profile.h"
#include "cogging_calibration.h"

/* 参数持久化契约：InterfaceParam_TypeDef 是 Flash 参数区的存储布局，
 * schema 版本与字段追加规则见各字段注释；本头文件只声明布局与读写接口。 */

/* Flash 参数记录识别字，与 InterfaceParam_TypeDef.magic_word 配对；
 * 属于已发布 schema 的一部分，数值禁止更改。 */
#define MAGIC_WORD ((uint32_t)0x454E4332U)

#define PARAM_SCHEMA_VERSION 11U

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
 * @brief 将编译期默认配置应用到指定对象。
 * @param motor 启动或受理 DEFAULTS 的电机对象。
 * @param encoder 所属编码器配置。
 * @note 调用方保证停机和独占配置；不写 Flash、不修改模式。
 */
void Param_Return_Default(MotorControl_TypeDef *motor, Encoder_TypeDef *encoder);

/**
 * @brief 导出 schema11 参数值记录。
 * @param param 调用方独占的记录空间。
 * @param motor 稳定的电机配置。
 * @param encoder 稳定的编码器配置。
 * @note 仅启动或停机前台使用；不保留指针，包含表校验。
 */
void Param_Upload(InterfaceParam_TypeDef *param,
                  const MotorControl_TypeDef *motor,
                  const Encoder_TypeDef *encoder);

/**
 * @brief 仅装载当前 schema 11 参数，其他版本或无效记录恢复编译期默认值。
 * @param param 只读 Flash 记录副本。
 * @param motor 中断启动前的电机配置。
 * @param encoder 中断启动前的编码器配置。
 * @note 仅启动调用；拒绝旧版本时清除运行态标定标志，不转换记录或执行 Flash I/O。
 */
void Param_Download(const InterfaceParam_TypeDef *param,
                    MotorControl_TypeDef *motor,
                    Encoder_TypeDef *encoder);
#endif
