#ifndef FOC_PARAM_H
#define FOC_PARAM_H

#include <stdbool.h>
#include <stdint.h>

#include "angle_feedback.h"
#include "position_cascade_config.h"
#include "motor_axis_profile.h"
/* schema12退役记录槽，仅用于保持已部署参数的大小和偏移；无运行算法。 */
typedef struct
{
    uint32_t magic, encoder_signature;
    float full_scale_a;
    uint32_t crc32;
    int16_t iq_q15[1024];
} CoggingMapRecord;

/* 参数持久化契约：InterfaceParam 是 Flash 参数区的存储布局，
 * schema 版本与字段追加规则见各字段注释；本头文件只声明布局与读写接口。
 * 字段顺序是已发布ABI，不按参数分类重排：基础配置、标定结果和兼容派生值在此编码，
 * 运行目标/积分/热历史不保存。归属、来源及计算时机见docs/architecture/parameter_sources_v1.md。 */

/* Flash 参数记录识别字，与 InterfaceParam.magic_word 配对；
 * 属于已发布 schema 的一部分，数值禁止更改。 */
#define MAGIC_WORD ((uint32_t)0x454E4332U)

#define PARAM_SCHEMA_VERSION 13U
#define PARAM_SCHEMA12_BYTES 4304U

typedef struct
{
    /* 设备输入；node在线格式沿用float，运行对象使用受校验的整数地址。 */
    float node_id;
    /* 当次启动重新采集的设备偏置；旧存储值不提供本次启动资格。 */
    float currentoffset_a;
    float currentoffset_b;
    float currentoffset_c;
    /* 电机模型基础输入：来自已核实型号或实测，不由生产标签自动选择。 */
    float motor_pole_pairs;
    float motor_phase_resistance_ohm;
    float motor_d_inductance_h;
    float motor_q_inductance_h;
    float motor_flux_wb;
    /* 装配标定结果；有效位由实际任务提交，零值本身不代表已标定。 */
    uint16_t encoder_electrical_zero_q15;
    uint16_t encoder_mechanical_zero_q15;
    uint8_t encoder_calib_flag;
    uint8_t encoder_reverse;
    uint8_t encoder_reserved[2];
    int16_t encoder_linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
    /* 电流PI默认由模型派生；有效存储值保留人工整定，不自动覆盖。 */
    float id_kp;
    float id_ki;
    float iq_kp;
    float iq_ki;
    /* 应用输入：速度/位置外环整定与包络，不与电流PI混用单位。 */
    float speed_acceleration_rad_s2;
    float speed_deceleration_rad_s2;
    float speed_kp;
    float speed_ki;
    float position_acceleration_rad_s2;
    float position_deceleration_rad_s2;
    float position_maximum_speed_rad_s;
    /* schema12 复用退役槽；bit0 为用户配置确认，其他位必须为零。 */
    uint32_t configuration_flags;
    /* schema12 整条记录 CRC32，计算时跳过此四字节槽。 */
    uint32_t record_crc32;
    float calibration_current_a;
    float current_limit_a;
    float speed_limit_rad_s;
    /* 退役心跳保留槽：装载忽略，导出清零，不参与控制租约。 */
    float can_hb;
    uint32_t schema_version;
    uint32_t magic_word;
    /* 退役位置积分字段保留槽：读取忽略，导出清零。 */
    uint32_t reserved_pos_ki;
    /* Appended in schema v6; identifies the current-sense scaling in Flash. */
    uint32_t current_sense_shunt_milliohm;
    /* 退役积分限值保留槽：读取忽略，导出清零。 */
    uint32_t reserved_pos_integral_limit;
    /* Appended in schema v8; legacy cascade outer-loop gains. */
    float cascade_pos_kp;
    float cascade_pos_kd;
    /* 退役摩擦模型占位：读取忽略，导出清零。 */
    float friction_coulomb_pos_a;
    float friction_coulomb_neg_a;
    float friction_viscous_pos_a_per_rad_s;
    float friction_viscous_neg_a_per_rad_s;
    uint32_t friction_model_valid;
    /* Optional AXS1 extension; legacy schema/calibration offsets are unchanged. */
    MotorAxisProfile axis_profile;
    /* 退役齿槽表占位：读取忽略，导出清零；schema12布局不变。 */
    CoggingMapRecord cogging;
    /* schema13尾追加；schema12前缀及其所有偏移保持不变。 */
    MotorWindingThermalConfig thermal;
    uint32_t speed_feedforward_enabled;
    float speed_feedforward_gain_a_per_rad_s2;
    float speed_feedforward_acceleration_rad_s2;
    float speed_feedforward_jerk_rad_s3;
    float speed_feedforward_current_limit_a;
} InterfaceParam;

/**
 * @brief 将编译期默认配置应用到指定对象。
 * @param motor 启动或受理 DEFAULTS 的电机对象。
 * @param encoder 所属编码器配置。
 * @note 调用方保证停机和独占配置；不写 Flash、不修改模式。
 */
void Param_Return_Default(MotorControlState *motor, EncoderState *encoder);

/**
 * @brief 导出含配置确认、识别字与整条 CRC32 的 schema13 参数记录。
 * @param param 调用方独占的记录空间。
 * @param motor 稳定的电机配置。
 * @param encoder 稳定的编码器配置。
 * @note 仅启动或停机前台使用；不保留指针，包含表校验。
 */
void Param_Upload(InterfaceParam *param,
                  const MotorControlState *motor,
                  const EncoderState *encoder);

/**
 * @brief 装载有效schema13或schema12前缀；旧前缀使用新增功能的保守默认值。
 * @param param 只读 Flash 记录副本。
 * @param motor 中断启动前的电机配置。
 * @param encoder 中断启动前的编码器配置。
 * @note 仅启动调用；其他版本清除标定资格，不自动转换schema10或写Flash。
 */
void Param_Download(const InterfaceParam *param, MotorControlState *motor, EncoderState *encoder);

/**
 * @brief 按schema12/13规定长度计算记录CRC32，跳过record_crc32槽。
 * @param param 调用方独占的完整记录，不能为空。
 * @return 标准反射 CRC32，首块种子为零。
 * @note 启动或停机前台调用；不修改输入，不在快慢环计算。
 */
uint32_t Param_RecordCrc(const InterfaceParam *param);
#endif
