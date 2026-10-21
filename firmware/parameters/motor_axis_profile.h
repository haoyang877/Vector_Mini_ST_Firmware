/* 轴记录服务只处理验证、身份与基线解析；电机值契约在 axis_config.h。 */
#ifndef MOTOR_AXIS_PROFILE_H
#define MOTOR_AXIS_PROFILE_H
#include "axis_config.h"

/**
 * @brief 把稳定轴编号映射为显示名。
 * @param joint_type 协议/存储使用的稳定轴编号。
 * @return 静态只读名称；未知编号返回 unknown。
 */
const char *MotorJointType_Name(uint32_t joint_type);
/**
 * @brief 建立带 CRC 的 AXS2 轴记录并检查配置范围。
 * @param output 调用方非空记录空间；不转移所有权。
 * @param joint_type 稳定轴类型编号。
 * @param config_revision 零表示仅身份且所有范围须为零；一仅支持 roll/pitch。
 * @param minimum_position_rad 位置下界，单位 rad。
 * @param maximum_position_rad 位置上界，单位 rad。
 * @param maximum_speed_rad_s 正速度上限，单位 rad/s。
 * @return 配置有效返回 true，否则返回 false，output 不变。
 * @note 前台/启动使用；不写 Flash、不使能电机。
 */
bool MotorAxisProfile_CreateJoint(MotorAxisProfile *output,
                                  uint32_t joint_type,
                                  uint32_t config_revision,
                                  float minimum_position_rad,
                                  float maximum_position_rad,
                                  float maximum_speed_rad_s);
/**
 * @brief 验证 AXS2 并解析关节控制基线，限制只能收窄包络。
 * @param profile 只读轴记录。
 * @param output 调用方基线空间；失败保持不变。
 * @return AXS2 修订一且范围合法返回 true，否则返回 false。
 * @note 包含 CRC，只在启动/恢复使用；AXS1 保留独立保存的增益。
 */
bool MotorAxisProfile_Resolve(const MotorAxisProfile *profile, MotorJointControlConfig *output);

/**
 * @brief 校验轴记录并复制到调用方空间。
 * @param stored 只读记录；全零/全擦除表示未配置。
 * @param output 非空调用方输出；校验失败时内容可能已经复制，不可使用。
 * @return 记录有效或未配置返回 true；损坏/未知布局返回 false。
 * @note 启动/恢复执行 CRC；调用方持有有效性并拒绝损坏配置。
 */
bool MotorAxisProfile_Load(const MotorAxisProfile *stored, MotorAxisProfile *output);
/**
 * @brief 创建带 CRC 的 roll/pitch 名称记录。
 * @param output 调用方非空记录空间；不转移所有权。
 * @param name 只读零结尾名称，仅允许 roll 或 pitch。
 * @param minimum_position_rad 位置下界，单位 rad。
 * @param maximum_position_rad 位置上界，单位 rad。
 * @param maximum_speed_rad_s 正速度上限，单位 rad/s。
 * @return 输入及记录有效返回 true，否则返回 false。
 * @note AXS1 记录构造，不写 Flash；失败时 output 可能写入候选，不可作为有效配置。
 */
bool MotorAxisProfile_Create(MotorAxisProfile *output,
                             const char *name,
                             float minimum_position_rad,
                             float maximum_position_rad,
                             float maximum_speed_rad_s);

#endif
