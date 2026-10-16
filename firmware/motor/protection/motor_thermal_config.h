#ifndef MOTOR_THERMAL_CONFIG_H
#define MOTOR_THERMAL_CONFIG_H

#include "motor_thermal.h"

/** @brief 电机绕组热配置；电流为d/q向量幅值A，时间s，热门限为比例。 */
typedef struct
{
    float continuous_vector_a;
    float peak_a;
    float heating_tau_s;
    float derating_enter_heat;
    float derating_exit_heat;
    float trip_heat;
    float clear_heat;
} MotorWindingThermalConfig;

/**
 * @brief 取得已提交电机配置的默认热模型；不修改应用电流上限。
 * @param config 接收七个物理配置值，非空。
 * @note 上电按冷态开始；STOP/CLEAR不调用初始化，也不清热历史。
 */
void MotorWindingThermal_Default(MotorWindingThermalConfig *config);
/**
 * @brief 校验电机热参数和热门限顺序。
 * @param config 候选值，单位为A、s或热比例。
 * @return 七个配置及派生模型全部有效返回true。
 */
bool MotorWindingThermal_Valid(const MotorWindingThermalConfig *config);
/**
 * @brief 从电机配置派生固定20kHz采样、1kHz更新的纯模型配置。
 * @param config 七个物理配置值。
 * @param derived 接收派生配置；输入无效时保持不变。
 * @return 完整配置有效并已输出返回true。
 * @note 相RMS为向量幅值除以根号2；冷却常数至少120s，不宣称已辨识绝对温度。
 */
bool MotorWindingThermal_Derive(const MotorWindingThermalConfig *config,
                                MotorThermalConfig *derived);

#endif
