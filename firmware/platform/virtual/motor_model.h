#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H
#include <stdbool.h>
#include <stdint.h>

/** 对象真实参数，与固件MotorConfiguration独立，不自动互相同步。 */
typedef struct
{
    float phase_resistance_ohm, d_inductance_h, q_inductance_h, flux_wb;
    uint32_t pole_pairs;
    float inertia_kg_m2, damping_nm_per_rad_s, load_torque_nm;
} VirtualMotorConfig;

/** 模型真值，只向虚拟平台和测试台架开放。 */
typedef struct
{
    float id_a, iq_a;
    double angle_rad; /**< 累计角使用双精度，防止小子步在多圈后被float舍入吞掉。 */
    float velocity_rad_s;
} MotorModelState;

/**
 * @brief 检查PMSM模型参数有限性及物理范围。
 * @param config 只读对象参数。
 * @return R/L/磁链/惯量及极对数为正、阻尼非负返回true。
 */
bool MotorModel_Valid(const VirtualMotorConfig *config);
/**
 * @brief 用显式中点法推进一次PMSM电气与机械方程。
 * @param config 有效对象参数，单位见字段名。
 * @param state 输入及输出模型真值。
 * @param voltage_v 三相相对中性点电压V。
 * @param enabled 功率使能，关闭时电流R/L衰减并保留机械惯性。
 * @param step_s 积分时间s，正且有限。
 * @return 输入与积分结果有限返回true，否则false。
 * @note 调用方负责子步收敛；关相不模拟高转速二极管回馈。
 */
bool MotorModel_Step(const VirtualMotorConfig *config,
                     MotorModelState *state,
                     const float voltage_v[3],
                     bool enabled,
                     float step_s);
/**
 * @brief 将模型dq电流转换为三相瞬时电流。
 * @param config 只读对象参数。
 * @param state 只读模型真值。
 * @param current_a 三相输出A，不保留地址。
 */
void MotorModel_PhaseCurrents(const VirtualMotorConfig *config,
                              const MotorModelState *state,
                              float current_a[3]);
#endif
