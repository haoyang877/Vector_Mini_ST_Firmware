#ifndef INVERTER_MODEL_H
#define INVERTER_MODEL_H
#include <stdbool.h>

/**
 * @brief 按当前PWM比较极性计算平均三相对中性点电压。
 * @param duty 三相比较比例，范围0..1，输入只读。
 * @param bus_voltage_v 理想刚性母线电压V，非负。
 * @param enabled 数字功率使能；关闭时输出零电压占位。
 * @param voltage_v 三相输出V，不保留地址。
 * @return 输入有限且范围合法返回true，否则false。
 * @note 关闭状态的开路电流由电机模型处理；不包含死区和开关损耗。
 */
bool InverterModel_Apply(const float duty[3],
                         float bus_voltage_v,
                         bool enabled,
                         float voltage_v[3]);
#endif
