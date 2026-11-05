#ifndef CONTROL_MATH_H
#define CONTROL_MATH_H

/* 通用数学工具：纯函数，无硬件依赖与可变全局。 */

#define _PI 3.1415926536f
#define _2PI 6.2831853072f
#define ONE_BY_2PI 0.1591549431f

#define ONE_BY_SQRT_3 0.577350269f
#define TWO_BY_SQRT_3 1.154700538f
#define SQRT_3_BY_2 0.866025403f

#define UTILS_LP_FAST(value, sample, filter_constant)                                              \
    (value -= (filter_constant) * ((value) - (sample)))

/**
 * @brief  把数值限制到闭区间 [low, high]。
 * @param  amt 输入值。
 * @param  low 下限。
 * @param  high 上限。
 * @return 限幅后的值；NaN 行为与逐次比较实现一致。
 */
float Math_Constrain(float amt, float low, float high);

/**
 * @brief  把角度折算到单圈范围。
 * @param  angle 任意角度，单位 rad。
 * @return 折算后的角度，单位 rad。
 */
float Math_NormalizeAngle(float angle);

/**
 * @brief  求绝对值。
 * @param  x 输入值。
 * @return |x|。
 */
float Math_Abs(float x);

/**
 * @brief  求较小值。
 * @param  x 第一个值。
 * @param  y 第二个值。
 * @return 两者中的较小值。
 */
float Math_Min(float x, float y);

/**
 * @brief 查表并作小角旋转计算正弦与余弦，单圈最大绝对误差不超过 1e-6。
 * @param theta_rad 输入角度，单位 rad；控制调用者应保持在 [-2pi, 2pi]。
 * @param sin_out 正弦输出；调用方提供非空、可写的独立 float 对象。
 * @param cos_out 余弦输出；调用方提供非空、可写的独立 float 对象，不得与 sin_out 重叠。
 * @note 无分配、无共享可变状态，可在 ISR 使用；单圈内工作量固定。
 * 多圈按 float 周期取模，极大角度不保证相对真实三角函数的精度；
 * 非有限输入的两个输出均为 NaN，正负零的正弦保留输入符号。
 */
void Math_SinCos(float theta_rad, float *sin_out, float *cos_out);

/**
 * @brief 单精度反正切，供观测器及标定参考角使用。
 * @param  y y 分量。
 * @param  x x 分量。
 * @return 角度，单位rad，范围[-pi, pi]；非有限输入返回NaN。
 * @note 零向量沿用pi/2方向；普通有限向量使用标准atan2f精度。
 */
float Math_Atan2(float y, float x);

/**
 * @brief  平方根近似。
 * @param  x 非负输入。
 * @return sqrt(x) 的近似值。
 */
float Math_Sqrt(float x);

/**
 * @brief  硬符号：按符号返回 ±1。
 * @param  x 输入值。
 * @return x < 0 返回 -1，否则返回 1（含 0 与 NaN 返回 1）。
 */
float Math_Sign(float x);

#endif
