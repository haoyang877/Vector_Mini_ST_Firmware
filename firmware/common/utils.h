#ifndef __FOC_UTILS_H__
#define __FOC_UTILS_H__

/* 通用数学工具：纯函数，无硬件依赖与可变全局。 */
#include <stdint.h>

#define _PI 3.1415926536f
#define _PI_2 1.5707963268f
#define _PI_3 1.0471975512f
#define _2PI 6.2831853072f
#define _3PI_2 4.7123889804f
#define ONE_BY_2PI 0.1591549431f

#define _SQRT_3 1.732050807f
#define ONE_BY_SQRT_3 0.577350269f
#define TWO_BY_SQRT_3 1.154700538f
#define SQRT_3_BY_2 0.866025403f

#define UTILS_LP_FAST(value, sample, filter_constant)                                              \
    (value -= (filter_constant) * ((value) - (sample)))

#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)                                               \
    UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

typedef struct
{
    float state_n, state_n_1, state_n_2;
    float a0, a1, a2;
    float b0, b1, b2;
    float gain0, gain1;
} IIR_Butterworth_TypeDef;

/**
 * @brief  把数值限制到闭区间 [low, high]。
 * @param  amt 输入值。
 * @param  low 下限。
 * @param  high 上限。
 * @return 限幅后的值；NaN 行为与逐次比较实现一致。
 */
float constrain(float amt, float low, float high);

/**
 * @brief  把角度折算到单圈范围。
 * @param  angle 任意角度，单位 rad。
 * @return 折算后的角度，单位 rad。
 */
float normalizeAngle(float angle);

/**
 * @brief  读取 float 的 IEEE-754 位模式。
 * @param  x 输入浮点数。
 * @return 对应的 32 位位模式；用于位级比较（NaN/±0 安全）。
 */
uint32_t FloatToIntBit(float x);

/**
 * @brief  由 IEEE-754 位模式重建 float。
 * @param  x 32 位位模式。
 * @return 对应浮点数。
 */
float IntBitToFloat(uint32_t x);

/**
 * @brief  执行一次二阶 IIR 巴特沃斯滤波。
 * @param  input 本次采样值。
 * @param  IIR_Butterworth_t 滤波器状态与系数指针。
 * @return 滤波输出；状态保存在调用方结构体中。
 */
float IIR_Butterworth(float input, IIR_Butterworth_TypeDef *IIR_Butterworth_t);

/**
 * @brief  求绝对值。
 * @param  x 输入值。
 * @return |x|。
 */
float fast_abs(float x);

/**
 * @brief  求平方。
 * @param  x 输入值。
 * @return x²。
 */
float fast_sq(float x);

/**
 * @brief  求较大值。
 * @param  x 第一个值。
 * @param  y 第二个值。
 * @return 两者中的较大值。
 */
float fast_max(float x, float y);

/**
 * @brief  求较小值。
 * @param  x 第一个值。
 * @param  y 第二个值。
 * @return 两者中的较小值。
 */
float fast_min(float x, float y);

/**
 * @brief 同表三次插值计算正弦与余弦，单圈最大绝对误差不超过 1e-6。
 * @param theta_rad 输入角度，单位 rad；控制调用者应保持在 [-2pi, 2pi]。
 * @param sin_out 正弦输出；调用方提供非空、可写的独立 float 对象。
 * @param cos_out 余弦输出；调用方提供非空、可写的独立 float 对象，不得与 sin_out 重叠。
 * @note 无分配、无共享可变状态，可在 ISR 使用；单圈内工作量固定。
 * 多圈按 float 周期取模，极大角度不保证相对真实三角函数的精度；
 * 非有限输入的两个输出均为 NaN，正负零的正弦保留输入符号。
 */
void Math_SinCos(float theta_rad, float *sin_out, float *cos_out);

/**
 * @brief  atan2 近似。
 * @param  y y 分量。
 * @param  x x 分量。
 * @return 角度，单位 rad，范围约 (-pi, pi]。
 */
float fast_atan2(float y, float x);

/**
 * @brief  平方根近似。
 * @param  x 非负输入。
 * @return sqrt(x) 的近似值。
 */
float fast_sqrt(float x);

/**
 * @brief  整数幂。
 * @param  x 底数。
 * @param  y 指数；仅支持 y >= 0。
 * @return x 的 y 次幂。
 */
float fast_pow(float x, int y);

/**
 * @brief  硬符号：按符号返回 ±1。
 * @param  x 输入值。
 * @return x < 0 返回 -1，否则返回 1（含 0 与 NaN 返回 1）。
 */
float sign_hard(float x);

/**
 * @brief  取 x 的绝对值并赋予 y 的符号。
 * @param  x 数值来源。
 * @param  y 符号来源；y < 0 取负。
 * @return |x| 带 y 的符号。
 */
float copy_sign(float x, float y);

#endif
