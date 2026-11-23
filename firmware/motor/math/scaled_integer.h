#ifndef SCALED_INTEGER_H
#define SCALED_INTEGER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t magnitude;
    bool negative, exact_half;
} ScaledInteger;

/**
 * @brief 对 binary32 输入与整数比例的精确乘积取最近整数，半值远离零。
 * @param value 有符号输入值。
 * @param scale 正整数单位比例。
 * @param rounded 成功时写入符号、舍入后的幅值及精确半值标记。
 * @return 输入有限且结果可用 uint32_t 幅值表示时为 true；失败保持输出。
 * @note 只做数值转换。线路哨兵、有效位和有符号端点由各自编码器决定。
 */
bool ScaledInteger_Round(float value, uint32_t scale, ScaledInteger *rounded);

#endif
