#ifndef __FOC_SENSING_H__
#define __FOC_SENSING_H__

#include "foc_algorithm.h"
#include "data_type.h"

/**
 * @brief  更新母线电压采样与滤波值。
 * @param  FOC FOC 状态指针，写入 Vbus 相关字段。
 * @param  MotorControl 电机控制状态指针，读取故障使能状态。
 * @note 快速环周期调用；禁用状态下采样用于复位计数器。
 */
void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);

/**
 * @brief  完成相电流采样换算与偏置补偿。
 * @param  FOC FOC 状态指针，写入 Ia/Ib/Ic。
 * @param  MotorControl 电机控制状态指针。
 * @note 快速环周期调用；换算系数来自平台感测契约。
 */
void Current_Cal(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);

/**
 * @brief  更新 MCU 温度换算与遥测。
 * @param  FOC FOC 状态指针，写入温度字段。
 * @note 1 kHz 监督任务调用，不得在 20 kHz 快速环调用。
 */
void Temperature_Update(FOC_TypeDef *FOC);

typedef struct
{
    uint32_t raw_ts, raw_vref, valid, missed_ms, sample_count;
    float vdda_mv, raw_celsius;
} McuTemperatureTelemetry;
extern volatile McuTemperatureTelemetry McuTemperature;

#endif
