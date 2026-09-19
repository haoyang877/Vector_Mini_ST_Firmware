#ifndef __FOC_SENSING_H__
#define __FOC_SENSING_H__

#include "control_config.h"
#include "foc_algorithm.h"
#include "data_type.h"

/* 温度失效门限：监督 tick 数（2 kHz，0.5 ms/tick），保持 MCU_TEMPERATURE_TIMEOUT_MS 契约。 */
#define MCU_TEMPERATURE_TIMEOUT_TICKS (MCU_TEMPERATURE_TIMEOUT_MS * SUPERVISOR_TICKS_PER_MS)

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
 * @note 2 kHz 监督任务调用，不得在 20 kHz 快速环调用。
 */
void Temperature_Update(FOC_TypeDef *FOC);

/* missed_ticks 以监督 tick（2 kHz，0.5 ms）计数；valid 为换算有效性标志。 */
typedef struct
{
    uint32_t raw_ts, raw_vref, valid, missed_ticks, sample_count;
    float vdda_mv, raw_celsius;
} McuTemperatureTelemetry;
extern volatile McuTemperatureTelemetry McuTemperature;

#endif
