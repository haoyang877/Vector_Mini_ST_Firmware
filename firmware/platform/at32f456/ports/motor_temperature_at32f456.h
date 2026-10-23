#ifndef MOTOR_TEMPERATURE_AT32F456_H
#define MOTOR_TEMPERATURE_AT32F456_H

#include <stdbool.h>
#include <stdint.h>

/* AT32私有的电机外部NTC诊断；不属于公共BSP，不复用MCU结温或保护字段。 */
typedef enum
{
    R40_MOTOR_TEMPERATURE_UNAVAILABLE = 0,
    R40_MOTOR_TEMPERATURE_DISABLED,
    R40_MOTOR_TEMPERATURE_VALID,
    R40_MOTOR_TEMPERATURE_OPEN,
    R40_MOTOR_TEMPERATURE_SHORT,
    R40_MOTOR_TEMPERATURE_OUT_OF_RANGE,
    R40_MOTOR_TEMPERATURE_BAD_CONFIG
} R40MotorTemperatureStatus;

typedef struct
{
    uint32_t sequence;
    uint16_t raw_counts;
    R40MotorTemperatureStatus status;
    float resistance_ohm;
    float celsius; /* 只有VALID时有效；其余状态为NaN。 */
} R40MotorTemperatureSample;

/**
 * @brief 读取新鲜辅助ADC并换算电机NTC温度，重复帧不重复计算。
 * @param result 输出当前诊断快照；NULL时返回false，不保存指针。
 * @return 消费新帧返回true（含故障帧）；重复帧、无数据或禁用时返回false。
 * @note 仅单一前台调用者使用，logf不在中断运行；过期/采样故障立即输出UNAVAILABLE。
 *       重复但仍新鲜的帧返回上次快照。不开启外设、不改变功率输出或上层保护。
 */
bool R40_MotorTemperaturePoll(R40MotorTemperatureSample *result);

#endif
