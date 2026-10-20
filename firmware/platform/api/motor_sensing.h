#ifndef MOTOR_SENSING_H
#define MOTOR_SENSING_H

#include <stdbool.h>
#include <stdint.h>

#include "current_sense_profile.h"

/* 电机感测契约：相电流/母线电压/MCU 温度的采样访问与换算常量（platform/api 层拥有）。
 * 电流/VBUS原始计数为四次和，使用前折算为12位域；保留历史换算运算顺序。
 * hw_conf.h 以历史名称回导这些常量，板级实现见 platform/stm32g4/ports/motor。 */

/* 固定板级输入：母线分压电阻（kΩ）和ADC2每个rank的累加次数。
 * 累加次数须与ADC2实际配置一致，不是可在线修改的参数。 */
#define MOTOR_SENSING_VBUS_R1_KOHM 10.0f
#define MOTOR_SENSING_VBUS_R2_KOHM 1.0f
#define MOTOR_SENSING_ADC_SAMPLE_COUNT 4U

/* 以下为编译期派生量；全部操作数为常量，调用点只保留与采样值的运算。 */
#define MOTOR_SENSING_ADC_SUM_TO_COUNTS (1.0f / (float)MOTOR_SENSING_ADC_SAMPLE_COUNT)
/* 母线电压：V/count；保持先除ADC量程、再乘分压比的历史顺序。 */
#define MOTOR_SENSING_VBUS_V_PER_COUNT                                                             \
    (float)(CURRENT_SENSE_PROFILE_ADC_REFERENCE_V / (float)CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS *  \
            (MOTOR_SENSING_VBUS_R1_KOHM + MOTOR_SENSING_VBUS_R2_KOHM) /                            \
            MOTOR_SENSING_VBUS_R2_KOHM)
/* 相电流：A/count；增益为硬件放大倍数，不能当作此换算系数使用。 */
#define MOTOR_SENSING_CURRENT_A_PER_COUNT                                                          \
    (float)(CURRENT_SENSE_PROFILE_ADC_REFERENCE_V / (float)CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS /  \
            CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN / CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM)
/* 相电流软件跳闸阈值（A）。 */
#define MOTOR_SENSING_OVERCURRENT_TRIP_A CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A

/** 三相电流采样通道选择。 */
typedef enum
{
    MOTOR_HW_CURRENT_PHASE_A = 0,
    MOTOR_HW_CURRENT_PHASE_B,
    MOTOR_HW_CURRENT_PHASE_C
} MotorHwCurrentPhase;

/** MCU 温度轮询结果：一次调用内采集到的状态与换算值。 */
typedef struct
{
    bool sample_ready;  /* 本次轮询取得新样本（JEOS）。 */
    bool conversion_ok; /* 新样本换算成功；仅 sample_ready 时有效。 */
    uint32_t raw_ts;    /* 内部温度传感器原始计数；仅 sample_ready 时有效。 */
    uint32_t raw_vref;  /* VREFINT 原始计数；仅 sample_ready 时有效。 */
    float celsius;      /* 芯片结温，单位 °C；仅 conversion_ok 时有效。 */
    float vdda_mv;      /* 估算 VDDA，单位 mV；仅 conversion_ok 时有效。 */
} MotorHwTemperaturePoll;

/**
 * @brief  读取相电流注入采样原始计数（四次和域）。
 * @param  phase 目标相。
 * @return 12位转换结果的四次和（0..16380）；未知相返回0。
 * @note 快速环上下文调用；只读寄存器，不阻塞。
 */
uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase);

/**
 * @brief  读取母线电压注入采样原始计数。
 * @return 12位转换结果的四次和（0..16380）。
 * @note 快速环上下文调用；只读寄存器，不阻塞。
 */
uint16_t motor_hw_vbus_sample_raw(void);

/**
 * @brief  轮询 MCU 温度采样：处理 JEOS、读取两个 rank、换算并启动下一次转换。
 * @param  result 输出结果；函数返回后不保存该指针。
 * @note 2 kHz监督任务上下文调用；内部包含寄存器访问与下一次转换的启动，
 *       不在快速环调用。结果字段的有效性见其注释。
 */
void motor_hw_temperature_poll(MotorHwTemperaturePoll *result);

#endif
