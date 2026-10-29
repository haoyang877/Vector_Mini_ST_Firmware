#ifndef SAMPLING_SCALE_H
#define SAMPLING_SCALE_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_config.h"

/* 公共采样派生量：固定电气输入由bsp_config.h选择的板数据提供。 */

/* 固定输入的编译期派生量；不分配存储、不在采样时重复计算。
 * 双极性ADC以额定参考的一半为中点：2mΩ为±82.5A，6mΩ为±27.5A。 */
#define CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS                                                       \
    ((1UL << CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS) - 1UL)
#define CURRENT_SENSE_PROFILE_ADC_MIDSCALE_V (CURRENT_SENSE_PROFILE_ADC_REFERENCE_V / 2.0f)
#define CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM ((float)CURRENT_SENSE_SHUNT_MILLIOHM / 1000.0f)
#define CURRENT_SENSE_PROFILE_FULL_SCALE_A                                                         \
    (CURRENT_SENSE_PROFILE_ADC_MIDSCALE_V /                                                        \
     (CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN * CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM))
#define CURRENT_SENSE_PROFILE_PATH_COMPENSATION_OHM PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM

/* 电机感测的固定换算尺度；原码和温度分别由BSP采样/温度契约拥有。
 * 电流/VBUS原始计数为四次和，使用前折算为12位域；保留历史换算运算顺序。
 * 采集生命周期与原码由bsp_internal.h及本板BSP实现。 */

/* 固定板级输入由bsp_config.h提供；以下只派生换算系数。
 * 全部操作数为常量，调用点只保留与采样值的运算。 */
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

#endif
