#ifndef CURRENT_SENSE_PROFILE_H
#define CURRENT_SENSE_PROFILE_H

#include "board_profile_defaults.h"

/* 公共采样派生量：固定输入与保护限值由board_profile_defaults.h选择的目标数据提供。 */

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

#endif
