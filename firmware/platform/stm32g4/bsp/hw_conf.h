#ifndef HW_CONF_H
#define HW_CONF_H

#include "main.h"
#include "control_config.h"
#include "motor_sensing.h"
#include "current_sense_profile.h"
#include "encoder_sensor.h"

/* 编码器传感器选择：每个型号一个 driver，未选中的 driver 编译为空。 */
#ifndef ENCODER_SENSOR_TYPE
#define ENCODER_SENSOR_TYPE ENCODER_SENSOR_TYPE_TLE5012B
#endif

/* 固定板级输入：TIM1时钟Hz，须与实际RCC配置一致。 */
#define PWM_TIM_CLOCK 170000000
/* 编译期派生：中心对齐PWM的计数周期及秒周期。 */
#define PWM_TIM_FREQ FOC_FREQ
#define PWM_TIM_PERIOD (PWM_TIM_CLOCK / PWM_TIM_FREQ / 2)
#define PWM_PERIOD FOC_PERIOD

/* 低侧电流采样的兼容名称；固定输入和派生量均只有一个定义源。 */
#define SENSING_RES CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM
#define CURRENT_AMP_GAIN CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN
/* 换算常量归 platform/api/motor_sensing.h 所有；此处保留历史名称。 */
#define SENSING_CURR_FACTOR MOTOR_SENSING_CURRENT_A_PER_COUNT
#define ADC2_SUM_TO_COUNTS MOTOR_SENSING_ADC_SUM_TO_COUNTS

/* 既有保护策略，不能用ADC硬件满量程替代。 */
#define CURRENT_SENSE_RELIABLE_LIMIT_A CURRENT_SENSE_PROFILE_RELIABLE_LIMIT_A
#define CURRENT_COMMAND_LIMIT_MAX_A CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A
#define CURRENT_CALIB_LIMIT_MAX_A CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A
#define CURRENT_OVERCURRENT_TRIP_A MOTOR_SENSING_OVERCURRENT_TRIP_A

/* 母线分压输入的兼容名称，单位kΩ。 */
#define VBUS_R1 MOTOR_SENSING_VBUS_R1_KOHM
#define VBUS_R2 MOTOR_SENSING_VBUS_R2_KOHM
/* 母线换算系数的兼容名称，单位V/count。 */
#define SENSING_VBUS_FACTOR MOTOR_SENSING_VBUS_V_PER_COUNT

#define TEMP_R2 3.3f

/* 历史器件估值：目前无调用，不代表TIM1已启用死区补偿。 */
#define MOS_DEADTIME 2.1e-7f
/* 体二极管压降估值，单位V。 */
#define MOS_VDIODE 0.5f

#define DBC_FACTOR MOS_DEADTIME / PWM_PERIOD / 1000000000.0f

#define CURRENT_ADC ADC2
#define IA_ADC_CHANNEL JDR1
#define IB_ADC_CHANNEL JDR2
#define IC_ADC_CHANNEL JDR3

#define VBUS_ADC ADC2
#define VBUS_ADC_CHANNEL JDR4

#define TEMP_ADC ADC1
#define TEMP_ADC_CHANNEL JDR1

/* 电机启动/标定默认参数归 firmware/motor/motor_startup_profile.h（motor 层）；
 * 板级配置不引用该头文件，也不再重复定义相关常量。 */

#endif
