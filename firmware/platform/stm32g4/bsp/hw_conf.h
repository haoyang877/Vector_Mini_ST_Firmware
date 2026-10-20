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
