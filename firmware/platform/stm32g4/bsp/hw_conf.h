#ifndef __HW_CONF_H__
#define __HW_CONF_H__

#include "main.h"
#include "control_config.h"
#include "motor_hardware_profile.h"
#include "motor_sensing.h"
#include "current_sense_profile.h"
#include "encoder_sensor.h"

/* 编码器传感器选择：每个型号一个 driver，未选中的 driver 编译为空。 */
#ifndef ENCODER_SENSOR_TYPE
#define ENCODER_SENSOR_TYPE ENCODER_SENSOR_TYPE_TLE5012B
#endif

#define PWM_TIM_CLOCK 170000000
/* 板级 PWM 定时器实现 control_config.h 的电流环频率契约。 */
#define PWM_TIM_FREQ FOC_FREQ
#define PWM_TIM_PERIOD (PWM_TIM_CLOCK / PWM_TIM_FREQ / 2)
#define PWM_PERIOD 5e-5f

/* Three-phase low-side current sensing; selected in current_sense_profile.h. */
#define SENSING_RES CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM
#define CURRENT_AMP_GAIN CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN
/* 换算常量归 platform/api/motor_sensing.h 所有；此处保留历史名称。 */
#define SENSING_CURR_FACTOR MOTOR_SENSING_CURRENT_A_PER_COUNT
#define ADC2_SUM_TO_COUNTS MOTOR_SENSING_ADC_SUM_TO_COUNTS

/* Keep normal control and software protection below the amplifier/ADC rails. */
#define CURRENT_SENSE_RELIABLE_LIMIT_A CURRENT_SENSE_PROFILE_RELIABLE_LIMIT_A
#define CURRENT_COMMAND_LIMIT_MAX_A CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A
#define CURRENT_CALIB_LIMIT_MAX_A CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A
#define CURRENT_OVERCURRENT_TRIP_A MOTOR_SENSING_OVERCURRENT_TRIP_A

/*bus voltagge R1 R2 (kohm)*/
#define VBUS_R1 MOTOR_SENSING_VBUS_R1_KOHM
#define VBUS_R2 MOTOR_SENSING_VBUS_R2_KOHM
/*bus voltage sensing factor (adc value/V)*/
#define SENSING_VBUS_FACTOR MOTOR_SENSING_VBUS_V_PER_COUNT

#define TEMP_R2 3.3f

/*MOSFET approximate deadtime (s)*/
#define MOS_DEADTIME 2.1e-7f
/*voltage of body diode (V)*/
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

/* 编码器标定（Mode 13 LUT / Mode 15 电角度零位）参数已上移
 * platform/api/control_config.h（经本文件 include 可见）；板级不再重复定义。 */

#endif
