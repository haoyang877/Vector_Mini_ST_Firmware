#ifndef R40_ANALOG_H
#define R40_ANALOG_H

/* R40模拟链路的实板输入。保护限值和完整APP目标选择留待独立定版。 */
#define R40_ADC_REFERENCE_V 3.3f
#define R40_ADC_MAX_COUNTS 4095U
#define R40_ADC_ACCUMULATIONS 4U
#define R40_PHASE_SHUNT_OHM 0.004f
#define R40_CURRENT_GAIN 20.0f
#define R40_BUS_SHUNT_OHM 0.002f
#define R40_BUS_CURRENT_GAIN 20.0f
#define R40_VBUS_TOP_KOHM 100.0f
#define R40_VBUS_BOTTOM_KOHM 3.3f

/* 输出轴只读诊断单独编译启用；不接入角度控制或改变公共BSP契约。 */
#ifndef R40_OUTPUT_SINCOS_DIAGNOSTICS
#define R40_OUTPUT_SINCOS_DIAGNOSTICS 0
#endif
#if R40_OUTPUT_SINCOS_DIAGNOSTICS != 0 && R40_OUTPUT_SINCOS_DIAGNOSTICS != 1
#error "R40 output sin/cos diagnostics selector must be 0 or 1"
#endif

/* 仅控制MCU内部温度/VINTRV；电机NTC单独选择，不改变公共接口的结温语义。 */
#ifndef R40_TEMPERATURE_SENSING_ENABLED
#define R40_TEMPERATURE_SENSING_ENABLED 0
#endif
#if R40_TEMPERATURE_SENSING_ENABLED != 0 && R40_TEMPERATURE_SENSING_ENABLED != 1
#error "R40 temperature sensing selector must be 0 or 1"
#endif

/* CN5: 3V3--NTC--TEMP_MOT_IN--R92(1k)--GND；R91(22Ω)只串在ADC滤波支路。
 * 用户确认R25=10kΩ、B=3950K；误差和B值标定温区尚未提供。 */
#ifndef R40_MOTOR_NTC_ENABLED
#define R40_MOTOR_NTC_ENABLED 1
#endif
#if R40_MOTOR_NTC_ENABLED != 0 && R40_MOTOR_NTC_ENABLED != 1
#error "R40 motor NTC selector must be 0 or 1"
#endif
#ifndef R40_MOTOR_NTC_R25_OHM
#define R40_MOTOR_NTC_R25_OHM 10000.0f
#endif
#ifndef R40_MOTOR_NTC_BETA_K
#define R40_MOTOR_NTC_BETA_K 3950.0f
#endif
#define R40_MOTOR_NTC_PULLDOWN_OHM 1000.0f
#define R40_MOTOR_NTC_RAIL_MARGIN_COUNTS 2U
/* 诊断换算有效窗口，不是传感器额定范围或电机过温跳闸阈值。 */
#define R40_MOTOR_NTC_MIN_C (-40.0f)
#define R40_MOTOR_NTC_MAX_C 150.0f

/* 必须来自此板实测的温度校准，不能把数据手册典型值当作逐颗出厂校准。
 * 尚未提供校准数据时，温度端口仍报告新原始样本，但conversion_ok保持false。
 * 斜率为实测有符号dVts/dT=(V2-V1)/(T2-T1)，不是手册公式的Avg_Slope。
 * 可由目标编译配置提供实测值；不修改公共BSP或参数记录ABI。 */
#ifndef R40_TEMPERATURE_CALIBRATED
#define R40_TEMPERATURE_CALIBRATED 0
#endif
#ifndef R40_TEMPERATURE_CAL_C
#define R40_TEMPERATURE_CAL_C 25.0f
#endif
#ifndef R40_TEMPERATURE_CAL_MV
#define R40_TEMPERATURE_CAL_MV 0.0f
#endif
#ifndef R40_TEMPERATURE_SLOPE_MV_PER_C
#define R40_TEMPERATURE_SLOPE_MV_PER_C 0.0f
#endif
#ifndef R40_INTERNAL_REFERENCE_MV
#define R40_INTERNAL_REFERENCE_MV 0.0f
#endif

#endif
