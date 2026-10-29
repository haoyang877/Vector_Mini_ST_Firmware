#ifndef R40_APP_COMMON_CONFIG_H
#define R40_APP_COMMON_CONFIG_H

/* AT32完整APP的公共装配输入；由各目标预包含，不进入算法或BSP接口。
 * 板型与电机仍为独立选择，本组合固定为R40/YQ-5708。 */
#define APP_TARGET_R40_F456 1
#define FOC_ACTIVE_HW_PROFILE 2
#define FOC_ACTIVE_MOTOR_PROFILE 2
#define BSP_APP_LIFECYCLE 1
#ifndef APP_ENABLED_DIAGNOSTIC_TELEMETRY
#define APP_ENABLED_DIAGNOSTIC_TELEMETRY 1
#endif
#define APP_ENFORCE_POWER_CONTRACT 1
#define APP_FAST_MAX_CYCLES 9000U
#define BSP_REQUIRED_MCU_TEMPERATURE 0

/* 完整APP共用同一MT6835采样路径：11.25MHz/16位DMA，避免轮询占用快环预算。
 * DMA2独占SPI2，不改变角度单位、CRC、ADC资源或50us保护。 */
#define BSP_ENCODER_FAST_SPI 1
#define BSP_ENCODER_WORD_TRANSFER 1
#define BSP_ENCODER_DMA_TRANSFER 1
#define BSP_ENCODER_TIMEOUT_US 40U

/* 未确认的热数据仅供既有测试目标使用，不能变成正式产品默认值。
 * 开放目标必须重新确认热数据；本头不解除关相或限时窗口。 */
#if !((defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK == 1) ||                                  \
      (defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST == 1))
#error "Temporary motor defaults require a phase-locked or bounded-test target"
#endif
#define PARAM_MOTOR_APP_TEST_DEFAULTS 1
#define PARAM_MOTOR_THERMAL_CONTINUOUS_VECTOR_A 3.0f
#define PARAM_MOTOR_THERMAL_PEAK_A 3.0f
#define PARAM_MOTOR_THERMAL_HEATING_TAU_S 3600.0f
#define PARAM_MOTOR_THERMAL_DERATING_ENTER_RATIO 0.8f
#define PARAM_MOTOR_THERMAL_DERATING_EXIT_RATIO 0.7f
#define PARAM_MOTOR_THERMAL_TRIP_RATIO 1.0f
#define PARAM_MOTOR_THERMAL_CLEAR_RATIO 0.5f

#endif
