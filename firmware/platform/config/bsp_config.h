#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/* 板型选择只在配置边界识别MCU/历史profile；普通产品和主机测试默认使用旧板。
 * 未知或冲突的显式板型必须报错，不能静默回退。 */
#define BSP_BOARD_STM32G4 1
#define BSP_BOARD_AT32F456 2
#ifndef BSP_BOARD
#if defined(FOC_ACTIVE_HW_PROFILE)
#define BSP_BOARD FOC_ACTIVE_HW_PROFILE
#elif defined(AT32F456CCU7) || defined(AT32F456xx)
#define BSP_BOARD BSP_BOARD_AT32F456
#else
#define BSP_BOARD BSP_BOARD_STM32G4
#endif
#endif
#define FOC_HW_PROFILE_VECTOR_MINI_ST BSP_BOARD_STM32G4
#define FOC_HW_PROFILE_R40_F456 BSP_BOARD_AT32F456
#ifndef FOC_ACTIVE_HW_PROFILE
#define FOC_ACTIVE_HW_PROFILE BSP_BOARD
#endif
#if BSP_BOARD != FOC_ACTIVE_HW_PROFILE
#error "BSP board and analog profile disagree"
#endif
#if (defined(AT32F456CCU7) || defined(AT32F456xx)) && BSP_BOARD != BSP_BOARD_AT32F456
#error "BSP board and AT32 target disagree"
#endif
#if defined(STM32G431xx) && BSP_BOARD != BSP_BOARD_STM32G4
#error "BSP board and STM32 target disagree"
#endif
#if BSP_BOARD == BSP_BOARD_AT32F456
#include "boards/at32f456.h"
#elif BSP_BOARD == BSP_BOARD_STM32G4
#include "boards/stm32g4.h"
#else
#error "Unknown BSP board"
#endif

/* 由构建目标输入的永久关相锁；BSP不解释应用的限时运行策略。 */
#ifndef BSP_PWM_HARD_LOCK
#define BSP_PWM_HARD_LOCK 0
#endif
#if BSP_PWM_HARD_LOCK != 0 && BSP_PWM_HARD_LOCK != 1
#error "Invalid BSP PWM hard lock"
#endif

#ifndef BSP_ENABLED_LED
#define BSP_ENABLED_LED 1
#endif
#ifndef BSP_ENABLED_UART
#define BSP_ENABLED_UART 0
#endif
#if (BSP_ENABLED_LED != 0 && BSP_ENABLED_LED != 1) ||                                              \
    (BSP_ENABLED_UART != 0 && BSP_ENABLED_UART != 1)
#error "Non-boolean LED/UART configuration"
#endif
#if (BSP_ENABLED_LED && !BSP_SUPPORTED_LED) || (BSP_ENABLED_UART && !BSP_SUPPORTED_UART)
#error "LED/UART has no board resource binding"
#endif

#ifndef BSP_ENABLED_PHASE_CURRENT
#define BSP_ENABLED_PHASE_CURRENT 1
#endif
#ifndef BSP_ENABLED_BUS_VOLTAGE
#define BSP_ENABLED_BUS_VOLTAGE 1
#endif
#ifndef BSP_ENABLED_BUS_CURRENT
#define BSP_ENABLED_BUS_CURRENT BSP_SUPPORTED_BUS_CURRENT
#endif
#ifndef BSP_ENABLED_ROTOR_ENCODER
#define BSP_ENABLED_ROTOR_ENCODER 1
#endif
#ifndef BSP_ENABLED_PWM
#define BSP_ENABLED_PWM 1
#endif
#ifndef BSP_ENABLED_CAN
#define BSP_ENABLED_CAN 1
#endif
#ifndef BSP_ENABLED_MCU_TEMPERATURE
#define BSP_ENABLED_MCU_TEMPERATURE (BSP_BOARD == BSP_BOARD_STM32G4)
#endif
#ifndef BSP_ENABLED_MOTOR_TEMPERATURE
#define BSP_ENABLED_MOTOR_TEMPERATURE BSP_SUPPORTED_MOTOR_TEMPERATURE
#endif
#ifndef BSP_ENABLED_OUTPUT_SINCOS
#define BSP_ENABLED_OUTPUT_SINCOS 0
#endif
#ifndef BSP_ENABLED_FAST_LOOP_PROFILE
#define BSP_ENABLED_FAST_LOOP_PROFILE 0
#endif

#ifndef BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE
#define BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE BSP_ENABLED_MCU_TEMPERATURE
#endif
#ifndef BSP_ENABLED_HARDWARE_VERSION
#define BSP_ENABLED_HARDWARE_VERSION 0
#endif
#define BSP_ENABLED_INTERNAL_ANALOG_SAMPLING                                                       \
    (BSP_ENABLED_MCU_TEMPERATURE || BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE)
#if BSP_ENABLED_HARDWARE_VERSION
#error "Hardware version ADC is not wired on current boards"
#endif
#if (BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE != 0 && BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE != 1) ||          \
    (BSP_ENABLED_HARDWARE_VERSION != 0 && BSP_ENABLED_HARDWARE_VERSION != 1)
#error "Non-boolean analog capability configuration"
#endif
/* supported不可由目标覆盖；硬件能力与使能的布尔值及依赖统一在此验收。 */
#if (BSP_SUPPORTED_PHASE_CURRENT != 0 && BSP_SUPPORTED_PHASE_CURRENT != 1) ||                      \
    (BSP_ENABLED_PHASE_CURRENT != 0 && BSP_ENABLED_PHASE_CURRENT != 1)
#error "Non-boolean BSP feature: PHASE_CURRENT"
#endif
#if BSP_ENABLED_PHASE_CURRENT && !BSP_SUPPORTED_PHASE_CURRENT
#error "Unsupported BSP feature: PHASE_CURRENT"
#endif
#if (BSP_SUPPORTED_BUS_VOLTAGE != 0 && BSP_SUPPORTED_BUS_VOLTAGE != 1) ||                          \
    (BSP_ENABLED_BUS_VOLTAGE != 0 && BSP_ENABLED_BUS_VOLTAGE != 1)
#error "Non-boolean BSP feature: BUS_VOLTAGE"
#endif
#if BSP_ENABLED_BUS_VOLTAGE && !BSP_SUPPORTED_BUS_VOLTAGE
#error "Unsupported BSP feature: BUS_VOLTAGE"
#endif
#if (BSP_SUPPORTED_BUS_CURRENT != 0 && BSP_SUPPORTED_BUS_CURRENT != 1) ||                          \
    (BSP_ENABLED_BUS_CURRENT != 0 && BSP_ENABLED_BUS_CURRENT != 1)
#error "Non-boolean BSP feature: BUS_CURRENT"
#endif
#if BSP_ENABLED_BUS_CURRENT && !BSP_SUPPORTED_BUS_CURRENT
#error "Unsupported BSP feature: BUS_CURRENT"
#endif
#ifdef BSP_ENABLED_PHASE_VOLTAGE
#error "Unsupported BSP feature: PHASE_VOLTAGE removed"
#endif
#if (BSP_SUPPORTED_ROTOR_ENCODER != 0 && BSP_SUPPORTED_ROTOR_ENCODER != 1) ||                      \
    (BSP_ENABLED_ROTOR_ENCODER != 0 && BSP_ENABLED_ROTOR_ENCODER != 1)
#error "Non-boolean BSP feature: ROTOR_ENCODER"
#endif
#if BSP_ENABLED_ROTOR_ENCODER && !BSP_SUPPORTED_ROTOR_ENCODER
#error "Unsupported BSP feature: ROTOR_ENCODER"
#endif
#if (BSP_SUPPORTED_PWM != 0 && BSP_SUPPORTED_PWM != 1) ||                                          \
    (BSP_ENABLED_PWM != 0 && BSP_ENABLED_PWM != 1)
#error "Non-boolean BSP feature: PWM"
#endif
#if BSP_ENABLED_PWM && !BSP_SUPPORTED_PWM
#error "Unsupported BSP feature: PWM"
#endif
#if (BSP_SUPPORTED_CAN != 0 && BSP_SUPPORTED_CAN != 1) ||                                          \
    (BSP_ENABLED_CAN != 0 && BSP_ENABLED_CAN != 1)
#error "Non-boolean BSP feature: CAN"
#endif
#if BSP_ENABLED_CAN && !BSP_SUPPORTED_CAN
#error "Unsupported BSP feature: CAN"
#endif
#if (BSP_SUPPORTED_MCU_TEMPERATURE != 0 && BSP_SUPPORTED_MCU_TEMPERATURE != 1) ||                  \
    (BSP_ENABLED_MCU_TEMPERATURE != 0 && BSP_ENABLED_MCU_TEMPERATURE != 1)
#error "Non-boolean BSP feature: MCU_TEMPERATURE"
#endif
#if BSP_ENABLED_MCU_TEMPERATURE && !BSP_SUPPORTED_MCU_TEMPERATURE
#error "Unsupported BSP feature: MCU_TEMPERATURE"
#endif
#if (BSP_SUPPORTED_MOTOR_TEMPERATURE != 0 && BSP_SUPPORTED_MOTOR_TEMPERATURE != 1) ||              \
    (BSP_ENABLED_MOTOR_TEMPERATURE != 0 && BSP_ENABLED_MOTOR_TEMPERATURE != 1)
#error "Non-boolean BSP feature: MOTOR_TEMPERATURE"
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE && !BSP_SUPPORTED_MOTOR_TEMPERATURE
#error "Unsupported BSP feature: MOTOR_TEMPERATURE"
#endif
#if (BSP_SUPPORTED_OUTPUT_SINCOS != 0 && BSP_SUPPORTED_OUTPUT_SINCOS != 1) ||                      \
    (BSP_ENABLED_OUTPUT_SINCOS != 0 && BSP_ENABLED_OUTPUT_SINCOS != 1)
#error "Non-boolean BSP feature: OUTPUT_SINCOS"
#endif
#if BSP_ENABLED_OUTPUT_SINCOS && !BSP_SUPPORTED_OUTPUT_SINCOS
#error "Unsupported BSP feature: OUTPUT_SINCOS"
#endif
#if (BSP_SUPPORTED_FAST_LOOP_PROFILE != 0 && BSP_SUPPORTED_FAST_LOOP_PROFILE != 1) ||              \
    (BSP_ENABLED_FAST_LOOP_PROFILE != 0 && BSP_ENABLED_FAST_LOOP_PROFILE != 1)
#error "Non-boolean BSP feature: FAST_LOOP_PROFILE"
#endif
#if BSP_ENABLED_FAST_LOOP_PROFILE && !BSP_SUPPORTED_FAST_LOOP_PROFILE
#error "Unsupported BSP feature: FAST_LOOP_PROFILE"
#endif
#define BSP_ENABLED_AUX_SAMPLING (BSP_ENABLED_MOTOR_TEMPERATURE || BSP_ENABLED_OUTPUT_SINCOS)

/* 初始中点不是实测零点；三相仍分别在启动采样后取得各自偏置。 */
#define PARAM_HW_CURRENT_OFFSET_A_COUNTS (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))
#define PARAM_HW_CURRENT_OFFSET_B_COUNTS (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))
#define PARAM_HW_CURRENT_OFFSET_C_COUNTS (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))

#if BSP_ENCODER_DEVICE_TYPE != 1U && BSP_ENCODER_DEVICE_TYPE != 3U
#error "Unsupported encoder device selection"
#endif

/* SPI事务期限由实际构建配置给出，MCU实现不识别APP或台架名字。 */
#ifndef BSP_ENCODER_TIMEOUT_US
#define BSP_ENCODER_TIMEOUT_US 20U
#endif

#endif
