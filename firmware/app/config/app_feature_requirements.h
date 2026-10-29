#ifndef APP_FEATURE_REQUIREMENTS_H
#define APP_FEATURE_REQUIREMENTS_H

#include "bsp_config.h"

/* 当前完整APP和台架都依赖相电流、母线电压、转子与PWM；电机台架不使用CAN。
 * 这些是现有目标的固定准入要求，不允许用功能开关旁路保护或控制依赖。 */
#ifndef BSP_TARGET_MOTOR_ONLY
#define BSP_TARGET_MOTOR_ONLY 0
#endif
#if BSP_TARGET_MOTOR_ONLY != 0 && BSP_TARGET_MOTOR_ONLY != 1
#error "Invalid motor-only target selector"
#endif
#if BSP_TARGET_MOTOR_ONLY && (BSP_BOARD != BSP_BOARD_AT32F456 || defined(APP_TARGET_R40_F456))
#error "Motor-only profile cannot replace a full APP"
#endif
#if defined(BSP_REQUIRED_PHASE_CURRENT) && BSP_REQUIRED_PHASE_CURRENT != 1
#error "Current targets require PHASE_CURRENT"
#endif
#ifndef BSP_REQUIRED_PHASE_CURRENT
#define BSP_REQUIRED_PHASE_CURRENT 1
#endif
#if defined(BSP_REQUIRED_BUS_VOLTAGE) && BSP_REQUIRED_BUS_VOLTAGE != 1
#error "Current targets require BUS_VOLTAGE"
#endif
#ifndef BSP_REQUIRED_BUS_VOLTAGE
#define BSP_REQUIRED_BUS_VOLTAGE 1
#endif
#ifndef BSP_REQUIRED_BUS_CURRENT
#define BSP_REQUIRED_BUS_CURRENT 0
#endif
#if defined(BSP_REQUIRED_ROTOR_ENCODER) && BSP_REQUIRED_ROTOR_ENCODER != 1
#error "Current targets require ROTOR_ENCODER"
#endif
#ifndef BSP_REQUIRED_ROTOR_ENCODER
#define BSP_REQUIRED_ROTOR_ENCODER 1
#endif
#if defined(BSP_REQUIRED_PWM) && BSP_REQUIRED_PWM != 1
#error "Current targets require PWM"
#endif
#ifndef BSP_REQUIRED_PWM
#define BSP_REQUIRED_PWM 1
#endif
#if defined(BSP_REQUIRED_CAN) && BSP_REQUIRED_CAN != (!BSP_TARGET_MOTOR_ONLY)
#error "Current targets require CAN"
#endif
#ifndef BSP_REQUIRED_CAN
#define BSP_REQUIRED_CAN (!BSP_TARGET_MOTOR_ONLY)
#endif
#ifndef BSP_REQUIRED_MCU_TEMPERATURE
#define BSP_REQUIRED_MCU_TEMPERATURE (BSP_BOARD == BSP_BOARD_STM32G4)
#endif
#if BSP_BOARD == BSP_BOARD_STM32G4 && !BSP_REQUIRED_MCU_TEMPERATURE
#error "STM32 APP requires MCU temperature protection"
#endif
#ifndef BSP_REQUIRED_MOTOR_TEMPERATURE
#define BSP_REQUIRED_MOTOR_TEMPERATURE 0
#endif
#ifndef BSP_REQUIRED_OUTPUT_SINCOS
#define BSP_REQUIRED_OUTPUT_SINCOS 0
#endif
#ifndef BSP_REQUIRED_FAST_LOOP_PROFILE
#define BSP_REQUIRED_FAST_LOOP_PROFILE 0
#endif

/* MCU温度在AT32完整APP仍允许只读原码，缺少标定不伪装成有效温度。
 * 完整APP的非必需温度只允许在原有永久关相/限时目标内选择。 */
#if defined(APP_TARGET_R40_F456) && APP_TARGET_R40_F456 && !BSP_REQUIRED_MCU_TEMPERATURE &&        \
    (!defined(R40_APP_PHASE_LOCK) || R40_APP_PHASE_LOCK != 1) &&                                   \
    (!defined(R40_APP_BOUNDED_TEST) || R40_APP_BOUNDED_TEST != 1)
#error "Optional MCU temperature requires locked or bounded target"
#endif

#if BSP_REQUIRED_PHASE_CURRENT && !BSP_ENABLED_PHASE_CURRENT
#error "Required BSP feature disabled: PHASE_CURRENT"
#endif
#if BSP_REQUIRED_PHASE_CURRENT != 0 && BSP_REQUIRED_PHASE_CURRENT != 1
#error "Non-boolean application requirement: PHASE_CURRENT"
#endif
#if BSP_REQUIRED_BUS_VOLTAGE && !BSP_ENABLED_BUS_VOLTAGE
#error "Required BSP feature disabled: BUS_VOLTAGE"
#endif
#if BSP_REQUIRED_BUS_VOLTAGE != 0 && BSP_REQUIRED_BUS_VOLTAGE != 1
#error "Non-boolean application requirement: BUS_VOLTAGE"
#endif
#if BSP_REQUIRED_BUS_CURRENT && !BSP_ENABLED_BUS_CURRENT
#error "Required BSP feature disabled: BUS_CURRENT"
#endif
#if BSP_REQUIRED_BUS_CURRENT != 0 && BSP_REQUIRED_BUS_CURRENT != 1
#error "Non-boolean application requirement: BUS_CURRENT"
#endif
#ifdef BSP_REQUIRED_PHASE_VOLTAGE
#error "Removed application feature: PHASE_VOLTAGE"
#endif
#if BSP_REQUIRED_ROTOR_ENCODER && !BSP_ENABLED_ROTOR_ENCODER
#error "Required BSP feature disabled: ROTOR_ENCODER"
#endif
#if BSP_REQUIRED_ROTOR_ENCODER != 0 && BSP_REQUIRED_ROTOR_ENCODER != 1
#error "Non-boolean application requirement: ROTOR_ENCODER"
#endif
#if BSP_REQUIRED_PWM && !BSP_ENABLED_PWM
#error "Required BSP feature disabled: PWM"
#endif
#if BSP_REQUIRED_PWM != 0 && BSP_REQUIRED_PWM != 1
#error "Non-boolean application requirement: PWM"
#endif
#if BSP_REQUIRED_CAN && !BSP_ENABLED_CAN
#error "Required BSP feature disabled: CAN"
#endif
#if BSP_REQUIRED_CAN != 0 && BSP_REQUIRED_CAN != 1
#error "Non-boolean application requirement: CAN"
#endif
#if BSP_REQUIRED_MCU_TEMPERATURE && !BSP_ENABLED_MCU_TEMPERATURE
#error "Required BSP feature disabled: MCU_TEMPERATURE"
#endif
#if BSP_REQUIRED_MCU_TEMPERATURE != 0 && BSP_REQUIRED_MCU_TEMPERATURE != 1
#error "Non-boolean application requirement: MCU_TEMPERATURE"
#endif
#if BSP_REQUIRED_MOTOR_TEMPERATURE && !BSP_ENABLED_MOTOR_TEMPERATURE
#error "Required BSP feature disabled: MOTOR_TEMPERATURE"
#endif
#if BSP_REQUIRED_MOTOR_TEMPERATURE != 0 && BSP_REQUIRED_MOTOR_TEMPERATURE != 1
#error "Non-boolean application requirement: MOTOR_TEMPERATURE"
#endif
#if BSP_REQUIRED_OUTPUT_SINCOS && !BSP_ENABLED_OUTPUT_SINCOS
#error "Required BSP feature disabled: OUTPUT_SINCOS"
#endif
#if BSP_REQUIRED_OUTPUT_SINCOS != 0 && BSP_REQUIRED_OUTPUT_SINCOS != 1
#error "Non-boolean application requirement: OUTPUT_SINCOS"
#endif
#if BSP_REQUIRED_FAST_LOOP_PROFILE && !BSP_ENABLED_FAST_LOOP_PROFILE
#error "Required BSP feature disabled: FAST_LOOP_PROFILE"
#endif
#if BSP_REQUIRED_FAST_LOOP_PROFILE != 0 && BSP_REQUIRED_FAST_LOOP_PROFILE != 1
#error "Non-boolean application requirement: FAST_LOOP_PROFILE"
#endif

#endif
