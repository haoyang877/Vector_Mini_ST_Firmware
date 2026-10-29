// clang-format off
#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

/* 可选板卡与电机型号。 */
#define BSP_BOARD_STM32G4           1U
#define BSP_BOARD_AT32F456          2U
#define FOC_MOTOR_PROFILE_HT8115_4  1U
#define FOC_MOTOR_PROFILE_YQ_5708   2U

/* 全局选择一套板卡和电机；切换工程时修改这两行。 */
#define BSP_BOARD                 BSP_BOARD_AT32F456
#define FOC_ACTIVE_MOTOR_PROFILE  FOC_MOTOR_PROFILE_YQ_5708

#if defined(FIRMWARE_TARGET_STM32_APP) && BSP_BOARD != BSP_BOARD_STM32G4
#error "Select an STM32 board for the STM32 project"
#endif
#if defined(FIRMWARE_TARGET_AT32_OFF) && BSP_BOARD != BSP_BOARD_AT32F456
#error "Select an AT32 board for the AT32 project"
#endif
#if (defined(AT32F456CCU7) || defined(AT32F456xx)) && !defined(FIRMWARE_TARGET_AT32_OFF)
#error "AT32 firmware requires the phase-locked application target"
#endif

#endif
// clang-format on
