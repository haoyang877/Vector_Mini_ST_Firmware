// clang-format off
#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

/* 工程通过BSP_BOARD选择硬件，公共源码不固定板型。 */
#define BSP_BOARD_STM32G4           1U
#define BSP_BOARD_AT32F456          2U
#define FOC_MOTOR_PROFILE_HT8115_4  1U
#define FOC_MOTOR_PROFILE_YQ_5708   2U

#ifndef BSP_BOARD
#error "Define BSP_BOARD in the project compiler settings"
#endif

/* 硬件采样与监督时基，两板共用。 */
#define FOC_FREQ 20000U
#define SUPERVISOR_DIVIDER 10U
#define SUPERVISOR_FREQ (FOC_FREQ / SUPERVISOR_DIVIDER)

/* 电机型号保留公共默认，工程可独立覆盖。 */
#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE  FOC_MOTOR_PROFILE_YQ_5708
#endif

#endif
// clang-format on
