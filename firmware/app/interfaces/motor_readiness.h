#ifndef MOTOR_READINESS_H
#define MOTOR_READINESS_H
#include <stdint.h>

/** @brief 启动受限原因位；准备阶段缺项不是故障码，也不新增主状态。 */
enum
{
    MOTOR_START_CONFIGURATION_REQUIRED = 1U,
    MOTOR_START_CALIBRATION_REQUIRED = 2U,
    MOTOR_START_ENCODER_UNAVAILABLE = 4U,
    MOTOR_START_BUS_UNAVAILABLE = 8U,
    MOTOR_START_HEALTH_UNAVAILABLE = 16U,
    MOTOR_START_MODE_UNSUPPORTED = 32U,
    MOTOR_START_POSITION_UNAVAILABLE = 64U,
    MOTOR_START_STATE_UNAVAILABLE = 128U,
    MOTOR_START_OWNER_BUSY = 256U
};

/** @brief 基础自检事实位；全部通过不表示配置已确认或编码器已标定。 */
enum
{
    MOTOR_SELF_TEST_OUTPUT_DISABLED = 1U,
    MOTOR_SELF_TEST_SAMPLES_PROGRESSING = 2U,
    MOTOR_SELF_TEST_BUS_VALID = 4U,
    MOTOR_SELF_TEST_CURRENTS_VALID = 8U,
    MOTOR_SELF_TEST_TEMPERATURE_VALID = 16U,
    MOTOR_SELF_TEST_CONFIGURATION_VALID = 32U,
    MOTOR_SELF_TEST_ENCODER_VALID = 64U,
    MOTOR_SELF_TEST_RESOURCES_RELEASED = 128U,
    MOTOR_SELF_TEST_ALL = 255U
};

/** @brief 所有者只读诊断值；bit0/1/2/3为电流/速度/位置/VQ，bit5/6为电压/电流开环。 */
typedef struct
{
    uint32_t start_blockers;
    uint32_t self_test_checks;
    uint8_t startable_modes;
    uint8_t calibration_flags;
} MotorReadiness;
#endif
