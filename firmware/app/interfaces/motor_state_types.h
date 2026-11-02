#ifndef MOTOR_STATE_TYPES_H
#define MOTOR_STATE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* APP内部五态，数值不直接作为通信或持久化编号。 */
typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_SELF_TEST,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MotorState;

/** @brief 请求受理结果；受理不代表已经启相或标定完成。 */
typedef enum
{
    MOTOR_CONTROL_ACCEPTED = 0,
    MOTOR_CONTROL_INVALID,
    MOTOR_CONTROL_DENIED,
    MOTOR_CONTROL_BUSY,
    MOTOR_CONTROL_OUT_OF_RANGE,
    MOTOR_CONTROL_FAULT
} MotorControlResult;

/** @brief 独立标定类型；标定不自动保存到Flash。 */
typedef enum
{
    MOTOR_CALIBRATION_NONE,
    MOTOR_CALIBRATION_ENCODER_LINEARIZATION,
    MOTOR_CALIBRATION_ELECTRICAL_ZERO,
    MOTOR_CALIBRATION_MECHANICAL_ZERO
} MotorCalibrationType;

/** @brief 一次性读取的状态值；模式和故障使用电机内部编号，不是线路编号。 */
typedef struct
{
    MotorState state;
    uint16_t mode, fault;
    bool measurements_valid;
    float position_reference_rad, position_rad;
    float speed_reference_rad_s, speed_rad_s;
    float bus_voltage_v, bus_current_a;
    float iq_reference_a, iq_a, temperature_c, vq_v, vd_v;
} MotorStatus;

/** @brief 同模式易失目标；位置rad、速度rad/s、电流A、电压V、电角频率Hz。 */
typedef struct
{
    uint16_t mode;
    float position_rad, speed_rad_s, iq_a, vq_v, frequency_hz, id_a, vd_v;
} MotorTarget;

#endif
