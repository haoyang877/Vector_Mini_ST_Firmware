#ifndef MOTOR_HARDWARE_PROFILE_H
#define MOTOR_HARDWARE_PROFILE_H

/* 电机硬件配置契约：阻尼环/摩擦前馈是否安装的选择与校验。
 * platform/api 层拥有该契约；板级配置（hw_conf.h）引用，motor 模块可直接包含。 */

/* 阻尼环相关的编码器标定 profile 选择。 */
#define MOTOR_DAMPING_RING_DISABLED 0U
#define MOTOR_DAMPING_RING_ENABLED 1U
#ifndef MOTOR_HAS_DAMPING_RING
/* 当前电机未安装摩擦轴/阻尼环。 */
#define MOTOR_HAS_DAMPING_RING MOTOR_DAMPING_RING_DISABLED
#endif

/* 位置阻抗的阻尼/摩擦前馈开关：刻意与 MOTOR_HAS_DAMPING_RING 独立，
 * 使标定可保留阻尼环启动 profile 而单独关闭前馈控制。 */
#define MOTOR_DAMPING_FEEDFORWARD_DISABLED 0U
#define MOTOR_DAMPING_FEEDFORWARD_ENABLED 1U
#ifndef MOTOR_DAMPING_FEEDFORWARD
#define MOTOR_DAMPING_FEEDFORWARD MOTOR_HAS_DAMPING_RING
#endif

#if MOTOR_DAMPING_FEEDFORWARD != MOTOR_DAMPING_FEEDFORWARD_ENABLED &&                              \
    MOTOR_DAMPING_FEEDFORWARD != MOTOR_DAMPING_FEEDFORWARD_DISABLED
#error                                                                                             \
    "MOTOR_DAMPING_FEEDFORWARD must be MOTOR_DAMPING_FEEDFORWARD_ENABLED or MOTOR_DAMPING_FEEDFORWARD_DISABLED"
#endif
#endif
