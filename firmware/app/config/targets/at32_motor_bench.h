#ifndef AT32_MOTOR_BENCH_H
#define AT32_MOTOR_BENCH_H

/* 现有四个电机台架使用本地调试邮箱，不初始化CAN；不改变其原输出许可。 */
#define BSP_TARGET_MOTOR_ONLY 1
#define BSP_REQUIRED_CAN 0
#define BSP_ENABLED_CAN 0

#endif
