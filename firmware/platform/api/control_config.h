#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制时基契约：电流环、速度环、位置环与监督 tick 的频率。
 * platform/api 层拥有该契约；板级配置（hw_conf.h）引用并据此
 * 派生定时器参数，motor 层算法直接包含本头文件。
 * 电机的启动/标定默认参数（无感启动、编码器标定、阻尼环变体）属于 motor 层，
 * 见 firmware/motor/motor_startup_profile.h 与 firmware/motor/motor_hardware_profile.h。 */

/* 固定调度输入：各频率仍独立命名，不能因为当前同为2kHz而隐式绑定。
 * 修改频率需要同步实际定时器和调度，不属于可在线修改的设备参数。 */
#define FOC_FREQ 20000U
#define SPEED_LOOP_FREQ 2000U
#define POSITION_LOOP_FREQ 2000U
#define CASCADE_POSITION_LOOP_FREQ 2000U
#define SUPERVISOR_FREQ 2000U

/* 编译期派生：秒/拍、快速环分频及每毫秒监督拍数。 */
#define FOC_PERIOD (1.0f / (float)FOC_FREQ)

/* 兼容既有算法名称：电流环与PWM/FOC同频。 */
#define Current_Ts (FOC_PERIOD)

/* 速度PI和编码器速度估计。 */
#if SPEED_LOOP_FREQ == 0U
#error "SPEED_LOOP_FREQ must be greater than zero"
#elif SPEED_LOOP_FREQ > FOC_FREQ
#error "SPEED_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % SPEED_LOOP_FREQ) != 0U
#error "SPEED_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define SPEED_LOOP_DIVIDER (FOC_FREQ / SPEED_LOOP_FREQ)
#define Speed_Ts (1.0f / (float)SPEED_LOOP_FREQ)

/* 位置轨迹和阻抗控制。 */
#if POSITION_LOOP_FREQ == 0U
#error "POSITION_LOOP_FREQ must be greater than zero"
#elif POSITION_LOOP_FREQ > FOC_FREQ
#error "POSITION_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % POSITION_LOOP_FREQ) != 0U
#error "POSITION_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define POSITION_LOOP_DIVIDER (FOC_FREQ / POSITION_LOOP_FREQ)
#define Position_Ts (1.0f / (float)POSITION_LOOP_FREQ)

/* 位置串级与轨迹控制。 */
#if CASCADE_POSITION_LOOP_FREQ == 0U
#error "CASCADE_POSITION_LOOP_FREQ must be greater than zero"
#elif CASCADE_POSITION_LOOP_FREQ > FOC_FREQ
#error "CASCADE_POSITION_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % CASCADE_POSITION_LOOP_FREQ) != 0U
#error "CASCADE_POSITION_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define CASCADE_POSITION_LOOP_DIVIDER (FOC_FREQ / CASCADE_POSITION_LOOP_FREQ)
#define Cascade_Position_Ts (1.0f / (float)CASCADE_POSITION_LOOP_FREQ)

/* 监督 tick（TIM7）时基契约：温度、通信健康与指示器分频在该节拍更新。
 * 板级 TIM7 周期必须与本契约一致（见 MX_TIM7_Init 的 USER CODE 覆写与 CubeMX 工程）。 */
#if (SUPERVISOR_FREQ % 1000U) != 0U
#error "SUPERVISOR_FREQ must be a multiple of 1000 Hz for ms-based supervision"
#endif
#define SUPERVISOR_TICKS_PER_MS (SUPERVISOR_FREQ / 1000U)
#define Supervisor_Ts (1.0f / (float)SUPERVISOR_FREQ)

#endif
