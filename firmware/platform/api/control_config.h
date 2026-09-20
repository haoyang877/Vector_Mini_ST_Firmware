#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制时基与遥测采样契约：电流环、速度环、位置环、级联位置环与监督 tick 的频率，
 * 以及 RTT 采样分频。platform/api 层拥有该契约；板级配置（hw_conf.h）引用并据此
 * 派生定时器参数，motor 层算法直接包含本头文件。
 * 电机的启动/标定默认参数（无感启动、编码器标定、阻尼环变体）属于 motor 层，
 * 见 firmware/motor/motor_startup_profile.h 与 firmware/motor/motor_hardware_profile.h。 */

/* 控制时基契约：电流环频率由控制层定义，板级定时器据此配置。 */
#define FOC_FREQ 20000U
#define FOC_PERIOD (1.0f / (float)FOC_FREQ)

/* Current loop executes at the PWM/FOC rate. */
#define Current_Ts (FOC_PERIOD)

/* All speed PI controllers and encoder velocity estimation run at 2 kHz. */
#define SPEED_LOOP_FREQ 2000U
#if SPEED_LOOP_FREQ == 0U
#error "SPEED_LOOP_FREQ must be greater than zero"
#elif SPEED_LOOP_FREQ > FOC_FREQ
#error "SPEED_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % SPEED_LOOP_FREQ) != 0U
#error "SPEED_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define SPEED_LOOP_DIVIDER (FOC_FREQ / SPEED_LOOP_FREQ)
#define Speed_Ts (1.0f / (float)SPEED_LOOP_FREQ)

/* Position trajectory and impedance controller run at 2 kHz. */
#define POSITION_LOOP_FREQ 2000U
#if POSITION_LOOP_FREQ == 0U
#error "POSITION_LOOP_FREQ must be greater than zero"
#elif POSITION_LOOP_FREQ > FOC_FREQ
#error "POSITION_LOOP_FREQ must not exceed FOC_FREQ"
#elif (FOC_FREQ % POSITION_LOOP_FREQ) != 0U
#error "POSITION_LOOP_FREQ must divide FOC_FREQ exactly"
#endif
#define POSITION_LOOP_DIVIDER (FOC_FREQ / POSITION_LOOP_FREQ)
#define Position_Ts (1.0f / (float)POSITION_LOOP_FREQ)

/* Legacy cascaded position and trajectory controller also run at 2 kHz. */
#define CASCADE_POSITION_LOOP_FREQ 2000U
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
#define SUPERVISOR_FREQ 2000U
#if (SUPERVISOR_FREQ % 1000U) != 0U
#error "SUPERVISOR_FREQ must be a multiple of 1000 Hz for ms-based supervision"
#endif
#define SUPERVISOR_TICKS_PER_MS (SUPERVISOR_FREQ / 1000U)
#define Supervisor_Ts (1.0f / (float)SUPERVISOR_FREQ)

/* RTT 遥测契约：上行 channel 1，采样频率必须整除 FOC_FREQ。 */
#define RTT_SAMPLE_RATE_HZ 2000U
#if RTT_SAMPLE_RATE_HZ == 0U
#error "RTT_SAMPLE_RATE_HZ must be greater than zero"
#elif RTT_SAMPLE_RATE_HZ > FOC_FREQ
#error "RTT_SAMPLE_RATE_HZ must not exceed FOC_FREQ"
#elif (FOC_FREQ % RTT_SAMPLE_RATE_HZ) != 0U
#error "RTT_SAMPLE_RATE_HZ must divide FOC_FREQ exactly"
#endif
#define RTT_SAMPLE_DIVIDER (FOC_FREQ / RTT_SAMPLE_RATE_HZ)

/* 固定 4 通道 int16 帧：位置/转速/Iq 指令/Iq 反馈，布局见 rtt_telemetry.c。 */
#define RTT_JSCOPE_DESCRIPTOR "JScope_i2i2i2i2"

#endif
