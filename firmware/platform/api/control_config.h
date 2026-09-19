#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制时基与无感启动默认参数契约。
 * platform/api 层拥有该契约；板级配置（hw_conf.h）引用并据此派生定时器参数，
 * motor 层算法直接包含本头文件。 */

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

/* Sensorless speed-mode startup and observer handoff. */
#define SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S 0.50f
#define SENSORLESS_ALIGN_HOLD_TIME_S 0.30f
#define SENSORLESS_ALIGN_CURRENT_A 2.0f
#define SENSORLESS_STARTUP_IQ_INITIAL_A 0.15f
#define SENSORLESS_STARTUP_IQ_A 0.50f
#define SENSORLESS_STARTUP_IQ_RAMP_TIME_S 2.00f
#define SENSORLESS_STARTUP_ID_A 0.5f
#define SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S 250.0f
#define SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S 420.0f
#define SENSORLESS_STARTUP_RAMP_TIME_S 2.00f
#define SENSORLESS_STARTUP_SPEED_LOCK_TIME_S 0.20f
#define SENSORLESS_SPEED_LOCK_FILTER_ALPHA 1.0f
#define SENSORLESS_OBSERVER_LOCK_RATIO 0.25f
#define SENSORLESS_ANGLE_HANDOFF_TIME_S 0.10f
#define SENSORLESS_STARTUP_LOCK_TIMEOUT_S 3.00f
#define SENSORLESS_ID_RAMP_DOWN_TIME_S 0.50f
#define SENSORLESS_OBSERVER_LOSS_TIME_S 0.20f
#define SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S 5000.0f
#define SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA 0.1042f

#endif
