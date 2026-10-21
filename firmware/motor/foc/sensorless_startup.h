#ifndef SENSORLESS_STARTUP_H
#define SENSORLESS_STARTUP_H

#include "motor_config.h"
#include "flux_observer.h"

/* 无感运行应用：启动序列与速度模式；观测器算法见 foc_sensorless.{c,h}。 */

/**
 * @brief  复位无感启动状态机。
 * @param  startup 无感启动状态指针。
 */
void SensorlessStartup_Reset(SensorlessStartupState *startup);

/**
 * @brief  执行无感启动状态机的单 tick 步骤。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  flux_observer 磁链观测器状态指针。
 * @param  startup 无感启动状态机指针。
 * @param  Config 启动参数集，编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；故障经 MotorFault_Report 上报并停止本 tick 输出。
 */
void SensorlessStartup_Run(const MotorConfiguration *configuration,
                           FocState *foc,
                           MotorControlState *motor_state,
                           PiController *controller,
                           FluxObserverState *flux_observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *Config);

/** 编码器标定（Mode 13）观测器启动参数集；由 foc_encoder_calibration 模块消费。 */
extern const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig;

#endif
