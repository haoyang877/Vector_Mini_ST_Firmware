#ifndef FOC_SENSORLESS_RUN_H
#define FOC_SENSORLESS_RUN_H

#include "foc_sensorless.h"

/* 无感运行应用：启动序列与速度模式；观测器算法见 foc_sensorless.{c,h}。 */

/**
 * @brief  复位无感启动状态机。
 * @param  Startup 无感启动状态指针。
 */
void SensorlessStartup_Reset(SensorlessStartup_TypeDef *Startup);

/**
 * @brief  执行无感启动状态机的单 tick 步骤。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  Fluxobserver 磁链观测器状态指针。
 * @param  Startup 无感启动状态机指针。
 * @param  Config 启动参数集，默认配置或编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；故障经 Set_ErrorNow 上报并停止本 tick 输出。
 */
void SensorlessStartup_Run(FOC_TypeDef *FOC,
                           MotorControl_TypeDef *MotorControl,
                           PI_Controller_TypeDef *controller,
                           Fluxobserver_TypeDef *Fluxobserver,
                           SensorlessStartup_TypeDef *Startup,
                           const SensorlessStartupConfig_TypeDef *Config);

/** 默认无感启动参数集。 */
extern const SensorlessStartupConfig_TypeDef SensorlessStartup_DefaultConfig;

/** 编码器标定（Mode 13）观测器启动参数集；由 foc_encoder_calibration 模块消费。 */
extern const SensorlessStartupConfig_TypeDef SensorlessStartup_EncoderCalibConfig;

#endif
