#ifndef __FOC_RUN_H__
#define __FOC_RUN_H__

#include "position_cascade.h"

/** Deferred outer controller entry; called only by the lower-priority motor worker. */
void MotorOuterLoop_Service(void);
/** Fast-owner queries; no worker state is borrowed. */
bool MotorOuterLoop_IsReady(void);
bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry);

/**
 * @brief 从CAN上下文有界读取快环发布的三个同周期内部参考，不运行控制器。
 * @param[out] values 三元素数组，依次规划位置rad、规划速度rad/s、速度环参考rad/s。
 * @param[out] sequence 设备发布序号低16位，允许回绕；重复读取不递增。
 * @return true表示完整有限且40ms内的新鲜规划快照，false时输出保持原值。
 * @note ADC快环为唯一发布者，CAN RX可被其抢占；最多重读三次，不关中断、不等待。
 */
bool MotorOuterLoop_GetMotionTelemetry(float values[3], uint16_t *sequence);

/**
 * @brief 发布已通过 CAN 业务校验的位置目标，供外部参考候选使用。
 * @param[in] position_rad 接受的机械目标，rad。
 * @param[in] received_ms 本机接收时的单调毫秒。
 * @note 仅 CAN RX 单写者调用；不运行控制器、不复位轨迹、不修改反馈。
 * 默认点到点构建为无副作用操作，快环以非阻塞一致快照交给控制工作线程。
 */
void MotorOuterLoop_PublishPositionTarget(float position_rad, uint32_t received_ms);

#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_sensorless.h"
#include "foc_pid.h"
#include "data_type.h"

/** Service completion, validate live inputs and release one job per ten fast ticks.
 * Must run in the fast context even when the motor is disabled, to discard stale jobs. */
void MotorOuterLoop_FastTick(MotorControl_TypeDef *motor, PI_Controller_TypeDef *pi,
    Encoder_TypeDef *encoder);

void Task_Current_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver);
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Encoder_TypeDef *Encoder);
extern const SensorlessStartupConfig_TypeDef SensorlessStartup_DefaultConfig;
/* 编码器标定的两配置由 MotorLoadProfile_Calibration() 选择。 */

/**
 * @brief 执行无感启动及闭环交接，快环每50 μs调用一次。
 * @param FOC 电流环及调制状态。
 * @param MotorControl 电机实时参数与命令，电流A、机械速度rad/s。
 * @param controller 速度PI状态，单一快环所有者。
 * @param Fluxobserver 电角度及电角速度观测状态，rad与rad/s。
 * @param Startup 启动状态机，同一快环持有。
 * @param Config 运行期间保持不变的只读启动档位；IDLE入场校验。
 * @note 非阻塞；可变电机参数、限流及故障仍逐拍检查，故障禁止继续出力。
 */
void Task_Sensorless_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Fluxobserver_TypeDef *Fluxobserver, SensorlessStartup_TypeDef *Startup, const SensorlessStartupConfig_TypeDef *Config);
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
void Task_Position_Mode_Reset(void);
void Task_Voltage_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Task_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);

#endif
