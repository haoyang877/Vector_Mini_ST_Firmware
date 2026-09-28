#ifndef MOTOR_EXECUTION_H
#define MOTOR_EXECUTION_H
#include "motor_task_result.h"

/* 控制任务与外环入口：调用方显式传入控制对象；不选择维护或读取生命周期。 */

#include "position_cascade.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_sensorless.h"
#include "foc_pid.h"
#include "data_type.h"

/**
 * @brief  外环 2 kHz 慢拍：位置级联/轨迹与速度 PI 直接执行并发布电流参考。
 * @param  motor 电机控制状态。
 * @param  pi 速度环控制器（速度模式使用）。
 * @param  encoder 编码器状态。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是位置/速度外环输出的唯一写者，
 *       20 kHz 快环只读取 MotorControl.iqRef。
 */
void MotorOuterLoop_SlowTick(MotorControl_TypeDef *motor,
                             PI_Controller_TypeDef *pi,
                             Encoder_TypeDef *encoder);
/**
 * @brief  查询外环是否就绪：供快速环启动确认使用。
 * @return 就绪返回 true；只读取外环自有状态。
 */
bool MotorOuterLoop_IsReady(void);
/**
 * @brief  读取外环遥测快照。
 * @param  telemetry 输出参数，调用方持有。
 * @return 快照有效返回 true；参数为空返回 false。
 */
bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry);

/**
 * @brief  电流模式任务：执行一拍 d/q 电流闭环。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  Encoder 编码器状态。
 * @param  Fluxobserver 磁链观测器状态。
 * @note 仅 20 kHz 快速环调用。
 */
void Task_Current_Mode(FOC_TypeDef *FOC,
                       MotorControl_TypeDef *MotorControl,
                       Encoder_TypeDef *Encoder,
                       Fluxobserver_TypeDef *Fluxobserver);
/**
 * @brief  速度模式任务：执行一拍速度外环（含斜坡、限幅与电流指令）。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  controller 速度 PI 控制器。
 * @param  Encoder 编码器状态。
 * @note 仅快速环调用；控制器就绪由运行状态机在准入时把关。
 */
void Task_Speed_Mode(FOC_TypeDef *FOC,
                     MotorControl_TypeDef *MotorControl,
                     PI_Controller_TypeDef *controller,
                     Encoder_TypeDef *Encoder);
/**
 * @brief  位置模式适配任务：外环就绪前锁定当前位置并完成入口准备。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  Encoder 编码器状态。
 * @note 生产路径由外环执行本适配器；外部入口供调试与夹具使用。
 */
void Task_Position_Mode(FOC_TypeDef *FOC,
                        MotorControl_TypeDef *MotorControl,
                        Encoder_TypeDef *Encoder);
/**
 * @brief  复位位置模式适配状态：请求外环在下一慢拍重建，并清零位置状态。
 * @note 模式切换与故障入口调用；可在非快速环上下文调用。
 */
void Task_Position_Mode_Reset(void);
/**
 * @brief  Vq 开环电压模式任务：固定电角度施加 q 轴电压。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  Encoder 编码器状态。
 * @note 仅快速环调用；需要有效编码器帧。
 */
void Task_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);

/**
 * @brief 执行调用方已准入的电流、速度、位置或 Vq 控制功能。
 * @param foc 电流环对象。
 * @param motor 配置和控制对象。
 * @param encoder 编码器状态。
 * @param observer 磁链观测器。
 * @return 当前功能事实，下一任务由主状态机决定。
 * @note 仅运行组装层在 20 kHz 调用；须先确认无故障且处于 STARTING/RUNNING。
 *       不反查生命周期、不选择维护、不启相，终态交给主状态机复核。
 */
MotorTaskResult MotorExecution_ControlStep(FOC_TypeDef *foc,
                                           MotorControl_TypeDef *motor,
                                           Encoder_TypeDef *encoder,
                                           Fluxobserver_TypeDef *observer);
#endif
