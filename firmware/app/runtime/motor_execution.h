#ifndef MOTOR_EXECUTION_H
#define MOTOR_EXECUTION_H
#include "motor_task_result.h"
#include "motor_control_command.h"

/* 控制任务与外环入口：调用方显式传入控制对象；不选择维护或读取生命周期。 */
#include "position_cascade.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_sensorless.h"
#include "foc_pid.h"
#include "motor_types.h"

/**
 * @brief 验证控制目标格式及当前参数允许的数值范围，不决定运行许可。
 * @param command SI 值命令；未使用字段须为零，所有浮点字段须有限。
 * @param motor 只读限值和轴配置，电流 A、速度 rad/s、位置 rad。
 * @param foc 只读母线滤波电压，单位 V。
 * @param encoder 只读当前机械位置，单位 rad。
 * @return 格式错误返回 INVALID，越界返回 OUT_OF_RANGE，否则 ACCEPTED。
 * @note 不修改对象、不访问硬件；调用者同步配置和测量，另行检查故障、状态与启相许可。
 */
MotorControlResult MotorExecution_ValidateTarget(const MotorControlCommand *command,
                                                 const MotorControlState *motor,
                                                 const FocState *foc,
                                                 const EncoderState *encoder);

/**
 * @brief  外环 2 kHz 慢拍：位置级联与速度 PI 直接执行并发布电流参考。
 * @param  motor 电机控制状态。
 * @param  pi 速度环控制器（速度模式使用）。
 * @param  encoder 编码器状态。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是位置/速度外环输出的唯一写者，
 *       20 kHz 快环只读取 motor_state.iq_target_a。
 */
void MotorOuterLoop_SlowTick(MotorControlState *motor, PiController *pi, EncoderState *encoder);
/**
 * @brief  查询外环是否就绪：供快速环启动确认使用。
 * @return 就绪返回 true；只读取外环自有状态。
 */
bool MotorOuterLoop_IsReady(void);
/**
 * @brief  电流模式任务：执行一拍 d/q 电流闭环。
 * @param  foc foc 状态。
 * @param  motor_state 电机控制状态。
 * @param  encoder 编码器状态。
 * @param  flux_observer 磁链观测器状态。
 * @note 仅 20 kHz 快速环调用。
 */
void Task_Current_Mode(FocState *foc,
                       MotorControlState *motor_state,
                       EncoderState *encoder,
                       FluxObserverState *flux_observer);
/**
 * @brief  请求外环在下一慢拍复位控制与就绪状态。
 * @note 模式切换与故障入口调用；可在非快速环上下文调用。
 */
void MotorOuterLoop_RequestReset(void);
/**
 * @brief  vq_v 开环电压模式任务：固定电角度施加 q 轴电压。
 * @param  foc foc 状态。
 * @param  motor_state 电机控制状态。
 * @param  encoder 编码器状态。
 * @note 仅快速环调用；需要有效编码器帧。
 */
void Task_Vq_Mode(FocState *foc, MotorControlState *motor_state, EncoderState *encoder);

/**
 * @brief 执行调用方已准入的电流、速度、位置或 vq_v 控制功能。
 * @param foc 电流环对象。
 * @param motor 配置和控制对象。
 * @param encoder 编码器状态。
 * @param observer 磁链观测器。
 * @return 当前功能事实，下一任务由主状态机决定。
 * @note 仅运行组装层在 20 kHz 调用；须先确认无故障且处于 STARTING/RUNNING。
 *       不反查生命周期、不选择维护、不启相，终态交给主状态机复核。
 */
MotorTaskResult MotorExecution_ControlStep(FocState *foc,
                                           MotorControlState *motor,
                                           EncoderState *encoder,
                                           FluxObserverState *observer);
#endif
