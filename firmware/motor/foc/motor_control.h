#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_task_result.h"
#include "position_cascade.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "flux_observer.h"
#include "foc_pid.h"
#include "motor_types.h"

/* 控制功能与外环入口；显式借用所属对象，不依赖服务命令或APP生命周期。 */

/**
 * @brief  外环 2 kHz 慢拍：位置级联与速度 PI 直接执行并发布电流参考。
 * @param configuration 只读活动配置。
 * @param  motor 电机控制状态。
 * @param  pi 速度环控制器（速度模式使用）。
 * @param  encoder 编码器状态。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是位置/速度外环输出的唯一写者，
 *       20 kHz 快环只读取 motor_state.iq_target_a。
 */
void MotorOuterLoop_SlowTick(const MotorConfiguration *configuration,
                             MotorControlState *motor,
                             PiController *pi,
                             EncoderState *encoder);
/**
 * @brief  查询外环是否就绪：供快速环启动确认使用。
 * @return 就绪返回 true；只读取外环自有状态。
 */
bool MotorOuterLoop_IsReady(void);
/**
 * @brief  请求外环在下一慢拍复位控制与就绪状态。
 * @note 模式切换与故障入口调用；可在非快速环上下文调用。
 */
void MotorOuterLoop_RequestReset(void);
/**
 * @brief 选择反馈/开环角度和电流/电压控制律，再执行共用FOC流水线。
 * @param configuration 只读活动配置。
 * @param foc 电流环对象。
 * @param motor 可写运行对象，不含配置。
 * @param encoder 编码器状态。
 * @param observer 磁链观测器。
 * @return 当前功能事实，下一任务由主状态机决定。
 * @note 仅运行组装层在 20 kHz 调用；须先确认无故障且处于 STARTING/RUNNING。
 *       不反查生命周期、不选择维护、不启相，终态交给主状态机复核。
 */
MotorTaskResult MotorControl_Step(const MotorConfiguration *configuration,
                                  FocState *foc,
                                  MotorControlState *motor,
                                  EncoderState *encoder,
                                  FluxObserverState *observer);
#endif
