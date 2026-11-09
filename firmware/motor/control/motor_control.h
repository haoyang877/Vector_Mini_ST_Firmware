#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_types.h"
#include "position_control.h"
#include "position_otg.h"
#include "speed_feedforward.h"
#include "encoder_feedback.h"
#include "foc.h"
#include "flux_observer.h"
#include "pi_controller.h"

/* 前台独占的计算副本。序号在停机、复位和模式切换后用于拒绝过期结果。 */
typedef struct
{
    PositionOtg trajectory;
    PositionOtgLimits limits;
    float target_rad;
    uint32_t serial;
} MotorPositionPlan;

typedef enum
{
    PLAN_IDLE,
    PLAN_QUEUED,
    PLAN_BUILDING,
    PLAN_READY,
    PLAN_INVALID
} PositionPlanState;

typedef struct
{
    MotorPositionPlan work;
    PositionPlanState state;
    uint32_t tick, apply_tick;
    float initial_position_rad;
    bool missed_deadline;
} PositionPlanQueue;

/** @brief 调用方独占的外环及前台位置规划状态。 */
typedef struct
{
    volatile bool reset_requested;
    bool ready;
    MotorControlMode mode;
    PositionOtg position_otg;
    SpeedFeedforwardState speed_feedforward;
    PositionCascadeState cascade;
    PositionPlanQueue plan;
    uint32_t plan_serial;
} MotorOuterLoop;

/* 控制功能与外环入口；显式借用所属对象，不依赖服务命令或APP生命周期。 */

/**
 * @brief  外环 慢拍：位置级联与速度 PI 直接执行并发布电流参考。
 * @param configuration 只读活动配置。
 * @param  motor 电机控制状态。
 * @param  pi 速度环控制器（速度模式使用）。
 * @param  encoder 编码器状态。
 * @note 仅由外环执行上下文调用；本函数是位置/速度外环输出的唯一写者，
 *       20 kHz 快环只读取 motor_state.iq_target_a。
 * @param outer 调用方持有的实例状态。
 */
void MotorOuterLoop_SlowTick(MotorOuterLoop *outer,
                             const MotorConfiguration *configuration,
                             MotorControlState *motor,
                             PiController *pi,
                             EncoderState *encoder);

/**
 * @brief 从外环取走一份待计算的位置计划。
 * @param work 前台私有的写入空间。
 * @return 有新计划时返回 true。
 * @note 前台调用者须屏蔽外环直到复制完成。
 * @param outer 调用方持有的实例状态。
 */
bool MotorOuterLoop_TakePositionPlan(MotorOuterLoop *outer, MotorPositionPlan *work);

/**
 * @brief 在前台求交接时刻的状态及其到新终点的段表。
 * @param work 由 Take 获得的独占副本。
 * @return 预测与建表成功时返回 true。
 * @note 计算期间允许外环与电流环抢占。
 */
bool MotorOuterLoop_BuildPositionPlan(MotorPositionPlan *work);

/**
 * @brief 发布已计算的段表；外环仅在约定的采样边界切换。
 * @param work 前台计算副本。
 * @param valid 建表是否成功。
 * @note 前台调用者须屏蔽外环直到复制完成；撤销或过期结果直接丢弃。
 * @param outer 调用方持有的实例状态。
 */
void MotorOuterLoop_PublishPositionPlan(MotorOuterLoop *outer,
                                        const MotorPositionPlan *work,
                                        bool valid);

/**
 * @brief  查询外环是否就绪：供快速环启动确认使用。
 * @return 就绪返回 true；只读取外环自有状态。
 * @param outer 调用方持有的实例状态。
 */
bool MotorOuterLoop_IsReady(const MotorOuterLoop *outer);
/**
 * @brief  请求外环在下一慢拍复位控制与就绪状态。
 * @note 模式切换与故障入口调用；可在非快速环上下文调用。
 * @param outer 调用方持有的实例状态。
 */
void MotorOuterLoop_RequestReset(MotorOuterLoop *outer);
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
