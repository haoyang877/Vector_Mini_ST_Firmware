#ifndef __FOC_FRICTION_IDENTIFICATION_H__
#define __FOC_FRICTION_IDENTIFICATION_H__

#include "data_type.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "friction_identification.h"
#include "motor_task_result.h"

/* 单次会话使用稳定只读配置，组合层拥有其生命周期；算法不读取产品 profile。 */
typedef struct
{
    FrictionIdentificationConfig_TypeDef core;
    float current_ratio_max; /* iqRef / 电流上限，无量纲。 */
} FocFrictionIdentificationConfig;

/**
 * @brief 初始化摩擦辨识适配器状态与分频计数。
 * @param config 只读配置；核心复制数值，NULL 形成无效配置。
 * @note 由启动组合层调用；不使能功率输出，也不修改电机参数。
 */
void FocFrictionIdentification_Init(const FocFrictionIdentificationConfig *config);

/**
 * @brief 中止当前辨识会话并复位输出。
 * @param motor 电机与控制状态。
 * @param speed_controller 速度环控制器状态；中止时复位。
 * @note 可在模式退出时调用；不写模式，不开关功率级。
 */
void FocFrictionIdentification_Abort(MotorControl_TypeDef *motor,
                                     PI_Controller_TypeDef *speed_controller);
/**
 * @brief 摩擦辨识周期任务：推进速度环与辨识内核，完成时请求停机。
 * @param foc 电流控制与观测量。
 * @param motor 电机与控制状态。
 * @param speed_controller 速度环控制器状态。
 * @param encoder 编码器状态。
 * @param config 从启动至取消/完成均须保持相同数值的只读配置。
 * @return 采集中返回 MOTOR_TASK_RUNNING；内核完成时返回 MOTOR_TASK_COMPLETED。
 * @note 仅由 20 kHz 快速中断调用；本任务不写模式或功率级，转换由运行状态机执行。
 */
MotorTaskResult FocFrictionIdentification_Task(FOC_TypeDef *foc,
                                               MotorControl_TypeDef *motor,
                                               PI_Controller_TypeDef *speed_controller,
                                               Encoder_TypeDef *encoder,
                                               const FocFrictionIdentificationConfig *config);
/**
 * @brief 把辨识结果写入电机参数台账。
 * @param motor 电机与控制状态；成功时写入库仑/粘性摩擦系数并置有效标志。
 * @return 结果有效、内核已完成且电机处于禁用模式时返回 true，否则返回 false。
 * @note 仅允许前台调用；不写 Flash，持久化由参数保存流程负责。
 */
bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor);

/**
 * @brief 确认辨识任务没有持有活动工作状态。
 * @return 本地任务释放后返回 true。
 * @note 运行所有者取消后查询，不执行 I/O。
 */
bool FocFrictionIdentification_IsReleased(void);
#endif
