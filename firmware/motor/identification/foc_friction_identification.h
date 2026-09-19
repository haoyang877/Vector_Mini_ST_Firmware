#ifndef __FOC_FRICTION_IDENTIFICATION_H__
#define __FOC_FRICTION_IDENTIFICATION_H__

#include "data_type.h"
#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "friction_identification.h"
#include "motor_work.h"

/**
 * @brief 初始化摩擦辨识适配器状态与分频计数。
 * @note 由启动组合层调用；不使能功率输出，也不修改电机参数。
 */
void FocFrictionIdentification_Init(void);
/**
 * @brief 启动一次摩擦辨识会话并复位输出。
 * @param motor 电机与控制状态；用于运行时配置校验与输出复位。
 * @param encoder 编码器状态；要求在线且已完成标定。
 * @param speed_controller 速度环控制器状态；启动时复位。
 * @return 配置、编码器与内核准入全部通过时返回 true，否则返回 false。
 * @note 仅由快速环模式任务调用；失败时内核已置为 FAILED，调用方按故障处理。
 */
bool FocFrictionIdentification_Start(MotorControl_TypeDef *motor,
                                     Encoder_TypeDef *encoder,
                                     PI_Controller_TypeDef *speed_controller);
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
 * @return 采集中返回 MOTOR_WORK_RUNNING；内核完成时返回 MOTOR_WORK_STOP。
 * @note 仅由 20 kHz 快速中断调用；本任务不写模式或功率级，转换由运行状态机执行。
 */
MotorWorkOutcome_TypeDef FocFrictionIdentification_Task(FOC_TypeDef *foc,
                                                        MotorControl_TypeDef *motor,
                                                        PI_Controller_TypeDef *speed_controller,
                                                        Encoder_TypeDef *encoder);
/**
 * @brief 把辨识结果写入电机参数台账。
 * @param motor 电机与控制状态；成功时写入库仑/粘性摩擦系数并置有效标志。
 * @return 结果有效、内核已完成且电机处于禁用模式时返回 true，否则返回 false。
 * @note 仅允许前台调用；不写 Flash，持久化由参数保存流程负责。
 */
bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor);
/**
 * @brief 查询辨识内核当前状态。
 * @return 内核状态枚举（空闲/运行/完成/失败）。
 * @note 可从前台或诊断路径调用；只读。
 */
FrictionIdentificationState_TypeDef FocFrictionIdentification_GetState(void);
/**
 * @brief 查询辨识内核的失败原因。
 * @return 失败原因枚举；未失败时返回无效原因值。
 * @note 只读。
 */
FrictionIdentificationReason_TypeDef FocFrictionIdentification_GetReason(void);
/**
 * @brief 查询当前速度点索引。
 * @return 进行中的速度点序号；空闲或完成时值由内核约定。
 * @note 只读。
 */
uint32_t FocFrictionIdentification_GetPointIndex(void);
/**
 * @brief 查询辨识进度百分比。
 * @return 0..100 的进度值，单位 %。
 * @note 只读。
 */
float FocFrictionIdentification_GetProgressPercent(void);
/**
 * @brief 查询已采集样本数。
 * @return 当前会话累计样本数。
 * @note 只读。
 */
uint32_t FocFrictionIdentification_GetSampleCount(void);
/**
 * @brief 查询辨识结果。
 * @return 内核结果只读指针；未完成时返回 NULL。
 * @note 指针所有权归内核，调用方不得修改。
 */
const FrictionIdentificationResult_TypeDef *FocFrictionIdentification_GetResult(void);
/**
 * @brief 查询原始样本数组。
 * @return 内核样本数组只读指针；上下文为空时返回 NULL。
 * @note 指针所有权归内核；容量由内核约定。
 */
const FrictionIdentificationSample_TypeDef *FocFrictionIdentification_GetSamples(void);

#endif
