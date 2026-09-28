#ifndef __FOC_ERRHANDLE_H__
#define __FOC_ERRHANDLE_H__

#include "data_type.h"

/**
 * @brief 读取兼容模式投影。
 * @return 当前模式编号。
 */
ModeNow_TypeDef Get_ModeNow(void);
/**
 * @brief 登记历史前台模式握手。
 * @param tModeNow 目标模式。
 * @note 仅存储与齿槽前台握手尚未迁移；新控制目标使用运行所有者命令入口。
 */
void Set_ModeNow(ModeNow_TypeDef tModeNow);
/**
 * @brief 登记保护路径故障投影。
 * @param tErrorNow 故障编号。
 * @note 故障源调用；清除只由运行所有者在恢复条件通过后提交。
 */
void Set_ErrorNow(ErrorNow_TypeDef tErrorNow);
/**
 * @brief 清理控制参考、控制器积分与历史启动状态。
 * @note 不证明硬件已经关相，调用方另外使用功率级契约确认。
 */
void Clear_RunningData(void);
/**
 * @brief 校验并登记历史模式请求。
 * @param mode_set 请求模式。
 * @return 请求通过现有模式准入返回 true，否则返回 false。
 * @note 调试兼容入口；不代表生命周期已受理或输出已经启用。
 */
bool ModeSwitch_Handle(ModeNow_TypeDef mode_set);
/** @brief 根据模式和故障投影更新变化标志。 */
void Detect_Mode_Error_Change(void);
/**
 * @brief 读取模式或故障变化标志。
 * @return 已检测到变化返回 true。
 */
bool Return_Mode_Error_Change(void);
/** @brief 清除已消费的模式或故障变化标志。 */
void Clear_Mode_Error_Change(void);

#endif
