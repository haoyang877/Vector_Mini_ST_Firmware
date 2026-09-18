#ifndef FOC_RUN_STATE_H
#define FOC_RUN_STATE_H

#include <stdbool.h>

/**
 * @brief 快速环故障检查：编码器离线或轴配置失效时置故障并停机。
 * @note 仅由 20 kHz 快速中断调用；不清除既有故障，也不写参数。
 */
void FocRunState_CheckFastFaults(void);
/**
 * @brief 刷新运行指示：无故障按模式点亮；有故障时按策略强制停机并显示故障码。
 * @note 仅由 20 kHz 快速中断调用；会修改 MotorControl.ModeNow 与 LED 状态。
 */
void FocRunState_HandleFaultIndication(void);
/**
 * @brief 管理功率级启停迁移：停机清理、使能前预载校验与延迟使能。
 * @return true 表示本周期延迟功率启动，调用方需推迟模式跟踪提交。
 * @note 仅由 20 kHz 快速中断调用；内部保留跨周期的启动准备状态。
 */
bool FocRunState_ManagePowerStage(void);
/**
 * @brief 提交模式与故障跟踪：检测变化、更新 ModeLast/ErrorLast 与前台镜像。
 * @param defer_position_power_start true 时保持 ModeLast 不更新，等待延迟使能完成。
 * @note 仅由 20 kHz 快速中断调用；Detect_Mode_Error_Change 在此触发变化上报。
 */
void FocRunState_CommitModeAndError(bool defer_position_power_start);

#endif
