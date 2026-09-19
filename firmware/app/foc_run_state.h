#ifndef FOC_RUN_STATE_H
#define FOC_RUN_STATE_H

#include <stdbool.h>

#include "motor_work.h"

/**
 * @brief 初始化运行状态机适配器：核心进入 BOOT 态，功率级记录为关闭。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void FocRunState_Init(void);
/**
 * @brief 上报前台 SAVE 会话结果：下一快周期完成或失败生命周期保存操作。
 * @param committed true 表示 Flash 写入成功（COMMITTED）；false 表示失败。
 * @note 供前台主循环在 flash 写完成后调用；结果只记录一次并由快速环消费。
 *       失败时核心按操作失败处理；兼容投影的 ErrorNow 仍由调用方按旧路径设置。
 */
void FocRunState_SaveFinished(bool committed);
/**
 * @brief 快速环故障检查：编码器离线或轴配置失效时置故障并停机。
 * @note 仅由 20 kHz 快速中断调用；不清除既有故障，也不写参数。
 */
void FocRunState_CheckFastFaults(void);
/**
 * @brief 执行一次运行状态机：应用 worker 结果，完成故障指示、功率级启停迁移与提交。
 * @param outcome 本周期模式 worker 的结果；未请求转换时使用 MOTOR_WORK_RUNNING。
 * @note 仅由 20 kHz 快速中断调用；顺序为 worker 结果应用 → 故障反应 →
 *       维护会话服务（SAVE 结果上报/派生完成/取消确认）→ 停机/使能迁移 →
 *       变化检测与影子提交（ModeLast/ErrorLast 与前台镜像）。
 */
void FocRunState_Tick(MotorWorkOutcome_TypeDef outcome);

#endif
