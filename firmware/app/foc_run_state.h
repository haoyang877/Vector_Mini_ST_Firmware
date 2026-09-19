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
 * @brief 上报前台 SAVE 会话结果：下一 2 kHz 慢拍完成或失败生命周期保存操作。
 * @param committed true 表示 Flash 写入成功（COMMITTED）；false 表示失败。
 * @note 供前台主循环在 flash 写完成后调用；结果只记录一次并由状态机消费。
 *       失败时核心按操作失败处理；兼容投影的 ErrorNow 仍由调用方按旧路径设置。
 */
void FocRunState_SaveFinished(bool committed);
/**
 * @brief 快速环故障检查：编码器离线或轴配置失效时置故障并停机。
 * @note 仅由 20 kHz 快速中断调用；不清除既有故障，也不写参数。
 */
void FocRunState_CheckFastFaults(void);
/**
 * @brief 故障紧急关断快车道：故障锁存且功率级开启时立即关断硬件。
 * @note 仅由 20 kHz 快速中断调用；本板无 TIM1 BKIN 与驱动器故障引脚，软件是
 *       唯一关断路径，本入口把"故障 → 功率级关断"保持在快速环时延内。
 */
void FocRunState_FastFaultStop(void);
/**
 * @brief 上报本周期 worker 结果：20 kHz 快环写入，2 kHz 状态机消费。
 * @param outcome 本周期模式 worker 的结果；MOTOR_WORK_RUNNING 不入箱。
 * @note 仅由 20 kHz 快速中断调用；与消费端通过短临界区交换，只保留最新的
 *       非 RUNNING 结果。
 */
void FocRunState_PostOutcome(MotorWorkOutcome_TypeDef outcome);
/**
 * @brief 执行一次运行状态机：消费 worker 结果，完成故障指示、功率级启停迁移与提交。
 * @note 仅由 2 kHz 监督中断调用；顺序为 worker 结果应用 → 故障反应 →
 *       维护会话服务（SAVE 结果上报/派生完成/取消确认）→ 停机/使能迁移 →
 *       变化检测与影子提交（ModeLast/ErrorLast 与前台镜像）。
 */
void FocRunState_Tick(void);

#endif
