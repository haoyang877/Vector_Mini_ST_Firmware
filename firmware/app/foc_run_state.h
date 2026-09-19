#ifndef FOC_RUN_STATE_H
#define FOC_RUN_STATE_H

#include <stdbool.h>

#include "motor_work.h"

/* 运行状态机状态：迁移条件与动作集中在 FocRunState_Tick 内，外部只读。 */
typedef enum
{
    RUN_DISABLED = 0, /* 功率级关闭，可接受模式请求 */
    RUN_PREPARING,    /* 位置/速度模式已准入：等控制器就绪后使能 */
    RUN_ENABLED,      /* 功率级开启，模式任务执行中 */
    RUN_FAULT         /* 故障锁存：功率级关闭，等待清除 */
} RunState_TypeDef;

/**
 * @brief 初始化运行状态机适配器：核心进入 BOOT 态，功率级记录为关闭。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void FocRunState_Init(void);
/**
 * @brief 快速环故障检查：编码器离线或轴配置失效时置故障并停机。
 * @note 仅由 20 kHz 快速中断调用；不清除既有故障，也不写参数。
 */
void FocRunState_CheckFastFaults(void);
/**
 * @brief 执行一次运行状态机：应用 worker 结果，完成故障指示、功率级启停迁移与提交。
 * @param outcome 本周期模式 worker 的结果；未请求转换时使用 MOTOR_WORK_RUNNING。
 * @note 仅由 20 kHz 快速中断调用；顺序为 worker 结果应用 → 故障反应 →
 *       停机/使能迁移 → 变化检测与影子提交（ModeLast/ErrorLast 与前台镜像）。
 */
void FocRunState_Tick(MotorWorkOutcome_TypeDef outcome);

#endif
