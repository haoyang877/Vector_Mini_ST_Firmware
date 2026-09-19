#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include <stdbool.h>

/**
 * @brief 初始化电机控制对象、参数缓存和控制调度状态。
 * @note 由启动组合层调用；完成初始化不代表配置有效，也不会自动使能功率输出。
 */
void MotorControl_Init(void);
/**
 * @brief 查询启动所需配置是否已通过校验。
 * @return 轴身份、参数和必要校准均有效时返回 true，否则返回 false。
 * @note 只读查询，不清除故障、不写参数，也不使能电机。
 */
bool MotorControl_IsConfigurationValid(void);
/**
 * @brief 执行一次 20 kHz 电机快速控制周期。
 * @note 仅由快速定时器 ISR 调用；调用链必须保持有界、不可阻塞且不得动态分配。
 */
void FOC20kHzIRQHandler(void);
/**
 * @brief 执行一次 2 kHz 温度换算、保护监督和慢速维护任务。
 * @note 由 TIM7 监督中断上下文调用，不得替代快速环中的立即关断路径。
 */
void FOC2kHzSupervisor(void);

#endif
