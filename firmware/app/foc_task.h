#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include "data_type.h"

/**
 * @brief 执行一次 20 kHz 电机快速控制周期。
 * @note 仅由快速定时器 ISR 调用；调用链必须保持有界、不可阻塞且不得动态分配。
 */
void FOC20kHzIRQHandler(void);
/**
 * @return 本次监督提交后的模式，供指示器使用。
 * @brief 执行一次 2 kHz 温度换算、保护监督和慢速维护任务。
 * @note 由 TIM7 监督中断上下文调用，不得替代快速环中的立即关断路径。
 */
ModeNow_TypeDef FOC2kHzSupervisor(void);

#endif
