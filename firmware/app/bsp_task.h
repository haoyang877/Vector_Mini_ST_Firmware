#ifndef __BSP_TASK_H__
#define __BSP_TASK_H__

/**
 * @brief 2 kHz 板级监督任务：推进指示器、CAN 维护与监督组合。
 * @note 仅由 TIM7 中断上下文调用；不得阻塞、不得动态分配。
 */
void BSP2kHzIRQHandler(void);

#endif
