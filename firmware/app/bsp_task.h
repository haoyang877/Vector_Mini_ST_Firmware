#ifndef __BSP_TASK_H__
#define __BSP_TASK_H__

/**
 * @brief 1 kHz 板级任务：推进指示器、CAN 维护与 1 kHz 监督。
 * @note 仅由 TIM7 中断上下文调用；不得阻塞、不得动态分配。
 */
void BSP1kHzIRQHandler(void);

#endif
