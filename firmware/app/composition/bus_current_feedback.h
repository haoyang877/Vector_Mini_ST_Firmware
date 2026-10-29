#ifndef BUS_CURRENT_FEEDBACK_H
#define BUS_CURRENT_FEEDBACK_H

#include <stdbool.h>

/* 独立硬件母线电流测量；仅有采样硬件的目标装配，不属于冻结BSP接口。 */
/**
 * @brief 初始化本次上电的零偏及滤波状态。
 * @note 启动前台、中断采样前调用一次，不写Flash。
 */
void BusCurrentFeedback_Init(void);
/**
 * @brief 消费辅助新帧，关相时累计128个连续稳定样本校零。
 * @note 2kHz监督上下文调用；重复帧不推进，运行中不跟踪零点。
 */
void BusCurrentFeedback_Sample(void);
/**
 * @brief 读取有符号滤波母线电流及有效性。
 * @param current_a 输出电流，单位A；未校零、失效或过期写NAN；空指针不写。
 * @return 新鲜且校零有效返回true，其余返回false。
 * @note 前台短锁取快照、锁外核对ADC；更新冲突最多重试一次，否则返回失效。
 *       不刷新样本年龄、不回退估算值。
 */
bool BusCurrentFeedback_Read(float *current_a);

#endif
