#ifndef POWER_STAGE_HW_H
#define POWER_STAGE_HW_H

/* 功率级输出契约：停止与恢复六路互补 PWM 输出，不暴露定时器句柄或寄存器。
 * 调用方包含故障处理与 20 kHz 快速环，因此实现必须非阻塞且可在中断上下文调用。
 * 停止/恢复只切换输出使能，比较寄存器与占空比保持不变。 */

/**
 * @brief 停止全部六路 PWM 输出，保留已写入的占空比。
 * @note 可在中断上下文调用；实现按固定顺序先停主输出再停互补输出，避免出现
 *       单相仍被驱动的中间状态。
 */
void power_stage_hw_stop(void);

/**
 * @brief 恢复全部六路 PWM 输出，占空比沿用停止前写入的比较寄存器。
 * @note 可在中断上下文调用；顺序与停止对称。
 */
void power_stage_hw_start(void);

#endif
