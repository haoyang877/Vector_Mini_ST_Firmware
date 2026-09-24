#ifndef MOTOR_HW_H
#define MOTOR_HW_H

/** Initialize the deferred outer-control execution context before sampling starts.
 * The fast motor interrupt must preempt this context. No motor algorithm lives here. */
void motor_hw_outer_init(void);
/** Request one deferred outer-control service. Caller owns the single-job mailbox;
 * it must not schedule a second job before consuming the first completion. */
void motor_hw_outer_schedule(void);
/** Publish/acquire shared motor data across the fast and deferred contexts.
 * This is a compiler and hardware memory barrier, never an interrupt lock. */
void motor_hw_outer_barrier(void);

/**
 * @brief 获取本次完整电流采样对应的 PWM 扇区。
 * @return 当前 SVM 编号 1～6；输出未使能、零矢量或比较值无效时返回 0。
 * @note 仅在电流快中断内、完整采样完成后且本拍 PWM 写入前调用。
 *       依赖上一拍 PWM 已生效；返回扇区不保证其余两相的采样窗口有效。
 */
unsigned motor_hw_current_sample_sector(void);

#endif
