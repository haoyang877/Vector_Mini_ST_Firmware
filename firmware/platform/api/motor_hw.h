#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdbool.h>

/** @brief 检查六路功率输出已按本板停机方式关断。
 * @return PWM 通道关闭且六路引脚均为高阻时为 true。
 * @note 只读硬件状态，不等同于独立测量物理门极电压。
 */
bool motor_hw_phase_outputs_disabled(void);

/** @brief 将六路功率引脚切换为高阻模拟模式。
 * @note 仅修改本板功率引脚的模式位，不修改定时器、上下拉或复用选择。
 *       调用方须先关闭三相 PWM 通道；启动初始化时也可在输出未使能前调用。
 *       由启动或电机快中断独占 GPIO 配置，不阻塞。
 */
void motor_hw_phase_outputs_high_impedance(void);

/** @brief 在采样前初始化外环与遥测共用的低优先级执行上下文。
 * @note 电机快中断必须可抢占该上下文；平台层不执行控制算法。
 */
void motor_hw_outer_init(void);
/** @brief 请求一次低优先级外环与遥测服务，重复请求可以合并。
 * @note 各生产者独占自己的邮箱，已有待处理数据时不得覆盖。
 */
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

/**
 * @brief 读取 MCU 内部温度，使用出厂校准值及内部参考电压补偿。
 * @return 最新温度，单位 ℃；无完整采样或校准数据无效时返回 NaN。
 * @note 由 1 kHz 监督任务独占调用；不阻塞，不代表绕组或功率器件温度。
 */
float motor_hw_mcu_temperature_c(void);

#endif
