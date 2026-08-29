#ifndef __FOC_PHASE_RESISTANCE_H__
#define __FOC_PHASE_RESISTANCE_H__

#include "foc_algorithm.h"
#include "phase_resistance.h"

/**
 * @file foc_phase_resistance.h
 * @brief FOC adapter for the hardware-independent phase-resistance core.
 *
 * 功能说明：本文件是相电阻算法和现有 FOC 控制之间的唯一适配层。
 * - 从 MotorControl 中读取测试电流上限，生成核心模块所需的测试配置；
 * - 将核心模块给出的 Id/Iq/电角度命令交给 FOC_Current() 执行；
 * - 将本周期的 Id、Iq、滤波电流、Vd、Vq 转换为小型采样结构并回传核心；
 * - 在间隔、完成或失败时复位电流环并释放 PWM；
 * - 将最终的三个原始矢量电阻、A/B/C 相阻和不平衡率写回 MotorControl。
 *
 * 调度层每个 FOC 周期调用 PhaseResistanceMode_Run()；函数返回运行、完成
 * 或具体失败状态。foc_task.c 负责把这些状态映射为模式切换和现有故障码。
 * RTT 的数据输出保持在 foc_task.c，本模块不处理 RTT 或任何通信。
 *
 * The adapter is the only layer that accesses FOC_TypeDef and
 * MotorControl_TypeDef. It translates the core command into FOC_Current()
 * and PWM calls, then translates d/q electrical quantities back into
 * PhaseResistanceSample_TypeDef. Mode switching and error mapping stay in
 * foc_task.c, while RTT remains unchanged.
 */

typedef enum
{
	PHASE_RESISTANCE_MODE_RUNNING = 0,
	PHASE_RESISTANCE_MODE_DONE,
	PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT,
	PHASE_RESISTANCE_MODE_INVALID_RESULT,
	PHASE_RESISTANCE_MODE_UNDER_VOLTAGE,
	PHASE_RESISTANCE_MODE_OVER_VOLTAGE
} PhaseResistanceModeStatus_TypeDef;

/** Execute one 20 kHz resistance-identification cycle. */
PhaseResistanceModeStatus_TypeDef PhaseResistanceMode_Run(FOC_TypeDef *foc,
	MotorControl_TypeDef *motor);

/** Stop the adapter output and discard any unfinished identification state. */
void PhaseResistanceMode_Cancel(FOC_TypeDef *foc, MotorControl_TypeDef *motor);

#endif
