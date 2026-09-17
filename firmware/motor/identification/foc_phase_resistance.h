#ifndef __FOC_PHASE_RESISTANCE_H__
#define __FOC_PHASE_RESISTANCE_H__

#include "foc_algorithm.h"
#include "phase_resistance.h"

/**
 * @file foc_phase_resistance.h
 * @brief FOC adapter for the hardware-independent phase-resistance core.
 *
 * 功能说明：本文件是相电阻算法和现有 FOC 控制之间的唯一适配层。
 * - 从每电机参数 profile 读取双电流测试配置，并受 MotorControl 电流上限保护；
 * - 将核心模块给出的 Id/Iq/电角度命令交给 FOC_Current() 执行；
 * - 将本周期电流与上一 PWM 周期实际输出的调制电压配对并回传核心；
 * - 在间隔、完成或失败时复位电流环并释放 PWM；
 * - 将六方向双电流拟合后的 A/B/C 相阻和不平衡率写回 MotorControl。
 * - 本模块不访问编码器或位置环；不同电机通过参数 profile 调整测试电流和时序。
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

/**
 * Live values paired exactly as the resistance core receives them. The FOC
 * adapter owns this snapshot; callers may only read it for diagnostics.
 */
typedef struct
{
	float electrical_angle;
	float id_ref;
	float id;
	float iq;
	float vd;
	float vq;
	float current_magnitude;
	float parallel_voltage;
	float vbus;
} PhaseResistanceModeTelemetry_TypeDef;

/** Execute one 20 kHz resistance-identification cycle. */
PhaseResistanceModeStatus_TypeDef PhaseResistanceMode_Run(FOC_TypeDef *foc,
	MotorControl_TypeDef *motor);

/** Read the latest active current/voltage sample used by the resistance fit. */
bool PhaseResistanceMode_GetTelemetry(PhaseResistanceModeTelemetry_TypeDef *telemetry);

/** Stop the adapter output and discard any unfinished identification state. */
void PhaseResistanceMode_Cancel(FOC_TypeDef *foc, MotorControl_TypeDef *motor);

#endif
