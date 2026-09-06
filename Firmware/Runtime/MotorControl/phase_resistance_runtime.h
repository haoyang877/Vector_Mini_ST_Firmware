#ifndef RUNTIME_PHASE_RESISTANCE_H
#define RUNTIME_PHASE_RESISTANCE_H

#include "current_control_runtime.h"
#include "phase_resistance.h"
#include "motor_profiles.h"

typedef struct
{
	uint32_t control_frequency_hz;
	float path_compensation_ohm;
	float undervoltage_trip_v;
	float overvoltage_trip_v;
} PhaseResistanceRuntimeBoardConfig;

/**
 * @file phase_resistance_runtime.h
 * @brief CurrentControl adapter for the hardware-independent phase-resistance core.
 *
 * 功能说明：本文件是相电阻算法和现有 CurrentControl 控制之间的唯一适配层。
 * - 从每电机参数 profile 读取双电流测试配置，并受 MotorControl 电流上限保护；
 * - 将核心模块给出的 Id/Iq/电角度命令交给 CurrentControlRuntime_RunClosedLoop() 执行；
 * - 将本周期电流与上一 PWM 周期实际输出的调制电压配对并回传核心；
 * - 在间隔、完成或失败时复位电流环并释放 PWM；
 * - 将六方向双电流拟合后的 A/B/C 相阻和不平衡率写回 MotorControl。
 * - 本模块不访问编码器或位置环；不同电机通过参数 profile 调整测试电流和时序。
 *
 * 调度层每个 CurrentControl 周期调用 PhaseResistanceRuntime_Run()；函数返回运行、完成
 * 或具体失败状态。motor_control_runtime.c 负责把这些状态映射为模式切换和现有故障码。
 * RTT 的数据输出保持在 motor_control_runtime.c，本模块不处理 RTT 或任何通信。
 *
 * The adapter is the only layer that accesses CurrentControlContext and
 * MotorControlContext. It translates the core command into CurrentControlRuntime_RunClosedLoop()
 * and PWM calls, then translates d/q electrical quantities back into
 * PhaseResistanceSample. Mode switching and error mapping stay in
 * motor_control_runtime.c, while RTT remains unchanged.
 */

typedef enum
{
	PHASE_RESISTANCE_MODE_RUNNING = 0,
	PHASE_RESISTANCE_MODE_DONE,
	PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT,
	PHASE_RESISTANCE_MODE_INVALID_RESULT,
	PHASE_RESISTANCE_MODE_UNDER_VOLTAGE,
	PHASE_RESISTANCE_MODE_OVER_VOLTAGE
} PhaseResistanceRuntimeStatus;

/**
 * Live values paired exactly as the resistance core receives them. The CurrentControl
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
} PhaseResistanceRuntimeTelemetry;

typedef struct
{
	PhaseResistanceContext core;
	PhaseResistanceRuntimeStatus status;
	float applied_mod_d;
	float applied_mod_q;
	PhaseResistanceRuntimeTelemetry telemetry;
	bool applied_voltage_valid;
	bool telemetry_valid;
	bool started;
} PhaseResistanceRuntimeContext;

/** Execute one 20 kHz resistance-identification cycle. */
PhaseResistanceRuntimeStatus PhaseResistanceRuntime_Run(
	PhaseResistanceRuntimeContext *context, CurrentControlContext *current_control,
	MotorControlContext *motor,
	const PhaseResistanceRuntimeBoardConfig *board_config,
	const MotorProfile *motor_profile);

/** Read the latest active current/voltage sample used by the resistance fit. */
bool PhaseResistanceRuntime_GetTelemetry(
	const PhaseResistanceRuntimeContext *context,
	PhaseResistanceRuntimeTelemetry *telemetry);

/** Stop the adapter output and discard any unfinished identification state. */
void PhaseResistanceRuntime_Cancel(PhaseResistanceRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor);

#endif
