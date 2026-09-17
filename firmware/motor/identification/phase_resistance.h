#ifndef __PHASE_RESISTANCE_H__
#define __PHASE_RESISTANCE_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * @file phase_resistance.h
 * @brief Hardware-independent three-phase resistance identification core.
 *
 * 功能说明：
 * 1. 依次输出 0、60、120、180、240、300 电角度的六个静止电流矢量；
 * 2. 每个方向使用低、高两档电流，依次经历斜坡、稳定确认、平均采样和间隔；
 * 3. 使用 U = R * I + Uoffset 的双电流拟合计算每个方向的等效电阻；
 * 4. 对相反方向取平均后反算 A/B/C 三相电阻，并给出不平衡率；
 * 5. 本模块不访问 FOC、MotorControl、PWM、编码器、RTT 或通信资源。
 *
 * 调用顺序：Start() 开始一次测试；每个 FOC 周期先调用 GetCommand() 取得
 * 电流命令，控制层执行该命令后调用 InputSample() 提交实际采样；完成后通过
 * GetResult() 读取结果。如需中断，调用 Cancel()。
 *
 * 相反方向配对为：0/180 度、120/300 度、240/60 度。vector_resistance[]
 * 依次保存这三组方向平均后的等效阻值，因此仍可用于原有的三相反算和结果输出。
 */

#define PHASE_RESISTANCE_VECTOR_COUNT 3U
#define PHASE_RESISTANCE_DIRECTION_COUNT 6U
#define PHASE_RESISTANCE_CURRENT_LEVEL_COUNT 2U

typedef enum
{
	PHASE_RESISTANCE_CORE_RUNNING = 0,
	PHASE_RESISTANCE_CORE_DONE,
	PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT,
	PHASE_RESISTANCE_CORE_INVALID_RESULT
} PhaseResistanceCoreStatus_TypeDef;

/** Configuration supplied once before a measurement starts. */
typedef struct
{
	float test_current_low;
	float test_current_high;
	float maximum_test_current;
	float minimum_test_current;
	uint32_t ramp_ticks;
	uint32_t settle_ticks;
	uint32_t sample_ticks;
	uint32_t pause_ticks;
	uint32_t timeout_ticks;
	float current_tolerance;
	float q_current_tolerance;
	float voltage_tolerance;
	float voltage_min_delta;
	float voltage_filter;
	float path_compensation_ohm;
	float balance_warning_pct;
	float balance_fault_pct;
} PhaseResistanceConfig_TypeDef;

/** One completed FOC control-cycle measurement supplied by the adapter. */
typedef struct
{
	float id;
	float iq;
	float id_filt;
	float iq_filt;
	float vd;
	float vq;
} PhaseResistanceSample_TypeDef;

/** Current-vector command returned to the FOC adapter for the next cycle. */
typedef struct
{
	bool inject_current;
	bool reset_current_controller;
	float id_ref;
	float iq_ref;
	float electrical_angle;
} PhaseResistanceCommand_TypeDef;

/**
 * Final public result. direction_resistance[] and direction_voltage_offset[]
 * are diagnostic values for the six commanded directions. The three entries
 * in vector_resistance[] are their paired averages used to solve Ra/Rb/Rc.
 */
typedef struct
{
	float vector_resistance[PHASE_RESISTANCE_VECTOR_COUNT];
	float direction_resistance[PHASE_RESISTANCE_DIRECTION_COUNT];
	float direction_voltage_offset[PHASE_RESISTANCE_DIRECTION_COUNT];
	float phase_resistance_a;
	float phase_resistance_b;
	float phase_resistance_c;
	float spread_pct;
	bool valid;
	bool warning;
	bool balanced;
} PhaseResistanceResult_TypeDef;

/**
 * Per-measurement state storage. The FOC adapter owns one instance and must
 * only modify it through the functions below.
 */
typedef struct
{
	PhaseResistanceConfig_TypeDef config;
	PhaseResistanceResult_TypeDef result;
	uint8_t step;
	uint8_t direction_index;
	uint8_t current_point_index;
	uint32_t ramp_count;
	uint32_t stable_count;
	uint32_t sample_count;
	uint32_t timeout_count;
	uint32_t pause_count;
	float filtered_voltage_d;
	float filtered_voltage_q;
	float previous_voltage_d;
	float previous_voltage_q;
	float sample_current_sum;
	float sample_voltage_sum;
	float direction_current[PHASE_RESISTANCE_DIRECTION_COUNT]
		[PHASE_RESISTANCE_CURRENT_LEVEL_COUNT];
	float direction_voltage[PHASE_RESISTANCE_DIRECTION_COUNT]
		[PHASE_RESISTANCE_CURRENT_LEVEL_COUNT];
} PhaseResistanceContext_TypeDef;

/** Reset all internal state without touching any motor-control or PWM object. */
void PhaseResistance_Init(PhaseResistanceContext_TypeDef *context);

/** Validate and start a new six-direction, two-current resistance measurement. */
bool PhaseResistance_Start(PhaseResistanceContext_TypeDef *context,
	const PhaseResistanceConfig_TypeDef *config);

/**
 * Get the d/q current and electrical-angle command for the current FOC cycle.
 * A false inject_current command requests that the adapter release PWM output.
 */
void PhaseResistance_GetCommand(const PhaseResistanceContext_TypeDef *context,
	PhaseResistanceCommand_TypeDef *command);

/**
 * Submit the measurement captured after the command has been applied. Pass
 * NULL only while the command requests no current injection during the pause.
 */
PhaseResistanceCoreStatus_TypeDef PhaseResistance_InputSample(
	PhaseResistanceContext_TypeDef *context,
	const PhaseResistanceSample_TypeDef *sample);

/** Copy the finished result. Returns false until a valid measurement completes. */
bool PhaseResistance_GetResult(const PhaseResistanceContext_TypeDef *context,
	PhaseResistanceResult_TypeDef *result);

/** Cancel the current measurement and reset the algorithm to its idle state. */
void PhaseResistance_Cancel(PhaseResistanceContext_TypeDef *context);

#endif
