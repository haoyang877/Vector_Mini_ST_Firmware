#ifndef __PHASE_RESISTANCE_H__
#define __PHASE_RESISTANCE_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * @file phase_resistance.h
 * @brief Hardware-independent three-phase resistance identification core.
 *
 * 功能说明：
 * 1. 电机静止时，依次输出 0、120、240 电角度的三个 d 轴电流矢量；
 * 2. 每个矢量依次经历电流斜坡、连续稳定确认、平均采样和关断间隔；
 * 3. 每个矢量使用采样窗口内的 d/q 电压、电流计算等效电阻；
 * 4. 由三个等效电阻反算 A/B/C 三相电阻，并计算三相不平衡率；
 * 5. 本模块只处理算法状态，不访问 FOC_TypeDef、MotorControl_TypeDef、
 *    PWM、RTT 或通信资源，因此可独立阅读和复用。
 *
 * 调用顺序：Start() 开始一次测试；每个 FOC 周期先调用 GetCommand() 取得
 * 电流命令，控制层执行该命令后调用 InputSample() 提交实际采样；完成后通过
 * GetResult() 读取结果。如需中断，调用 Cancel()。
 *
 * The core injects three stationary d-axis current vectors at 0, 120 and
 * 240 electrical degrees. For each vector, the caller performs the configured
 * current ramp, settling and averaging windows through the commands produced
 * by this module. The vector resistance is calculated as:
 *
 *     R_vector = sum(Vd * Id + Vq * Iq) / sum(Id * Id + Iq * Iq)
 *
 * The three vector resistances are then combined to obtain the A/B/C phase
 * resistances. This header deliberately has no FOC, PWM, MotorControl, RTT
 * or communication dependency. The integration layer supplies a compact
 * sample structure and applies the compact current command structure.
 */

#define PHASE_RESISTANCE_VECTOR_COUNT 3U

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
	float test_current;
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
	float balance_limit_pct;
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
 * Final public result. vector_resistance[] stores the raw three-vector data.
 * phase_resistance_a/b/c are the final phase resistances in ohms; spread_pct
 * is the max-to-min phase difference relative to their mean.
 */
typedef struct
{
	float vector_resistance[PHASE_RESISTANCE_VECTOR_COUNT];
	float phase_resistance_a;
	float phase_resistance_b;
	float phase_resistance_c;
	float spread_pct;
	bool valid;
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
	uint8_t vector_index;
	uint32_t ramp_count;
	uint32_t stable_count;
	uint32_t sample_count;
	uint32_t timeout_count;
	uint32_t pause_count;
	float filtered_voltage_d;
	float filtered_voltage_q;
	float previous_voltage_d;
	float previous_voltage_q;
	float power_sum;
	float current_square_sum;
} PhaseResistanceContext_TypeDef;

/** Reset all internal state without touching any motor-control or PWM object. */
void PhaseResistance_Init(PhaseResistanceContext_TypeDef *context);

/** Validate and start a new three-vector resistance measurement. */
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

/** Cancel a measurement and clear its core state. */
void PhaseResistance_Cancel(PhaseResistanceContext_TypeDef *context);

#endif
