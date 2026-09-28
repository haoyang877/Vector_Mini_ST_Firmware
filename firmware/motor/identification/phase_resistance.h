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
    float direction_current[PHASE_RESISTANCE_DIRECTION_COUNT][PHASE_RESISTANCE_CURRENT_LEVEL_COUNT];
    float direction_voltage[PHASE_RESISTANCE_DIRECTION_COUNT][PHASE_RESISTANCE_CURRENT_LEVEL_COUNT];
} PhaseResistanceContext_TypeDef;

/**
 * @brief 清零相电阻辨识状态，不操作控制器或 PWM。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 */
void PhaseResistance_Init(PhaseResistanceContext_TypeDef *context);

/**
 * @brief 验证配置并开始六方向、双电流测量。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 * @param config 只读配置，电流 A、阻值 Ω、时间为调用周期数；启动时复制。
 * @return 配置通过返回 true；不通过返回 false。
 * @note 不接触硬件；实际限流及停机由适配器负责。
 */
bool PhaseResistance_Start(PhaseResistanceContext_TypeDef *context,
                           const PhaseResistanceConfig_TypeDef *config);

/**
 * @brief 读取本周期电流矢量或暂停命令。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 * @param command 非空调用方输出，电流 A、电角度 rad。
 * @note inject_current 为 false 时由适配器释放 PWM，核心不操作输出。
 */
void PhaseResistance_GetCommand(const PhaseResistanceContext_TypeDef *context,
                                PhaseResistanceCommand_TypeDef *command);

/**
 * @brief 提交与上一实际 PWM 输出配对的采样并推进测量。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 * @param sample 电流 A、电压 V；仅非注入间隔允许 NULL。
 * @return 运行、完成、稳定超时或无效结果状态。
 * @note 每个控制周期提交一次，不执行 I/O。
 */
PhaseResistanceCoreStatus_TypeDef
PhaseResistance_InputSample(PhaseResistanceContext_TypeDef *context,
                            const PhaseResistanceSample_TypeDef *sample);

/**
 * @brief 复制已完成且有效的辨识结果。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 * @param result 非空调用方结果空间，阻值 Ω、不平衡率 %。
 * @return 测量完成且结果有效返回 true，否则返回 false。
 */
bool PhaseResistance_GetResult(const PhaseResistanceContext_TypeDef *context,
                               PhaseResistanceResult_TypeDef *result);

/**
 * @brief 复位相电阻测量为初始状态。
 * @param context 调用方拥有的非空辨识状态；同一实例须串行访问。
 * @note 不关断 PWM，实际输出关闭由适配器负责。
 */
void PhaseResistance_Cancel(PhaseResistanceContext_TypeDef *context);

#endif
