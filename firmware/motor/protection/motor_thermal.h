/** @file
 * @brief 绕组及逐相一阶热模型；不访问硬件，调用方独占状态并提供实际电流。
 */
#ifndef MOTOR_THERMAL_H
#define MOTOR_THERMAL_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
    float continuous_vector_a;
    float continuous_phase_rms_a;
    float peak_a;
    float heating_tau_s;
    float cooling_tau_s;
    float update_period_s;
    uint32_t samples_per_update;
    float derating_enter_heat;
    float derating_exit_heat;
    float trip_heat;
    float clear_heat;
} MotorThermalConfig;

/** @brief 通信及MTH1记录的固定参数顺序；门限在模型中使用比例。 */
typedef enum
{
    MOTOR_THERMAL_CONTINUOUS_VECTOR = 0,
    MOTOR_THERMAL_PEAK,
    MOTOR_THERMAL_HEATING_TAU,
    MOTOR_THERMAL_DERATING_ENTER,
    MOTOR_THERMAL_DERATING_EXIT,
    MOTOR_THERMAL_TRIP,
    MOTOR_THERMAL_CLEAR,
    MOTOR_THERMAL_PARAMETER_COUNT
} MotorThermalParameter;

typedef struct
{
    MotorThermalConfig config;
    float heat[4]; /* 整体、A/B/C相；1为参考热预算，内部不钳至停机门限。 */
    float square_sum_a2[3];
    float heating_alpha;
    float cooling_alpha;
    float max_heat;
    float available_a;
    uint32_t sample_count;
    bool valid;
    bool derating;
    bool fault_latched;
} MotorThermalState;

/**
 * @brief 用三次展开预计算一阶热系数，避免引入指数库。
 * @param update_period_s 固定模型步长，s。
 * @param tau_s 有效热时间常数，s。
 * @return 近似1-exp(-dt/tau)；dt/tau≤0.01时截断相对误差小于4.3e-8。
 * @note 实际单精度结果另有浮点舍入误差，离线扫查覆盖整个配置范围。
 * @note 只在初始化或停机参数准备调用；不是每拍积分。
 */
static inline float MotorThermal_Alpha(float update_period_s, float tau_s)
{
    float x = update_period_s / tau_s;
    return x * (1.0f - x * (0.5f - x * (1.0f / 6.0f)));
}

/**
 * @brief 校验完整热配置和四个动作门限的顺序。
 * @param config 待校验配置；电流为A、时间为s、热门限为比例。
 * @return 所有参数有限、范围及相互关系有效时为true。
 */
bool MotorThermal_ConfigValid(const MotorThermalConfig *config);

/**
 * @brief 应用预计算配置，保留热历史、采样窗口及故障锁存。
 * @param state 调用方独占的有效模型。
 * @param config 完整有效配置，采样时序必须保持不变。
 * @param heating_alpha 已在非快上下文计算的升温离散系数。
 * @param cooling_alpha 已在非快上下文计算的冷却离散系数。
 * @return 应用成功时为true；拒绝时状态不变。
 * @note 仅输出关闭时调用；持续电流降低时放大H，增大时不减少已有H。
 */
bool MotorThermal_Reconfigure(MotorThermalState *state,
                              const MotorThermalConfig *config,
                              float heating_alpha,
                              float cooling_alpha);

/**
 * @brief 初始化热模型，热态启动由调用方明确给出，不能用于清故障。
 * @param state 调用方独占的状态。
 * @param config 已选定的模型参数；向量为幅值、相电流为RMS，单位A。
 * @param initial_heat 四个状态的保守初值；非负，1为参考热预算。
 * @return 配置有效时为true；无效时状态保持禁止出力。
 * @note 仅初始化时计算指数；STOP、模式切换和清故障不得调用本函数。
 */
bool MotorThermal_Init(MotorThermalState *state,
                       const MotorThermalConfig *config,
                       float initial_heat);

/**
 * @brief 累计一次实际三相电流平方，不先平均带符号电流。
 * @param state 调用方独占的模型状态。
 * @param ia_a A相实际电流，A。
 * @param ib_a B相实际电流，A。
 * @param ic_a C相实际电流，A。
 * @return 有效且未超过窗口样本数时为true；异常时锁定禁止出力。
 */
bool MotorThermal_Accumulate(MotorThermalState *state, float ia_a, float ib_a, float ic_a);

/**
 * @brief 消费完整窗口，更新四个热状态、降额和热故障锁存。
 * @param state 调用方独占的模型状态。
 * @return 窗口和模型有效时为true；缺样或计算异常时为false。
 * @note 热故障后仍需调用以估计冷却；本函数不会自动清故障。
 */
bool MotorThermal_Update(MotorThermalState *state);

/**
 * @brief 锁定无效反馈或失去调度保证的模型，保留已累计热量。
 * @param state 调用方独占的模型状态。
 */
void MotorThermal_Invalidate(MotorThermalState *state);

/**
 * @brief 仅在有效模型且所有热状态不高于配置恢复门限时解除热故障锁存。
 * @param state 调用方独占的模型状态。
 * @return 允许清除时为true；热量和累计窗口不会归零。
 */
bool MotorThermal_ClearFault(MotorThermalState *state);

/**
 * @brief 解析配置峰值与热边界，保留配置电流及控制增益的原口径。
 * @param configured_a 保存的电流上限，A。
 * @param available_a 热保护允许电流，A。
 * @param thermal_active 是否使用热边界；兼容尚未接入的调用者。
 * @return 实际可用上限，A；无效输入返回0。
 */
static inline float
MotorThermal_ResolveLimit(float configured_a, float available_a, bool thermal_active)
{
    if (!isfinite(configured_a) || configured_a <= 0.0f)
    {
        return 0.0f;
    }
    if (!thermal_active)
    {
        return configured_a;
    }
    if (!isfinite(available_a) || available_a < 0.0f)
    {
        return 0.0f;
    }
    return fminf(configured_a, available_a);
}

/**
 * @brief 等比例限制d/q总向量，不能分别夹紧后超过总电流上限。
 * @param limit_a 实际允许向量幅值，A。
 * @param id_a 输入输出d轴电流参考，A。
 * @param iq_a 输入输出q轴电流参考，A。
 * @return 输入有效时为true；异常时可用输出清零并返回false。
 */
bool MotorThermal_LimitVector(float limit_a, float *id_a, float *iq_a);

#endif
