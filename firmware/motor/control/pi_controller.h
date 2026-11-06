#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

/**
 * 调用方持有的 PI 对象；无全局状态、硬件依赖或动态分配。
 * 首次 Run 前须零初始化或 Reset，并完成配置；同一实例的配置与计算必须串行。
 * reference/feedback/error 使用同一输入单位；output 相关字段使用同一输出单位。
 * 电流环可为 A → V，速度环可为 rad/s → A 或归一化量，因此不固定物理量后缀。
 * 调用方负责非空、有限值、有效范围和中间计算不溢出；内核不重复检查。
 */
typedef struct
{
    /* 配置：Configure 只更新此组，Reset 保留此组；采样周期单位固定为 s。 */
    float proportional_gain;
    float integral_gain;
    float sample_period_s;
    float output_min;
    float output_max;

    /* 运行状态：Run 更新本周期计算，TrackOutput 修正实际输出与积分。 */
    float reference;
    float feedback;
    float error;
    float proportional_output;
    float integral_output;
    float output;
} PiController;

/**
 * @brief 设置 PI 增益、采样周期与输出范围，不复位积分项。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param proportional_gain 比例增益，单位为输出量/输入量。
 * @param integral_gain 积分增益，单位为输出量/(输入量·s)。
 * @param sample_period_s 正采样周期，单位 s。
 * @param output_min 输出下限，与输出量同单位。
 * @param output_max 输出上限，不小于下限。
 * @note 不初始化运行字段，不对已有积分或输出重新限幅；调用方负责有效范围。
 */
void PiController_Configure(PiController *controller,
                            float proportional_gain,
                            float integral_gain,
                            float sample_period_s,
                            float output_min,
                            float output_max);

/**
 * @brief 执行条件积分抗饱和 PI 并保存本周期状态。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param reference 参考值，与 feedback 同单位。
 * @param feedback 反馈值，与 reference 同单位。
 * @return 上下限内的控制输出；不额外检查 NaN。
 * @note 首次调用前须零初始化或 Reset 并配置有效参数；可在控制周期调用，无 I/O。
 */
float PiController_Run(PiController *controller, float reference, float feedback);

/**
 * @brief 以实际应用输出修正积分项，并记录实际输出。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param applied_output 与控制输出同单位的实际应用量。
 * @note 在配置且运行状态已建立后调用；积分项限幅，output 原样保留传入值，允许越界。
 */
void PiController_TrackOutput(PiController *controller, float applied_output);

/**
 * @brief 清零 PI 参考、反馈、积分和输出，保留配置。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @note 不改变增益、周期和上下限，不操作硬件；首次运行还需完成配置。
 */
void PiController_Reset(PiController *controller);

#endif
