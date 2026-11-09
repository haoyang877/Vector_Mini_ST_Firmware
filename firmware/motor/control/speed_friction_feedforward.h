#ifndef SPEED_FRICTION_FEEDFORWARD_H
#define SPEED_FRICTION_FEEDFORWARD_H

/** @brief 编译配置形成的只读摩擦模型；参数须有限并满足注释范围。 */
typedef struct
{
    float scale;               /**< 补偿比例，[0,1]。 */
    float coulomb_a;           /**< 库仑摩擦电流，非负 A。 */
    float viscous_a_per_rad_s; /**< 粘性系数，非负 A/(rad/s)。 */
    float zero_width_rad_s;    /**< 零速平滑宽度，正 rad/s。 */
    float time_constant_s;     /**< 平滑时间常数，正 s。 */
    float slew_a_per_s;        /**< 变化率上限，正 A/s。 */
    float limit_a;             /**< 幅值上限，非负 A。 */
} SpeedFrictionFeedforwardConfig;

/**
 * @brief 用规划后的速度参考计算有界摩擦前馈。
 * @param config 非空只读编译配置，须满足字段约束。
 * @param previous_a 外环持有的上一拍输出，A；停机或切模式清零。
 * @param target_rad_s 已规划的机械速度参考，rad/s。
 * @param sample_period_s 实际外环周期，(0,0.001] s。
 * @param current_limit_a 当前可用总电流，非负 A。
 * @return 有限输入时返回补偿电流；非法输入或溢出返回 NaN，由调用方处理故障。
 * @note 纯计算、无硬件访问；限流收窄立即生效，优先于变化率限制。
 */
float SpeedFrictionFeedforward_Update(const SpeedFrictionFeedforwardConfig *config,
                                      float previous_a,
                                      float target_rad_s,
                                      float sample_period_s,
                                      float current_limit_a);
#endif
