#ifndef __FOC_PID_H__
#define __FOC_PID_H__

typedef struct
{
    float Kp;
    float Ki;
    float Ts;
    float Umin;
    float Umax;
    float Ref;
    float Fbk;
    float Err;
    float Up;
    float Ui;
    float Out;
} PI_Controller_TypeDef;

/**
 * @brief 设置 PI 增益、采样周期与输出范围，不复位积分项。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param proportional_gain 比例增益，单位为输出量/输入量。
 * @param integral_gain 积分增益，单位为输出量/(输入量·s)。
 * @param sample_period 正采样周期，单位 s。
 * @param output_min 输出下限，与输出量同单位。
 * @param output_max 输出上限，不小于下限。
 * @note 调用方负责有限值及上下限检查；可用于确定性控制周期，无 I/O。
 */
void PI_Controller_Configure(PI_Controller_TypeDef *controller,
                             float proportional_gain,
                             float integral_gain,
                             float sample_period,
                             float output_min,
                             float output_max);
/**
 * @brief 执行条件积分抗饱和 PI 并保存本周期状态。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param reference 参考值，与 feedback 同单位。
 * @param feedback 反馈值，与 reference 同单位。
 * @return 上下限内的控制输出；不额外检查 NaN。
 * @note 需先配置有限参数；无 I/O，不分配内存。
 */
float PI_Controller_Run(PI_Controller_TypeDef *controller, float reference, float feedback);
/**
 * @brief 以实际应用输出修正积分项，并记录实际输出。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @param applied_output 与控制输出同单位的实际应用量。
 * @note 调用方负责有限值；积分项限幅，Out 保留传入值。
 */
void PI_Controller_TrackOutput(PI_Controller_TypeDef *controller, float applied_output);
/**
 * @brief 清零 PI 参考、反馈、积分和输出，保留配置。
 * @param controller 调用方拥有的非空控制器状态；同一实例须串行访问。
 * @note 无 I/O；调用方负责串行访问。
 */
void PI_Controller_Reset(PI_Controller_TypeDef *controller);

#endif
