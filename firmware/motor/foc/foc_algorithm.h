#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

/* foc 算法核心：坐标变换、SVPWM 与电流/电压环；PWM 输出统一经 motor_hw 契约，
 * 不直接访问定时器寄存器。 */

#include "motor_config.h"
#include "motor_types.h"
#include "foc_pid.h"

/**
 * @brief  foc 运行时状态：母线/电流、坐标变换量、调制比与电流环控制器。
 * @note   单位：电压 V、电流 A、调制比/占空比为 0..1 标幺、角度为电角度 rad、
 *         temperature_c 为功率板温度 °C；由快速环写入，其他上下文只读。
 */
typedef struct
{
    float bus_voltage_v, bus_voltage_filtered_v;
    float bus_current_a, bus_current_filtered_a;
    float power_filtered_w;
    float voltage_alpha_v, voltage_beta_v;
    float vd_v, vq_v;
    float phase_a_current_a, phase_b_current_a, phase_c_current_a;
    float current_alpha_a, current_beta_a;
    float current_alpha_filtered_a, current_beta_filtered_a;
    float id_a, iq_a, id_filtered_a, iq_filtered_a;
    PiController id_pi;
    PiController iq_pi;
    /*dq voltage of p.u.*/
    float mod_d, mod_q;
    /*alpha-beta voltage of p.u.*/
    float mod_alpha, mod_beta;
    /*duty of voltage utilization */
    float duty;
    /*duty cycle of abc*/
    float duty_a, duty_b, duty_c;
    int32_t sector;
    /*temprature of power board*/
    float temperature_c;
} FocState;

/**
 * @brief 计算当前母线允许的 d/q 电压幅值上限，复用控制器调制深度限制。
 * @param bus_voltage_v 母线电压，单位 V。
 * @return 有限正母线电压对应的最大幅值 V；无效或非正输入返回零。
 * @note 纯计算、非阻塞；这是算法限幅，不替代产品功率与温度保护。
 */
float FOC_MaximumVoltage(float bus_voltage_v);

/**
 * @brief 将活动d/q与速度PI增益、采样周期绑定到显式控制器。
 * @param configuration 调用期间稳定的只读电机及应用配置。
 * @param foc d/q控制器所有者。
 * @param speed_pi 速度控制器所有者。
 * @note 仅初始化或受保护的配置提交边界调用；保留积分、输出和动态限幅。
 */
void FOC_ConfigureControllers(const MotorConfiguration *configuration,
                              FocState *foc,
                              PiController *speed_pi);

/**
 * @brief 按本拍母线电压刷新d/q电压额度。
 * @param foc 母线采样及d/q控制器所有者。
 * @note 快速环在母线采样后调用；不改变增益、采样周期、积分或速度控制器。
 */
void FOC_UpdateCurrentControllerLimits(FocState *foc);

/**
 * @brief  Clarke 变换：三相 → 静止 α/β。
 * @param  a 三相 A 相值。
 * @param  b 三相 B 相值。
 * @param  c 三相 C 相值。
 * @param  alpha 输出 α 轴分量，调用方持有。
 * @param  beta 输出 β 轴分量，调用方持有。
 */
void Clarke_Transform(float a, float b, float c, float *alpha, float *beta);
/**
 * @brief  Clarke 反变换：α/β → 三相。
 * @param  alpha α 轴分量。
 * @param  beta β 轴分量。
 * @param  a 输出 A 相值。
 * @param  b 输出 B 相值。
 * @param  c 输出 C 相值。
 */
void Inverse_Clarke_Transform(float alpha, float beta, float *a, float *b, float *c);
/**
 * @brief  Park 变换：α/β → 旋转 d/q。
 * @param  alpha α 轴分量。
 * @param  beta β 轴分量。
 * @param  theta 电角度，单位 rad。
 * @param  d 输出 d 轴分量。
 * @param  q 输出 q 轴分量。
 */
void Park_Transform(float alpha, float beta, float theta, float *d, float *q);
/**
 * @brief  Park 反变换：d/q → α/β。
 * @param  d d 轴分量。
 * @param  q q 轴分量。
 * @param  theta 电角度，单位 rad。
 * @param  alpha 输出 α 轴分量。
 * @param  beta 输出 β 轴分量。
 */
void Inverse_Park_Transform(float d, float q, float theta, float *alpha, float *beta);
/**
 * @brief  SVPWM 零序注入：由三相占空比求注入后占空比。
 * @param  a A 相占空比（0..1）。
 * @param  b B 相占空比（0..1）。
 * @param  c C 相占空比（0..1）。
 * @param  duty_a 输出 A 相占空比。
 * @param  duty_b 输出 B 相占空比。
 * @param  duty_c 输出 C 相占空比。
 */
void SVM_ZeroInjection(float a, float b, float c, float *duty_a, float *duty_b, float *duty_c);
/**
 * @brief  SVPWM 扇区判定与矢量时间计算。
 * @param  alpha α 轴调制分量。
 * @param  beta β 轴调制分量。
 * @param  duty_a 输出 A 相占空比。
 * @param  duty_b 输出 B 相占空比。
 * @param  duty_c 输出 C 相占空比。
 * @param  sector 输出扇区号（1..6），调用方持有。
 * @note   仅快速环调用；输入需为有限值。
 */
void SVM_SectorJudge(
    float alpha, float beta, float *duty_a, float *duty_b, float *duty_c, int32_t *sector);

/**
 * @brief  电流环：执行一拍 d/q 电流闭环并更新三相 PWM。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态。
 * @param  motor_state 运行目标与动态热额度；必要时锁存故障。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s（解耦/前馈用）。
 * @note   调用前由所有者将活动Kp/Ki及Current_Ts绑定到id_pi/iq_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；PWM 输出经 motor_hw 契约。
 */
void FOC_Current(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 float phase,
                 float phase_vel);
/**
 * @brief  电流环（带外部 q 轴参考）：供辨识与调试注入 iq。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态。
 * @param  motor_state 电机控制状态。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s。
 * @param  iq_reference q 轴电流参考，单位 A。
 * @note   调用前由所有者将活动Kp/Ki及Current_Ts绑定到id_pi/iq_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；参考仍受 current_limit_a 限制。
 */
void FOC_CurrentWithReference(const MotorConfiguration *configuration,
                              FocState *foc,
                              MotorControlState *motor_state,
                              float phase,
                              float phase_vel,
                              float iq_reference);
/**
 * @brief  复位电流环控制器（清积分与输出）。
 * @param  foc foc 状态。
 * @note   模式切换与故障清理时调用；不写硬件。
 */
void FOC_CurrentController_Reset(FocState *foc);
/**
 * @brief  vq_v 开环电压模式：固定电角度施加 q 轴电压执行一拍。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态。
 * @param  motor_state 电机控制状态。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s。
 * @note   调用前由所有者将活动Kp/Ki及Current_Ts绑定到id_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；需要有效编码器帧。
 */
void FOC_Vq_Mode(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 float phase,
                 float phase_vel);
#endif
