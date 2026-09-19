#ifndef __FOC_ALGORITHM_H__
#define __FOC_ALGORITHM_H__

/* FOC 算法核心：坐标变换、SVPWM 与电流/电压环；PWM 输出统一经 motor_hw 契约，
 * 不直接访问定时器寄存器。 */

#include "data_type.h"
#include "foc_pid.h"

/**
 * @brief  FOC 运行时状态：母线/电流、坐标变换量、调制比与电流环控制器。
 * @note   单位：电压 V、电流 A、调制比/占空比为 0..1 标幺、角度为电角度 rad、
 *         temp 为功率板温度 °C；由快速环写入，其他上下文只读。
 */
typedef struct
{
    float Vbus, Vbus_filt;
    float Ibus, Ibus_filt;
    float Power_filt;
    float Valpha, Vbeta;
    float Vd, Vq;
    float Ia, Ib, Ic;
    float Ialpha, Ibeta;
    float Ialpha_filt, Ibeta_filt;
    float Id, Iq, Id_filt, Iq_filt;
    PI_Controller_TypeDef id_pi;
    PI_Controller_TypeDef iq_pi;
    /*dq voltage of p.u.*/
    float mod_d, mod_q;
    /*alpha-beta voltage of p.u.*/
    float mod_alpha, mod_beta;
    /*duty of voltage utilization */
    float duty;
    /*duty cycle of abc*/
    float dtc_a, dtc_b, dtc_c;
    int32_t sector;
    /*temprature of power board*/
    float temp;
} FOC_TypeDef;

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
 * @param  tA 输出 A 相占空比。
 * @param  tB 输出 B 相占空比。
 * @param  tC 输出 C 相占空比。
 */
void SVM_ZeroInjection(float a, float b, float c, float *tA, float *tB, float *tC);
/**
 * @brief  SVPWM 扇区判定与矢量时间计算。
 * @param  alpha α 轴调制分量。
 * @param  beta β 轴调制分量。
 * @param  tA 输出 A 相占空比。
 * @param  tB 输出 B 相占空比。
 * @param  tC 输出 C 相占空比。
 * @param  sector 输出扇区号（1..6），调用方持有。
 * @note   仅快速环调用；输入需为有限值。
 */
void SVM_SectorJudge(float alpha, float beta, float *tA, float *tB, float *tC, int32_t *sector);

/**
 * @brief  电压环：按 d/q 电压设定生成调制与三相占空比（电流开环）。
 * @param  FOC FOC 状态。
 * @param  Vd_set d 轴电压设定。
 * @param  Vq_set q 轴电压设定。
 * @param  phase 输出电角度，单位 rad。
 * @note   仅快速环调用；依赖母线电压与角度反馈有效。
 */
void FOC_Voltage(FOC_TypeDef *FOC, float Vd_set, float Vq_set, float phase);
/**
 * @brief  电流环：执行一拍 d/q 电流闭环并更新三相 PWM。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态（含电流指令与限制）。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s（解耦/前馈用）。
 * @note   仅快速环调用；PWM 输出经 motor_hw 契约。
 */
void FOC_Current(FOC_TypeDef *FOC,
                 MotorControl_TypeDef *MotorControl,
                 float phase,
                 float phase_vel);
/**
 * @brief  电流环（带外部 q 轴参考）：供辨识与调试注入 iq。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s。
 * @param  iq_reference q 轴电流参考，单位 A。
 * @note   仅快速环调用；参考仍受 current_limit 限制。
 */
void FOC_CurrentWithReference(FOC_TypeDef *FOC,
                              MotorControl_TypeDef *MotorControl,
                              float phase,
                              float phase_vel,
                              float iq_reference);
/**
 * @brief  复位电流环控制器（清积分与输出）。
 * @param  FOC FOC 状态。
 * @note   模式切换与故障清理时调用；不写硬件。
 */
void FOC_CurrentController_Reset(FOC_TypeDef *FOC);
/**
 * @brief  Vq 开环电压模式：固定电角度施加 q 轴电压执行一拍。
 * @param  FOC FOC 状态。
 * @param  MotorControl 电机控制状态。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s。
 * @note   仅快速环调用；需要有效编码器帧。
 */
void FOC_Vq_Mode(FOC_TypeDef *FOC,
                 MotorControl_TypeDef *MotorControl,
                 float phase,
                 float phase_vel);
#endif
