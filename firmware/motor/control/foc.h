#ifndef FOC_H
#define FOC_H

/* foc 算法核心：坐标变换、SVPWM 与电流/电压环；PWM 输出统一经 BSP 契约，
 * 不直接访问定时器寄存器。 */

#include "motor_config.h"
#include "motor_types.h"
#include "pi_controller.h"
#include "coordinate_types.h"

/**
 * @brief FOC控制器、坐标变换、调制与直接持有的测量状态。
 * @note 电压V、电流A，调制比及占空比为标幺；estimated字段为控制估算。
 */
typedef struct
{
    /* 轴在上电时绑定实际 PWM，独立台架沿用板级默认输出。 */
    void (*set_duty)(float phase_a, float phase_b, float phase_c);
    float bus_current_estimated_a, bus_current_estimated_filtered_a;
    float power_filtered_w;
    AlphaBetaVector voltage_ab_v;
    DqVector voltage_dq_v;
    AlphaBetaVector current_ab_a;
    AlphaBetaVector current_ab_filtered_a;
    DqVector current_dq_a, current_dq_filtered_a;
    PiController id_pi;
    PiController iq_pi;
    /*dq voltage of p.u.*/
    DqVector mod_dq;
    /*alpha-beta voltage of p.u.*/
    AlphaBetaVector mod_ab;
    /*duty of voltage utilization */
    float duty;
    /*duty cycle of abc*/
    ABCVect duty_abc;
    int32_t sector;
    /* 硬件实测量；与上面的母线电流/功率估算值独立。 */
    float bus_voltage_v, bus_voltage_filtered_v;
    /* 一阶低通系数，无量纲[0,1]；0保持历史，1直接跟随本拍。 */
    float bus_voltage_filter_alpha;
    /* 来自硬件通道；未提供或未校零时为NAN，与FOC估算电流独立。 */
    float bus_current_a, bus_current_filtered_a;
    /* 硬件母线电流的一阶低通系数，无量纲[0,1]。 */
    float bus_current_filter_alpha;
    ABCVect phase_current_a;
    float temperature_c;
} FocState;

/**
 * @brief 计算当前母线允许的 d/q 电压幅值上限，复用控制器调制深度限制。
 * @param bus_voltage_v 母线电压，单位 V。
 * @return 有限正母线电压对应的最大幅值 V；无效或非正输入返回零。
 * @note 纯计算、非阻塞；这是算法限幅，不替代产品功率与温度保护。
 */
float Foc_MaxVoltage(float bus_voltage_v);

/**
 * @brief 将活动d/q与速度PI增益、采样周期绑定到显式控制器。
 * @param configuration 调用期间稳定的只读电机及应用配置。
 * @param foc d/q控制器所有者。
 * @param speed_pi 速度控制器所有者。
 * @note 仅初始化或受保护的配置提交边界调用；保留积分、输出和动态限幅。
 */
void Foc_ConfigureControllers(const MotorConfiguration *configuration,
                              FocState *foc,
                              PiController *speed_pi);

/**
 * @brief 按本拍母线电压刷新d/q电压额度。
 * @param foc d/q控制器所有者。
 * @note 快速环在母线采样后调用；不改变增益、采样周期、积分或速度控制器。
 */
void Foc_UpdateCurrentLimits(FocState *foc);

/**
 * @brief 等幅Clarke变换：三相静止坐标转到α/β，a相与α轴重合。
 * @param abc 只读三相输入，要求a+b+c=0，分量使用相同单位。
 * @param ab 调用方持有的α/β输出，单位与输入一致，不与输入重叠。
 * @note 纯计算、固定工作量；可在快速环调用，不保留指针。
 */
void Clarke_Transform(const ABCVect *abc, AlphaBetaVector *ab);
/**
 * @brief 逆Clarke变换：α/β静止坐标转到无零序三相坐标。
 * @param ab 只读α/β输入，两个分量使用相同单位。
 * @param abc 调用方持有的三相输出，单位与输入一致，不与输入重叠。
 */
void Clarke_Inverse(const AlphaBetaVector *ab, ABCVect *abc);
/**
 * @brief Park变换：α/β静止坐标转到d/q旋转坐标。
 * @param ab 只读α/β输入，两个分量使用相同单位。
 * @param theta 电角度，单位rad，满足Math_SinCos输入契约。
 * @param dq 调用方持有的d/q输出，单位与输入一致，不与输入重叠。
 */
void Park_Transform(const AlphaBetaVector *ab, float theta, DqVector *dq);
/**
 * @brief 逆Park变换：d/q旋转坐标转到α/β静止坐标。
 * @param dq 只读d/q输入，两个分量使用相同单位。
 * @param theta 电角度，单位rad，满足Math_SinCos输入契约。
 * @param ab 调用方持有的α/β输出，单位与输入一致，不与输入重叠。
 */
void Park_Inverse(const DqVector *dq, float theta, AlphaBetaVector *ab);
/**
 * @brief 计算居中零矢量分配的SVPWM占空比和扇区。
 * @param ab 只读有限α/β标幺调制量，沿用现有归一化方式。
 * @param duty 调用方持有的三相占空比输出，不与输入重叠。
 * @return 扇区号1..6，零向量归入扇区1。
 * @note 纯计算、固定工作量；调用方先限幅，在线性调制区输出为0..1。
 *       不额外截断占空比，保留既有PWM极性和扇区边界归属。
 */
int32_t Svm_SectorJudge(const AlphaBetaVector *ab, ABCVect *duty);

/** @brief 共用FOC控制律；角度来源由调用方独立选择。 */
typedef enum
{
    FOC_CONTROL_CURRENT,
    FOC_CONTROL_ID_VQ,
    FOC_CONTROL_VOLTAGE
} FocControlLaw;

/**
 * @brief 执行实测电流变换、控制律、电压限幅、SVPWM及一次PWM更新。
 * @param configuration 本拍稳定的活动配置。
 * @param foc 电流变换、调制与PI对象。
 * @param motor 电流保护额度和故障对象。
 * @param law d/q电流闭环、Id闭环加Vq给定或纯电压给定。
 * @param phase 本拍控制电角度，rad。
 * @param phase_vel 电角速度，rad/s，用于1.5个控制周期的输出角度预测。
 * @param d_reference 电流律/Id律为A，纯电压律为V。
 * @param q_reference 电流律为A，其余为V。
 * @note 仅快速环；调用方先完成采样、过流/过压和输出准入；本函数不启相。
 */
void Foc_Step(const MotorConfiguration *configuration,
              FocState *foc,
              MotorControlState *motor,
              FocControlLaw law,
              float phase,
              float phase_vel,
              float d_reference,
              float q_reference);

/**
 * @brief  电流环：执行一拍 d/q 电流闭环并更新三相 PWM。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态。
 * @param  motor_state 运行目标与动态热额度；必要时锁存故障。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s（解耦/前馈用）。
 * @note   调用前由所有者将活动Kp/Ki及FOC_TS_S绑定到id_pi/iq_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；PWM 输出经 motor_hw 契约。
 */
void Foc_RunCurrent(const MotorConfiguration *configuration,
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
 * @note   调用前由所有者将活动Kp/Ki及FOC_TS_S绑定到id_pi/iq_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；参考仍受 current_limit_a 限制。
 */
void Foc_RunCurrentRef(const MotorConfiguration *configuration,
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
void Foc_ResetCurrent(FocState *foc);
/**
 * @brief  Vq给定、Id闭环到零：使用传入电角度执行一拍。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态。
 * @param  motor_state 电机控制状态。
 * @param  phase 电角度，单位 rad。
 * @param  phase_vel 电角速度，单位 rad/s。
 * @note   调用前由所有者将活动Kp/Ki及FOC_TS_S绑定到id_pi；本入口只刷新动态电压限幅。
 *         仅快速环调用；需要有效编码器帧。
 */
void Foc_RunVq(const MotorConfiguration *configuration,
               FocState *foc,
               MotorControlState *motor_state,
               float phase,
               float phase_vel);
#endif
