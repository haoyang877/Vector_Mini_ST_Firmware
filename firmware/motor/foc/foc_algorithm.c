#include "foc_algorithm.h"

#include <stdbool.h>

#include "control_config.h"
#include "fast_loop_profile.h"
#include "motor_hw.h"
#include "control_math.h"
#include "motor_thermal.h"
#include "foc_errhandle.h"

/* foc 变换、调制与电流环入口。本文件不访问定时器寄存器：三相 PWM 输出统一
 * 经 platform/api/motor_hw.h 契约由板级端口（platform/stm32g4/ports）实现。 */

#define FOC_MAX_MODULATION (0.95f * SQRT_3_BY_2)

float FOC_MaximumVoltage(float bus_voltage_v)
{
    return isfinite(bus_voltage_v) && bus_voltage_v > 0.0f
               ? FOC_MAX_MODULATION * bus_voltage_v / 1.5f
               : 0.0f;
}

/* 计算 d/q 轴调制比并按最大调制深度限幅；母线电压无效时清零输出。
 * 返回值只表示本次是否产生有效调制，不表示功率级已使能。 */
static bool foc_set_voltage_modulation(FocState *foc, float voltage_d, float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (foc->bus_voltage_filtered_v <= 0.0f)
    {
        foc->mod_d = 0.0f;
        foc->mod_q = 0.0f;
        foc->duty = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / foc->bus_voltage_filtered_v;
    foc->mod_d = voltage_to_modulation * voltage_d;
    foc->mod_q = voltage_to_modulation * voltage_q;
    modulation_magnitude = Math_Sqrt(foc->mod_d * foc->mod_d + foc->mod_q * foc->mod_q);

    if (modulation_magnitude > FOC_MAX_MODULATION)
    {
        float modulation_scale = FOC_MAX_MODULATION / modulation_magnitude;
        foc->mod_d *= modulation_scale;
        foc->mod_q *= modulation_scale;
        modulation_magnitude = FOC_MAX_MODULATION;
    }

    if (modulation_magnitude > 0.0f)
    {
        foc->duty = Math_Sign(foc->mod_q) * modulation_magnitude / FOC_MAX_MODULATION;
    }
    else
    {
        foc->duty = 0.0f;
    }

    return true;
}

/* Clarke 变换：三相静止坐标到 alpha-beta 坐标（等幅值，a 相与 alpha 轴重合）。 */
void Clarke_Transform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta = (b - c) * ONE_BY_SQRT_3;
}

/* 逆 Clarke 变换：alpha-beta 坐标到三相静止坐标。 */
void Inverse_Clarke_Transform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + SQRT_3_BY_2 * beta;
    *c = -0.5f * alpha - SQRT_3_BY_2 * beta;
}

/* Park 变换：alpha-beta 坐标到随电角度 theta 旋转的 d-q 坐标。 */
void Park_Transform(float alpha, float beta, float theta, float *d, float *q)
{
    float sin;
    float cos;
    Math_SinCos(theta, &sin, &cos);

    *d = alpha * cos + beta * sin;
    *q = -alpha * sin + beta * cos;
}

/* 逆 Park 变换：d-q 坐标到 alpha-beta 坐标。 */
void Inverse_Park_Transform(float d, float q, float theta, float *alpha, float *beta)
{
    float sin;
    float cos;
    Math_SinCos(theta, &sin, &cos);

    *alpha = d * cos - q * sin;
    *beta = d * sin + q * cos;
}

/* 空间矢量调制：按 alpha-beta 平面象限与边界判定 1~6 扇区，
 * 输出三相占空比 duty_a/duty_b/duty_c（0~1，含零矢量居中分配）。 */
void SVM_SectorJudge(
    float alpha, float beta, float *duty_a, float *duty_b, float *duty_c, int32_t *sector)
{
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            /* 象限 I */
            if (ONE_BY_SQRT_3 * beta > alpha)
            {
                *sector = 2;
            }
            else
            {
                *sector = 1;
            }
        }
        else
        {
            /* 象限 II */
            if (-ONE_BY_SQRT_3 * beta > alpha)
            {
                *sector = 3;
            }
            else
            {
                *sector = 2;
            }
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            /* 象限 IV */
            if (-ONE_BY_SQRT_3 * beta > alpha)
            {
                *sector = 5;
            }
            else
            {
                *sector = 6;
            }
        }
        else
        {
            /* 象限 III */
            if (ONE_BY_SQRT_3 * beta > alpha)
            {
                *sector = 4;
            }
            else
            {
                *sector = 5;
            }
        }
    }

    switch (*sector)
    {
    /* 扇区 v1-v2 */
    case 1:
    {
        float t1 = alpha - ONE_BY_SQRT_3 * beta;
        float t2 = TWO_BY_SQRT_3 * beta;

        *duty_a = (1.0f - t1 - t2) * 0.5f;
        *duty_b = *duty_a + t1;
        *duty_c = *duty_b + t2;
        break;
    }

    /* 扇区 v2-v3 */
    case 2:
    {
        float t2 = alpha + ONE_BY_SQRT_3 * beta;
        float t3 = -alpha + ONE_BY_SQRT_3 * beta;

        *duty_b = (1.0f - t2 - t3) * 0.5f;
        *duty_a = *duty_b + t3;
        *duty_c = *duty_a + t2;
        break;
    }

    /* 扇区 v3-v4 */
    case 3:
    {
        float t3 = TWO_BY_SQRT_3 * beta;
        float t4 = -alpha - ONE_BY_SQRT_3 * beta;

        *duty_b = (1.0f - t3 - t4) * 0.5f;
        *duty_c = *duty_b + t3;
        *duty_a = *duty_c + t4;
        break;
    }

    /* 扇区 v4-v5 */
    case 4:
    {
        float t4 = -alpha + ONE_BY_SQRT_3 * beta;
        float t5 = -TWO_BY_SQRT_3 * beta;

        *duty_c = (1.0f - t4 - t5) * 0.5f;
        *duty_b = *duty_c + t5;
        *duty_a = *duty_b + t4;
        break;
    }

    /* 扇区 v5-v6 */
    case 5:
    {
        float t5 = -alpha - ONE_BY_SQRT_3 * beta;
        float t6 = alpha - ONE_BY_SQRT_3 * beta;

        *duty_c = (1.0f - t5 - t6) * 0.5f;
        *duty_a = *duty_c + t5;
        *duty_b = *duty_a + t6;
        break;
    }

    /* 扇区 v6-v1 */
    case 6:
    {
        float t6 = -TWO_BY_SQRT_3 * beta;
        float t1 = alpha + ONE_BY_SQRT_3 * beta;

        *duty_a = (1.0f - t6 - t1) * 0.5f;
        *duty_c = *duty_a + t1;
        *duty_b = *duty_c + t6;
        break;
    }
    }
}

/* 两种控制策略共享采样变换，保持各自 PI 与电压给定。 */
static inline void foc_measure_currents(FocState *foc, float phase)
{
    Clarke_Transform(foc->phase_a_current_a,
                     foc->phase_b_current_a,
                     foc->phase_c_current_a,
                     &foc->current_alpha_a,
                     &foc->current_beta_a);
    Park_Transform(foc->current_alpha_a, foc->current_beta_a, phase, &foc->id_a, &foc->iq_a);
}

/* 滤波、遥测及 PWM 的计算与输出顺序不可改变。 */
static inline void foc_apply_modulation(FocState *foc, float phase, float phase_vel)
{
    Inverse_Park_Transform(
        foc->mod_d, foc->mod_q, phase + phase_vel * Current_Ts, &foc->mod_alpha, &foc->mod_beta);

    UTILS_LP_FAST(foc->id_filtered_a, foc->id_a, 0.01f);
    UTILS_LP_FAST(foc->iq_filtered_a, foc->iq_a, 0.01f);

    foc->bus_current_a = foc->mod_d * foc->id_a + foc->mod_q * foc->iq_a;
    UTILS_LP_FAST(foc->bus_current_filtered_a, foc->bus_current_a, 0.01f);
    foc->power_filtered_w = foc->bus_voltage_filtered_v * foc->bus_current_filtered_a;

    SVM_SectorJudge(
        foc->mod_alpha, foc->mod_beta, &foc->duty_a, &foc->duty_b, &foc->duty_c, &foc->sector);
    motor_hw_pwm_set_duty(foc->duty_a, foc->duty_b, foc->duty_c);
}

/* 电流环：使用 motor_state->iq_target_a 作为 q 轴参考执行一次 d/q 电流闭环。 */
void FOC_Current(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 float phase,
                 float phase_vel)
{
    FOC_CurrentWithReference(
        configuration, foc, motor_state, phase, phase_vel, motor_state->iq_target_a);
}

/* 电流环：使用显式 q 轴电流参考执行一次 d/q 电流闭环并输出三相占空比。 */
void FOC_CurrentWithReference(const MotorConfiguration *configuration,
                              FocState *foc,
                              MotorControlState *motor_state,
                              float phase,
                              float phase_vel,
                              float iq_reference)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    FAST_PROFILE_BEGIN(FAST_PROFILE_CURRENT);

    /* 最终提交再次按快环热额度限制d/q总向量，不能分别裁剪后超过总限流。 */
    float id_reference = motor_state->id_target_a;
    float current_limit_a = MotorThermal_ResolveLimit(
        configuration->application.current_limit_a, motor_state->thermal_current_limit_a, true);
    if (!MotorThermal_LimitVector(current_limit_a, &id_reference, &iq_reference))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_WINDING_THERMAL);
        FAST_PROFILE_END(FAST_PROFILE_CURRENT);
        return;
    }
    foc_measure_currents(foc, phase);

    if (foc->bus_voltage_filtered_v > 0.0f)
    {
        max_voltage = FOC_MaximumVoltage(foc->bus_voltage_filtered_v);
        PiController_Configure(&foc->id_pi,
                               configuration->model.id_kp,
                               configuration->model.id_ki,
                               Current_Ts,
                               -max_voltage,
                               max_voltage);
        PiController_Configure(&foc->iq_pi,
                               configuration->model.iq_kp,
                               configuration->model.iq_ki,
                               Current_Ts,
                               -max_voltage,
                               max_voltage);

        voltage_d = PiController_Run(&foc->id_pi, id_reference, foc->id_a);
        voltage_q = PiController_Run(&foc->iq_pi, iq_reference, foc->iq_a);
        foc_set_voltage_modulation(foc, voltage_d, voltage_q);

        PiController_TrackOutput(&foc->id_pi, foc->mod_d * foc->bus_voltage_filtered_v / 1.5f);
        PiController_TrackOutput(&foc->iq_pi, foc->mod_q * foc->bus_voltage_filtered_v / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(foc);
        foc->mod_d = 0.0f;
        foc->mod_q = 0.0f;
        foc->duty = 0.0f;
    }

    foc_apply_modulation(foc, phase, phase_vel);
    FAST_PROFILE_END(FAST_PROFILE_CURRENT);
}

/* 复位 d/q 电流环状态。 */
void FOC_CurrentController_Reset(FocState *foc)
{
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}

/* q 轴电压模式：d 轴电流闭环到 0，q 轴电压限幅后直接输出。 */
void FOC_Vq_Mode(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 float phase,
                 float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    foc_measure_currents(foc, phase);
    float limit_a = MotorThermal_ResolveLimit(
        configuration->application.current_limit_a, motor_state->thermal_current_limit_a, true);
    /* 电压模式无法按参考限流；当前热额度为零或实测越界时必须关断。 */
    if (!(limit_a > 0.0f) || !(foc->id_a * foc->id_a + foc->iq_a * foc->iq_a <= limit_a * limit_a))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_WINDING_THERMAL);
        return;
    }

    if (foc->bus_voltage_filtered_v > 0.0f)
    {
        max_voltage = FOC_MaximumVoltage(foc->bus_voltage_filtered_v);
        PiController_Configure(&foc->id_pi,
                               configuration->model.id_kp,
                               configuration->model.id_ki,
                               Current_Ts,
                               -max_voltage,
                               max_voltage);

        voltage_d = PiController_Run(&foc->id_pi, 0.0f, foc->id_a);
        voltage_q = Math_Constrain(motor_state->vq_target_v, -max_voltage, max_voltage);
        foc_set_voltage_modulation(foc, voltage_d, voltage_q);
        PiController_TrackOutput(&foc->id_pi, foc->mod_d * foc->bus_voltage_filtered_v / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(foc);
        foc->mod_d = 0.0f;
        foc->mod_q = 0.0f;
        foc->duty = 0.0f;
    }

    foc_apply_modulation(foc, phase, phase_vel);
}
