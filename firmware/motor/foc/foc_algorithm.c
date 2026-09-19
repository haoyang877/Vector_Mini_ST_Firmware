#include "foc_algorithm.h"

#include <stdbool.h>

#include "control_config.h"
#include "fast_loop_profile.h"
#include "motor_hw.h"
#include "utils.h"

/* FOC 变换、调制与电流环入口。本文件不访问定时器寄存器：三相 PWM 输出统一
 * 经 platform/api/motor_hw.h 契约由板级端口（platform/stm32g4/ports）实现。 */

#define FOC_MAX_MODULATION (0.95f * SQRT_3_BY_2)

/* 计算 d/q 轴调制比并按最大调制深度限幅；母线电压无效时清零输出。
 * 返回值只表示本次是否产生有效调制，不表示功率级已使能。 */
static bool FOC_SetVoltageModulation(FOC_TypeDef *FOC, float voltage_d, float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (FOC->Vbus_filt <= 0.0f)
    {
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / FOC->Vbus_filt;
    FOC->mod_d = voltage_to_modulation * voltage_d;
    FOC->mod_q = voltage_to_modulation * voltage_q;
    modulation_magnitude = fast_sqrt(FOC->mod_d * FOC->mod_d + FOC->mod_q * FOC->mod_q);

    if (modulation_magnitude > FOC_MAX_MODULATION)
    {
        float modulation_scale = FOC_MAX_MODULATION / modulation_magnitude;
        FOC->mod_d *= modulation_scale;
        FOC->mod_q *= modulation_scale;
        modulation_magnitude = FOC_MAX_MODULATION;
    }

    if (modulation_magnitude > 0.0f)
    {
        FOC->duty = sign_hard(FOC->mod_q) * modulation_magnitude / FOC_MAX_MODULATION;
    }
    else
    {
        FOC->duty = 0.0f;
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
    float sin = fast_sin(theta);
    float cos = fast_cos(theta);

    *d = alpha * cos + beta * sin;
    *q = -alpha * sin + beta * cos;
}

/* 逆 Park 变换：d-q 坐标到 alpha-beta 坐标。 */
void Inverse_Park_Transform(float d, float q, float theta, float *alpha, float *beta)
{
    float sin = fast_sin(theta);
    float cos = fast_cos(theta);

    *alpha = d * cos - q * sin;
    *beta = d * sin + q * cos;
}

/* 空间矢量调制：按 alpha-beta 平面象限与边界判定 1~6 扇区，
 * 输出三相占空比 tA/tB/tC（0~1，含零矢量居中分配）。 */
void SVM_SectorJudge(float alpha, float beta, float *tA, float *tB, float *tC, int32_t *sector)
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

        *tA = (1.0f - t1 - t2) * 0.5f;
        *tB = *tA + t1;
        *tC = *tB + t2;
        break;
    }

    /* 扇区 v2-v3 */
    case 2:
    {
        float t2 = alpha + ONE_BY_SQRT_3 * beta;
        float t3 = -alpha + ONE_BY_SQRT_3 * beta;

        *tB = (1.0f - t2 - t3) * 0.5f;
        *tA = *tB + t3;
        *tC = *tA + t2;
        break;
    }

    /* 扇区 v3-v4 */
    case 3:
    {
        float t3 = TWO_BY_SQRT_3 * beta;
        float t4 = -alpha - ONE_BY_SQRT_3 * beta;

        *tB = (1.0f - t3 - t4) * 0.5f;
        *tC = *tB + t3;
        *tA = *tC + t4;
        break;
    }

    /* 扇区 v4-v5 */
    case 4:
    {
        float t4 = -alpha + ONE_BY_SQRT_3 * beta;
        float t5 = -TWO_BY_SQRT_3 * beta;

        *tC = (1.0f - t4 - t5) * 0.5f;
        *tB = *tC + t5;
        *tA = *tB + t4;
        break;
    }

    /* 扇区 v5-v6 */
    case 5:
    {
        float t5 = -alpha - ONE_BY_SQRT_3 * beta;
        float t6 = alpha - ONE_BY_SQRT_3 * beta;

        *tC = (1.0f - t5 - t6) * 0.5f;
        *tA = *tC + t5;
        *tB = *tA + t6;
        break;
    }

    /* 扇区 v6-v1 */
    case 6:
    {
        float t6 = -TWO_BY_SQRT_3 * beta;
        float t1 = alpha + ONE_BY_SQRT_3 * beta;

        *tA = (1.0f - t6 - t1) * 0.5f;
        *tC = *tA + t1;
        *tB = *tC + t6;
        break;
    }
    }
}

/* 过渡兼容层：PWM 命令的历史入口，供标定等调用方使用；寄存器写入已移至
 * platform/stm32g4/ports/motor/motor_pwm_stm32g4.c。调用方迁移到
 * motor_hw_pwm_* 契约后删除本节。 */
void Set_A_Duty(float duty)
{
    motor_hw_pwm_set_phase_duty(MOTOR_HW_PWM_PHASE_A, duty);
}

void Set_B_Duty(float duty)
{
    motor_hw_pwm_set_phase_duty(MOTOR_HW_PWM_PHASE_B, duty);
}

void Set_C_Duty(float duty)
{
    motor_hw_pwm_set_phase_duty(MOTOR_HW_PWM_PHASE_C, duty);
}

/* 电压环：按设定 d/q 电压直接生成调制与三相占空比，不做电流闭环。 */
void FOC_Voltage(FOC_TypeDef *FOC, float Vd_set, float Vq_set, float phase)
{
    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC_SetVoltageModulation(FOC, Vd_set, Vq_set);

    Inverse_Park_Transform(FOC->mod_d, FOC->mod_q, phase, &FOC->mod_alpha, &FOC->mod_beta);
    SVM_SectorJudge(
        FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    motor_hw_pwm_set_duty(FOC->dtc_a, FOC->dtc_b, FOC->dtc_c);
}

/* 电流环：使用 MotorControl->iqRef 作为 q 轴参考执行一次 d/q 电流闭环。 */
void FOC_Current(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel)
{
    FOC_CurrentWithReference(FOC, MotorControl, phase, phase_vel, MotorControl->iqRef);
}

/* 电流环：使用显式 q 轴电流参考执行一次 d/q 电流闭环并输出三相占空比。 */
void FOC_CurrentWithReference(FOC_TypeDef *FOC,
                              MotorControl_TypeDef *MotorControl,
                              float phase,
                              float phase_vel,
                              float iq_reference)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    FAST_PROFILE_BEGIN(FAST_PROFILE_CURRENT);

    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    if (FOC->Vbus_filt > 0.0f)
    {
        max_voltage = FOC_MAX_MODULATION * FOC->Vbus_filt / 1.5f;
        PI_Controller_Configure(&FOC->id_pi,
                                MotorControl->id_Kp,
                                MotorControl->id_Ki,
                                Current_Ts,
                                -max_voltage,
                                max_voltage);
        PI_Controller_Configure(&FOC->iq_pi,
                                MotorControl->iq_Kp,
                                MotorControl->iq_Ki,
                                Current_Ts,
                                -max_voltage,
                                max_voltage);

        voltage_d = PI_Controller_Run(&FOC->id_pi, MotorControl->idRef, FOC->Id);
        voltage_q = PI_Controller_Run(&FOC->iq_pi, iq_reference, FOC->Iq);
        FOC_SetVoltageModulation(FOC, voltage_d, voltage_q);

        PI_Controller_TrackOutput(&FOC->id_pi, FOC->mod_d * FOC->Vbus_filt / 1.5f);
        PI_Controller_TrackOutput(&FOC->iq_pi, FOC->mod_q * FOC->Vbus_filt / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(FOC);
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
    }

    Inverse_Park_Transform(
        FOC->mod_d, FOC->mod_q, phase + phase_vel * Current_Ts, &FOC->mod_alpha, &FOC->mod_beta);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC->Ibus = FOC->mod_d * FOC->Id + FOC->mod_q * FOC->Iq;
    UTILS_LP_FAST(FOC->Ibus_filt, FOC->Ibus, 0.01f);
    FOC->Power_filt = FOC->Vbus_filt * FOC->Ibus_filt;

    SVM_SectorJudge(
        FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    motor_hw_pwm_set_duty(FOC->dtc_a, FOC->dtc_b, FOC->dtc_c);
    FAST_PROFILE_END(FAST_PROFILE_CURRENT);
}

/* 复位 d/q 电流环状态。 */
void FOC_CurrentController_Reset(FOC_TypeDef *FOC)
{
    PI_Controller_Reset(&FOC->id_pi);
    PI_Controller_Reset(&FOC->iq_pi);
}

/* q 轴电压模式：d 轴电流闭环到 0，q 轴电压限幅后直接输出。 */
void FOC_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, float phase, float phase_vel)
{
    float max_voltage;
    float voltage_d;
    float voltage_q;

    Clarke_Transform(FOC->Ia, FOC->Ib, FOC->Ic, &FOC->Ialpha, &FOC->Ibeta);
    Park_Transform(FOC->Ialpha, FOC->Ibeta, phase, &FOC->Id, &FOC->Iq);

    if (FOC->Vbus_filt > 0.0f)
    {
        max_voltage = FOC_MAX_MODULATION * FOC->Vbus_filt / 1.5f;
        PI_Controller_Configure(&FOC->id_pi,
                                MotorControl->id_Kp,
                                MotorControl->id_Ki,
                                Current_Ts,
                                -max_voltage,
                                max_voltage);

        voltage_d = PI_Controller_Run(&FOC->id_pi, 0.0f, FOC->Id);
        voltage_q = constrain(MotorControl->vqRef, -max_voltage, max_voltage);
        FOC_SetVoltageModulation(FOC, voltage_d, voltage_q);
        PI_Controller_TrackOutput(&FOC->id_pi, FOC->mod_d * FOC->Vbus_filt / 1.5f);
    }
    else
    {
        FOC_CurrentController_Reset(FOC);
        FOC->mod_d = 0.0f;
        FOC->mod_q = 0.0f;
        FOC->duty = 0.0f;
    }

    Inverse_Park_Transform(
        FOC->mod_d, FOC->mod_q, phase + phase_vel * Current_Ts, &FOC->mod_alpha, &FOC->mod_beta);

    UTILS_LP_FAST(FOC->Id_filt, FOC->Id, 0.01f);
    UTILS_LP_FAST(FOC->Iq_filt, FOC->Iq, 0.01f);

    FOC->Ibus = FOC->mod_d * FOC->Id + FOC->mod_q * FOC->Iq;
    UTILS_LP_FAST(FOC->Ibus_filt, FOC->Ibus, 0.01f);
    FOC->Power_filt = FOC->Vbus_filt * FOC->Ibus_filt;

    SVM_SectorJudge(
        FOC->mod_alpha, FOC->mod_beta, &FOC->dtc_a, &FOC->dtc_b, &FOC->dtc_c, &FOC->sector);
    motor_hw_pwm_set_duty(FOC->dtc_a, FOC->dtc_b, FOC->dtc_c);
}

/* 过渡兼容层：三相全开（占空比 1.0）与三相全关（占空比 0.0），
 * 供标定短接测试使用；调用方迁移到 motor_hw_pwm_* 契约后删除。 */
void PWM_TurnOnHighSides(void)
{
    motor_hw_pwm_set_duty(1.0f, 1.0f, 1.0f);
}

void PWM_TurnOnLowSides(void)
{
    motor_hw_pwm_set_duty(0.0f, 0.0f, 0.0f);
}
