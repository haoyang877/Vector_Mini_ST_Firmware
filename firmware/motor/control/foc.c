#include "foc_internal.h"

#include <stdbool.h>

#include "application_config.h"
#include "control_math.h"
#include "motor_thermal.h"
#include "motor_fault.h"

/* foc 变换、调制和电流计算；PWM 提交由调用方协调。 */

#define FOC_MAX_MODULATION (0.95f * SQRT_3_BY_2)

float Foc_MaxVoltage(float bus_voltage_v)
{
    return isfinite(bus_voltage_v) && bus_voltage_v > 0.0f
               ? FOC_MAX_MODULATION * bus_voltage_v / 1.5f
               : 0.0f;
}

/* 固定配置只在提交边界绑定，避免快环重复写入或重置控制器积分。 */
void Foc_ConfigureControllers(const MotorConfiguration *configuration,
                              FocState *foc,
                              PiController *speed_pi)
{
    foc->id_pi.proportional_gain = configuration->model.id_kp;
    foc->id_pi.integral_gain = configuration->model.id_ki;
    foc->id_pi.sample_period_s = FOC_TS_S;
    foc->iq_pi.proportional_gain = configuration->model.iq_kp;
    foc->iq_pi.integral_gain = configuration->model.iq_ki;
    foc->iq_pi.sample_period_s = FOC_TS_S;
    speed_pi->proportional_gain = configuration->application.speed_kp;
    speed_pi->integral_gain = configuration->application.speed_ki;
    speed_pi->sample_period_s = SPEED_LOOP_TS_S;
}

void Foc_UpdateCurrentLimits(FocState *foc, const MotorMeasurement *measurement)
{
    float maximum_voltage_v = Foc_MaxVoltage(measurement->bus_voltage_filtered_v);
    foc->id_pi.output_min = foc->iq_pi.output_min = -maximum_voltage_v;
    foc->id_pi.output_max = foc->iq_pi.output_max = maximum_voltage_v;
}

/* 计算 d/q 轴调制比并按最大调制深度限幅；母线电压无效时清零输出。
 * 返回值只表示本次是否产生有效调制，不表示功率级已使能。 */
static bool foc_set_voltage_modulation(FocState *foc,
                                       const MotorMeasurement *measurement,
                                       float voltage_d,
                                       float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (measurement->bus_voltage_filtered_v <= 0.0f)
    {
        foc->mod_d = 0.0f;
        foc->mod_q = 0.0f;
        foc->duty = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / measurement->bus_voltage_filtered_v;
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
void Clarke_Inverse(float alpha, float beta, float *a, float *b, float *c)
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
void Park_Inverse(float d, float q, float theta, float *alpha, float *beta)
{
    float sin;
    float cos;
    Math_SinCos(theta, &sin, &cos);

    *alpha = d * cos - q * sin;
    *beta = d * sin + q * cos;
}

/* 空间矢量调制：按 alpha-beta 平面象限与边界判定 1~6 扇区，
 * 输出三相占空比 duty_a/duty_b/duty_c（0~1，含零矢量居中分配）。 */
void Svm_SectorJudge(
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

/* 所有普通模式和标定入口共用同一条FOC流水线；策略只选择d/q控制律。 */
bool Foc_Prepare(const MotorConfiguration *configuration,
                 FocState *foc,
                 const MotorMeasurement *measurement,
                 MotorControlState *motor,
                 FocControlLaw law,
                 float phase,
                 float phase_vel,
                 float d_reference,
                 float q_reference)
{
    if ((unsigned)law > FOC_CONTROL_VOLTAGE ||
        (law == FOC_CONTROL_VOLTAGE && (!isfinite(d_reference) || !isfinite(q_reference))))
    {
        MotorFault_Report(motor, MOTOR_FAULT_INVALID_PARAMETERS);
        return false;
    }
    bool current_loop = law == FOC_CONTROL_CURRENT;
    float limit_a = MotorThermal_ResolveLimit(
        configuration->application.current_limit_a, motor->thermal_current_limit_a, true);
    if (current_loop && !MotorThermal_LimitVector(limit_a, &d_reference, &q_reference))
    {
        MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
        return false;
    }

    /* 实测abc -> alpha/beta -> 所选控制角度下的d/q；三路过流已在采样阶段检查。 */
    Clarke_Transform(measurement->phase_a_current_a,
                     measurement->phase_b_current_a,
                     measurement->phase_c_current_a,
                     &foc->current_alpha_a,
                     &foc->current_beta_a);
    Park_Transform(foc->current_alpha_a, foc->current_beta_a, phase, &foc->id_a, &foc->iq_a);
    if (!current_loop && (!(limit_a > 0.0f) ||
                          !(foc->id_a * foc->id_a + foc->iq_a * foc->iq_a <= limit_a * limit_a)))
    {
        MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
        return false;
    }

    /* 控制律只产生Vd/Vq，共用后续电压限幅、SVPWM和唯一PWM提交。 */
    if (measurement->bus_voltage_filtered_v > 0.0f)
    {
        float maximum_v = Foc_MaxVoltage(measurement->bus_voltage_filtered_v);
        float voltage_d = d_reference;
        float voltage_q = q_reference;
        switch (law)
        {
        case FOC_CONTROL_CURRENT:
            foc->id_pi.output_min = foc->iq_pi.output_min = -maximum_v;
            foc->id_pi.output_max = foc->iq_pi.output_max = maximum_v;
            voltage_d = PiController_Run(&foc->id_pi, d_reference, foc->id_a);
            voltage_q = PiController_Run(&foc->iq_pi, q_reference, foc->iq_a);
            break;
        case FOC_CONTROL_ID_VQ:
            foc->id_pi.output_min = -maximum_v;
            foc->id_pi.output_max = maximum_v;
            voltage_d = PiController_Run(&foc->id_pi, d_reference, foc->id_a);
            voltage_q = Math_Constrain(q_reference, -maximum_v, maximum_v);
            break;
        case FOC_CONTROL_VOLTAGE:
            break;
        }
        foc_set_voltage_modulation(foc, measurement, voltage_d, voltage_q);
    }
    else
    {
        Foc_ResetCurrent(foc);
        foc->mod_d = foc->mod_q = foc->duty = 0.0f;
    }

    /* 电压矢量已限幅：逆Park按1.5个控制周期预测角度，SVPWM之后只提交一次。 */
    Park_Inverse(foc->mod_d,
                 foc->mod_q,
                 phase + phase_vel * (1.5f * FOC_TS_S),
                 &foc->mod_alpha,
                 &foc->mod_beta);
    Svm_SectorJudge(
        foc->mod_alpha, foc->mod_beta, &foc->duty_a, &foc->duty_b, &foc->duty_c, &foc->sector);
    return true;
}

void Foc_Complete(FocState *foc, const MotorMeasurement *measurement, FocControlLaw law)
{
    bool current_loop = law == FOC_CONTROL_CURRENT;
    UTILS_LP_FAST(foc->id_filtered_a, foc->id_a, 0.01f);
    UTILS_LP_FAST(foc->iq_filtered_a, foc->iq_a, 0.01f);
    foc->bus_current_a = foc->mod_d * foc->id_a + foc->mod_q * foc->iq_a;
    UTILS_LP_FAST(foc->bus_current_filtered_a, foc->bus_current_a, 0.01f);
    foc->power_filtered_w = measurement->bus_voltage_filtered_v * foc->bus_current_filtered_a;

    /* 跟踪影响下一拍积分；电压开环不修改PI积分。 */
    if (measurement->bus_voltage_filtered_v > 0.0f && law != FOC_CONTROL_VOLTAGE)
    {
        PiController_TrackOutput(&foc->id_pi,
                                 foc->mod_d * measurement->bus_voltage_filtered_v / 1.5f);
        if (current_loop)
        {
            PiController_TrackOutput(&foc->iq_pi,
                                     foc->mod_q * measurement->bus_voltage_filtered_v / 1.5f);
        }
    }
}

void Foc_ResetCurrent(FocState *foc)
{
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}
