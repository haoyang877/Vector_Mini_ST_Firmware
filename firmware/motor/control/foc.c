#include "foc_internal.h"

#include <stdbool.h>
#include <math.h>

#include "application_config.h"
#include "control_math.h"
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

void Foc_UpdateCurrentLimits(FocState *foc)
{
    float maximum_voltage_v = Foc_MaxVoltage(foc->bus_voltage_filtered_v);
    foc->id_pi.output_min = foc->iq_pi.output_min = -maximum_voltage_v;
    foc->id_pi.output_max = foc->iq_pi.output_max = maximum_voltage_v;
}

/* 计算 d/q 轴调制比并按最大调制深度限幅；母线电压无效时清零输出。
 * 返回值只表示本次是否产生有效调制，不表示功率级已使能。 */
static bool foc_set_voltage_modulation(FocState *foc, float voltage_d, float voltage_q)
{
    float voltage_to_modulation;
    float modulation_magnitude;

    if (foc->bus_voltage_filtered_v <= 0.0f)
    {
        foc->mod_dq.d = 0.0f;
        foc->mod_dq.q = 0.0f;
        foc->duty = 0.0f;
        return false;
    }

    voltage_to_modulation = 1.5f / foc->bus_voltage_filtered_v;
    foc->mod_dq.d = voltage_to_modulation * voltage_d;
    foc->mod_dq.q = voltage_to_modulation * voltage_q;
    modulation_magnitude = Math_Sqrt(foc->mod_dq.d * foc->mod_dq.d + foc->mod_dq.q * foc->mod_dq.q);

    if (modulation_magnitude > FOC_MAX_MODULATION)
    {
        float modulation_scale = FOC_MAX_MODULATION / modulation_magnitude;
        foc->mod_dq.d *= modulation_scale;
        foc->mod_dq.q *= modulation_scale;
        modulation_magnitude = FOC_MAX_MODULATION;
    }

    if (modulation_magnitude > 0.0f)
    {
        foc->duty = Math_Sign(foc->mod_dq.q) * modulation_magnitude / FOC_MAX_MODULATION;
    }
    else
    {
        foc->duty = 0.0f;
    }

    return true;
}

/* 变换只组织坐标，不改变单位、相序或现有数值算法。 */
void Clarke_Transform(const ABCVect *abc, AlphaBetaVector *ab)
{
    ab->alpha = abc->a;
    ab->beta = (abc->b - abc->c) * ONE_BY_SQRT_3;
}

void Clarke_Inverse(const AlphaBetaVector *ab, ABCVect *abc)
{
    abc->a = ab->alpha;
    abc->b = -0.5f * ab->alpha + SQRT_3_BY_2 * ab->beta;
    abc->c = -0.5f * ab->alpha - SQRT_3_BY_2 * ab->beta;
}

void Park_Transform(const AlphaBetaVector *ab, float theta, DqVector *dq)
{
    float sin;
    float cos;
    Math_SinCos(theta, &sin, &cos);
    dq->d = ab->alpha * cos + ab->beta * sin;
    dq->q = -ab->alpha * sin + ab->beta * cos;
}

void Park_Inverse(const DqVector *dq, float theta, AlphaBetaVector *ab)
{
    float sin;
    float cos;
    Math_SinCos(theta, &sin, &cos);
    ab->alpha = dq->d * cos - dq->q * sin;
    ab->beta = dq->d * sin + dq->q * cos;
}

int32_t Svm_SectorJudge(const AlphaBetaVector *ab, ABCVect *duty)
{
    float alpha = ab->alpha;
    float beta = ab->beta;
    float beta_scaled = ONE_BY_SQRT_3 * beta;
    int32_t sector;
    if (beta >= 0.0f)
    {
        sector = alpha >= 0.0f ? (beta_scaled > alpha ? 2 : 1) : (-beta_scaled > alpha ? 3 : 2);
    }
    else
    {
        sector = alpha >= 0.0f ? (-beta_scaled > alpha ? 5 : 6) : (beta_scaled > alpha ? 4 : 5);
    }

    switch (sector)
    {
    /* 扇区 v1-v2 */
    case 1:
    {
        float t1 = alpha - ONE_BY_SQRT_3 * beta;
        float t2 = TWO_BY_SQRT_3 * beta;

        duty->a = (1.0f - t1 - t2) * 0.5f;
        duty->b = duty->a + t1;
        duty->c = duty->b + t2;
        break;
    }

    /* 扇区 v2-v3 */
    case 2:
    {
        float t2 = alpha + ONE_BY_SQRT_3 * beta;
        float t3 = -alpha + ONE_BY_SQRT_3 * beta;

        duty->b = (1.0f - t2 - t3) * 0.5f;
        duty->a = duty->b + t3;
        duty->c = duty->a + t2;
        break;
    }

    /* 扇区 v3-v4 */
    case 3:
    {
        float t3 = TWO_BY_SQRT_3 * beta;
        float t4 = -alpha - ONE_BY_SQRT_3 * beta;

        duty->b = (1.0f - t3 - t4) * 0.5f;
        duty->c = duty->b + t3;
        duty->a = duty->c + t4;
        break;
    }

    /* 扇区 v4-v5 */
    case 4:
    {
        float t4 = -alpha + ONE_BY_SQRT_3 * beta;
        float t5 = -TWO_BY_SQRT_3 * beta;

        duty->c = (1.0f - t4 - t5) * 0.5f;
        duty->b = duty->c + t5;
        duty->a = duty->b + t4;
        break;
    }

    /* 扇区 v5-v6 */
    case 5:
    {
        float t5 = -alpha - ONE_BY_SQRT_3 * beta;
        float t6 = alpha - ONE_BY_SQRT_3 * beta;

        duty->c = (1.0f - t5 - t6) * 0.5f;
        duty->a = duty->c + t5;
        duty->b = duty->a + t6;
        break;
    }

    /* 扇区 v6-v1 */
    case 6:
    {
        float t6 = -TWO_BY_SQRT_3 * beta;
        float t1 = alpha + ONE_BY_SQRT_3 * beta;

        duty->a = (1.0f - t6 - t1) * 0.5f;
        duty->c = duty->a + t1;
        duty->b = duty->c + t6;
        break;
    }
    }
    return sector;
}

bool Foc_Prepare(const MotorConfiguration *configuration,
                 FocState *foc,
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
    float limit_a = configuration->application.current_limit_a;
    /* 标定保持 NONE 控制模式，启动、交接和制动均执行同一矢量预算。 */
    if (configuration->preset != NULL && motor->control_mode == MOTOR_CONTROL_NONE)
    {
        limit_a = fminf(limit_a, configuration->model.calibration_current_a);
    }
    if (!(limit_a > 0.0f && limit_a <= 100.0f) ||
        (current_loop && !(fabsf(d_reference) <= 100.0f && fabsf(q_reference) <= 100.0f)))
    {
        MotorFault_Report(motor, MOTOR_FAULT_INVALID_PARAMETERS);
        return false;
    }
    if (current_loop)
    {
        float current_squared_a2 = d_reference * d_reference + q_reference * q_reference;
        if (current_squared_a2 > limit_a * limit_a)
        {
            float scale = limit_a / sqrtf(current_squared_a2);
            d_reference *= scale;
            q_reference *= scale;
        }
    }

    /* 实测abc转到控制角度下的d/q，供电流控制和电压模式电流检查使用。 */
    Clarke_Transform(&foc->phase_current_a, &foc->current_ab_a);
    Park_Transform(&foc->current_ab_a, phase, &foc->current_dq_a);
    if (!current_loop &&
        !(foc->current_dq_a.d * foc->current_dq_a.d + foc->current_dq_a.q * foc->current_dq_a.q <=
          limit_a * limit_a))
    {
        MotorFault_Report(motor, MOTOR_FAULT_OVERCURRENT);
        return false;
    }

    /* 控制律只产生Vd/Vq，共用后续电压限幅、SVPWM和唯一PWM提交。 */
    if (foc->bus_voltage_filtered_v > 0.0f)
    {
        float maximum_v = Foc_MaxVoltage(foc->bus_voltage_filtered_v);
        float voltage_d = d_reference;
        float voltage_q = q_reference;
        switch (law)
        {
        case FOC_CONTROL_CURRENT:
            foc->id_pi.output_min = foc->iq_pi.output_min = -maximum_v;
            foc->id_pi.output_max = foc->iq_pi.output_max = maximum_v;
            voltage_d = PiController_Run(&foc->id_pi, d_reference, foc->current_dq_a.d);
            voltage_q = PiController_Run(&foc->iq_pi, q_reference, foc->current_dq_a.q);
            break;
        case FOC_CONTROL_ID_VQ:
            foc->id_pi.output_min = -maximum_v;
            foc->id_pi.output_max = maximum_v;
            voltage_d = PiController_Run(&foc->id_pi, d_reference, foc->current_dq_a.d);
            voltage_q = Math_Constrain(q_reference, -maximum_v, maximum_v);
            break;
        case FOC_CONTROL_VOLTAGE:
            break;
        }
        foc_set_voltage_modulation(foc, voltage_d, voltage_q);
    }
    else
    {
        Foc_ResetCurrent(foc);
        foc->mod_dq.d = foc->mod_dq.q = foc->duty = 0.0f;
    }

    /* 电压矢量已限幅：逆Park按1.5个控制周期预测角度，SVPWM之后只提交一次。 */
    Park_Inverse(&foc->mod_dq, phase + phase_vel * (1.5f * FOC_TS_S), &foc->mod_ab);
    foc->sector = Svm_SectorJudge(&foc->mod_ab, &foc->duty_abc);
    return true;
}

void Foc_Complete(FocState *foc, FocControlLaw law)
{
    bool current_loop = law == FOC_CONTROL_CURRENT;
    /* 限幅后的命令电压估计，同时供遥测和原PI输出跟踪使用。 */
    foc->voltage_dq_v.d = foc->mod_dq.d * foc->bus_voltage_filtered_v / 1.5f;
    foc->voltage_dq_v.q = foc->mod_dq.q * foc->bus_voltage_filtered_v / 1.5f;
    UTILS_LP_FAST(foc->current_dq_filtered_a.d, foc->current_dq_a.d, 0.01f);
    UTILS_LP_FAST(foc->current_dq_filtered_a.q, foc->current_dq_a.q, 0.01f);
    foc->bus_current_estimated_a =
        foc->mod_dq.d * foc->current_dq_a.d + foc->mod_dq.q * foc->current_dq_a.q;
    UTILS_LP_FAST(foc->bus_current_estimated_filtered_a, foc->bus_current_estimated_a, 0.01f);
    foc->power_filtered_w = foc->bus_voltage_filtered_v * foc->bus_current_estimated_filtered_a;

    /* 跟踪影响下一拍积分；电压开环不修改PI积分。 */
    if (foc->bus_voltage_filtered_v > 0.0f && law != FOC_CONTROL_VOLTAGE)
    {
        PiController_TrackOutput(&foc->id_pi, foc->voltage_dq_v.d);
        if (current_loop)
        {
            PiController_TrackOutput(&foc->iq_pi, foc->voltage_dq_v.q);
        }
    }
}

void Foc_ResetCurrent(FocState *foc)
{
    foc->voltage_dq_v = (DqVector){0};
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}
