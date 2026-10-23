#include "closedloop_bench.h"

#include <math.h>
#include <string.h>
#include "control_config.h"
#include "control_math.h"
#include "speed_control.h"

/* 此装配默认YQ-5708；编译选项可独立选择电机，不按板名切换模型。 */
#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE FOC_MOTOR_PROFILE_YQ_5708
#endif
#include "../../../motor/motor_profile_selection.h"

#define R40_CL_POLE_PAIRS ((float)PARAM_MOTOR_POLE_PAIRS)
#define R40_CL_VOLTAGE_LIMIT_V 1.5f
#define R40_CL_SPEED_LIMIT_RAD_S (_2PI)

/* 台架末段20kHz环形记录；故障/完成后停止更新，主机仅在关相后读取。 */
volatile float g_closedloop_waveform[1024][4];
volatile uint32_t g_closedloop_waveform_samples;
/* 全程记录：4秒动态电流阶段500Hz，其余阶段200Hz；原始位置只来自传感器。 */
volatile float g_closedloop_motion[2048][8];
volatile uint32_t g_closedloop_motion_samples;

static void fail(R40ClBench *bench, R40OlModel *guard, uint32_t reason)
{
    if (bench->telemetry.failure == 0U)
    {
        bench->telemetry.failure = reason;
    }
    R40Ol_Stop(guard, R40_OL_OUTPUT_ERROR);
}

void R40Cl_Init(R40ClBench *bench)
{
    memset(bench, 0, sizeof(*bench));
    bench->speed_reference_rpm = 20.0f;
    bench->configuration.model.motor_pole_pairs = PARAM_MOTOR_POLE_PAIRS;
    bench->configuration.model.motor_phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM;
    bench->configuration.model.motor_d_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H;
    bench->configuration.model.motor_q_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H;
    bench->configuration.model.motor_flux_wb = PARAM_MOTOR_FLUX_WB;
    bench->configuration.model.id_kp = PARAM_MOTOR_DEFAULT_ID_KP;
    bench->configuration.model.iq_kp = PARAM_MOTOR_DEFAULT_IQ_KP;
    bench->configuration.model.id_ki = PARAM_MOTOR_DEFAULT_ID_KI;
    bench->configuration.model.iq_ki = PARAM_MOTOR_DEFAULT_IQ_KI;
    bench->configuration.application.current_limit_a = 0.8f;
    /* 降低速度环候选增益，避开实机约100Hz往返振荡；通过标准与保护不变。 */
    bench->configuration.application.speed_kp = 0.10f;
    bench->configuration.application.speed_ki = 10.0f;
    bench->configuration.application.speed_acceleration_rad_s2 = 2.5f;
    bench->configuration.application.speed_deceleration_rad_s2 = 2.5f;
    bench->motor.thermal_current_limit_a = 0.8f;
    FOC_ConfigureControllers(&bench->configuration, &bench->foc, &bench->speed_pi);
    Encoder_ParamInit(&bench->configuration.device.encoder, &bench->encoder);
}

bool R40Cl_ReadEncoder(R40ClBench *bench, uint8_t stage, uint16_t *raw)
{
    Encoder_CompleteSample(&bench->configuration, &bench->encoder, stage);
    if (bench->encoder.read_status != ENCODER_READ_OK)
    {
        return false;
    }
    *raw = bench->encoder.raw_q15;
    return true;
}

bool R40Cl_Begin(R40ClBench *bench, R40OlModel *guard, uint32_t mode, uint32_t now_ms)
{
    uint32_t required = mode > 0U && mode <= 4U ? (1U << (mode - 1U)) - 1U : UINT32_MAX;
    if ((mode == 4U &&
         (!isfinite(bench->speed_reference_rpm) || bench->speed_reference_rpm <= 0.0f ||
          bench->speed_reference_rpm >= 60.0f)) ||
        bench->telemetry.failure != 0U || bench->telemetry.passed_mask != required ||
        (guard->state != R40_OL_READY && guard->state != R40_OL_DONE))
    {
        return false;
    }
    if (guard->state == R40_OL_DONE)
    {
        guard->state = R40_OL_READY;
    }
    if (!R40Ol_Begin(guard, now_ms))
    {
        return false;
    }
    bench->telemetry.mode = mode;
    bench->stage_start_counts = bench->position_counts;
    g_closedloop_waveform_samples = 0U;
    g_closedloop_motion_samples = 0U;
    bench->telemetry.elapsed_ms = 0U;
    bench->telemetry.control_samples = 0U;
    bench->telemetry.quality_samples = 0U;
    bench->telemetry.tracking_mae_a = 0.0f;
    bench->telemetry.tracking_max_a = 0.0f;
    bench->error_sum_a = 0.0f;
    bench->speed_error_sum_rpm = 0.0f;
    bench->telemetry.speed_samples = 0U;
    bench->telemetry.speed_mae_rpm = 0.0f;
    memset(bench->angle_sums, 0, sizeof(bench->angle_sums));
    memset(bench->angle_samples, 0, sizeof(bench->angle_samples));
    FOC_CurrentController_Reset(&bench->foc);
    PiController_Reset(&bench->speed_pi);
    Encoder_ResetVelocity(&bench->encoder);
    bench->motor.speed_ramped_rad_s = 0.0f;
    bench->motor.iq_target_a = 0.0f;
    bench->configuration.application.current_limit_a =
        mode == 2U ? 1.6f : (mode == 3U ? 1.5f : 0.8f);
    bench->motor.thermal_current_limit_a = bench->configuration.application.current_limit_a;
    return true;
}

static void update_angle(R40ClBench *bench, uint16_t angle)
{
    if (!bench->angle_initialized)
    {
        bench->position_counts = angle;
        bench->angle_initialized = true;
    }
    else
    {
        bench->position_counts += (int16_t)(uint16_t)(angle - bench->last_angle);
    }
    bench->last_angle = angle;
    float direction = bench->telemetry.direction == 0 ? 1.0f : (float)bench->telemetry.direction;
    bench->telemetry.velocity_rad_s = direction * Encoder_GetMecVelContinuous(&bench->encoder);
}

static float ramp(uint32_t ticks, uint32_t duration)
{
    float up = fminf(1.0f, (float)ticks / 4000.0f);
    float down = fminf(1.0f, (float)(duration - ticks) / 4000.0f);
    return fminf(up, down);
}

static void finish(R40ClBench *bench, R40OlModel *guard)
{
    R40ClTelemetry *t = &bench->telemetry;
    if (t->quality_samples < 1000U || !(t->tracking_mae_a < 0.15f))
    {
        fail(bench, guard, 1U);
        return;
    }
    if (t->mode == 2U)
    {
        float averages[3];
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (bench->angle_samples[i] < 1000U)
            {
                fail(bench, guard, 2U);
                return;
            }
            averages[i] = (float)bench->angle_sums[i] / (float)bench->angle_samples[i];
        }
        t->calibration_travel_counts = averages[1] - averages[0];
        t->calibration_return_counts = averages[2] - averages[0];
        float expected = 65536.0f;
        if (fabsf(fabsf(t->calibration_travel_counts) - expected) > expected * 0.01f ||
            fabsf(t->calibration_return_counts) > 150.0f)
        {
            fail(bench, guard, 2U);
            return;
        }
        t->direction = t->calibration_travel_counts > 0.0f ? 1 : -1;
        /* 正反接近的零位等权平均，减少单侧静摩擦偏置；中点减去一机械圈。 */
        t->zero_q15 =
            0.5f * (0.5f * (averages[0] + averages[2]) + averages[1] - t->direction * expected);
        bench->configuration.device.encoder.reverse = t->direction < 0 ? 1U : 0U;
        uint16_t zero = (uint16_t)lroundf(t->zero_q15);
        if (t->direction < 0)
        {
            zero = (uint16_t)(0U - zero);
        }
        if (!Encoder_SetElectricalZeroQ15(
                &bench->configuration.device.encoder, &bench->encoder, zero))
        {
            fail(bench, guard, 2U);
            return;
        }
        Encoder_ResetVelocity(&bench->encoder);
    }
    if (t->mode == 4U && (t->speed_samples < 10000U || !(t->speed_mae_rpm < 1.5f)))
    {
        fail(bench, guard, 4U);
        return;
    }
    /* 电压阶段必须有实测正向运动；Id跟踪成立不能替代角度传感器的运动证据。 */
    if (t->mode == 3U &&
        t->direction * (float)(bench->position_counts - bench->stage_start_counts) <
            65536.0f / 72.0f)
    {
        fail(bench, guard, 5U);
        return;
    }
    t->passed_mask |= 1U << (t->mode - 1U);
    guard->state = R40_OL_DONE;
}

void R40Cl_Step(R40ClBench *bench, R40OlModel *guard, uint16_t angle_q15)
{
    R40ClTelemetry *t = &bench->telemetry;
    if (guard->state == R40_OL_FAULT)
    {
        return;
    }
    update_angle(bench, angle_q15);
    if (guard->state != R40_OL_RUNNING)
    {
        /* 原保护的10秒上限先到时，仍必须完成本阶段验收，不能绕过质量检查。 */
        if (guard->state == R40_OL_DONE && (t->passed_mask & (1U << (t->mode - 1U))) == 0U)
        {
            finish(bench, guard);
        }
        return;
    }
    uint32_t ticks = t->control_samples++;
    uint32_t duration = t->mode == 1U ? 80000U : 200000U;
    t->elapsed_ms = ticks / 20U;
    if (ticks >= duration)
    {
        finish(bench, guard);
        return;
    }
    if (ticks == 0U)
    {
        t->velocity_rad_s = 0.0f;
    }
    float phase = 0.0f;
    float id = 0.0f, iq = 0.0f;
    if (t->mode == 1U)
    {
        /* 先避开任意停留角的反向平衡点，再缓升电流；动态频段开始前回到固定零相位。 */
        phase = ticks < 10000U
                    ? 0.5f * _PI
                    : (ticks < 20000U ? 0.5f * _PI * (1.0f - (float)(ticks - 10000U) / 10000.0f)
                                      : 0.0f);
        float current_ramp =
            ticks < 20000U ? 0.3f : 0.3f + 0.3f * fminf(1.0f, (float)(ticks - 20000U) / 10000.0f);
        id = current_ramp * ramp(ticks, duration);
        /* 固定电角度隔离编码器换相影响；小幅q轴扫频检查电流动态，不用于持续加速。 */
        if (ticks >= 40000U && ticks < 76000U)
        {
            if (ticks % SPEED_LOOP_DIVIDER == 0U)
            {
                uint32_t frequency_hz = ticks < 50000U ? 2U : (ticks < 60000U ? 10U : 30U);
                uint32_t phase_ticks = ((ticks - 40000U) * frequency_hz) % 20000U;
                float sine, cosine;
                Math_SinCos((float)phase_ticks * (_2PI / 20000.0f), &sine, &cosine);
                bench->motor.iq_target_a = 0.15f * sine;
            }
            iq = bench->motor.iq_target_a;
        }
        else
        {
            bench->motor.iq_target_a = 0.0f;
        }
    }
    else if (t->mode == 2U)
    {
        id = 1.6f * fminf(1.0f, (float)ticks / 4000.0f);
        /* 初始与最终零位均从正角度接近，避免随机初始位置的齿槽/静摩擦滞回
         * 被误记为回零误差；不放宽150计数的重复性门限。 */
        if (ticks < 10000U)
        {
            phase = 0.5f * _PI;
        }
        else if (ticks < 20000U)
        {
            phase = 0.5f * _PI * (1.0f - (float)(ticks - 10000U) / 10000.0f);
        }
        else if (ticks >= 40000U && ticks < 100000U)
        {
            float cycles = R40_CL_POLE_PAIRS * (float)(ticks - 40000U) / 60000.0f;
            phase = _2PI * (cycles - (uint32_t)cycles);
        }
        else if (ticks >= 100000U && ticks < 120000U)
        {
            phase = 0.0f;
        }
        else if (ticks >= 120000U && ticks < 180000U)
        {
            float cycles = R40_CL_POLE_PAIRS * (1.0f - (float)(ticks - 120000U) / 60000.0f);
            phase = _2PI * (cycles - (uint32_t)cycles);
        }
        const uint32_t starts[3] = {30000U, 110000U, 190000U};
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (ticks >= starts[i] && ticks < starts[i] + 8000U)
            {
                bench->angle_sums[i] += bench->position_counts;
                ++bench->angle_samples[i];
            }
        }
    }
    else
    {
        phase = Encoder_GetElePhase(&bench->encoder);
        if (fabsf(t->velocity_rad_s) > R40_CL_SPEED_LIMIT_RAD_S)
        {
            fail(bench, guard, 4U);
            return;
        }
        if (t->mode == 3U)
        {
            if (t->quality_samples >= 1000U &&
                t->direction * (float)(bench->position_counts - bench->stage_start_counts) >=
                    65536.0f / 72.0f)
            {
                finish(bench, guard);
                return;
            }
            /* 已完成预定位与电零位；复用现有Vq模式，Id闭环为0，速度不闭环。 */
            /* 按实测5度终止的短行程检查；不以恒定Vq尝试维持低速。 */
            bench->motor.vq_target_v = 0.55f * ramp(ticks, duration);
        }
        else
        {
            if (ticks % SPEED_LOOP_DIVIDER == 0U)
            {
                bench->motor.speed_target_rad_s =
                    ticks < 80000U ? bench->speed_reference_rpm * _2PI / 60.0f
                                   : (ticks < 100000U || ticks >= 180000U
                                          ? 0.0f
                                          : -bench->speed_reference_rpm * _2PI / 60.0f);
                SpeedMode_UpdateControl(&bench->configuration,
                                        &bench->motor,
                                        &bench->speed_pi,
                                        t->direction * t->velocity_rad_s);
            }
            iq = bench->motor.iq_target_a;
        }
    }
    /* 校准保持点2pi与0等价；保持输入在半开区间，避免原三角函数反复走fmodf慢路径。 */
    if (phase >= _2PI)
    {
        phase -= _2PI;
    }
    bench->motor.id_target_a = id;
    bench->foc.bus_voltage_filtered_v = guard->bus_v;
    /* 原中间层电流语义为负向采样差，与其SVM占空比方向配对。 */
    bench->foc.phase_a_current_a = -guard->current_a[0];
    bench->foc.phase_b_current_a = -guard->current_a[1];
    bench->foc.phase_c_current_a = -guard->current_a[2];
    if (t->mode == 3U)
    {
        FOC_Vq_Mode(&bench->configuration, &bench->foc, &bench->motor, phase, 0.0f);
    }
    else
    {
        FOC_CurrentWithReference(
            &bench->configuration, &bench->foc, &bench->motor, phase, 0.0f, iq);
    }
    t->id_reference_a = id;
    t->iq_reference_a = iq;
    t->id_a = bench->foc.id_a;
    t->iq_a = bench->foc.iq_a;
    t->vd_v = bench->foc.mod_d * guard->bus_v / 1.5f;
    t->vq_v = bench->foc.mod_q * guard->bus_v / 1.5f;
    t->phase_rad = phase;
    t->phase_sum_a = guard->current_a[0] + guard->current_a[1] + guard->current_a[2];
    uint32_t wave_index = g_closedloop_waveform_samples & 1023U;
    g_closedloop_waveform[wave_index][0] = t->id_a;
    g_closedloop_waveform[wave_index][1] = t->iq_a;
    g_closedloop_waveform[wave_index][2] = t->vd_v;
    g_closedloop_waveform[wave_index][3] = t->vq_v;
    ++g_closedloop_waveform_samples;
    uint32_t motion_divider = t->mode == 1U ? 40U : 100U;
    if (ticks % motion_divider == 0U && g_closedloop_motion_samples < 2048U)
    {
        uint32_t index = g_closedloop_motion_samples;
        g_closedloop_motion[index][0] = (float)bench->position_counts;
        g_closedloop_motion[index][1] = t->velocity_rad_s;
        g_closedloop_motion[index][2] = t->id_a;
        g_closedloop_motion[index][3] = t->iq_a;
        g_closedloop_motion[index][4] = iq;
        g_closedloop_motion[index][5] = t->vd_v;
        g_closedloop_motion[index][6] = t->vq_v;
        g_closedloop_motion[index][7] = phase;
        ++g_closedloop_motion_samples;
    }
    if (t->mode == 4U &&
        ((ticks >= 40000U && ticks < 70000U) || (ticks >= 140000U && ticks < 170000U)))
    {
        bench->speed_error_sum_rpm +=
            fabsf(bench->motor.speed_target_rad_s - t->direction * t->velocity_rad_s) * 60.0f /
            _2PI;
        t->speed_mae_rpm = bench->speed_error_sum_rpm / ++t->speed_samples;
    }
    float voltage_limit_v = t->mode == 2U ? 2.0f : R40_CL_VOLTAGE_LIMIT_V;
    if (!isfinite(t->vd_v) || !isfinite(t->vq_v) ||
        t->vd_v * t->vd_v + t->vq_v * t->vq_v > voltage_limit_v * voltage_limit_v ||
        bench->motor.fault_code != MOTOR_FAULT_NONE)
    {
        fail(bench, guard, 3U);
        return;
    }
    /* 跳过每秒阶跃后的300ms，只以稳定窗口评价内环误差。 */
    bool quality_window = ticks % 20000U >= 6000U && duration - ticks > 4000U;
    if (t->mode == 3U)
    {
        quality_window = ticks >= 400U;
    }
    if (quality_window)
    {
        /* Vq模式无Iq参考，不把有功转矩电流当作Id跟踪误差。 */
        float error = fabsf(id - t->id_a) + (t->mode == 3U ? 0.0f : fabsf(iq - t->iq_a));
        bench->error_sum_a += error;
        t->tracking_mae_a = bench->error_sum_a / ++t->quality_samples;
        t->tracking_max_a = fmaxf(t->tracking_max_a, error);
        if (t->quality_samples >= 4000U && t->tracking_mae_a > 0.25f)
        {
            fail(bench, guard, 1U);
            return;
        }
    }
    guard->duty[0] = bench->foc.duty_a;
    guard->duty[1] = bench->foc.duty_b;
    guard->duty[2] = bench->foc.duty_c;
}
