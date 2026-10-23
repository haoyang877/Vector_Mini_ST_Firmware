#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "closedloop_bench.h"
#include "motor_hw.h"
#include "motor_fault.h"
#include "encoder_sensor.h"
#include "critical_hw.h"
#include "control_math.h"

static float s_duty[3], s_current[3];
static uint16_t s_angle;
static float s_mechanical_gain;
static float s_periodic_disturbance_a = 0.15f;
static bool s_voltage_stalled;

extern volatile float g_closedloop_motion[2048][8];
extern volatile uint32_t g_closedloop_motion_samples;

void encoder_sensor_init(void)
{
}

bool encoder_sensor_begin(void)
{
    return true;
}

uint8_t encoder_sensor_begin_readback(bool started)
{
    return started ? 1U : 0U;
}

EncoderSensorStatus encoder_sensor_complete(uint8_t stage, EncoderSensorSample *sample)
{
    (void)stage;
    *sample =
        (EncoderSensorSample){.status = ENCODER_SENSOR_OK, .angle_q15 = s_angle, .crc_ok = true};
    return ENCODER_SENSOR_OK;
}

uint32_t critical_hw_enter(void)
{
    return 0U;
}

void critical_hw_exit(uint32_t state)
{
    (void)state;
}

void motor_hw_pwm_set_duty(float a, float b, float c)
{
    s_duty[0] = a;
    s_duty[1] = b;
    s_duty[2] = c;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(isfinite(s_duty[i]) && s_duty[i] > 0.4f && s_duty[i] < 0.6f);
    }
}

void MotorFault_Report(MotorControlState *motor, MotorFaultCode code)
{
    motor->fault_code = code;
}

static void initialize(R40ClBench *bench, R40OlModel *guard)
{
    R40Cl_Init(bench);
    R40Ol_Init(guard);
    uint16_t raw[4] = {8192U, 8192U, 8192U, 4704U};
    for (uint32_t i = 0U; i < 22048U; ++i)
    {
        R40Ol_Step(guard, raw, true, true, i / 20U);
        s_angle = 10000U;
        uint16_t angle;
        assert(R40Cl_ReadEncoder(bench, 1U, &angle));
        R40Cl_Step(bench, guard, 10000U);
    }
    assert(guard->state == R40_OL_READY);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        s_current[i] = 0.0f;
        s_duty[i] = 0.5f;
    }
}

static void run(R40ClBench *bench, R40OlModel *guard, unsigned mode)
{
    assert(R40Cl_Begin(bench, guard, mode, 2000U));
    double rotor_phase = 0.0, previous_phase = 0.0;
    float position_rad = 0.0f, speed_rad_s = 0.0f;
    for (uint32_t i = 0U; i < 210000U; ++i)
    {
        uint32_t ms = 2000U + i / 20U;
        if (i % 1000U == 0U)
        {
            R40Ol_Heartbeat(guard, ms);
        }
        float mean = (s_duty[0] + s_duty[1] + s_duty[2]) / 3.0f;
        uint16_t raw[4] = {0U, 0U, 0U, 4704U};
        for (unsigned p = 0U; p < 3U; ++p)
        {
            float voltage =
                guard->state == R40_OL_RUNNING ? (s_duty[p] - mean) * guard->bus_v : 0.0f;
            s_current[p] += (voltage - 0.55f * s_current[p]) * 0.00005f / 0.0001425f;
            raw[p] = (uint16_t)lroundf(8192.0f + s_current[p] * 397.090909f);
        }
        uint16_t angle = 10000U;
        if (mode == 3U && !s_voltage_stalled)
        {
            /* 独立的1rpm传感器轨迹仅测试运动准入，不作为电机动力学或速度性能证据。 */
            angle = (uint16_t)(10000 + lroundf((float)i * 65536.0f / 1200000.0f));
        }
        if (mode == 4U && s_mechanical_gain > 0.0f)
        {
            /* 电流环驱动机械积分器，加入14阶周期扰动；不把控制输出当作速度反馈。 */
            float alpha, beta, direct, torque_current;
            Clarke_Transform(-s_current[0], -s_current[1], -s_current[2], &alpha, &beta);
            Park_Transform(alpha, beta, position_rad * 14.0f, &direct, &torque_current);
            speed_rad_s += s_mechanical_gain *
                           (torque_current - s_periodic_disturbance_a * sinf(position_rad * 14.0f) -
                            0.02f * speed_rad_s) *
                           0.00005f;
            position_rad += speed_rad_s * 0.00005f;
            angle = (uint16_t)(10000 + lroundf(position_rad * 65536.0f / 6.283185307f));
        }
        if (mode == 2U)
        {
            double delta = (double)bench->telemetry.phase_rad - previous_phase;
            if (delta > 3.141592654f)
            {
                delta -= 6.283185307f;
            }
            else if (delta < -3.141592654f)
            {
                delta += 6.283185307f;
            }
            rotor_phase += delta;
            previous_phase = bench->telemetry.phase_rad;
            angle = (uint16_t)(10000 + lround(rotor_phase * 65536.0 / (14.0 * 6.283185307)));
        }
        R40Ol_Step(guard, raw, true, true, ms);
        s_angle = angle;
        assert(R40Cl_ReadEncoder(bench, 1U, &angle));
        R40Cl_Step(bench, guard, angle);
        assert(bench->telemetry.phase_rad >= 0.0f && bench->telemetry.phase_rad < 6.283185307f);
        if (guard->state == R40_OL_DONE || guard->state == R40_OL_FAULT)
        {
            break;
        }
    }
}

int main(void)
{
    R40ClBench bench;
    R40OlModel guard;
    initialize(&bench, &guard);
    assert(!R40Cl_Begin(&bench, &guard, 2U, 2000U));
    assert(!R40Cl_Begin(&bench, &guard, 0U, 2000U));
    run(&bench, &guard, 1U);
    assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 1U);
    assert(bench.telemetry.tracking_mae_a < 0.06f);
    assert(g_closedloop_motion_samples >= 1999U && g_closedloop_motion_samples <= 2000U);
    /* 实际RL电流响应要跟上三个频段，不能仅以参考发生器存在判为通过。 */
    const unsigned starts[] = {1050U, 1300U, 1550U};
    const unsigned ends[] = {1250U, 1500U, 1900U};
    for (unsigned band = 0U; band < 3U; ++band)
    {
        float reference_square = 0.0f, measured_square = 0.0f, error_square = 0.0f;
        for (unsigned i = starts[band]; i < ends[band]; ++i)
        {
            float reference = g_closedloop_motion[i][4];
            float measured = g_closedloop_motion[i][3];
            reference_square += reference * reference;
            measured_square += measured * measured;
            error_square += (reference - measured) * (reference - measured);
        }
        assert(reference_square > 1.0f);
        assert(measured_square > 0.65f * reference_square);
        assert(measured_square < 1.1f * reference_square);
        assert(error_square < 0.15f * reference_square);
    }
    assert(!R40Cl_Begin(&bench, &guard, 1U, 2000U));
    run(&bench, &guard, 2U);
    assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 3U);
    assert(bench.telemetry.direction == 1);
    fprintf(stderr,
            "calibration zero %.3f travel %.3f return %.3f\n",
            bench.telemetry.zero_q15,
            bench.telemetry.calibration_travel_counts,
            bench.telemetry.calibration_return_counts);
    assert(fabsf(bench.telemetry.zero_q15 - 10000.0f) < 2.0f);
    assert(fabsf(bench.telemetry.calibration_travel_counts - 65536.0f) < 2.0f);
    s_voltage_stalled = true;
    run(&bench, &guard, 3U);
    assert(guard.state == R40_OL_FAULT && bench.telemetry.failure == 5U);
    assert(!R40Cl_Begin(&bench, &guard, 4U, 2000U));
    s_voltage_stalled = false;
    initialize(&bench, &guard);
    run(&bench, &guard, 1U);
    run(&bench, &guard, 2U);
    run(&bench, &guard, 3U);
    assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 7U);
    /* 独立1rpm反馈转过5度后结束，明显早于10秒硬时限。 */
    assert(bench.telemetry.elapsed_ms < 1000U &&
           bench.position_counts - bench.stage_start_counts >= 910);
    assert(bench.telemetry.tracking_mae_a < 0.02f);
    /* 私有台架速度参考必须有限且低于保护速度；拒绝时不得启相。 */
    const float invalid_speeds[] = {NAN, INFINITY, -1.0f, 0.0f, 60.0f};
    for (unsigned i = 0U; i < sizeof(invalid_speeds) / sizeof(invalid_speeds[0]); ++i)
    {
        bench.speed_reference_rpm = invalid_speeds[i];
        assert(!R40Cl_Begin(&bench, &guard, 4U, 2000U));
        assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 7U);
    }
    bench.speed_reference_rpm = 20.0f;
    /* 不动的编码器不能通过速度验收，即使电流闭环跟踪完美。 */
    run(&bench, &guard, 4U);
    assert(guard.state == R40_OL_FAULT && bench.telemetry.failure == 4U);
    assert(!R40Cl_Begin(&bench, &guard, 4U, 2000U));
    for (unsigned gain = 300U; gain <= 900U; gain += 300U)
    {
        initialize(&bench, &guard);
        run(&bench, &guard, 1U);
        run(&bench, &guard, 2U);
        run(&bench, &guard, 3U);
        /* 保留原较高带宽组合对周期负载的覆盖，不将其作为新板默认整定。 */
        bench.configuration.application.speed_kp = 0.35f;
        bench.configuration.application.speed_ki = 10.0f;
        bench.speed_reference_rpm = 8.0f;
        bench.configuration.application.speed_acceleration_rad_s2 = 1.0f;
        bench.configuration.application.speed_deceleration_rad_s2 = 1.0f;
        s_mechanical_gain = (float)gain;
        run(&bench, &guard, 4U);
        printf("speed plant gain %u, error %.3f rpm, failure %u\n",
               gain,
               bench.telemetry.speed_mae_rpm,
               bench.telemetry.failure);
        assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 15U);
    }
    initialize(&bench, &guard);
    run(&bench, &guard, 1U);
    run(&bench, &guard, 2U);
    run(&bench, &guard, 3U);
    s_periodic_disturbance_a = 0.0f;
    run(&bench, &guard, 4U);
    assert(guard.state == R40_OL_DONE && bench.telemetry.passed_mask == 15U);
    s_periodic_disturbance_a = 0.15f;
    initialize(&bench, &guard);
    run(&bench, &guard, 1U);
    run(&bench, &guard, 2U);
    run(&bench, &guard, 3U);
    run(&bench, &guard, 4U);
    /* 强扰动按同一误差线判定，不能因候选参数改变而误报通过。 */
    if (guard.state == R40_OL_DONE)
    {
        assert(bench.telemetry.speed_mae_rpm < 1.5f && bench.telemetry.passed_mask == 15U);
    }
    else
    {
        assert(guard.state == R40_OL_FAULT && bench.telemetry.failure == 4U &&
               bench.telemetry.speed_mae_rpm >= 1.5f);
    }
    s_mechanical_gain = 0.0f;
    initialize(&bench, &guard);
    assert(R40Cl_Begin(&bench, &guard, 1U, 2000U));
    uint16_t high[4] = {9500U, 8192U, 8192U, 4704U};
    R40Ol_Step(&guard, high, true, true, 2001U);
    assert(guard.state == R40_OL_FAULT && guard.fault == R40_OL_OVERCURRENT);
    assert(!R40Cl_Begin(&bench, &guard, 1U, 2002U));
    initialize(&bench, &guard);
    assert(R40Cl_Begin(&bench, &guard, 1U, 2000U));
    uint16_t zero[4] = {8192U, 8192U, 8192U, 4704U};
    R40Ol_Step(&guard, zero, true, true, 2300U);
    assert(guard.state == R40_OL_FAULT && guard.fault == R40_OL_LEASE_EXPIRED);
    puts("PASS R40 closed-loop sequencing, real PI/RL plant, angle calibration, stalled speed "
         "rejection, 3A and lease stop");
    return 0;
}
