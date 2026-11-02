#include "motor_bench.h"

#include <math.h>
#include <string.h>

#include "application_config.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "control_math.h"
#include "encoder_calibration.h"
#include "motor_lifecycle.h"

/* 固定测试入口：4KiB记录区；默认禁用动力，控制算法和功率许可仍归原所有者。 */
#define BENCH_SAMPLES 128U
#define BENCH_VERSION 1U
#define BENCH_MAGIC 0x3154424DU
#define BENCH_START_TIMEOUT_TICKS FOC_FREQ

enum
{
    OP_INFO,
    OP_LIMITS,
    OP_RUN,
    OP_CAPTURE,
    OP_READ,
    OP_KEEPALIVE,
    OP_STOP,
    OP_LUT,
    OP_SNAPSHOT
};
enum
{
    RUN_NONE,
    RUN_ID,
    RUN_IQ_PULSE,
    RUN_SPEED,
    RUN_LUT,
    RUN_ZERO,
    RUN_MECHANICAL_ZERO,
    RUN_REMOTE,
    RUN_ID_SINE
};
enum
{
    RUN_IDLE,
    RUN_WAIT,
    RUN_ACTIVE,
    RUN_DONE,
    RUN_FAILED
};
enum
{
    CAP_IDLE,
    CAP_WAIT,
    CAP_RECORDING,
    CAP_FROZEN
};
enum
{
    GROUP_PHASE = 1,
    GROUP_DQ,
    GROUP_SPEED,
    GROUP_SINCOS,
    GROUP_ANGLE
};
enum
{
    WHY_NONE,
    WHY_CURRENT,
    WHY_SPEED,
    WHY_TRAVEL,
    WHY_LEASE,
    WHY_DURATION,
    WHY_START_TIMEOUT,
    WHY_STATE,
    WHY_ENCODER,
    WHY_FAULT,
    WHY_CANCEL,
    WHY_MEASUREMENT
};

typedef struct
{
    uint32_t tick;
    float values[7];
} BenchSample;

typedef struct
{
    float current_a, speed_rad_s, travel_rad;
    uint32_t duration_ms, lease_ms;
} BenchLimits;

static struct
{
    const MotorConfiguration *configuration;
    FocState *foc;
    MotorControlState *motor;
    EncoderState *encoder;
    FluxObserverState *observer;
    BenchLimits limits;
    bool limits_valid;
    volatile uint32_t tick;
    uint32_t heartbeat_tick, run_id, run_epoch, started_tick, step_tick;
    uint32_t duration_ticks, warmup_ticks, travel_counts;
    uint16_t previous_angle, run_kind, run_state, reason;
    float reference;
    float sine_bias_a, sine_phase_rad, sine_step_rad;
    bool reference_applied, step_edge, calibration_completed;
    uint16_t group, trigger, trigger_phase, pre_samples, capture_state;
    uint16_t count, write_index, first_index, invalid_samples;
    uint32_t capture_id, capture_run_id, stride, divider, delay_ticks;
    BenchSample samples[BENCH_SAMPLES], latest;
    bool last_valid;
    uint16_t last_sequence, last_length;
    uint8_t last_request[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint8_t last_reply[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint32_t last_reply_length;
} s_bench;

typedef char BenchSampleSizeIs32[sizeof(BenchSample) == 32U ? 1 : -1];

static uint32_t milliseconds_to_ticks(uint32_t value)
{
    return (uint32_t)(((uint64_t)value * FOC_FREQ + 999U) / 1000U);
}

static float read_float(const uint8_t *data)
{
    uint32_t bits = YgProtocol_ReadU32Le(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void write_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    YgProtocol_WriteU32Le(data, bits);
}

static bool running(void)
{
    return s_bench.run_state == RUN_WAIT || s_bench.run_state == RUN_ACTIVE;
}

static void freeze_capture(void)
{
    if (s_bench.capture_state == CAP_WAIT)
    {
        s_bench.count = 0U;
    }
    if (s_bench.capture_state == CAP_WAIT || s_bench.capture_state == CAP_RECORDING)
    {
        s_bench.capture_state = CAP_FROZEN;
    }
}

static void stop_run(bool failed, uint16_t reason)
{
    s_bench.reason = reason;
    s_bench.run_state = failed ? RUN_FAILED : RUN_DONE;
    freeze_capture();
    (void)MotorLifecycle_RequestRemoteStop();
}

void MotorBench_Init(const MotorConfiguration *configuration,
                     FocState *foc,
                     MotorControlState *motor,
                     EncoderState *encoder,
                     FluxObserverState *observer)
{
    memset(&s_bench, 0, sizeof(s_bench));
    s_bench.configuration = configuration;
    s_bench.foc = foc;
    s_bench.motor = motor;
    s_bench.encoder = encoder;
    s_bench.observer = observer;
    s_bench.group = GROUP_DQ;
    s_bench.stride = 1U;
}

static void start_tracking(void)
{
    s_bench.run_state = RUN_ACTIVE;
    s_bench.run_epoch = MotorLifecycle_WorkEpoch();
    s_bench.started_tick = s_bench.tick;
    s_bench.previous_angle = s_bench.encoder->directed_q15;
    s_bench.travel_counts = 0U;
    s_bench.reference_applied = false;
    s_bench.calibration_completed = false;
}

void MotorBench_BeforeControl(void)
{
    ++s_bench.tick;
    s_bench.step_edge = false;
    MotorState state = MotorLifecycle_GetState();
    /* 原标定和117入口共用监护；不代写116目标，未配置限值仍禁止启相。 */
    if (!running() &&
        (state == MOTOR_STATE_CALIBRATION || s_bench.motor->control_mode != MOTOR_CONTROL_NONE) &&
        s_bench.limits_valid && MotorLifecycle_WorkEpoch() != s_bench.run_epoch &&
        (uint32_t)(s_bench.tick - s_bench.heartbeat_tick) <=
            milliseconds_to_ticks(s_bench.limits.lease_ms) &&
        s_bench.run_id != UINT32_MAX)
    {
        MotorCalibrationType kind = MotorLifecycle_GetCalibrationType();
        s_bench.run_kind = state != MOTOR_STATE_CALIBRATION                  ? RUN_REMOTE
                           : kind == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ? RUN_LUT
                           : kind == MOTOR_CALIBRATION_ELECTRICAL_ZERO       ? RUN_ZERO
                                                                             : RUN_MECHANICAL_ZERO;
        s_bench.duration_ticks = milliseconds_to_ticks(s_bench.limits.duration_ms);
        s_bench.warmup_ticks = 0U;
        ++s_bench.run_id;
        if (s_bench.run_kind == RUN_REMOTE)
        {
            s_bench.run_state = RUN_WAIT;
            s_bench.run_epoch = MotorLifecycle_WorkEpoch();
            s_bench.started_tick = s_bench.tick;
            s_bench.reason = WHY_NONE;
        }
        else
        {
            start_tracking();
        }
    }
    if (!running())
    {
        if (state == MOTOR_STATE_CALIBRATION || s_bench.motor->control_mode != MOTOR_CONTROL_NONE)
        {
            if (s_bench.run_epoch != MotorLifecycle_WorkEpoch() || s_bench.run_state == RUN_IDLE)
            {
                stop_run(true, WHY_STATE);
            }
            else
            {
                (void)MotorLifecycle_RequestRemoteStop();
            }
        }
        return;
    }
    if (!s_bench.limits_valid || (uint32_t)(s_bench.tick - s_bench.heartbeat_tick) >
                                     milliseconds_to_ticks(s_bench.limits.lease_ms))
    {
        stop_run(true, WHY_LEASE);
        return;
    }
    if (s_bench.run_state == RUN_WAIT)
    {
        if (state == MOTOR_STATE_RUNNING)
        {
            start_tracking();
        }
        else if (state == MOTOR_STATE_FAULT ||
                 (uint32_t)(s_bench.tick - s_bench.started_tick) >= BENCH_START_TIMEOUT_TICKS)
        {
            stop_run(true, WHY_START_TIMEOUT);
            return;
        }
        else
        {
            return;
        }
    }
    bool calibration = s_bench.run_kind >= RUN_LUT && s_bench.run_kind <= RUN_MECHANICAL_ZERO;
    if (calibration && state == MOTOR_STATE_IDLE && MotorLifecycle_IsStopped())
    {
        s_bench.run_state = s_bench.calibration_completed ? RUN_DONE : RUN_FAILED;
        s_bench.reason = s_bench.calibration_completed ? WHY_NONE : WHY_STATE;
        freeze_capture();
        return;
    }
    if (s_bench.motor->fault_code != MOTOR_FAULT_NONE)
    {
        stop_run(true, WHY_FAULT);
        return;
    }
    if (s_bench.run_epoch != MotorLifecycle_WorkEpoch() ||
        state != (calibration ? MOTOR_STATE_CALIBRATION : MOTOR_STATE_RUNNING))
    {
        stop_run(true, WHY_STATE);
        return;
    }
    float speed = s_bench.encoder->mechanical_velocity_continuous_rad_s;
    const ABCVect *phase = &s_bench.foc->phase_current_a;
    if (!isfinite(speed) || !isfinite(phase->a) || !isfinite(phase->b) || !isfinite(phase->c))
    {
        stop_run(true, WHY_MEASUREMENT);
        return;
    }
    if (!s_bench.encoder->has_valid_sample)
    {
        stop_run(true, WHY_ENCODER);
        return;
    }
    if (fabsf(phase->a) > s_bench.limits.current_a || fabsf(phase->b) > s_bench.limits.current_a ||
        fabsf(phase->c) > s_bench.limits.current_a)
    {
        stop_run(true, WHY_CURRENT);
        return;
    }
    if (fabsf(speed) > s_bench.limits.speed_rad_s)
    {
        stop_run(true, WHY_SPEED);
        return;
    }
    uint16_t angle = s_bench.encoder->directed_q15;
    int32_t delta = (int16_t)(uint16_t)(angle - s_bench.previous_angle);
    s_bench.previous_angle = angle;
    if (!BspPwm_ArePhasesDisabled())
    {
        s_bench.travel_counts += (uint32_t)(delta < 0 ? -delta : delta);
    }
    if ((float)s_bench.travel_counts > s_bench.limits.travel_rad * (65536.0f / CONFIG_TWO_PI))
    {
        stop_run(true, WHY_TRAVEL);
        return;
    }
    uint32_t elapsed = s_bench.tick - s_bench.started_tick;
    uint32_t tail_ticks = s_bench.run_kind == RUN_IQ_PULSE ? milliseconds_to_ticks(5U) : 0U;
    if (elapsed >= s_bench.warmup_ticks + s_bench.duration_ticks + tail_ticks)
    {
        stop_run(false, WHY_DURATION);
        return;
    }
    if (!s_bench.reference_applied && elapsed >= s_bench.warmup_ticks)
    {
        s_bench.reference_applied = true;
        s_bench.step_edge = true;
        s_bench.step_tick = s_bench.tick;
    }
    if (!calibration && s_bench.run_kind != RUN_REMOTE)
    {
        float reference = s_bench.reference_applied ? s_bench.reference : 0.0f;
        if (s_bench.run_kind == RUN_ID_SINE)
        {
            reference = s_bench.sine_bias_a;
            if (s_bench.reference_applied)
            {
                float sine, cosine;
                Math_SinCos(s_bench.sine_phase_rad, &sine, &cosine);
                reference += s_bench.reference * sine;
                s_bench.sine_phase_rad += s_bench.sine_step_rad;
                if (s_bench.sine_phase_rad >= CONFIG_TWO_PI)
                {
                    s_bench.sine_phase_rad -= CONFIG_TWO_PI;
                }
            }
        }
        if (s_bench.run_kind == RUN_IQ_PULSE &&
            elapsed >= s_bench.warmup_ticks + s_bench.duration_ticks)
        {
            reference = 0.0f;
        }
        if (s_bench.run_kind == RUN_SPEED)
        {
            s_bench.motor->speed_target_rad_s = reference;
        }
        else
        {
            s_bench.motor->id_target_a =
                s_bench.run_kind == RUN_ID || s_bench.run_kind == RUN_ID_SINE ? reference : 0.0f;
            s_bench.motor->iq_target_a = s_bench.run_kind == RUN_IQ_PULSE ? reference : 0.0f;
        }
    }
}

static BenchSample read_sample(void)
{
    const FocState *f = s_bench.foc;
    const EncoderState *e = s_bench.encoder;
    const MotorControlState *m = s_bench.motor;
    BenchSample sample = {.tick = s_bench.tick};
    switch (s_bench.group)
    {
    case GROUP_PHASE:
        sample.values[0] = BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_A);
        sample.values[1] = BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_B);
        sample.values[2] = BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_C);
        sample.values[3] = f->phase_current_a.a;
        sample.values[4] = f->phase_current_a.b;
        sample.values[5] = f->phase_current_a.c;
        sample.values[6] = e->electrical_angle_rad;
        break;
    case GROUP_DQ:
        sample.values[0] = m->id_target_a;
        sample.values[1] = m->iq_target_a;
        sample.values[2] = f->current_dq_a.d;
        sample.values[3] = f->current_dq_a.q;
        sample.values[4] = f->voltage_dq_v.d;
        sample.values[5] = f->voltage_dq_v.q;
        sample.values[6] = e->mechanical_velocity_continuous_rad_s;
        break;
    case GROUP_SPEED:
        sample.values[0] = m->speed_target_rad_s;
        sample.values[1] = m->speed_ramped_rad_s;
        sample.values[2] = e->mechanical_velocity_continuous_rad_s;
        sample.values[3] = m->iq_target_a;
        sample.values[4] = f->current_dq_a.q;
        sample.values[5] = f->bus_voltage_v;
        sample.values[6] = e->mechanical_position_rad;
        break;
    case GROUP_SINCOS:
        sample.values[0] = BspAdc_ReadRawChannel(BSP_ADC_OUTPUT_SIN);
        sample.values[1] = BspAdc_ReadRawChannel(BSP_ADC_OUTPUT_COS);
        sample.values[2] = e->mechanical_velocity_continuous_rad_s;
        sample.values[3] = e->linearized_q15;
        sample.values[4] = f->bus_voltage_v;
        sample.values[5] = sample.values[6] = 0.0f;
        break;
    case GROUP_ANGLE:
        sample.values[0] = e->raw_q15;
        sample.values[1] = e->directed_q15;
        sample.values[2] = e->linearized_q15;
        sample.values[3] = s_bench.configuration->model.motor_pole_pairs > 0
                               ? FluxObserver_GetElecPos(s_bench.observer) /
                                     (float)s_bench.configuration->model.motor_pole_pairs
                               : NAN;
        sample.values[4] = e->electrical_angle_rad;
        sample.values[5] = f->current_dq_a.d;
        sample.values[6] = f->current_dq_a.q;
        break;
    default:
        break;
    }
    return sample;
}

void MotorBench_AfterControl(MotorTaskResult outcome)
{
    if (running() && s_bench.run_kind >= RUN_LUT && s_bench.run_kind <= RUN_MECHANICAL_ZERO &&
        outcome.result == MOTOR_TASK_COMPLETED)
    {
        s_bench.calibration_completed = true;
    }
    s_bench.latest = read_sample();
    if (s_bench.capture_state != CAP_WAIT && s_bench.capture_state != CAP_RECORDING)
    {
        return;
    }
    bool trigger =
        s_bench.trigger == 0U ||
        (s_bench.trigger == 1U && s_bench.run_state == RUN_ACTIVE &&
         s_bench.run_id == s_bench.capture_run_id && s_bench.step_edge) ||
        (s_bench.trigger == 2U && s_bench.run_state == RUN_ACTIVE &&
         s_bench.run_id == s_bench.capture_run_id && s_bench.reference_applied &&
         (uint32_t)(s_bench.tick - s_bench.step_tick) >= s_bench.delay_ticks) ||
        (s_bench.trigger == 3U && EncoderCalibrationTask_GetStep() == s_bench.trigger_phase);
    if (s_bench.capture_state == CAP_WAIT && trigger)
    {
        uint16_t pre = s_bench.count < s_bench.pre_samples ? s_bench.count : s_bench.pre_samples;
        s_bench.first_index = (s_bench.write_index + BENCH_SAMPLES - pre) % BENCH_SAMPLES;
        s_bench.count = pre;
        s_bench.capture_state = CAP_RECORDING;
        s_bench.divider = 0U;
    }
    if (s_bench.divider++ != 0U)
    {
        if (s_bench.divider >= s_bench.stride)
        {
            s_bench.divider = 0U;
        }
        return;
    }
    if (s_bench.stride == 1U)
    {
        s_bench.divider = 0U;
    }
    bool valid = true;
    for (uint32_t i = 0U; i < 7U; ++i)
    {
        valid = valid && isfinite(s_bench.latest.values[i]);
    }
    if (!valid && s_bench.invalid_samples != UINT16_MAX)
    {
        ++s_bench.invalid_samples;
    }
    s_bench.samples[s_bench.write_index] = s_bench.latest;
    s_bench.write_index = (s_bench.write_index + 1U) % BENCH_SAMPLES;
    if (s_bench.count < BENCH_SAMPLES)
    {
        ++s_bench.count;
    }
    if (s_bench.capture_state == CAP_RECORDING && s_bench.count == BENCH_SAMPLES)
    {
        s_bench.capture_state = CAP_FROZEN;
    }
}

void MotorBench_CalibrationPublished(bool succeeded)
{
    if (running() && s_bench.run_kind == RUN_LUT)
    {
        s_bench.calibration_completed = succeeded;
    }
}

static uint16_t set_limits(const uint8_t *p)
{
    BenchLimits limits = {read_float(p),
                          read_float(p + 4U),
                          read_float(p + 8U),
                          YgProtocol_ReadU32Le(p + 12U),
                          YgProtocol_ReadU32Le(p + 16U)};
    if (!MotorLifecycle_IsStopped() || running())
    {
        return 6U;
    }
    if (!isfinite(s_bench.configuration->application.current_limit_a) ||
        s_bench.configuration->application.current_limit_a <= 0.0f || !isfinite(limits.current_a) ||
        limits.current_a <= 0.0f ||
        limits.current_a > s_bench.configuration->application.current_limit_a ||
        !isfinite(limits.speed_rad_s) || limits.speed_rad_s <= 0.0f ||
        limits.speed_rad_s > 500.0f || !isfinite(limits.travel_rad) || limits.travel_rad <= 0.0f ||
        limits.travel_rad > 10000.0f || limits.duration_ms == 0U || limits.duration_ms > 120000U ||
        limits.lease_ms < 100U || limits.lease_ms > 1000U)
    {
        return 11U;
    }
    uint32_t mask = BspSystem_EnterCritical();
    s_bench.limits = limits;
    s_bench.heartbeat_tick = s_bench.tick;
    s_bench.limits_valid = true;
    BspSystem_ExitCritical(mask);
    return 0U;
}

static uint16_t request_run(const uint8_t *p, uint32_t length)
{
    uint16_t kind = YgProtocol_ReadU16Le(p);
    bool sine = kind == RUN_ID_SINE;
    if (length != (sine ? 24U : 16U))
    {
        return 4U;
    }
    float reference = read_float(p + 4U);
    uint32_t duration_us = YgProtocol_ReadU32Le(p + 8U);
    uint32_t warmup_us = YgProtocol_ReadU32Le(p + 12U);
    if (!s_bench.limits_valid || !MotorLifecycle_IsStopped() || running())
    {
        return 5U;
    }
    if ((!sine && (kind < RUN_ID || kind > RUN_SPEED)) || YgProtocol_ReadU16Le(p + 2U) != 0U ||
        !isfinite(reference) || duration_us == 0U ||
        duration_us > s_bench.limits.duration_ms * 1000U || warmup_us > 100000U ||
        (uint64_t)duration_us + warmup_us + (kind == RUN_IQ_PULSE ? 5000U : 0U) >
            (uint64_t)s_bench.limits.duration_ms * 1000U ||
        s_bench.run_id == UINT32_MAX || (kind == RUN_IQ_PULSE && duration_us > 5000U) ||
        (kind == RUN_ID && reference < 0.0f) ||
        (kind == RUN_SPEED
             ? !isfinite(s_bench.configuration->application.speed_limit_rad_s) ||
                   s_bench.configuration->application.speed_limit_rad_s <= 0.0f ||
                   fabsf(reference) > s_bench.limits.speed_rad_s ||
                   fabsf(reference) > s_bench.configuration->application.speed_limit_rad_s
             : fabsf(reference) > s_bench.limits.current_a))
    {
        return 11U;
    }
    float frequency_hz = sine ? read_float(p + 16U) : 0.0f;
    float bias_a = sine ? read_float(p + 20U) : 0.0f;
    if (sine && (!isfinite(frequency_hz) || frequency_hz < 1.0f ||
                 frequency_hz > (float)FOC_FREQ / 5.0f || !isfinite(bias_a) || reference <= 0.0f ||
                 bias_a < reference || bias_a + reference > s_bench.limits.current_a))
    {
        return 11U;
    }
    uint32_t mask = BspSystem_EnterCritical();
    ++s_bench.run_id;
    s_bench.run_kind = kind;
    s_bench.reference = reference;
    s_bench.sine_bias_a = bias_a;
    s_bench.sine_phase_rad = 0.0f;
    s_bench.sine_step_rad = CONFIG_TWO_PI * frequency_hz / (float)FOC_FREQ;
    s_bench.duration_ticks = (uint32_t)(((uint64_t)duration_us * FOC_FREQ + 999999U) / 1000000U);
    s_bench.warmup_ticks = (uint32_t)(((uint64_t)warmup_us * FOC_FREQ + 999999U) / 1000000U);
    s_bench.started_tick = s_bench.heartbeat_tick = s_bench.tick;
    s_bench.reference_applied = false;
    s_bench.reason = WHY_NONE;
    s_bench.run_state = RUN_WAIT;
    BspSystem_ExitCritical(mask);
    MotorControlResult result = MotorLifecycle_RequestEnable(
        kind == RUN_SPEED ? MOTOR_CONTROL_SPEED : MOTOR_CONTROL_CURRENT);
    if (result != MOTOR_CONTROL_ACCEPTED)
    {
        s_bench.run_state = RUN_FAILED;
        s_bench.reason = WHY_STATE;
        return result == MOTOR_CONTROL_FAULT ? 17U : 5U;
    }
    return 1U;
}

static uint16_t arm_capture(const uint8_t *p)
{
    uint16_t group = YgProtocol_ReadU16Le(p), stride = YgProtocol_ReadU16Le(p + 2U);
    uint16_t trigger = YgProtocol_ReadU16Le(p + 4U), pre = YgProtocol_ReadU16Le(p + 6U);
    uint32_t delay_ms = YgProtocol_ReadU32Le(p + 8U);
    if (s_bench.capture_state == CAP_WAIT || s_bench.capture_state == CAP_RECORDING)
    {
        return 6U;
    }
    if (group < GROUP_PHASE || group > GROUP_ANGLE || stride == 0U || trigger > 3U ||
        pre > BENCH_SAMPLES / 4U || delay_ms > 120000U || YgProtocol_ReadU16Le(p + 14U) != 0U ||
        s_bench.capture_id == UINT32_MAX ||
        (group == GROUP_SINCOS && stride < FOC_FREQ / SUPERVISOR_FREQ))
    {
        return 11U;
    }
    uint32_t mask = BspSystem_EnterCritical();
    s_bench.group = group;
    s_bench.stride = stride;
    s_bench.trigger = trigger;
    /* 停止时ARM等待下一次RUN；不能用上一次运行遗留的参考更新时间触发新记录。 */
    s_bench.capture_run_id = s_bench.run_id + (running() ? 0U : 1U);
    s_bench.trigger_phase = YgProtocol_ReadU16Le(p + 12U);
    s_bench.pre_samples = pre;
    s_bench.delay_ticks = milliseconds_to_ticks(delay_ms);
    s_bench.count = s_bench.write_index = s_bench.first_index = s_bench.invalid_samples = 0U;
    s_bench.divider = 0U;
    ++s_bench.capture_id;
    s_bench.capture_state = CAP_WAIT;
    BspSystem_ExitCritical(mask);
    return 1U;
}

static void encode_sample(uint8_t *p, const BenchSample *sample)
{
    YgProtocol_WriteU32Le(p, sample->tick);
    for (uint32_t i = 0U; i < 7U; ++i)
    {
        write_float(p + 4U + 4U * i, sample->values[i]);
    }
}

yg_protocol_result_t MotorBench_Handle(void *context,
                                       const yg_protocol_message_t *request,
                                       yg_protocol_service_result_t *result)
{
    (void)context;
    if (result == NULL || !YgProtocol_IsAckUnicastRequest(request) || request->source_node != 2U ||
        request->payload_length < 4U || request->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    uint16_t op = YgProtocol_ReadU16Le(request->payload);
    uint16_t code = 4U;
    uint8_t *p = result->response_payload;
    const uint8_t *data = request->payload + 4U;
    uint32_t length = request->payload_length - 4U;
    bool mutation = op == OP_LIMITS || op == OP_RUN || op == OP_CAPTURE || op == OP_STOP;
    if (mutation && s_bench.last_valid && s_bench.last_sequence == request->sequence)
    {
        if (request->payload_length == s_bench.last_length &&
            memcmp(s_bench.last_request, request->payload, request->payload_length) == 0)
        {
            memcpy(p, s_bench.last_reply, s_bench.last_reply_length);
            result->response_payload_length = s_bench.last_reply_length;
            return YG_PROTOCOL_OK;
        }
        YgProtocol_WriteU16Le(p, code);
        YgProtocol_WriteU16Le(p + 2U, op);
        result->response_payload_length = 4U;
        return YG_PROTOCOL_OK;
    }
    if (mutation && s_bench.last_valid &&
        (int16_t)(uint16_t)(request->sequence - s_bench.last_sequence) <= 0)
    {
        YgProtocol_WriteU16Le(p, 5U);
        YgProtocol_WriteU16Le(p + 2U, op);
        result->response_payload_length = 4U;
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = 4U;
    if (YgProtocol_ReadU16Le(request->payload + 2U) == BENCH_VERSION)
    {
        if (op == OP_INFO && length == 0U)
        {
            uint32_t mask = BspSystem_EnterCritical();
            YgProtocol_WriteU32Le(p + 4U, BENCH_MAGIC);
            YgProtocol_WriteU32Le(p + 8U, s_bench.tick);
            YgProtocol_WriteU32Le(p + 12U, s_bench.run_id);
            YgProtocol_WriteU32Le(p + 16U, s_bench.capture_id);
            YgProtocol_WriteU16Le(p + 20U, s_bench.run_state);
            YgProtocol_WriteU16Le(p + 22U, s_bench.capture_state);
            YgProtocol_WriteU16Le(p + 24U, s_bench.run_kind);
            YgProtocol_WriteU16Le(p + 26U, s_bench.reason);
            YgProtocol_WriteU16Le(p + 28U, s_bench.count);
            YgProtocol_WriteU16Le(p + 30U, s_bench.group);
            YgProtocol_WriteU16Le(p + 32U, s_bench.invalid_samples);
            YgProtocol_WriteU16Le(p + 34U, s_bench.configuration->device.encoder.calib_flag);
            YgProtocol_WriteU16Le(p + 36U,
                                  s_bench.configuration->device.encoder.electrical_zero_q15);
            YgProtocol_WriteU16Le(p + 38U, (uint16_t)s_bench.configuration->model.motor_pole_pairs);
            YgProtocol_WriteU16Le(p + 40U, (uint16_t)s_bench.motor->fault_code);
            YgProtocol_WriteU16Le(
                p + 42U,
                (uint16_t)((s_bench.limits_valid ? 1U : 0U) |
                           (s_bench.configuration->device.encoder.reverse ? 2U : 0U) |
                           (BspPwm_ArePhasesDisabled() ? 4U : 0U) |
                           (s_bench.encoder->has_valid_sample ? 8U : 0U) |
                           (s_bench.last_valid ? 16U : 0U) |
                           ((uint16_t)EncoderCalibrationTask_GetStep() << 8U)));
            YgProtocol_WriteU16Le(p + 44U, s_bench.last_sequence);
            BspSystem_ExitCritical(mask);
            result->response_payload_length = 46U;
            code = 0U;
        }
        else if (op == OP_LIMITS && length == 20U)
        {
            code = set_limits(data);
        }
        else if (op == OP_RUN && (length == 16U || length == 24U))
        {
            code = request_run(data, length);
            YgProtocol_WriteU32Le(p + 4U, s_bench.run_id);
            result->response_payload_length = 8U;
        }
        else if (op == OP_CAPTURE && length == 16U)
        {
            code = arm_capture(data);
            YgProtocol_WriteU32Le(p + 4U, s_bench.capture_id);
            result->response_payload_length = 8U;
        }
        else if (op == OP_READ && length == 8U)
        {
            uint16_t index = YgProtocol_ReadU16Le(data + 4U);
            code = 6U;
            if (s_bench.capture_state == CAP_FROZEN && MotorLifecycle_IsStopped() &&
                YgProtocol_ReadU32Le(data) == s_bench.capture_id && index < s_bench.count &&
                YgProtocol_ReadU16Le(data + 6U) == 0U)
            {
                YgProtocol_WriteU32Le(p + 4U, s_bench.capture_id);
                YgProtocol_WriteU16Le(p + 8U, index);
                YgProtocol_WriteU16Le(p + 10U, s_bench.count);
                encode_sample(p + 12U,
                              &s_bench.samples[(s_bench.first_index + index) % BENCH_SAMPLES]);
                result->response_payload_length = 44U;
                code = 0U;
            }
        }
        else if (op == OP_KEEPALIVE && length == 4U)
        {
            code = YgProtocol_ReadU32Le(data) == s_bench.run_id ? 0U : 5U;
            if (code == 0U)
            {
                s_bench.heartbeat_tick = s_bench.tick;
            }
        }
        else if (op == OP_STOP && length == 0U)
        {
            uint32_t mask = BspSystem_EnterCritical();
            stop_run(false, WHY_CANCEL);
            s_bench.limits_valid = false;
            BspSystem_ExitCritical(mask);
            code = 0U;
        }
        else if (op == OP_LUT && length == 4U)
        {
            uint16_t index = YgProtocol_ReadU16Le(data), count = YgProtocol_ReadU16Le(data + 2U);
            code = 6U;
            if (MotorLifecycle_IsStopped() && count > 0U && count <= 16U &&
                (uint32_t)index + count <= ENCODER_OFFSET_LUT_SIZE)
            {
                YgProtocol_WriteU16Le(p + 4U, index);
                YgProtocol_WriteU16Le(p + 6U, count);
                for (uint16_t i = 0U; i < count; ++i)
                {
                    YgProtocol_WriteU16Le(p + 8U + 2U * i,
                                          (uint16_t)s_bench.configuration->device.encoder
                                              .linearization_lut_q15[index + i]);
                }
                result->response_payload_length = 8U + 2U * count;
                code = 0U;
            }
        }
        else if (op == OP_SNAPSHOT && length == 0U)
        {
            uint32_t mask = BspSystem_EnterCritical();
            encode_sample(p + 4U, &s_bench.latest);
            YgProtocol_WriteU16Le(p + 36U, s_bench.group);
            YgProtocol_WriteU16Le(p + 38U, s_bench.run_state);
            YgProtocol_WriteU16Le(p + 40U, (uint16_t)s_bench.motor->fault_code);
            YgProtocol_WriteU16Le(p + 42U, s_bench.configuration->device.encoder.calib_flag);
            BspSystem_ExitCritical(mask);
            result->response_payload_length = 44U;
            code = 0U;
        }
    }
    YgProtocol_WriteU16Le(p, code);
    YgProtocol_WriteU16Le(p + 2U, op);
    if (mutation && request->payload_length <= sizeof(s_bench.last_request))
    {
        s_bench.last_sequence = request->sequence;
        s_bench.last_length = (uint16_t)request->payload_length;
        memcpy(s_bench.last_request, request->payload, request->payload_length);
        memcpy(s_bench.last_reply, p, result->response_payload_length);
        s_bench.last_reply_length = result->response_payload_length;
        s_bench.last_valid = true;
    }
    return YG_PROTOCOL_OK;
}
