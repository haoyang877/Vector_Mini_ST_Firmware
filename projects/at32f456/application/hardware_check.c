#include "hardware_check.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_system.h"
#include <math.h>
#include <string.h>

/* 单位：三相与母线均用 1/4 ADC cnt；角轨迹用机械圈，时间用 DWT 核心周期。 */
#define STATIC_SAMPLES (FOC_FREQ / 10U)
#define QUEUE_SIZE 128U
#define ROTOR_COUNTS 2097152
#define TWO_PI_F 6.28318530718f
#define PERIOD_CYCLES (AT32_CORE_HZ / FOC_FREQ)

enum
{
    IDLE,
    STATIC,
    ARMED,
    DRIVING,
    DONE,
    ABORTED
};
enum
{
    UNTESTED,
    PASS,
    FAIL,
    ABSENT
};
enum
{
    IA,
    IB,
    IC,
    IBUS,
    VBUS,
    NTC,
    ROTOR_STATIC,
    HALL_STATIC,
    ROTOR_DRIVE,
    HALL_DRIVE
};

typedef struct
{
    uint32_t cycles, frame, epoch;
    float voltage_v, frequency_hz;
    uint16_t current_q4[4], hall[2];
    bool powered, open_current, fault, stopping;
} HardwareSample;

typedef struct
{
    int32_t sum;
    int64_t squares;
    float mean_counts, rms_counts;
} NoiseWindow;

typedef struct
{
    uint32_t samples, first_cycles, last_cycles;
    float last_turns, travel_turns, min_turns, max_turns;
    float backward_turns, forward_turns;
    uint32_t last_progress;
    float progress_turns;
    bool moving;
} AngleTrack;

typedef struct
{
    ProtocolHardwareReport report;
    uint32_t previous_run, base_epoch, driven_epoch, stage_ms, last_cycles, samples, revision;
    uint16_t start_sequence;
    uint8_t mode;
    bool hall_installed, hall_calibrated, seen_frame, have_cycles;
    int32_t last_rotor, rotor_travel, rotor_min, rotor_max;
    NoiseWindow noise[6];
    float min_v, max_v, temperature_c, jitter_turns[2];
    uint16_t hall_min[2], hall_max[2];
    LinearHall hall;
    AngleTrack hall_rest, legs[2][2], settle[2], path[2];
    float last_speed_turns_s;
    int first_sign;
    unsigned leg, settled_samples;
    uint32_t leg_samples[2];
} HardwareCheckState;

/* 前台独占统计和报告，Watch 可直接读取本轮 s_check。ADC 单写 write，前台单写 read。 */
static HardwareCheckState s_check;
static HardwareSample s_queue[QUEUE_SIZE];
static volatile uint32_t s_write, s_read;
static volatile bool s_overrun;
volatile bool g_hardware_check_capture;

void HardwareCheck_CaptureFast(const MotorAxis *axis)
{
    uint32_t write = s_write;
    if (write - s_read >= QUEUE_SIZE)
    {
        s_overrun = true;
        return;
    }
    HardwareSample *sample = &s_queue[write % QUEUE_SIZE];
    sample->cycles = BspSystem_CycleCount();
    sample->frame = BspEncoder_ReadCompletedFrame();
    sample->voltage_v = axis->foc.bus_voltage_v;
    sample->epoch = axis->state_machine.epoch;
    sample->powered = !axis->hardware->power.phases_disabled();
    sample->open_current = axis->motor.control_mode == MOTOR_CONTROL_OPEN_CURRENT &&
                           axis->state_machine.motion_lease_active;
    sample->frequency_hz = axis->motor.open_loop_frequency_hz;
    sample->fault = axis->motor.fault_code != MOTOR_FAULT_NONE;
    sample->stopping = axis->state_machine.stop_pending;
    if (s_check.hall_installed)
    {
        uint8_t slot = axis->hall_published_slot;
        sample->hall[0] = axis->hall_samples[slot].sin_counts;
        sample->hall[1] = axis->hall_samples[slot].cos_counts;
    }
    if (!sample->powered)
    {
        sample->current_q4[0] = (uint16_t)(4.0f * BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_A));
        sample->current_q4[1] = (uint16_t)(4.0f * BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_B));
        sample->current_q4[2] = (uint16_t)(4.0f * BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_C));
        sample->current_q4[3] = (uint16_t)(4.0f * BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT));
    }
    BspSystem_MemoryBarrier();
    s_write = write + 1U;
}

static void end_check(MotorAxis *axis, uint8_t stage)
{
    g_hardware_check_capture = false;
    s_check.report.stage = stage;
    if (s_check.driven_epoch != 0U && axis->state_machine.epoch == s_check.driven_epoch &&
        !MotorStateMachine_IsStopped(&axis->state_machine))
    {
        MotorStateMachine_RequestStop(&axis->state_machine);
    }
}

static ProtocolCode handle_request(void *context,
                                   uint8_t mode,
                                   uint16_t sequence,
                                   uint32_t expected_run,
                                   ProtocolHardwareReport *report)
{
    MotorAxis *axis = context;
    if (mode == 0U)
    {
        *report = s_check.report;
        return PROTOCOL_RESULT_OK;
    }
    if (s_check.report.run_id != 0U && expected_run == s_check.previous_run &&
        sequence == s_check.start_sequence && mode == s_check.mode)
    {
        *report = s_check.report;
        return PROTOCOL_RESULT_ACCEPTED;
    }
    if (expected_run != s_check.report.run_id || expected_run == UINT32_MAX)
    {
        return PROTOCOL_RESULT_DENIED;
    }
    if (g_hardware_check_capture || !MotorStateMachine_IsStopped(&axis->state_machine) ||
        axis->state_machine.parameters_updating || axis->state_machine.starts_locked)
    {
        return PROTOCOL_RESULT_BUSY;
    }
    memset(&s_check, 0, sizeof(s_check));
    s_check.report.run_id = expected_run + 1U;
    s_check.previous_run = expected_run;
    s_check.start_sequence = sequence;
    s_check.mode = mode;
    s_check.base_epoch = axis->state_machine.epoch;
    s_check.revision = axis->parameters.revision;
    s_check.stage_ms = BspSystem_TimeMs();
    s_check.report.stage = STATIC;
    s_check.min_v = INFINITY;
    s_check.max_v = -INFINITY;
    s_check.hall_min[0] = s_check.hall_min[1] = UINT16_MAX;
    s_check.hall_installed = MotorAxisPreset_HasLinearHall(axis->configuration.preset);
    if (s_check.hall_installed)
    {
        /* 配置与 LUT 仅在停机前台更新。外环独占的 val 由本工作区每次重新计算。 */
        memcpy(&s_check.hall, &axis->linear_hall, offsetof(LinearHall, val));
        s_check.hall_calibrated =
            s_check.hall.calibration_success &&
            (s_check.hall.lut_status & LINEAR_HALL_LUT_OK) != 0U &&
            LinearHall_CoefficientsInRange(s_check.hall.sin_off_counts, s_check.hall.sin_amp) &&
            LinearHall_CoefficientsInRange(s_check.hall.cos_off_counts, s_check.hall.cos_amp);
    }
    else if (axis->configuration.preset != NULL)
    {
        s_check.report.items[HALL_STATIC] = s_check.report.items[HALL_DRIVE] = ABSENT;
    }
    s_read = s_write;
    s_overrun = false;
    BspSystem_MemoryBarrier();
    g_hardware_check_capture = true;
    *report = s_check.report;
    return PROTOCOL_RESULT_ACCEPTED;
}

/* 只有同一主站受理的模式 10、同一 127 启动序号才将本检查绑定到运动代际。 */
static void observe_start(void *context, uint8_t mode, uint16_t sequence)
{
    MotorAxis *axis = context;
    if (s_check.report.stage == ARMED && mode == 10U && sequence == s_check.start_sequence &&
        axis->state_machine.epoch == s_check.base_epoch + 1U)
    {
        s_check.driven_epoch = axis->state_machine.epoch;
    }
}

ProtocolHardwareCheck HardwareCheck_Bind(MotorAxis *axis)
{
    return (ProtocolHardwareCheck){
        .context = axis, .handle = handle_request, .started = observe_start};
}

/* MT6835 连续读：从 ANGLE[20] 到 STATUS[0] 的三个字节，CRC-8 0x07，初值 0。 */
static bool frame_valid(uint32_t frame)
{
    static const uint8_t s_crc_nibble[16] = {
        0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45};
    uint8_t crc = 0U;
    for (int shift = 28; shift >= 8; shift -= 4)
    {
        crc = (uint8_t)((crc << 4U) ^ s_crc_nibble[(crc >> 4U) ^ ((frame >> shift) & 15U)]);
    }
    return crc == (uint8_t)frame && (frame & 0x700U) == 0U;
}

static void accumulate(NoiseWindow *window, int32_t centered_q4)
{
    window->sum += centered_q4;
    window->squares += centered_q4 * centered_q4;
}

static bool noise_passes(NoiseWindow *window, bool check_bias)
{
    int64_t n = STATIC_SAMPLES;
    int64_t sum = window->sum;
    int64_t variance = n * window->squares - sum * sum;
    window->mean_counts = 2048.0f + (float)sum / (4.0f * (float)n);
    window->rms_counts = sqrtf((float)variance) / (4.0f * (float)n);
    return variance < (40 * n) * (40 * n) && (!check_bias || (sum > -1200 * n && sum < 1200 * n));
}

static float hall_turns(const HardwareSample *sample)
{
    LinearHall *hall = &s_check.hall;
    LinearHall_Update(hall, sample->hall[0], sample->hall[1]);
    if (!isfinite(hall->val.sin_cal) || !isfinite(hall->val.cos_cal) ||
        (hall->val.sin_cal == 0.0f && hall->val.cos_cal == 0.0f))
    {
        return NAN;
    }
    return hall->val.angle_rad / TWO_PI_F;
}

/* 单圈角用相邻最短差展开；记录两种方向的最大逆行量，避免把返回原位误作静止。 */
static bool track_angle(
    AngleTrack *track, float turns, uint32_t cycles, float speed_turns_s, float margin_turns)
{
    if (!isfinite(turns))
    {
        return false;
    }
    bool plausible = true;
    if (track->samples == 0U)
    {
        track->first_cycles = cycles;
    }
    else
    {
        float delta = turns - track->last_turns;
        if (delta > 0.5f)
        {
            delta -= 1.0f;
        }
        else if (delta < -0.5f)
        {
            delta += 1.0f;
        }
        float dt = (float)(cycles - track->last_cycles) / (float)AT32_CORE_HZ;
        plausible = fabsf(delta) <= speed_turns_s * dt + margin_turns;
        track->travel_turns += delta;
        track->backward_turns =
            fmaxf(track->backward_turns, track->max_turns - track->travel_turns);
        track->forward_turns = fmaxf(track->forward_turns, track->travel_turns - track->min_turns);
        track->min_turns = fminf(track->min_turns, track->travel_turns);
        track->max_turns = fmaxf(track->max_turns, track->travel_turns);
    }
    ++track->samples;
    track->last_turns = turns;
    track->last_cycles = cycles;
    return plausible;
}

static void static_sample(const HardwareSample *sample)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        accumulate(&s_check.noise[i], (int32_t)sample->current_q4[i] - 8192);
    }
    int32_t angle = (int32_t)(sample->frame >> 11U);
    if (!frame_valid(sample->frame))
    {
        s_check.report.items[ROTOR_STATIC] = FAIL;
    }
    if (sample->frame != 0U)
    {
        s_check.seen_frame = true;
    }
    if (s_check.samples != 0U)
    {
        int32_t delta = angle - s_check.last_rotor;
        if (delta > ROTOR_COUNTS / 2)
        {
            delta -= ROTOR_COUNTS;
        }
        else if (delta < -ROTOR_COUNTS / 2)
        {
            delta += ROTOR_COUNTS;
        }
        s_check.rotor_travel += delta;
        if (s_check.rotor_travel < s_check.rotor_min)
        {
            s_check.rotor_min = s_check.rotor_travel;
        }
        if (s_check.rotor_travel > s_check.rotor_max)
        {
            s_check.rotor_max = s_check.rotor_travel;
        }
    }
    s_check.last_rotor = angle;
    if (s_check.hall_installed)
    {
        for (unsigned i = 0U; i < 2U; ++i)
        {
            accumulate(&s_check.noise[4U + i], 4 * ((int32_t)sample->hall[i] - 2048));
        }
        if (s_check.hall_calibrated && s_check.samples % (FOC_FREQ / 1000U) == 0U)
        {
            (void)track_angle(&s_check.hall_rest, hall_turns(sample), sample->cycles, 0.0f, 1.0f);
        }
    }
    ++s_check.samples;
}

static uint8_t temperature_result(const MotorAxis *axis)
{
    uint32_t saved = BspSystem_EnterControlCritical();
    MotorTemperatureTelemetry temperature = axis->motor_temperature;
    uint32_t now = BspSystem_TimeMs();
    BspSystem_ExitControlCritical(saved);
    s_check.temperature_c = temperature.celsius;
    if (!temperature.valid || !isfinite(temperature.celsius) || now - temperature.sample_ms > 30U)
    {
        return FAIL;
    }
    return temperature.celsius >= 0.0f && temperature.celsius <= 60.0f ? PASS : UNTESTED;
}

static void finish_static(MotorAxis *axis)
{
    uint8_t *items = s_check.report.items;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        items[i] = noise_passes(&s_check.noise[i], true) ? PASS : FAIL;
    }
    if (items[VBUS] != FAIL)
    {
        items[VBUS] = PASS;
    }
    int32_t span = s_check.rotor_max - s_check.rotor_min;
    if (items[ROTOR_STATIC] != FAIL)
    {
        items[ROTOR_STATIC] = span > 58 ? FAIL : (s_check.seen_frame ? PASS : UNTESTED);
    }
    s_check.jitter_turns[0] = (float)(span + 2) / ROTOR_COUNTS;
    if (s_check.hall_installed)
    {
        bool sin_ok = noise_passes(&s_check.noise[4], false);
        bool cos_ok = noise_passes(&s_check.noise[5], false);
        items[HALL_STATIC] = sin_ok && cos_ok ? PASS : FAIL;
        /* ADC 量化及本点实测峰峰角共同构成动态裕量。 */
        s_check.jitter_turns[1] =
            s_check.hall_rest.max_turns - s_check.hall_rest.min_turns +
            2.0f / (TWO_PI_F * fmaxf(1.0f, fminf(s_check.hall.sin_amp, s_check.hall.cos_amp)));
    }
    uint8_t ntc = temperature_result(axis);
    if (items[NTC] == PASS)
    {
        items[NTC] = ntc;
    }
    if (s_check.mode == 1U || items[NTC] != PASS)
    {
        end_check(axis, items[NTC] == UNTESTED || s_check.mode == 2U ? ABORTED : DONE);
        return;
    }
    for (unsigned i = IA; i <= HALL_STATIC; ++i)
    {
        if (items[i] != PASS && items[i] != ABSENT)
        {
            end_check(axis, ABORTED);
            return;
        }
    }
    s_check.report.stage = ARMED;
    s_check.stage_ms = BspSystem_TimeMs();
}

static uint8_t measure_direction(const AngleTrack *angle, float margin)
{
    if (angle->samples < 2U)
    {
        return 0U;
    }
    if (fabsf(angle->travel_turns) <= margin)
    {
        return 3U;
    }
    return angle->travel_turns > 0.0f ? 1U : 2U;
}

static void finish_drive(MotorAxis *axis)
{
    for (unsigned sensor = 0U; sensor < 2U; ++sensor)
    {
        uint8_t *item = &s_check.report.items[ROTOR_DRIVE + sensor];
        if (*item == ABSENT || (sensor == 1U && !s_check.hall_calibrated))
        {
            continue;
        }
        uint8_t directions[2];
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            const AngleTrack *track = &s_check.legs[leg][sensor];
            float margin = s_check.jitter_turns[sensor];
            directions[leg] = measure_direction(track, margin);
            s_check.report.directions |= (uint8_t)(directions[leg] << (4U * sensor + 2U * leg));
            if (track->backward_turns > margin && track->forward_turns > margin)
            {
                *item = FAIL;
            }
        }
        if (*item != FAIL && directions[0] != 0U && directions[1] != 0U)
        {
            if (directions[0] != 3U && directions[1] != 3U)
            {
                *item = directions[0] != directions[1] ? PASS : FAIL;
            }
        }
    }
    /* 已标定的机械传动方向把独立 Hall 和电机编码器变换到同一坐标。
       此处不在检测中估计或翻转方向；不一致归入输出轴动态关系项。 */
    const OutputEncoderParams *relation = &axis->output_encoder.params;
    if (s_check.report.items[ROTOR_DRIVE] == PASS && s_check.report.items[HALL_DRIVE] == PASS &&
        (relation->status & OUTPUT_ENCODER_CAL_OK) != 0U &&
        (relation->direction == 1 || relation->direction == -1))
    {
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            unsigned motor = (s_check.report.directions >> (2U * leg)) & 3U;
            unsigned hall = (s_check.report.directions >> (4U + 2U * leg)) & 3U;
            if ((motor == hall) != (relation->direction == 1))
            {
                s_check.report.items[HALL_DRIVE] = FAIL;
            }
        }
    }
    end_check(axis, DONE);
}

static void drive_sample(MotorAxis *axis, const HardwareSample *sample)
{
    uint8_t *items = s_check.report.items;
    if (!frame_valid(sample->frame))
    {
        items[ROTOR_DRIVE] = FAIL;
    }
    if (s_check.hall_installed)
    {
        for (unsigned i = 0U; i < 2U; ++i)
        {
            uint16_t raw = sample->hall[i];
            if (raw < s_check.hall_min[i])
            {
                s_check.hall_min[i] = raw;
            }
            if (raw > s_check.hall_max[i])
            {
                s_check.hall_max[i] = raw;
            }
            if (raw < 95U || raw > 4000U)
            {
                items[HALL_DRIVE] = FAIL;
            }
        }
    }
    int sign = (sample->frequency_hz > 0.0f) - (sample->frequency_hz < 0.0f);
    bool hall_tick = s_check.samples++ % (FOC_FREQ / 1000U) == 0U;
    float rotor = (float)(sample->frame >> 11U) / ROTOR_COUNTS;
    if (axis->configuration.device.encoder.reverse != 0U)
    {
        rotor = -rotor;
    }
    float hall = hall_tick && s_check.hall_calibrated ? hall_turns(sample) : NAN;
    /* 跨越启动、零频保持和反转的相邻角也必须连续。零频允许上一段的减速余量。 */
    float speed = fminf(axis->configuration.application.speed_limit_rad_s / TWO_PI_F,
                        2.0f * fabsf(sample->frequency_hz) /
                            (float)axis->configuration.model.motor_pole_pairs);
    float transition_speed = fmaxf(s_check.last_speed_turns_s, speed);
    if (sign != 0)
    {
        s_check.last_speed_turns_s = speed;
    }
    if (!track_angle(
            &s_check.path[0], rotor, sample->cycles, transition_speed, s_check.jitter_turns[0]))
    {
        items[ROTOR_DRIVE] = FAIL;
    }
    if (hall_tick && s_check.hall_calibrated &&
        !track_angle(&s_check.path[1],
                     hall,
                     sample->cycles,
                     transition_speed / axis->configuration.preset->gear_ratio,
                     s_check.jitter_turns[1]))
    {
        items[HALL_DRIVE] = FAIL;
    }
    if (sign == 0)
    {
        if (s_check.first_sign != 0 && s_check.leg == 0U)
        {
            (void)track_angle(&s_check.settle[0], rotor, sample->cycles, 0.0f, 1.0f);
            if (hall_tick && s_check.hall_calibrated)
            {
                (void)track_angle(&s_check.settle[1], hall, sample->cycles, 0.0f, 1.0f);
            }
            if (s_check.settle[0].max_turns - s_check.settle[0].min_turns >
                    s_check.jitter_turns[0] ||
                s_check.settle[1].max_turns - s_check.settle[1].min_turns > s_check.jitter_turns[1])
            {
                memset(s_check.settle, 0, sizeof(s_check.settle));
                s_check.settled_samples = 0U;
            }
            else
            {
                ++s_check.settled_samples;
            }
        }
    }
    else
    {
        if (s_check.first_sign == 0)
        {
            s_check.first_sign = sign;
        }
        if (sign != s_check.first_sign && s_check.leg == 0U)
        {
            if (s_check.settled_samples < FOC_FREQ / 25U)
            {
                end_check(axis, ABORTED);
                return;
            }
            s_check.leg = 1U;
        }
        if (s_check.leg == 1U && sign == s_check.first_sign)
        {
            end_check(axis, ABORTED);
            return;
        }
        if (sign == s_check.first_sign && s_check.settled_samples != 0U)
        {
            memset(s_check.settle, 0, sizeof(s_check.settle));
            s_check.settled_samples = 0U;
        }
        ++s_check.leg_samples[s_check.leg];
        AngleTrack *tracks = s_check.legs[s_check.leg];
        for (unsigned sensor = 0U; sensor < 2U; ++sensor)
        {
            if (sensor == 1U && (!s_check.hall_calibrated || !hall_tick))
            {
                continue;
            }
            AngleTrack *track = &tracks[sensor];
            float margin = s_check.jitter_turns[sensor];
            float axis_speed =
                sensor == 0U ? speed : speed / axis->configuration.preset->gear_ratio;
            (void)track_angle(
                track, sensor == 0U ? rotor : hall, sample->cycles, axis_speed, margin);
            if (track->samples == 1U)
            {
                track->last_progress = sample->cycles;
            }
            else if (fabsf(track->travel_turns - track->progress_turns) > margin)
            {
                /* 同一非零目标下，已经运动的轨迹长时间保持后再次跃进。慢轴按
                   此段速度和静态噪声拓宽观测时间，首次起动延迟单独处理。 */
                float elapsed_s =
                    (float)(sample->cycles - track->last_progress) / (float)AT32_CORE_HZ;
                if (track->moving && elapsed_s > 0.15f + 8.0f * margin / axis_speed)
                {
                    items[ROTOR_DRIVE + sensor] = FAIL;
                }
                track->last_progress = sample->cycles;
                track->progress_turns = track->travel_turns;
                track->moving = true;
            }
            if (track->backward_turns > margin && track->forward_turns > margin)
            {
                items[ROTOR_DRIVE + sensor] = FAIL;
            }
        }
    }
    if (items[ROTOR_DRIVE] == FAIL || items[HALL_DRIVE] == FAIL)
    {
        end_check(axis, ABORTED);
    }
}

static void consume_sample(MotorAxis *axis, const HardwareSample *sample)
{
    if (s_check.have_cycles && (sample->cycles - s_check.last_cycles > PERIOD_CYCLES * 3U / 2U ||
                                sample->cycles == s_check.last_cycles))
    {
        end_check(axis, ABORTED);
        return;
    }
    s_check.last_cycles = sample->cycles;
    s_check.have_cycles = true;
    s_check.min_v = fminf(s_check.min_v, sample->voltage_v);
    s_check.max_v = fmaxf(s_check.max_v, sample->voltage_v);
    bool supply_ok = sample->voltage_v >= 27.0f && sample->voltage_v <= 29.0f;
    if (!supply_ok)
    {
        s_check.report.items[VBUS] = FAIL;
    }
    if (s_check.report.stage == STATIC)
    {
        if (sample->powered || sample->epoch != s_check.base_epoch)
        {
            end_check(axis, ABORTED);
            return;
        }
        static_sample(sample);
        if (s_check.samples == STATIC_SAMPLES)
        {
            finish_static(axis);
        }
        return;
    }
    if (!supply_ok || sample->fault)
    {
        end_check(axis, ABORTED);
        return;
    }
    if (s_check.report.stage == ARMED && !sample->powered && sample->epoch == s_check.base_epoch)
    {
        return;
    }
    if (sample->epoch != s_check.driven_epoch || !sample->powered || !sample->open_current)
    {
        if (s_check.report.stage == ARMED && s_check.driven_epoch != 0U &&
            sample->epoch == s_check.driven_epoch && !sample->powered)
        {
            return;
        }
        if (s_check.report.stage == DRIVING && !sample->powered && !sample->fault &&
            s_check.leg_samples[0] >= FOC_FREQ / 20U && s_check.leg_samples[1] >= FOC_FREQ / 20U &&
            MotorStateMachine_IsStopped(&axis->state_machine))
        {
            finish_drive(axis);
        }
        else if (s_check.report.stage == DRIVING && sample->stopping &&
                 sample->epoch == s_check.driven_epoch)
        {
            return;
        }
        else
        {
            end_check(axis, ABORTED);
        }
        return;
    }
    if (s_check.report.stage == ARMED)
    {
        s_check.report.stage = DRIVING;
        s_check.stage_ms = BspSystem_TimeMs();
    }
    drive_sample(axis, sample);
}

void HardwareCheck_Process(MotorAxis *axis)
{
    if (!g_hardware_check_capture)
    {
        return;
    }
    uint32_t now = BspSystem_TimeMs();
    uint8_t stage = s_check.report.stage;
    uint32_t timeout_ms = stage == STATIC ? 500U : (stage == ARMED ? 3000U : 8000U);
    if (s_overrun || axis->parameters.revision != s_check.revision ||
        now - s_check.stage_ms > timeout_ms)
    {
        end_check(axis, ABORTED);
        return;
    }
    uint8_t ntc = temperature_result(axis);
    if (ntc != PASS || s_check.samples == 0U)
    {
        /* 无效、过期优先于未知环境，保留整窗已知异常。 */
        if (s_check.report.items[NTC] != FAIL)
        {
            s_check.report.items[NTC] = ntc;
        }
    }
    if (stage != STATIC && ntc != PASS)
    {
        end_check(axis, ABORTED);
        return;
    }
    uint32_t end = s_write;
    BspSystem_MemoryBarrier();
    while (s_read != end && g_hardware_check_capture)
    {
        HardwareSample sample = s_queue[s_read % QUEUE_SIZE];
        BspSystem_MemoryBarrier();
        ++s_read;
        consume_sample(axis, &sample);
    }
}
