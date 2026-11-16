#include "hardware_check.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_system.h"
#include <math.h>
#include <string.h>

/* 单位：三相与母线均用 1/4 ADC cnt；角轨迹用机械圈，时间用 DWT 核心周期。 */
#define STATIC_SAMPLES (FOC_FREQ / 10U)
#define QUEUE_SIZE HARDWARE_CHECK_QUEUE_SIZE
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
    ABORTED,
    RESISTANCE
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

/* 前台拥有报告与缓存；ADC单写队列，维护区在检测结束后允许复用。 */
static HardwareCheckState *s_state;
static ProtocolHardwareReport s_report;
static PhaseResistanceReport s_resistance;
static uint32_t s_previous_run;
static uint16_t s_start_sequence;
static uint8_t s_mode;
static struct
{
    float values[11][4];
    uint32_t samples[11];
    uint8_t valid[11];
} s_measurements;
static void freeze_measurements(void);
static volatile uint32_t s_write, s_read;
static volatile bool s_overrun;
volatile bool g_hardware_check_capture;

void HardwareCheck_CaptureFast(MotorAxis *axis)
{
    uint32_t write = s_write;
    if (write - s_read >= QUEUE_SIZE)
    {
        s_overrun = true;
        return;
    }
    HardwareCheckSample value = {0};
    HardwareCheckSample *sample = &value;
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
    sample->stopped = !sample->powered && sample->epoch != s_state->base_epoch &&
                      MotorStateMachine_IsStopped(&axis->state_machine);
    if (s_state->hall_installed)
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
    axis->calibration.storage.hardware_check.queue[write % QUEUE_SIZE] = value;
    BspSystem_MemoryBarrier();
    s_write = write + 1U;
}

static void end_check(MotorAxis *axis, uint8_t stage)
{
    if (s_report.stage == RESISTANCE && stage == ABORTED && s_resistance.failure == 0U)
    {
        s_resistance.failure = s_report.items[VBUS] == FAIL ? 7U : 5U;
    }
    g_hardware_check_capture = false;
    freeze_measurements();
    BspSystem_MemoryBarrier();
    axis->state_machine.maintenance_capture = false;
    s_report.stage = stage;
    if (s_state->resistance_epoch != 0U && axis->state_machine.epoch == s_state->resistance_epoch &&
        !MotorStateMachine_IsStopped(&axis->state_machine))
    {
        MotorStateMachine_RequestStop(&axis->state_machine);
    }
    if (s_state->driven_epoch != 0U && axis->state_machine.epoch == s_state->driven_epoch &&
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
        *report = s_report;
        return PROTOCOL_RESULT_OK;
    }
    if (s_report.run_id != 0U && expected_run == s_previous_run && sequence == s_start_sequence &&
        mode == s_mode)
    {
        *report = s_report;
        return PROTOCOL_RESULT_ACCEPTED;
    }
    if (expected_run != s_report.run_id || expected_run == UINT32_MAX)
    {
        return PROTOCOL_RESULT_DENIED;
    }
    if (g_hardware_check_capture || axis->state_machine.maintenance_capture ||
        axis->state_machine.calibration_type != MOTOR_CALIBRATION_NONE ||
        !MotorStateMachine_IsStopped(&axis->state_machine) ||
        axis->state_machine.parameters_updating || axis->state_machine.starts_locked)
    {
        return PROTOCOL_RESULT_BUSY;
    }
    memset(s_state, 0, sizeof(*s_state));
    memset(&s_report, 0, sizeof(s_report));
    memset(&s_resistance, 0, sizeof(s_resistance));
    memset(&s_measurements, 0, sizeof(s_measurements));
    s_report.run_id = expected_run + 1U;
    s_previous_run = expected_run;
    s_start_sequence = sequence;
    s_mode = mode;
    s_state->base_epoch = axis->state_machine.epoch;
    s_state->revision = axis->parameters.revision;
    s_state->stage_ms = BspSystem_TimeMs();
    s_report.stage = STATIC;
    s_state->ntc_window = PASS;
    s_state->min_v = INFINITY;
    s_state->max_v = -INFINITY;
    s_state->minimum_temperature_c = INFINITY;
    s_state->maximum_temperature_c = -INFINITY;
    s_state->hall_min[0] = s_state->hall_min[1] = UINT16_MAX;
    s_state->hall_installed = MotorAxisPreset_HasLinearHall(axis->configuration.preset);
    if (s_state->hall_installed)
    {
        /* 配置与 LUT 仅在停机前台更新。外环独占的 val 由本工作区每次重新计算。 */
        memcpy(&s_state->hall, &axis->linear_hall, offsetof(LinearHall, val));
        s_state->hall_calibrated =
            s_state->hall.calibration_success &&
            (s_state->hall.lut_status & LINEAR_HALL_LUT_OK) != 0U &&
            LinearHall_CoefficientsInRange(s_state->hall.sin_off_counts, s_state->hall.sin_amp) &&
            LinearHall_CoefficientsInRange(s_state->hall.cos_off_counts, s_state->hall.cos_amp);
    }
    else if (axis->configuration.preset != NULL)
    {
        s_report.items[HALL_STATIC] = s_report.items[HALL_DRIVE] = ABSENT;
    }
    s_read = s_write;
    s_overrun = false;
    BspSystem_MemoryBarrier();
    axis->state_machine.maintenance_capture = true;
    BspSystem_MemoryBarrier();
    g_hardware_check_capture = true;
    *report = s_report;
    return PROTOCOL_RESULT_ACCEPTED;
}

/* 只有同一主站受理的模式 10、同一 127 启动序号才将本检查绑定到运动代际。 */
static void observe_start(void *context, uint8_t mode, uint16_t sequence)
{
    MotorAxis *axis = context;
    if (s_report.stage == ARMED && mode == 10U && sequence == s_start_sequence &&
        axis->state_machine.epoch == s_state->base_epoch + 1U)
    {
        s_state->driven_epoch = axis->state_machine.epoch;
    }
}

/* 仅在采集停止后读取；所有数值来自当前轮，读取不改变检测判据和统计。 */
static void
read_noise(const HardwareCheckNoiseWindow *noise, uint32_t samples, float *mean, float *rms)
{
    int64_t n = samples;
    int64_t sum = noise->sum;
    int64_t variance = n * noise->squares - sum * sum;
    *mean = 2048.0f + (float)sum / (4.0f * (float)n);
    *rms = sqrtf((float)variance) / (4.0f * (float)n);
}

static ProtocolCode build_measurement(void *context,
                                      uint8_t item,
                                      uint32_t expected_run,
                                      ProtocolHardwareMeasurement *measurement)
{
    (void)context;
    if (item > 10U)
    {
        return PROTOCOL_RESULT_BAD_FIELD;
    }
    if (expected_run == 0U || expected_run != s_report.run_id)
    {
        return PROTOCOL_RESULT_DENIED;
    }
    if (g_hardware_check_capture)
    {
        return PROTOCOL_RESULT_BUSY;
    }
    *measurement =
        (ProtocolHardwareMeasurement){.run_id = s_report.run_id, .values = {NAN, NAN, NAN, NAN}};
    if (item <= IBUS || item == HALL_STATIC)
    {
        measurement->samples = s_state->static_samples;
        if (s_state->static_samples != 0U && (item <= IBUS || s_state->hall_installed))
        {
            unsigned channel = item <= IBUS ? item : 4U;
            read_noise(&s_state->noise[channel],
                       s_state->static_samples,
                       &measurement->values[0],
                       &measurement->values[1]);
            if (item == HALL_STATIC)
            {
                read_noise(&s_state->noise[5],
                           s_state->static_samples,
                           &measurement->values[2],
                           &measurement->values[3]);
            }
        }
    }
    else if (item == VBUS && s_state->voltage_samples != 0U)
    {
        measurement->samples = s_state->voltage_samples;
        measurement->values[0] = s_state->min_v;
        measurement->values[1] = s_state->max_v;
        measurement->values[2] = (float)s_state->voltage_outside_samples;
        measurement->values[3] = (float)s_state->voltage_invalid_samples;
    }
    else if (item == NTC && s_state->temperature_samples != 0U)
    {
        measurement->samples = s_state->temperature_samples;
        measurement->values[0] = s_state->temperature_c;
        measurement->values[1] = s_state->minimum_temperature_c;
        measurement->values[2] = s_state->maximum_temperature_c;
    }
    else if (item == ROTOR_STATIC && s_state->static_samples != 0U && s_state->seen_frame)
    {
        measurement->samples = s_state->static_samples;
        measurement->values[0] =
            (float)(s_state->rotor_max - s_state->rotor_min) * 360.0f / ROTOR_COUNTS;
    }
    else if (item == ROTOR_DRIVE || (item == HALL_DRIVE && s_state->hall_calibrated))
    {
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            const HardwareCheckAngleTrack *track = &s_state->legs[leg][item - ROTOR_DRIVE];
            measurement->samples += track->samples;
            if (track->samples != 0U)
            {
                measurement->values[leg] = track->travel_turns * 360.0f;
            }
            uint32_t windows = track->forward_windows + track->backward_windows;
            if (windows != 0U)
            {
                uint32_t agreed =
                    track->travel_turns > 0.0f ? track->forward_windows : track->backward_windows;
                measurement->values[leg + 2U] = (float)agreed / (float)windows;
            }
        }
    }
    else if (item == 10U && s_state->hall_installed && s_state->hall_min[0] != UINT16_MAX)
    {
        measurement->samples = s_state->drive_samples;
        measurement->values[0] = (float)s_state->hall_min[0];
        measurement->values[1] = (float)s_state->hall_max[0];
        measurement->values[2] = (float)s_state->hall_min[1];
        measurement->values[3] = (float)s_state->hall_max[1];
    }
    for (unsigned i = 0U; i < 4U; ++i)
    {
        if (isfinite(measurement->values[i]))
        {
            measurement->valid |= (uint8_t)(1U << i);
        }
        else
        {
            measurement->values[i] = 0.0f;
        }
    }
    return PROTOCOL_RESULT_OK;
}

static void freeze_measurements(void)
{
    for (uint8_t item = 0U; item < 11U; ++item)
    {
        ProtocolHardwareMeasurement value;
        (void)build_measurement(NULL, item, s_report.run_id, &value);
        memcpy(s_measurements.values[item], value.values, sizeof(value.values));
        s_measurements.samples[item] = value.samples;
        s_measurements.valid[item] = value.valid;
    }
}

static ProtocolCode read_measurement(void *context,
                                     uint8_t item,
                                     uint32_t expected_run,
                                     ProtocolHardwareMeasurement *measurement)
{
    (void)context;
    if (item > 10U)
    {
        return PROTOCOL_RESULT_BAD_FIELD;
    }
    if (expected_run == 0U || expected_run != s_report.run_id)
    {
        return PROTOCOL_RESULT_DENIED;
    }
    if (g_hardware_check_capture)
    {
        return PROTOCOL_RESULT_BUSY;
    }
    measurement->run_id = expected_run;
    measurement->samples = s_measurements.samples[item];
    measurement->valid = s_measurements.valid[item];
    memcpy(measurement->values, s_measurements.values[item], sizeof(measurement->values));
    return PROTOCOL_RESULT_OK;
}

static ProtocolCode
read_resistance(void *context, uint32_t expected_run, ProtocolHardwareResistance *measurement)
{
    (void)context;
    if (g_hardware_check_capture || expected_run == 0U || expected_run != s_report.run_id ||
        s_mode != 3U)
    {
        return PROTOCOL_RESULT_DENIED;
    }
    const PhaseResistanceReport *result = &s_resistance;
    *measurement = (ProtocolHardwareResistance){.run_id = expected_run,
                                                .result = result->result,
                                                .open_phases = result->open_phases,
                                                .low_phases = result->low_phases,
                                                .high_phases = result->high_phases,
                                                .open_lines = result->open_lines,
                                                .failure = result->failure,
                                                .valid = result->valid};
    measurement->values[0] = result->nominal_ohm;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        measurement->values[1U + i] = result->phase_ohm[i];
        measurement->values[4U + i] = result->line_ohm[i];
    }
    return PROTOCOL_RESULT_OK;
}

ProtocolHardwareCheck HardwareCheck_Bind(MotorAxis *axis)
{
    s_state = &axis->calibration.storage.hardware_check.state;
    return (ProtocolHardwareCheck){.context = axis,
                                   .handle = handle_request,
                                   .started = observe_start,
                                   .measure = read_measurement,
                                   .resistance = read_resistance};
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

static void accumulate(HardwareCheckNoiseWindow *window, int32_t centered_q4)
{
    window->sum += centered_q4;
    window->squares += centered_q4 * centered_q4;
}

static bool
noise_passes(HardwareCheckNoiseWindow *window, int32_t rms_limit_counts, bool check_bias)
{
    int64_t n = STATIC_SAMPLES;
    int64_t sum = window->sum;
    int64_t variance = n * window->squares - sum * sum;
    window->mean_counts = 2048.0f + (float)sum / (4.0f * (float)n);
    window->rms_counts = sqrtf((float)variance) / (4.0f * (float)n);
    int64_t rms_limit_q4 = 4 * n * rms_limit_counts;
    return variance < rms_limit_q4 * rms_limit_q4 &&
           (!check_bias || (sum > -1200 * n && sum < 1200 * n));
}

static float hall_turns(const HardwareCheckSample *sample)
{
    LinearHall *hall = &s_state->hall;
    LinearHall_Update(hall, sample->hall[0], sample->hall[1]);
    if (!isfinite(hall->val.sin_cal) || !isfinite(hall->val.cos_cal) ||
        (hall->val.sin_cal == 0.0f && hall->val.cos_cal == 0.0f))
    {
        return NAN;
    }
    return hall->val.angle_rad / TWO_PI_F;
}

/* 单圈角用相邻最短差展开；记录两种方向的最大逆行量，避免把返回原位误作静止。 */
static void track_angle(HardwareCheckAngleTrack *track, float turns, uint32_t cycles)
{
    if (!isfinite(turns))
    {
        return;
    }
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
}

/* 原码先跨零展开，再用完整 50ms 均值更新趋势；返回 -1 无效、0 待窗、1 新趋势点。
 * 原始单拍增量不再用作动态失败条件，raw 中仍保留原始轨迹供诊断。 */
static int track_trend(HardwareCheckAngleWindow *window,
                       HardwareCheckAngleTrack *trend,
                       float turns,
                       uint32_t cycles,
                       float static_margin)
{
    if (!isfinite(turns))
    {
        return -1;
    }
    track_angle(&window->raw, turns, cycles);
    if (window->samples == 0U)
    {
        window->first_cycles = cycles;
    }
    ++window->samples;
    window->mean_turns += (window->raw.travel_turns - window->mean_turns) / (float)window->samples;
    if (cycles - window->first_cycles < AT32_CORE_HZ / 20U)
    {
        return 0;
    }
    float previous = trend->travel_turns;
    track_angle(trend, window->mean_turns, cycles);
    float delta = trend->travel_turns - previous;
    if (delta > static_margin)
    {
        ++trend->forward_windows;
    }
    else if (delta < -static_margin)
    {
        ++trend->backward_windows;
    }
    window->mean_turns = 0.0f;
    window->samples = 0U;
    return 1;
}

static void static_sample(const HardwareCheckSample *sample)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        accumulate(&s_state->noise[i], (int32_t)sample->current_q4[i] - 8192);
    }
    int32_t angle = (int32_t)(sample->frame >> 11U);
    if (!frame_valid(sample->frame))
    {
        s_report.items[ROTOR_STATIC] = FAIL;
    }
    if (sample->frame != 0U)
    {
        s_state->seen_frame = true;
    }
    if (s_state->static_samples != 0U)
    {
        int32_t delta = angle - s_state->last_rotor;
        if (delta > ROTOR_COUNTS / 2)
        {
            delta -= ROTOR_COUNTS;
        }
        else if (delta < -ROTOR_COUNTS / 2)
        {
            delta += ROTOR_COUNTS;
        }
        s_state->rotor_travel += delta;
        if (s_state->rotor_travel < s_state->rotor_min)
        {
            s_state->rotor_min = s_state->rotor_travel;
        }
        if (s_state->rotor_travel > s_state->rotor_max)
        {
            s_state->rotor_max = s_state->rotor_travel;
        }
    }
    s_state->last_rotor = angle;
    if (s_state->hall_installed)
    {
        for (unsigned i = 0U; i < 2U; ++i)
        {
            accumulate(&s_state->noise[4U + i], 4 * ((int32_t)sample->hall[i] - 2048));
        }
        if (s_state->hall_calibrated && s_state->static_samples % (FOC_FREQ / 1000U) == 0U)
        {
            track_angle(&s_state->hall_rest, hall_turns(sample), sample->cycles);
        }
    }
    ++s_state->static_samples;
}

static uint8_t temperature_result(const MotorAxis *axis)
{
    uint32_t saved = BspSystem_EnterControlCritical();
    MotorTemperatureTelemetry temperature = axis->motor_temperature;
    uint32_t now = BspSystem_TimeMs();
    BspSystem_ExitControlCritical(saved);
    s_state->temperature_c = temperature.celsius;
    if (isfinite(temperature.celsius))
    {
        s_state->minimum_temperature_c = fminf(s_state->minimum_temperature_c, temperature.celsius);
        s_state->maximum_temperature_c = fmaxf(s_state->maximum_temperature_c, temperature.celsius);
        ++s_state->temperature_samples;
    }
    if (!temperature.valid || !isfinite(temperature.celsius) || now - temperature.sample_ms > 30U)
    {
        return FAIL;
    }
    return temperature.celsius >= 0.0f && temperature.celsius <= 60.0f ? PASS : UNTESTED;
}

static void finish_static(MotorAxis *axis)
{
    uint8_t *items = s_report.items;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        items[i] = noise_passes(&s_state->noise[i], i == IBUS ? 50 : 10, true) ? PASS : FAIL;
    }
    if (items[VBUS] != FAIL)
    {
        items[VBUS] = PASS;
    }
    int32_t span = s_state->rotor_max - s_state->rotor_min;
    if (items[ROTOR_STATIC] != FAIL)
    {
        items[ROTOR_STATIC] = span > 58 ? FAIL : (s_state->seen_frame ? PASS : UNTESTED);
    }
    s_state->jitter_turns[0] = (float)(span + 2) / ROTOR_COUNTS;
    if (s_state->hall_installed)
    {
        bool sin_ok = noise_passes(&s_state->noise[4], 10, false);
        bool cos_ok = noise_passes(&s_state->noise[5], 10, false);
        items[HALL_STATIC] = sin_ok && cos_ok ? PASS : FAIL;
        /* ADC 量化及本点实测峰峰角共同构成动态裕量。 */
        s_state->jitter_turns[1] =
            s_state->hall_rest.max_turns - s_state->hall_rest.min_turns +
            2.0f / (TWO_PI_F * fmaxf(1.0f, fminf(s_state->hall.sin_amp, s_state->hall.cos_amp)));
    }
    items[NTC] = s_state->ntc_window;
    if (s_mode == 1U || items[NTC] != PASS)
    {
        end_check(axis, items[NTC] == UNTESTED || s_mode != 1U ? ABORTED : DONE);
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
    if (s_mode == 3U)
    {
        s_resistance.nominal_ohm = axis->configuration.model.motor_phase_resistance_ohm;
        if (MotorStateMachine_RequestCalibration(
                &axis->state_machine, MOTOR_CALIBRATION_PHASE_RESISTANCE) != MOTOR_CONTROL_ACCEPTED)
        {
            s_resistance.failure = 8U;
            end_check(axis, ABORTED);
            return;
        }
        s_state->resistance_epoch = axis->state_machine.epoch;
        s_report.stage = RESISTANCE;
    }
    else
    {
        s_report.stage = ARMED;
    }
    s_state->stage_ms = BspSystem_TimeMs();
}

static uint8_t measure_direction(const HardwareCheckAngleTrack *angle, float margin)
{
    if (angle->samples < 3U)
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
        uint8_t *item = &s_report.items[ROTOR_DRIVE + sensor];
        if (*item == ABSENT || (sensor == 1U && !s_state->hall_calibrated))
        {
            continue;
        }
        uint8_t directions[2];
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            const HardwareCheckAngleTrack *track = &s_state->legs[leg][sensor];
            float margin = s_state->jitter_turns[sensor];
            directions[leg] = measure_direction(track, margin);
            s_report.directions |= (uint8_t)(directions[leg] << (4U * sensor + 2U * leg));
            uint32_t forward = track->forward_windows;
            uint32_t backward = track->backward_windows;
            uint32_t agreed = directions[leg] == 1U ? forward : backward;
            unsigned expected = (s_state->first_sign > 0) != (leg != 0U) ? 1U : 2U;
            if (sensor == 0U && directions[leg] != 0U && directions[leg] != 3U &&
                directions[leg] != expected)
            {
                *item = FAIL;
            }
            if (directions[leg] != 0U && directions[leg] != 3U &&
                agreed * 3U < (forward + backward) * 2U)
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
    /* 电机角必须与三相开环命令同向；霍尔只验证自身趋势，不作安装方向关联验收。 */
    end_check(axis, DONE);
}

/* 预转至少 500ms，且连续两个 50ms 窗口的机械速度幅值均在目标的 ±20% 内，
 * 才开始本段轨迹验证。
 * 安装方向由后续独立旋向判定处理；未稳定只等待，不累计动态通过证据。 */
static bool rotation_settled(float rotor, uint32_t cycles, float frequency_hz, unsigned pole_pairs)
{
    HardwareCheckAngleTrack *warmup = &s_state->warmup;
    track_angle(warmup, rotor, cycles);
    uint32_t elapsed = cycles - warmup->first_cycles;
    if (elapsed < AT32_CORE_HZ / 20U)
    {
        return false;
    }
    float expected =
        fabsf(frequency_hz) * (float)elapsed / ((float)AT32_CORE_HZ * (float)pole_pairs);
    if (fabsf(fabsf(warmup->travel_turns) - expected) <= 0.2f * expected)
    {
        ++s_state->stable_windows;
    }
    else
    {
        s_state->stable_windows = 0U;
    }
    memset(warmup, 0, sizeof(*warmup));
    return s_state->stable_windows >= 2U &&
           cycles - s_state->warmup_begin_cycles >= AT32_CORE_HZ / 2U;
}

static void drive_sample(MotorAxis *axis, const HardwareCheckSample *sample)
{
    uint8_t *items = s_report.items;
    bool invalid_sample = false;
    if (!frame_valid(sample->frame))
    {
        items[ROTOR_DRIVE] = FAIL;
        invalid_sample = true;
    }
    if (s_state->hall_installed)
    {
        for (unsigned i = 0U; i < 2U; ++i)
        {
            uint16_t raw = sample->hall[i];
            if (raw < s_state->hall_min[i])
            {
                s_state->hall_min[i] = raw;
            }
            if (raw > s_state->hall_max[i])
            {
                s_state->hall_max[i] = raw;
            }
            if (raw < 95U || raw > 4000U)
            {
                items[HALL_DRIVE] = FAIL;
                invalid_sample = true;
            }
        }
    }
    if (invalid_sample)
    {
        end_check(axis, ABORTED);
        return;
    }
    int sign = (sample->frequency_hz > 0.0f) - (sample->frequency_hz < 0.0f);
    bool hall_tick = s_state->drive_samples++ % (FOC_FREQ / 1000U) == 0U;
    float rotor = (float)(sample->frame >> 11U) / ROTOR_COUNTS;
    if (axis->configuration.device.encoder.reverse != 0U)
    {
        rotor = -rotor;
    }
    float hall = hall_tick && s_state->hall_calibrated ? hall_turns(sample) : NAN;
    if (sign == 0)
    {
        if (s_state->warmup_frequency_hz != 0.0f)
        {
            memset(s_state->path, 0, sizeof(s_state->path));
            memset(s_state->settle, 0, sizeof(s_state->settle));
            s_state->settled_samples = 0U;
        }
        s_state->warmup_frequency_hz = 0.0f;
        if (s_state->first_sign == 0)
        {
            return;
        }
    }
    else
    {
        if (s_state->first_sign == 0)
        {
            s_state->first_sign = sign;
        }
        if (sign != s_state->first_sign && s_state->leg == 0U)
        {
            if (s_state->settled_samples < FOC_FREQ / 25U)
            {
                end_check(axis, ABORTED);
                return;
            }
            s_state->leg = 1U;
        }
        if (s_state->leg == 1U && sign == s_state->first_sign)
        {
            end_check(axis, ABORTED);
            return;
        }
        if (sign == s_state->first_sign && s_state->settled_samples != 0U)
        {
            memset(s_state->settle, 0, sizeof(s_state->settle));
            s_state->settled_samples = 0U;
        }
        if (sample->frequency_hz != s_state->warmup_frequency_hz)
        {
            s_state->warmup_frequency_hz = sample->frequency_hz;
            memset(&s_state->warmup, 0, sizeof(s_state->warmup));
            memset(s_state->path, 0, sizeof(s_state->path));
            memset(s_state->legs[s_state->leg], 0, sizeof(s_state->legs[s_state->leg]));
            s_state->leg_samples[s_state->leg] = 0U;
            s_state->stable_windows = 0U;
            s_state->rotation_ready = false;
            s_state->warmup_begin_cycles = sample->cycles;
        }
        if (!s_state->rotation_ready)
        {
            s_state->rotation_ready = rotation_settled(rotor,
                                                       sample->cycles,
                                                       sample->frequency_hz,
                                                       axis->configuration.model.motor_pole_pairs);
            if (!s_state->rotation_ready)
            {
                return;
            }
        }
    }
    /* 每个传感器独立观察 50ms 均值趋势，不对单拍速度或反向抖动判失败。 */
    HardwareCheckAngleTrack *tracks = sign == 0 ? s_state->settle : s_state->legs[s_state->leg];
    int updated[2] = {
        track_trend(&s_state->path[0], &tracks[0], rotor, sample->cycles, s_state->jitter_turns[0]),
        0};
    if (hall_tick && s_state->hall_calibrated)
    {
        updated[1] = track_trend(
            &s_state->path[1], &tracks[1], hall, sample->cycles, s_state->jitter_turns[1]);
    }
    for (unsigned sensor = 0U; sensor < 2U; ++sensor)
    {
        if (updated[sensor] < 0)
        {
            items[ROTOR_DRIVE + sensor] = FAIL;
            end_check(axis, ABORTED);
            return;
        }
    }
    if (sign == 0)
    {
        if (s_state->first_sign != 0 && s_state->leg == 0U)
        {
            if (s_state->settle[0].max_turns - s_state->settle[0].min_turns >
                s_state->jitter_turns[0])
            {
                memset(s_state->settle, 0, sizeof(s_state->settle));
                s_state->settled_samples = 0U;
            }
            else if (updated[0] == 1 && s_state->settle[0].samples >= 2U)
            {
                s_state->settled_samples += FOC_FREQ / 20U;
            }
        }
    }
    else
    {
        ++s_state->leg_samples[s_state->leg];
        float speed =
            fabsf(sample->frequency_hz) / (float)axis->configuration.model.motor_pole_pairs;
        for (unsigned sensor = 0U; sensor < 2U; ++sensor)
        {
            if (updated[sensor] != 1)
            {
                continue;
            }
            HardwareCheckAngleTrack *track = &tracks[sensor];
            float margin = s_state->jitter_turns[sensor];
            float axis_speed =
                sensor == 0U ? speed : speed / axis->configuration.preset->gear_ratio;
            float elapsed_s = (float)(sample->cycles - track->last_progress) / (float)AT32_CORE_HZ;
            if (track->moving && elapsed_s > 0.15f + 8.0f * margin / axis_speed)
            {
                items[ROTOR_DRIVE + sensor] = FAIL;
            }
            if (track->samples == 1U)
            {
                track->last_progress = sample->cycles;
            }
            else if (fabsf(track->travel_turns - track->progress_turns) > margin)
            {
                /* 同一非零目标下，已经运动的轨迹长时间保持后再次跃进。慢轴按
                   此段速度和静态噪声拓宽观测时间，首次起动延迟单独处理。 */
                track->last_progress = sample->cycles;
                track->progress_turns = track->travel_turns;
                track->moving = true;
            }
        }
    }
    /* 趋势异常只锁存本项失败，继续取得其他项证据；原码/故障等保护仍立即中止。 */
}

static void consume_sample(MotorAxis *axis, const HardwareCheckSample *sample)
{
    if (s_state->have_cycles && (sample->cycles - s_state->last_cycles > PERIOD_CYCLES * 3U / 2U ||
                                 sample->cycles == s_state->last_cycles))
    {
        end_check(axis, ABORTED);
        return;
    }
    s_state->last_cycles = sample->cycles;
    s_state->have_cycles = true;
    ++s_state->voltage_samples;
    s_state->min_v = fminf(s_state->min_v, sample->voltage_v);
    s_state->max_v = fmaxf(s_state->max_v, sample->voltage_v);
    bool supply_ok = sample->voltage_v >= 27.0f && sample->voltage_v <= 29.0f;
    if (!supply_ok)
    {
        ++s_state->voltage_outside_samples;
        if (!isfinite(sample->voltage_v))
        {
            ++s_state->voltage_invalid_samples;
        }
        s_report.items[VBUS] = FAIL;
    }
    if (s_report.stage == STATIC)
    {
        if (sample->powered || sample->epoch != s_state->base_epoch)
        {
            end_check(axis, ABORTED);
            return;
        }
        static_sample(sample);
        if (s_state->static_samples == STATIC_SAMPLES)
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
    if (s_report.stage == RESISTANCE)
    {
        return;
    }
    if (s_report.stage == ARMED && !sample->powered && sample->epoch == s_state->base_epoch)
    {
        return;
    }
    if (sample->epoch != s_state->driven_epoch || !sample->powered || !sample->open_current)
    {
        if (s_report.stage == ARMED && s_state->driven_epoch != 0U &&
            sample->epoch == s_state->driven_epoch && !sample->powered)
        {
            return;
        }
        if (s_report.stage == DRIVING && !sample->powered && !sample->fault &&
            s_state->leg_samples[0] >= FOC_FREQ * 3U / 20U &&
            s_state->leg_samples[1] >= FOC_FREQ * 3U / 20U && sample->stopped &&
            sample->epoch == s_state->driven_epoch + 1U)
        {
            finish_drive(axis);
        }
        else if (s_report.stage == DRIVING && sample->stopping &&
                 sample->epoch == s_state->driven_epoch)
        {
            return;
        }
        else
        {
            end_check(axis, ABORTED);
        }
        return;
    }
    if (s_report.stage == ARMED)
    {
        s_report.stage = DRIVING;
        s_state->stage_ms = BspSystem_TimeMs();
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
    uint8_t stage = s_report.stage;
    uint32_t timeout_ms =
        stage == STATIC ? 500U : (stage == ARMED ? 3000U : (stage == RESISTANCE ? 25000U : 8000U));
    if (s_overrun || axis->parameters.revision != s_state->revision ||
        now - s_state->stage_ms > timeout_ms)
    {
        end_check(axis, ABORTED);
        return;
    }
    uint8_t ntc = temperature_result(axis);
    if (ntc != PASS)
    {
        /* 只在整窗完成后发布正常。无效或过期读数可立即证实故障。 */
        if (s_state->ntc_window != FAIL)
        {
            s_state->ntc_window = ntc;
        }
        if (stage != STATIC || ntc == FAIL)
        {
            s_report.items[NTC] = ntc;
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
        HardwareCheckSample sample =
            axis->calibration.storage.hardware_check.queue[s_read % QUEUE_SIZE];
        BspSystem_MemoryBarrier();
        ++s_read;
        consume_sample(axis, &sample);
    }
    if (g_hardware_check_capture && s_report.stage == RESISTANCE &&
        MotorStateMachine_IsStopped(&axis->state_machine))
    {
        if (axis->calibration.resistance_epoch == s_state->resistance_epoch)
        {
            s_resistance = axis->calibration.workspace.resistance.report;
        }
        if (axis->state_machine.calibration_result_epoch != s_state->resistance_epoch ||
            axis->state_machine.calibration_result != MOTOR_CALIBRATION_RESULT_COMPLETED)
        {
            s_resistance.failure = 5U;
        }
        if (s_resistance.result != 1U || s_resistance.failure != 0U)
        {
            end_check(axis, ABORTED);
        }
        else
        {
            s_state->base_epoch = axis->state_machine.epoch;
            s_report.stage = ARMED;
            s_state->stage_ms = BspSystem_TimeMs();
        }
    }
}
