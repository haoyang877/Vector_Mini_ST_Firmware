#include "virtual_platform.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "app.h"
#include "board_hw.h"
#include "control_config.h"
#include "critical_hw.h"
#include "encoder_sensor.h"
#include "indicator_hw.h"
#include "motor_hw.h"
#include "motor_sensing.h"
#include "param_store.h"
#include "power_stage_hw.h"
#include "time_hw.h"
#include "vector_mini_st_g431.h"
#include "yg_protocol_canfd_irq.h"
#include "inverter_model.h"

/* 静态或最小物理对象替身；所有APP状态、校正和控制均由原生产源码推进。 */
#define QUEUE_CAPACITY 32U
#if (1000000U % FOC_FREQ) != 0U || (FOC_FREQ % SUPERVISOR_FREQ) != 0U
#error "Virtual time requires integer microseconds and an integer supervisor divider"
#endif
typedef struct
{
    CommHwCanFrame frames[QUEUE_CAPACITY];
    uint64_t tickets[QUEUE_CAPACITY];
    unsigned head, count;
} FrameQueue;

static VirtualConfig s_config;
static VirtualSnapshot s_snapshot;
static FrameQueue s_rx, s_tx;
static bool s_initialized, s_board_started, s_can_started, s_encoder_started;
static uint32_t s_critical_depth;
static bool s_critical_error;
static uint8_t s_storage[TARGET_RAW_BYTES];
static uint32_t s_storage_bytes;
static uint64_t s_ticket, s_ticket_started_us;
static CommHwCanTxCompletion s_completion;
static bool s_completion_ready;

bool VirtualPlatform_LoadParameters(const void *record, uint32_t size_bytes)
{
    if (s_initialized || record == NULL || size_bytes == 0U || size_bytes > sizeof(s_storage))
    {
        return false;
    }
    memcpy(s_storage, record, size_bytes);
    s_storage_bytes = size_bytes;
    return true;
}

static bool enqueue(FrameQueue *queue, const CommHwCanFrame *frame, uint64_t ticket)
{
    if (frame == NULL || frame->length > 64U || queue->count == QUEUE_CAPACITY)
    {
        return false;
    }
    unsigned index = (queue->head + queue->count) % QUEUE_CAPACITY;
    queue->frames[index] = *frame;
    queue->tickets[index] = ticket;
    ++queue->count;
    return true;
}

static bool dequeue(FrameQueue *queue, CommHwCanFrame *frame, uint64_t *ticket)
{
    if (frame == NULL || queue->count == 0U)
    {
        return false;
    }
    *frame = queue->frames[queue->head];
    *ticket = queue->tickets[queue->head];
    queue->head = (queue->head + 1U) % QUEUE_CAPACITY;
    --queue->count;
    return true;
}

bool VirtualPlatform_Init(const VirtualConfig *config)
{
    if (s_initialized || config == NULL || !isfinite(config->bus_voltage_v) ||
        config->bus_voltage_v < 0.0f ||
        config->bus_voltage_v >
            MOTOR_SENSING_VBUS_V_PER_COUNT * CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS ||
        !isfinite(config->temperature_c))
    {
        return false;
    }
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        if (config->phase_offset_counts[phase] > CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            return false;
        }
    }
    if (config->motor_connected &&
        (!MotorModel_Valid(&config->motor) || config->integration_substeps == 0U ||
         config->integration_substeps > 100U))
    {
        return false;
    }
    s_config = *config;
    s_snapshot.rotor.angle_rad = (float)config->encoder_angle_q15 * (6.283185307f / 65536.0f);
    s_snapshot.phases_disabled = true;
    s_initialized = true;
    App_Init();
    return true;
}

bool VirtualPlatform_Advance(uint32_t fast_ticks)
{
    if (!s_initialized || !s_board_started || s_critical_depth != 0U || s_critical_error)
    {
        return false;
    }
    for (uint32_t tick = 0U; tick < fast_ticks; ++tick)
    {
        if (s_config.motor_connected)
        {
            float voltage_v[3];
            if (!InverterModel_Apply(s_snapshot.duty,
                                     s_config.bus_voltage_v,
                                     !s_snapshot.phases_disabled,
                                     voltage_v))
            {
                power_stage_hw_stop();
                return false;
            }
            for (unsigned step = 0U; step < s_config.integration_substeps; ++step)
            {
                if (!MotorModel_Step(&s_config.motor,
                                     &s_snapshot.rotor,
                                     voltage_v,
                                     !s_snapshot.phases_disabled,
                                     1.0f /
                                         ((float)FOC_FREQ * (float)s_config.integration_substeps)))
                {
                    power_stage_hw_stop();
                    return false;
                }
            }
            MotorModel_PhaseCurrents(
                &s_config.motor, &s_snapshot.rotor, s_snapshot.phase_current_a);
            for (unsigned phase = 0U; phase < 3U; ++phase)
            {
                s_snapshot.peak_phase_current_a = fmaxf(s_snapshot.peak_phase_current_a,
                                                        fabsf(s_snapshot.phase_current_a[phase]));
            }
        }
        s_snapshot.time_us += 1000000U / FOC_FREQ;
        if (s_rx.count != 0U && s_can_started)
        {
            YgProtocolCanfd_RxIrqHandler();
        }
        App_FastTick();
        ++s_snapshot.fast_ticks;
        if (s_snapshot.fast_ticks % (FOC_FREQ / SUPERVISOR_FREQ) == 0U)
        {
            App_SupervisorTick();
            ++s_snapshot.supervisor_ticks;
        }
        App_Process();
        if ((!s_config.motor_connected && s_snapshot.power_was_enabled) ||
            s_snapshot.sample_saturated || s_critical_depth != 0U || s_critical_error)
        {
            power_stage_hw_stop();
            return false;
        }
    }
    return true;
}

VirtualSnapshot VirtualPlatform_Read(void)
{
    return s_snapshot;
}

bool VirtualPlatform_Receive(const CommHwCanFrame *frame)
{
    return s_initialized && enqueue(&s_rx, frame, 0U);
}

bool VirtualPlatform_TakeTx(CommHwCanFrame *frame)
{
    uint64_t ticket;
    if (!dequeue(&s_tx, frame, &ticket))
    {
        return false;
    }
    if (ticket != 0U && ticket == s_ticket)
    {
        s_completion = (CommHwCanTxCompletion){ticket, true};
        s_completion_ready = true;
    }
    return true;
}

void board_hw_start(bool motor_phases_enabled)
{
    s_board_started = true;
    if (motor_phases_enabled)
    {
        power_stage_hw_start();
    }
}

bool board_hw_read_uid(uint8_t uid[BOARD_HW_UID_BYTES])
{
    if (uid == NULL)
    {
        return false;
    }
    /* 固定虚拟身份，与实机UID无关。 */
    memcpy(uid, "VIRTUAL00001", BOARD_HW_UID_BYTES);
    return true;
}

bool board_hw_read_identity(uint8_t record[64])
{
    if (record == NULL)
    {
        return false;
    }
    memset(record, 0xFF, 64U);
    return true; /* 空白生产区，由真实解码器判断无效。 */
}

uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase)
{
    if ((unsigned)phase >= 3U)
    {
        return 0U;
    }
    /* 电路放大极性与原Current_Cal一致；只编码传感器，绝不写控制器电流。 */
    float counts = (float)s_config.phase_offset_counts[phase] -
                   s_snapshot.phase_current_a[phase] / MOTOR_SENSING_CURRENT_A_PER_COUNT;
    if (!isfinite(counts) || counts < 0.0f || counts > CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
    {
        s_snapshot.sample_saturated = true;
        return 0U;
    }
    return (uint16_t)(lroundf(counts) * MOTOR_SENSING_ADC_SAMPLE_COUNT);
}

uint16_t motor_hw_vbus_sample_raw(void)
{
    return (uint16_t)(lroundf(s_config.bus_voltage_v / MOTOR_SENSING_VBUS_V_PER_COUNT) *
                      MOTOR_SENSING_ADC_SAMPLE_COUNT);
}

void motor_hw_temperature_poll(MotorHwTemperaturePoll *result)
{
    *result = (MotorHwTemperaturePoll){.sample_ready = true,
                                       .conversion_ok = true,
                                       .celsius = s_config.temperature_c,
                                       .vdda_mv = 3300.0f};
}

void motor_hw_pwm_set_duty(float duty_a, float duty_b, float duty_c)
{
    s_snapshot.duty[0] = duty_a;
    s_snapshot.duty[1] = duty_b;
    s_snapshot.duty[2] = duty_c;
}

void power_stage_hw_stop(void)
{
    s_snapshot.phases_disabled = true;
}

void power_stage_hw_start(void)
{
    s_snapshot.phases_disabled = false;
    s_snapshot.power_was_enabled = true;
}

bool power_stage_hw_phases_disabled(void)
{
    return s_snapshot.phases_disabled;
}

void encoder_sensor_init(void)
{
}
bool encoder_sensor_begin(void)
{
    s_encoder_started = true;
    return true;
}
uint8_t encoder_sensor_begin_readback(bool started)
{
    return started && s_encoder_started ? 2U : 0U;
}
EncoderSensorStatus encoder_sensor_complete(uint8_t sample_stage, EncoderSensorSample *out)
{
    (void)sample_stage; /* 当前静态传感器同步回退与异步结果一致。 */
    s_encoder_started = false;
    *out = (EncoderSensorSample){
        .status = ENCODER_SENSOR_OK, .angle_q15 = s_config.encoder_angle_q15, .crc_ok = true};
    if (s_config.motor_connected)
    {
        double angle = fmod(s_snapshot.rotor.angle_rad, 6.283185307179586);
        if (angle < 0.0f)
        {
            angle += 6.283185307179586;
        }
        /* TLE5012B有效15位，接口沿用左移一位的16位单圈表示。 */
        out->angle_q15 = (uint16_t)((uint32_t)(angle * (32768.0 / 6.283185307179586)) << 1U);
    }
    return out->status;
}
EncoderSensorType encoder_sensor_type(void)
{
    return ENCODER_SENSOR_TYPE_TLE5012B;
}
void encoder_sensor_capture_config(void)
{
}
uint16_t encoder_sensor_config_word(uint8_t index)
{
    (void)index;
    return 0U;
}

void comm_hw_can_start_fd(void)
{
    s_can_started = true;
}
bool comm_hw_can_receive(CommHwCanFrame *frame)
{
    uint64_t ticket;
    return s_can_started && dequeue(&s_rx, frame, &ticket);
}
bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    if (!s_can_started || s_ticket != 0U || frame == NULL || !frame->extended || frame->remote ||
        !frame->fd || !frame->bitrate_switch || frame->identifier > 0x1FFFFFFFU ||
        (frame->length > 8U && frame->length != 12U && frame->length != 16U &&
         frame->length != 20U && frame->length != 24U && frame->length != 32U &&
         frame->length != 48U && frame->length != 64U) ||
        !enqueue(&s_tx, frame, 0U))
    {
        return false;
    }
    ++s_snapshot.tx_frames;
    return true;
}
bool comm_hw_can_try_send_tracked_frame(const CommHwCanFrame *frame, uint64_t ticket)
{
    if (ticket == 0U || !comm_hw_can_try_send_frame(frame))
    {
        return false;
    }
    s_tx.tickets[(s_tx.head + s_tx.count - 1U) % QUEUE_CAPACITY] = ticket;
    s_ticket = ticket;
    s_ticket_started_us = s_snapshot.time_us;
    return true;
}
bool comm_hw_can_take_tx_completion(CommHwCanTxCompletion *completion)
{
    if (completion == NULL || s_ticket == 0U)
    {
        return false;
    }
    if (!s_completion_ready && s_snapshot.time_us - s_ticket_started_us < 100000U)
    {
        return false;
    }
    if (!s_completion_ready)
    {
        /* 超时取消尚未发送的跟踪帧，不能在失败结果后又被台架发出。 */
        unsigned count = s_tx.count;
        for (unsigned index = 0U; index < count; ++index)
        {
            CommHwCanFrame frame;
            uint64_t ticket;
            (void)dequeue(&s_tx, &frame, &ticket);
            if (ticket != s_ticket)
            {
                (void)enqueue(&s_tx, &frame, ticket);
            }
        }
    }
    *completion = s_completion_ready ? s_completion : (CommHwCanTxCompletion){s_ticket, false};
    s_ticket = 0U;
    s_completion_ready = false;
    return true;
}
bool comm_hw_can_service_bus_off(void)
{
    return false; /* 首期静态健康链路，不宣称覆盖bus-off恢复。 */
}
bool comm_hw_can_is_healthy(void)
{
    return s_can_started;
}

bool param_store_read(void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > s_storage_bytes)
    {
        return false;
    }
    memcpy(record, s_storage, size_bytes);
    return true;
}
bool param_store_write(const void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > sizeof(s_storage) ||
        !s_snapshot.phases_disabled || s_critical_depth == 0U)
    {
        return false;
    }
    memcpy(s_storage, record, size_bytes);
    s_storage_bytes = size_bytes;
    return memcmp(s_storage, record, size_bytes) == 0;
}
uint32_t time_hw_now_ms(void)
{
    return (uint32_t)(s_snapshot.time_us / 1000U);
}
uint32_t critical_hw_enter(void)
{
    return s_critical_depth++;
}
void critical_hw_exit(uint32_t state)
{
    /* 不模拟抢占；错误嵌套锁存为非零，下一次Advance失败。 */
    if (s_critical_depth == state + 1U)
    {
        s_critical_depth = state;
    }
    else
    {
        s_critical_error = true;
    }
}
void indicator_hw_set_led(bool error_channel, uint8_t blink_num)
{
    (void)error_channel;
    (void)blink_num;
}
void indicator_hw_led_task(void)
{
}
void indicator_hw_set_color(IndicatorHwColor color)
{
    (void)color;
}
