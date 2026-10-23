#include "r40_bench.h"

#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "critical_hw.h"
#include "motor_hw.h"
#include "motor_sensing.h"
#include "param_store.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "r40_storage_layout.h"
#include "time_hw.h"

/* 命令结果：0空闲、1执行中、2通过、3失败、4拒绝；存储测试只接受原本全FF的两扇区。
 * 所有对象均归此验证模块所有；不把这些诊断命令接入产品CAN协议。 */
volatile R40BenchMailbox g_r40_bench = {.magic = R40_BENCH_MAGIC, .version = 1U};
volatile R40BenchSnapshot g_r40_snapshot;
static uint8_t s_storage[2U * R40_FLASH_SECTOR_BYTES];
static uint8_t s_pattern[R40_PARAM_RAW_BYTES];
static uint32_t s_nonce, s_can_rx, s_can_tx_ok, s_can_tx_failed;
static uint64_t s_ticket;
static bool s_tx_pending, s_snapshot_held;

static bool outputs_disabled(void)
{
    return power_stage_hw_phases_disabled() && (TMR1->brk & (1U << 15U)) == 0U &&
           (GPIOA->idt & 0xE00U) == 0U && (GPIOB->idt & 0x13U) == 0U;
}

static bool bytes_are(const uint8_t *data, uint32_t size, uint8_t value)
{
    for (uint32_t i = 0U; i < size; ++i)
    {
        if (data[i] != value)
        {
            return false;
        }
    }
    return true;
}

static bool pwm_test(void)
{
    const float duty[] = {0.0f, 0.125f, 0.5f, 0.875f, 1.0f};
    uint32_t top = TMR1->pr;
    for (uint32_t i = 0U; i < 5U; ++i)
    {
        motor_hw_pwm_set_duty(duty[i], duty[4U - i], 0.5f);
        if (!outputs_disabled() || TMR1->c2dt != (uint32_t)(duty[i] * (float)top) ||
            TMR1->c3dt != (uint32_t)(duty[4U - i] * (float)top) || TMR1->c4dt != top / 2U)
        {
            return false;
        }
    }
    motor_hw_pwm_set_phase_duty(MOTOR_HW_PWM_PHASE_B, 0.25f);
    g_r40_bench.values[0] = TMR1->c2dt;
    g_r40_bench.values[1] = TMR1->c3dt;
    g_r40_bench.values[2] = TMR1->c4dt;
    bool ok =
        TMR1->c2dt == top && TMR1->c3dt == top / 4U && TMR1->c4dt == top / 2U && outputs_disabled();
    motor_hw_pwm_set_duty(0.5f, 0.5f, 0.5f);
    return ok && outputs_disabled();
}

static bool storage_test(uint32_t command)
{
    if (!param_store_read(s_storage, sizeof(s_storage)))
    {
        g_r40_bench.detail = 1U;
        return false;
    }
    for (uint32_t i = 0U; i < sizeof(s_pattern); ++i)
    {
        s_pattern[i] = (uint8_t)((i * 37U + 0x53U) ^ (i >> 3U));
    }
    bool allowed = command == 2U ? bytes_are(s_storage, sizeof(s_storage), 0xFFU)
                                 : memcmp(s_storage, s_pattern, sizeof(s_pattern)) == 0 &&
                                       bytes_are(s_storage + sizeof(s_pattern),
                                                 sizeof(s_storage) - sizeof(s_pattern),
                                                 0xFFU);
    if (!allowed)
    {
        g_r40_bench.detail = 2U;
        return false;
    }
    if (command == 3U)
    {
        memset(s_pattern, 0xFF, sizeof(s_pattern));
    }
    /* 写入之后不恢复采样；调用者停止并等待显式复位，避免暂停中的ADC触发造成假样本。 */
    __disable_irq();
    power_stage_hw_stop();
    tmr_counter_enable(TMR1, FALSE);
    tmr_counter_enable(TMR6, FALSE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    bool written = param_store_write(s_pattern, sizeof(s_pattern));
    bool read = written && param_store_read(s_storage, sizeof(s_storage));
    g_r40_bench.values[0] = written;
    g_r40_bench.values[1] = read;
    g_r40_bench.values[2] = sizeof(s_pattern);
    bool same =
        read && memcmp(s_storage, s_pattern, sizeof(s_pattern)) == 0 &&
        bytes_are(s_storage + sizeof(s_pattern), sizeof(s_storage) - sizeof(s_pattern), 0xFFU);
    g_r40_bench.detail = same ? 0U : 3U;
    return same && outputs_disabled();
}

bool R40_BenchPoll(void)
{
    if (s_tx_pending)
    {
        CommHwCanTxCompletion done;
        if (comm_hw_can_take_tx_completion(&done))
        {
            if (done.success && done.ticket == s_ticket)
            {
                ++s_can_tx_ok;
            }
            else
            {
                ++s_can_tx_failed;
            }
            s_tx_pending = false;
        }
    }
    uint32_t mask = critical_hw_enter();
    uint32_t command = g_r40_bench.command;
    uint32_t inverse = g_r40_bench.command_inverse;
    uint32_t nonce = g_r40_bench.nonce;
    uint32_t confirmation = g_r40_bench.arguments[0];
    critical_hw_exit(mask);
    if (g_r40_bench.magic != R40_BENCH_MAGIC || g_r40_bench.version != 1U || command == 0U ||
        nonce == 0U || nonce == s_nonce || inverse != ~command)
    {
        return false;
    }
    s_nonce = nonce;
    g_r40_bench.result = 1U;
    g_r40_bench.detail = 0U;
    bool stop = false;
    if (!outputs_disabled())
    {
        g_r40_bench.result = 3U;
        g_r40_bench.detail = 4U;
        stop = true;
    }
    else if (command == 1U)
    {
        bool ok = pwm_test();
        g_r40_bench.result = ok ? 2U : 3U;
        stop = !ok;
    }
    else if ((command == 2U || command == 3U) && confirmation == 0x53544F52U)
    {
        g_r40_bench.result = storage_test(command) ? 2U : 3U;
        stop = true;
    }
    else if (command == 5U || command == 6U)
    {
        s_snapshot_held = command == 5U;
        g_r40_bench.result = 2U;
    }
    else if (command == 7U && !s_tx_pending)
    {
        /* 由SWD显式触发单帧发送，独立诊断TX路径；不能伪造接收计数。 */
        CommHwCanFrame frame = {.identifier = R40_BENCH_CAN_RESPONSE,
                                .length = 8U,
                                .extended = true,
                                .fd = true,
                                .bitrate_switch = true};
        memcpy(frame.data, "R40BTX!!", 8U);
        s_ticket = (2ULL << 32U) | nonce;
        s_tx_pending = comm_hw_can_try_send_tracked_frame(&frame, s_ticket);
        /* 这里只确认入队，实际发送结果仍由tx_ok/tx_failed记录。 */
        g_r40_bench.result = s_tx_pending ? 2U : 3U;
    }
    else
    {
        g_r40_bench.result = 4U;
    }
    g_r40_bench.completed_nonce = nonce;
    return stop;
}

void R40_BenchCanReceive(const CommHwCanFrame *frame)
{
    if (frame == NULL || frame->identifier != R40_BENCH_CAN_REQUEST || frame->length < 8U ||
        memcmp(frame->data, "R40B", 4U) != 0 || !outputs_disabled())
    {
        return;
    }
    ++s_can_rx;
    if (!s_tx_pending)
    {
        CommHwCanFrame reply = *frame;
        reply.identifier = R40_BENCH_CAN_RESPONSE;
        s_ticket = (1ULL << 32U) | s_can_rx;
        if (comm_hw_can_try_send_tracked_frame(&reply, s_ticket))
        {
            s_tx_pending = true;
        }
        else
        {
            ++s_can_tx_failed;
        }
    }
}

void R40_BenchPublish(uint32_t supervisor_ticks,
                      uint32_t fast_samples,
                      uint32_t encoder_samples,
                      uint32_t encoder_errors,
                      uint32_t max_cycles,
                      uint32_t angle,
                      uint32_t angle_min,
                      uint32_t angle_max,
                      const R40MotorTemperatureSample *temperature)
{
    static uint32_t s_published_ms = UINT32_MAX;
    uint32_t now = time_hw_now_ms();
    if (s_snapshot_held || now == s_published_ms || temperature == NULL)
    {
        return;
    }
    s_published_ms = now;
    uint32_t state = critical_hw_enter();
    uint32_t version = (g_r40_snapshot.begin + 2U) & ~1U;
    g_r40_snapshot.begin = version | 1U;
    g_r40_snapshot.milliseconds = now;
    g_r40_snapshot.supervisor_ticks = supervisor_ticks;
    g_r40_snapshot.fast_samples = fast_samples;
    g_r40_snapshot.encoder_samples = encoder_samples;
    g_r40_snapshot.encoder_errors = encoder_errors;
    g_r40_snapshot.fast_max_cycles = max_cycles;
    g_r40_snapshot.angle = angle;
    g_r40_snapshot.angle_min = angle_min;
    g_r40_snapshot.angle_max = angle_max;
    g_r40_snapshot.cctrl = TMR1->cctrl;
    g_r40_snapshot.brk = TMR1->brk;
    g_r40_snapshot.gpioa_input = GPIOA->idt;
    g_r40_snapshot.gpiob_input = GPIOB->idt;
    g_r40_snapshot.phase_a = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_A);
    g_r40_snapshot.phase_b = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_B);
    g_r40_snapshot.phase_c = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_C);
    g_r40_snapshot.vbus = motor_hw_vbus_sample_raw();
    R40AuxSamples aux;
    g_r40_snapshot.bus_current = R40_AdcReadAux(&aux) ? aux.bus_current_counts : UINT32_MAX;
    g_r40_snapshot.ntc_sequence = temperature->sequence;
    g_r40_snapshot.ntc_raw = temperature->raw_counts;
    g_r40_snapshot.ntc_status = temperature->status;
    uint32_t bits;
    memcpy(&bits, &temperature->resistance_ohm, sizeof(bits));
    g_r40_snapshot.ntc_ohm_bits = bits;
    memcpy(&bits, &temperature->celsius, sizeof(bits));
    g_r40_snapshot.ntc_celsius_bits = bits;
    g_r40_snapshot.can_rx = s_can_rx;
    g_r40_snapshot.can_tx_ok = s_can_tx_ok;
    g_r40_snapshot.can_tx_failed = s_can_tx_failed;
    g_r40_snapshot.can_ctrl = CAN1->ctrlstat;
    g_r40_snapshot.core_hz = system_core_clock;
    g_r40_snapshot.end = version;
    g_r40_snapshot.begin = version;
    critical_hw_exit(state);
}
