/* 验证实机诊断入口不会默认写Flash，并覆盖显式写入、重启校验、恢复和关相拒绝。 */
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "../../../firmware/platform/at32f456/foundation/r40_bench.c"
#include "../../../firmware/platform/at32f456/ports/motor_pwm_at32f456.c"

tmr_type fake_tmr1 = {.pr = 4500U}, fake_tmr6;
gpio_type fake_gpioa, fake_gpiob;
can_type fake_can;
uint32_t system_core_clock = 180000000U;
static uint32_t s_mask, s_ms, s_writes;
static uint8_t s_flash[4096];
static bool s_fail_write, s_can_sent;
static CommHwCanFrame s_sent;
static uint64_t s_sent_ticket;

uint32_t critical_hw_enter(void)
{
    uint32_t old = s_mask;
    s_mask = 1U;
    return old;
}
void critical_hw_exit(uint32_t old)
{
    s_mask = old;
}
void __disable_irq(void)
{
    s_mask = 1U;
}
void power_stage_hw_stop(void)
{
    fake_tmr1.cctrl = 0U;
    fake_tmr1.brk = 0U;
}
bool power_stage_hw_phases_disabled(void)
{
    return fake_tmr1.cctrl == 0U;
}
void tmr_counter_enable(tmr_type *timer, bool enabled)
{
    timer->running = enabled;
}
void dma_channel_enable(unsigned channel, bool enabled)
{
    (void)channel;
    assert(!enabled);
}
uint32_t time_hw_now_ms(void)
{
    return s_ms;
}
uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase)
{
    return (uint16_t)phase;
}
uint16_t motor_hw_vbus_sample_raw(void)
{
    return 1234U;
}
bool R40_AdcReadAux(R40AuxSamples *out)
{
    *out = (R40AuxSamples){100U, 372U, 1U};
    return true;
}
bool param_store_read(void *out, uint32_t size)
{
    memcpy(out, s_flash, size);
    return true;
}
bool param_store_write(const void *in, uint32_t size)
{
    assert(s_mask && !fake_tmr1.running && !fake_tmr6.running);
    assert(size == 2288U && outputs_disabled());
    ++s_writes;
    if (s_fail_write)
    {
        return false;
    }
    memset(s_flash, 0xff, sizeof(s_flash));
    memcpy(s_flash, in, size);
    return true;
}
bool comm_hw_can_try_send_tracked_frame(const CommHwCanFrame *frame, uint64_t ticket)
{
    s_sent = *frame;
    s_sent_ticket = ticket;
    s_can_sent = true;
    return true;
}
bool comm_hw_can_take_tx_completion(CommHwCanTxCompletion *out)
{
    if (!s_can_sent)
    {
        return false;
    }
    *out = (CommHwCanTxCompletion){s_sent_ticket, true};
    s_can_sent = false;
    return true;
}

static void request(uint32_t command, uint32_t nonce, uint32_t confirmation)
{
    g_r40_bench.command = command;
    g_r40_bench.command_inverse = ~command;
    g_r40_bench.nonce = nonce;
    g_r40_bench.arguments[0] = confirmation;
}

int main(void)
{
    memset(s_flash, 0xff, sizeof(s_flash));
    assert(!R40_BenchPoll() && s_writes == 0U);
    request(2U, 1U, 0x53544f52U);
    g_r40_bench.command_inverse = 0;
    assert(!R40_BenchPoll() && s_writes == 0U);
    request(2U, 1U, 0);
    assert(!R40_BenchPoll() && g_r40_bench.result == 4U && s_writes == 0U);
    request(1U, 2U, 0);
    assert(!R40_BenchPoll() && g_r40_bench.result == 2U);
    assert(g_r40_bench.values[0] == 4500U && g_r40_bench.values[1] == 1125U);
    assert(fake_tmr1.c2dt == 2250U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 2250U);
    request(2U, 3U, 0x53544f52U);
    s_flash[4000] = 0;
    assert(R40_BenchPoll() && g_r40_bench.result == 3U && s_writes == 0U);
    memset(s_flash, 0xff, sizeof(s_flash));
    request(2U, 4U, 0x53544f52U);
    fake_tmr1.running = fake_tmr6.running = true;
    assert(R40_BenchPoll() && g_r40_bench.result == 2U && s_writes == 1U);
    assert(!R40_BenchPoll() && s_writes == 1U);
    /* 重启会清空镜像状态；验证仍仅取决于持久化Flash中的完整标记。 */
    s_nonce = 0;
    s_mask = 0;
    request(3U, 1U, 0x53544f52U);
    assert(R40_BenchPoll() && g_r40_bench.result == 2U && s_writes == 2U);
    assert(bytes_are(s_flash, sizeof(s_flash), 0xff));
    request(2U, 5U, 0x53544f52U);
    s_fail_write = true;
    assert(R40_BenchPoll() && g_r40_bench.result == 3U && g_r40_bench.detail == 3U);
    s_fail_write = false;
    request(1U, 6U, 0);
    fake_gpioa.idt = 1U << 9U;
    assert(R40_BenchPoll() && g_r40_bench.result == 3U && g_r40_bench.detail == 4U);
    fake_gpioa.idt = 0U;
    g_r40_bench.command = 0;
    CommHwCanFrame frame = {.identifier = R40_BENCH_CAN_REQUEST,
                            .length = 8U,
                            .extended = true,
                            .fd = true,
                            .bitrate_switch = true};
    R40_BenchCanReceive(&frame);
    assert(!s_can_sent);
    memcpy(frame.data, "R40B1234", 8U);
    R40_BenchCanReceive(&frame);
    assert(s_can_sent && s_sent.identifier == R40_BENCH_CAN_RESPONSE && s_sent_ticket > UINT32_MAX);
    assert(!R40_BenchPoll() && s_can_tx_ok == 1U);
    R40MotorTemperatureSample temp = {.sequence = 7,
                                      .raw_counts = 372,
                                      .status = R40_MOTOR_TEMPERATURE_VALID,
                                      .resistance_ohm = 10000,
                                      .celsius = 25};
    R40_BenchPublish(2, 20, 20, 0, 2000, 42, 40, 50, &temp);
    assert(g_r40_snapshot.begin == g_r40_snapshot.end && (g_r40_snapshot.begin & 1U) == 0U);
    assert(g_r40_snapshot.phase_c == 2 && g_r40_snapshot.ntc_raw == 372);
    request(5U, 7U, 0U);
    assert(!R40_BenchPoll() && g_r40_bench.result == 2U);
    s_ms = 10U;
    R40_BenchPublish(3, 40, 40, 0, 2000, 43, 40, 50, &temp);
    assert(g_r40_snapshot.fast_samples == 20U);
    request(6U, 8U, 0U);
    assert(!R40_BenchPoll());
    R40_BenchPublish(3, 40, 40, 0, 2000, 43, 40, 50, &temp);
    assert(g_r40_snapshot.fast_samples == 40U);
    uint32_t received_before = s_can_rx;
    request(7U, 9U, 0U);
    assert(!R40_BenchPoll() && g_r40_bench.result == 2U && s_can_sent);
    assert(s_sent.identifier == R40_BENCH_CAN_RESPONSE && s_sent.length == 8U);
    assert(s_sent.extended && s_sent.fd && s_sent.bitrate_switch && !s_sent.remote);
    assert(memcmp(s_sent.data, "R40BTX!!", 8U) == 0 && s_can_rx == received_before);
    assert(s_sent_ticket == ((2ULL << 32U) | 9U));
    assert(!R40_BenchPoll() && s_can_tx_ok == 2U && outputs_disabled());
    fake_gpioa.idt = 1U << 9U;
    request(7U, 10U, 0U);
    assert(R40_BenchPoll() && g_r40_bench.result == 3U && !s_can_sent);
    puts("PASS bench: explicit commands only, PWM restoration, Flash guards/persistence "
         "restoration, CAN echo, coherent snapshot");
    return 0;
}
