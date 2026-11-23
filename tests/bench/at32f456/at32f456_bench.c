#include "bsp_adc.h"
#include "at32f456_bench.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "board_config.h"
#include "bsp_flash.h"
#include "bsp_pwm.h"
#include "../../../firmware/platform/bsp/bsp_internal.h"
#include "board_config.h"

/* 已发布的台架诊断字节，前四字节同时用于请求匹配。 */
static const uint8_t s_diagnostic_payload[8] = {0x52, 0x34, 0x30, 0x42, 0x54, 0x58, 0x21, 0x21};

/* 命令结果：0空闲、1执行中、2通过、3失败、4拒绝；存储测试只接受原本全FF的两扇区。
 * 所有对象均归此验证模块所有；不把这些诊断命令接入产品CAN协议。 */
volatile AT32BenchMailbox g_at32_bench = {.magic = AT32_BENCH_MAGIC, .version = 1U};
volatile AT32BenchSnapshot g_at32_snapshot;
static uint8_t s_storage[2U * AT32_FLASH_SECTOR_BYTES];
static uint8_t s_pattern[AT32_PARAM_RAW_BYTES];
static uint32_t s_nonce, s_can_rx, s_can_tx_ok, s_can_tx_failed;
static bool s_tx_pending, s_snapshot_held;

static bool outputs_disabled(void)
{
    return BspPwm_ArePhasesDisabled() && (TMR1->brk & (1U << 15U)) == 0U &&
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
        BspPwm_SetDuty(duty[i], duty[4U - i], 0.5f);
        if (!outputs_disabled() || TMR1->c2dt != (uint32_t)(duty[i] * (float)top) ||
            TMR1->c3dt != (uint32_t)(duty[4U - i] * (float)top) || TMR1->c4dt != top / 2U)
        {
            return false;
        }
    }
    BspPwm_SetPhaseDuty(BSP_PWM_PHASE_B, 0.25f);
    g_at32_bench.values[0] = TMR1->c2dt;
    g_at32_bench.values[1] = TMR1->c3dt;
    g_at32_bench.values[2] = TMR1->c4dt;
    bool ok =
        TMR1->c2dt == top && TMR1->c3dt == top / 4U && TMR1->c4dt == top / 2U && outputs_disabled();
    BspPwm_SetDuty(0.5f, 0.5f, 0.5f);
    return ok && outputs_disabled();
}

static bool storage_test(uint32_t command)
{
    if (BspFlash_Read(s_storage, sizeof(s_storage)) != BSP_OK)
    {
        g_at32_bench.detail = 1U;
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
        g_at32_bench.detail = 2U;
        return false;
    }
    if (command == 3U)
    {
        memset(s_pattern, 0xFF, sizeof(s_pattern));
    }
    /* 写入之后不恢复采样；调用者停止并等待显式复位，避免暂停中的ADC触发造成假样本。 */
    __disable_irq();
    BspPwm_Disable();
    tmr_counter_enable(TMR1, FALSE);
    tmr_counter_enable(TMR6, FALSE);
    bool written = (BspFlash_Write(s_pattern, sizeof(s_pattern)) == BSP_OK);
    bool read = written && (BspFlash_Read(s_storage, sizeof(s_storage)) == BSP_OK);
    g_at32_bench.values[0] = written;
    g_at32_bench.values[1] = read;
    g_at32_bench.values[2] = sizeof(s_pattern);
    bool same =
        read && memcmp(s_storage, s_pattern, sizeof(s_pattern)) == 0 &&
        bytes_are(s_storage + sizeof(s_pattern), sizeof(s_storage) - sizeof(s_pattern), 0xFFU);
    g_at32_bench.detail = same ? 0U : 3U;
    return same && outputs_disabled();
}

bool AT32_BenchPoll(void)
{
    if (s_tx_pending && !BspCan_IsTransmitBusy())
    {
        if (BspCan_IsTransmitComplete())
        {
            ++s_can_tx_ok;
        }
        else
        {
            ++s_can_tx_failed;
        }
        s_tx_pending = false;
    }
    /* 调试器最后提交 nonce；读取后再核对，避免采用跨次命令的参数。 */
    uint32_t nonce = g_at32_bench.nonce;
    uint32_t command = g_at32_bench.command;
    uint32_t inverse = g_at32_bench.command_inverse;
    uint32_t confirmation = g_at32_bench.arguments[0];
    if (g_at32_bench.magic != AT32_BENCH_MAGIC || g_at32_bench.version != 1U || command == 0U ||
        nonce == 0U || nonce == s_nonce || nonce != g_at32_bench.nonce || inverse != ~command)
    {
        return false;
    }
    s_nonce = nonce;
    g_at32_bench.result = 1U;
    g_at32_bench.detail = 0U;
    bool stop = false;
    if (!outputs_disabled())
    {
        g_at32_bench.result = 3U;
        g_at32_bench.detail = 4U;
        stop = true;
    }
    else if (command == 1U)
    {
        bool ok = pwm_test();
        g_at32_bench.result = ok ? 2U : 3U;
        stop = !ok;
    }
    else if ((command == 2U || command == 3U) && confirmation == 0x53544F52U)
    {
        g_at32_bench.result = storage_test(command) ? 2U : 3U;
        stop = true;
    }
    else if (command == 5U || command == 6U)
    {
        s_snapshot_held = command == 5U;
        g_at32_bench.result = 2U;
    }
    else if (command == 7U && !s_tx_pending)
    {
        /* 由SWD显式触发单帧发送，独立诊断TX路径；不能伪造接收计数。 */
        BspCanFrame frame = {.identifier = AT32_BENCH_CAN_RESPONSE,
                             .length = 8U,
                             .extended = true,
                             .fd = true,
                             .bitrate_switch = true};
        memcpy(frame.data, s_diagnostic_payload, sizeof(s_diagnostic_payload));
        s_tx_pending = BspCan_TrySend(&frame);
        /* 这里只确认入队，实际发送结果仍由tx_ok/tx_failed记录。 */
        g_at32_bench.result = s_tx_pending ? 2U : 3U;
    }
    else
    {
        g_at32_bench.result = 4U;
    }
    g_at32_bench.completed_nonce = nonce;
    return stop;
}

void AT32_BenchCanReceive(const BspCanFrame *frame)
{
    if (frame == NULL || frame->identifier != AT32_BENCH_CAN_REQUEST || frame->length < 8U ||
        memcmp(frame->data, s_diagnostic_payload, 4U) != 0 || !outputs_disabled())
    {
        return;
    }
    ++s_can_rx;
    if (!s_tx_pending)
    {
        BspCanFrame reply = *frame;
        reply.identifier = AT32_BENCH_CAN_RESPONSE;
        if (BspCan_TrySend(&reply))
        {
            s_tx_pending = true;
        }
        else
        {
            ++s_can_tx_failed;
        }
    }
}

void AT32_BenchPublish(AT32BenchSnapshot *sample,
                       uint32_t revision,
                       const volatile uint32_t *adc_revision)
{
    static uint32_t s_published_ms = UINT32_MAX;
    uint32_t now = BspSystem_TimeMs();
    if (s_snapshot_held || now == s_published_ms)
    {
        return;
    }
    sample->milliseconds = now;
    sample->cctrl = TMR1->cctrl;
    sample->brk = TMR1->brk;
    sample->gpioa_input = GPIOA->idt;
    sample->gpiob_input = GPIOB->idt;
    sample->phase_a =
        (uint16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_A) * AT32_ADC_ACCUMULATIONS);
    sample->phase_b =
        (uint16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_B) * AT32_ADC_ACCUMULATIONS);
    sample->phase_c =
        (uint16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_C) * AT32_ADC_ACCUMULATIONS);
    sample->vbus = (uint16_t)(BspAdc_ReadRawChannel(BSP_ADC_BUS_VOLTAGE) * AT32_ADC_ACCUMULATIONS);
    float bus_counts = BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT);
    sample->bus_current = isfinite(bus_counts) ? (uint32_t)bus_counts : UINT32_MAX;
    sample->can_rx = s_can_rx;
    sample->can_tx_ok = s_can_tx_ok;
    sample->can_tx_failed = s_can_tx_failed;
    sample->can_ctrl = CAN1->ctrlstat;
    sample->core_hz = system_core_clock;
    BspSystem_MemoryBarrier();
    if (revision != *adc_revision)
    {
        return;
    }
    s_published_ms = now;
    uint32_t version = (g_at32_snapshot.begin + 2U) & ~1U;
    sample->begin = version | 1U;
    sample->end = version;
    g_at32_snapshot.begin = sample->begin;
    BspSystem_MemoryBarrier();
    g_at32_snapshot = *sample;
    BspSystem_MemoryBarrier();
    g_at32_snapshot.begin = version;
}
