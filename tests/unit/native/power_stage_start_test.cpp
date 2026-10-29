/* 官方HAL函数由Python夹具从原文件提取；比较完整MMIO序列及每次访问时的软件状态。 */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tim.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "bsp_internal.h"
#include <stdlib.h>
void __disable_irq(void)
{
    abort();
}
void __WFI(void)
{
    abort();
}

TIM_TypeDef g_tim1, g_tim8;
TIM_HandleTypeDef htim1;
static uint32_t s_irq_mask;
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t before = s_irq_mask;
    s_irq_mask = 1U;
    return before;
}
void BspSystem_ExitCritical(uint32_t state)
{
    s_irq_mask = state;
}

enum
{
    REGISTER_COUNT = 18,
    BDTR_INDEX = 4,
    TRACE_CAPACITY = 256
};

struct TraceEvent
{
    uint32_t write, identity, value, after;
    uint32_t channel[6], complementary[4], state, lock;
};

struct Result
{
    TraceEvent trace[TRACE_CAPACITY];
    uint32_t count, registers[2 * REGISTER_COUNT];
    BspResult enable_result;
    uint32_t channel[6], complementary[4], state, lock, fallback_count;
    uint32_t fallback_kind[6], fallback_channel[6];
};

struct Scenario
{
    uint32_t alternate, state_variant, ccer, bdtr, sms, cen, break_mode;
};

static TraceEvent s_trace[TRACE_CAPACITY];
static uint32_t s_trace_count, s_break_mode, s_moe_writes, s_fallback_count;
static uint32_t s_fallback_kind[6], s_fallback_channel[6];
static TIM_HandleTypeDef *s_observed;

static void record_access(uint32_t write, const TimRegister *reg, uint32_t value)
{
    assert(s_trace_count < TRACE_CAPACITY);
    TraceEvent &event = s_trace[s_trace_count++];
    event.write = write;
    event.identity = reg->identity;
    event.value = value;
    event.after = reg->value;
    for (unsigned index = 0; index < 6; ++index)
    {
        event.channel[index] = s_observed->ChannelState[index];
    }
    for (unsigned index = 0; index < 4; ++index)
    {
        event.complementary[index] = s_observed->ChannelNState[index];
    }
    event.state = s_observed->State;
    event.lock = s_observed->Lock;
}

TimRegister::operator uint32_t() const
{
    record_access(0, this, value);
    return value;
}

TimRegister &TimRegister::operator=(uint32_t next)
{
    value = next;
    if (identity % REGISTER_COUNT == BDTR_INDEX)
    {
        ++s_moe_writes;
        if (s_break_mode == 1 || (s_break_mode == 2 && s_moe_writes <= 2))
        {
            value &= ~TIM_BDTR_MOE;
        }
    }
    record_access(1, this, next);
    return *this;
}

TimRegister &TimRegister::operator&=(uint32_t mask)
{
    uint32_t current = *this;
    return *this = current & mask;
}

TimRegister &TimRegister::operator|=(uint32_t mask)
{
    uint32_t current = *this;
    return *this = current | mask;
}

#include "official_hal_reference.inc"

static void record_fallback(uint32_t kind, uint32_t channel)
{
    assert(s_fallback_count < 6);
    s_fallback_kind[s_fallback_count] = kind;
    s_fallback_channel[s_fallback_count++] = channel;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
    record_fallback(0, channel);
    return Ref_HAL_TIM_PWM_Start(handle, channel);
}

HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
    record_fallback(1, channel);
    return Ref_HAL_TIMEx_OCN_Start(handle, channel);
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle, uint32_t channel)
{
    return Ref_HAL_TIM_PWM_Stop(handle, channel);
}

HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *handle, uint32_t channel)
{
    return Ref_HAL_TIMEx_OCN_Stop(handle, channel);
}

static void reference_start(TIM_HandleTypeDef *handle)
{
    Ref_HAL_TIM_PWM_Start(handle, TIM_CHANNEL_1);
    Ref_HAL_TIM_PWM_Start(handle, TIM_CHANNEL_2);
    Ref_HAL_TIM_PWM_Start(handle, TIM_CHANNEL_3);
    Ref_HAL_TIMEx_OCN_Start(handle, TIM_CHANNEL_1);
    Ref_HAL_TIMEx_OCN_Start(handle, TIM_CHANNEL_2);
    Ref_HAL_TIMEx_OCN_Start(handle, TIM_CHANNEL_3);
}

static void reference_stop(TIM_HandleTypeDef *handle)
{
    Ref_HAL_TIM_PWM_Stop(handle, TIM_CHANNEL_1);
    Ref_HAL_TIM_PWM_Stop(handle, TIM_CHANNEL_2);
    Ref_HAL_TIM_PWM_Stop(handle, TIM_CHANNEL_3);
    Ref_HAL_TIMEx_OCN_Stop(handle, TIM_CHANNEL_1);
    Ref_HAL_TIMEx_OCN_Stop(handle, TIM_CHANNEL_2);
    Ref_HAL_TIMEx_OCN_Stop(handle, TIM_CHANNEL_3);
}

static void register_list(TIM_TypeDef *timer, TimRegister **result)
{
    TimRegister *items[] = {&timer->CR1,
                            &timer->CR2,
                            &timer->SMCR,
                            &timer->CCER,
                            &timer->BDTR,
                            &timer->CNT,
                            &timer->PSC,
                            &timer->ARR,
                            &timer->RCR,
                            &timer->CCR1,
                            &timer->CCR2,
                            &timer->CCR3,
                            &timer->CCR4,
                            &timer->DIER,
                            &timer->SR,
                            &timer->EGR,
                            &timer->CCMR1,
                            &timer->CCMR2};
    memcpy(result, items, sizeof(items));
}

static void initialize(const Scenario &scenario, TIM_HandleTypeDef *handle)
{
    for (unsigned alternate = 0; alternate < 2; ++alternate)
    {
        TIM_TypeDef *timer = alternate ? TIM8 : TIM1;
        TimRegister *registers[REGISTER_COUNT];
        register_list(timer, registers);
        for (unsigned index = 0; index < REGISTER_COUNT; ++index)
        {
            registers[index]->identity = alternate * REGISTER_COUNT + index;
            registers[index]->value = 0x13570000U + index * 0x101U;
        }
        timer->CR1.value = 0xA0U | scenario.cen;
        timer->CR2.value = scenario.state_variant & 1U;
        timer->CCER.value = scenario.ccer;
        timer->BDTR.value = scenario.bdtr;
        timer->SMCR.value = 0xA5A00080U | scenario.sms;
    }
    handle->Instance = scenario.alternate ? TIM8 : TIM1;
    handle->State = 0x10203040U;
    handle->Lock = 0x50607080U;
    for (unsigned index = 0; index < 6; ++index)
    {
        handle->ChannelState[index] = HAL_TIM_CHANNEL_STATE_READY;
    }
    for (unsigned index = 0; index < 4; ++index)
    {
        handle->ChannelNState[index] = HAL_TIM_CHANNEL_STATE_READY;
    }
    handle->ChannelState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
    handle->ChannelState[4] = HAL_TIM_CHANNEL_STATE_RESET;
    handle->ChannelNState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
    if (scenario.state_variant)
    {
        unsigned slot = (scenario.state_variant - 1) % 6;
        HAL_TIM_ChannelStateTypeDef state =
            scenario.state_variant <= 6 ? HAL_TIM_CHANNEL_STATE_BUSY : HAL_TIM_CHANNEL_STATE_RESET;
        if (slot < 3)
        {
            handle->ChannelState[slot] = state;
        }
        else
        {
            handle->ChannelNState[slot - 3] = state;
        }
    }
    s_observed = handle;
    s_trace_count = s_moe_writes = s_fallback_count = 0;
    s_break_mode = scenario.break_mode;
}

/* 新API增加只读输出确认；参考侧单独读取同一硬件状态，不替代HAL行为参考。 */
static BspResult reference_result(TIM_HandleTypeDef *handle)
{
    uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE;
    uint32_t channels = handle->Instance->CCER & mask;
    bool enabled = (handle->Instance->BDTR & TIM_BDTR_MOE) != 0U;
    return channels == mask && enabled ? BSP_OK : BSP_IO_ERROR;
}

static Result capture(const Scenario &scenario, bool reference, bool cycle)
{
    TIM_HandleTypeDef reference_handle;
    TIM_HandleTypeDef *handle = reference ? &reference_handle : &htim1;
    initialize(scenario, handle);
    BspResult observed;
    if (reference)
    {
        reference_start(handle);
        observed = reference_result(handle);
        if (cycle)
        {
            reference_stop(handle);
            TIM1->BDTR &= ~TIM_BDTR_MOE;
            TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                            TIM_CCER_CC3E | TIM_CCER_CC3NE);
            reference_start(handle);
            observed = reference_result(handle);
        }
    }
    else
    {
        McuPwm_EnableOutputs();
        observed = reference_result(&htim1);
        if (cycle)
        {
            McuPwm_DisableOutputs();
            McuPwm_EnableOutputs();
            observed = reference_result(&htim1);
        }
    }
    Result result = {};
    result.enable_result = observed;
    result.count = s_trace_count;
    memcpy(result.trace, s_trace, s_trace_count * sizeof(TraceEvent));
    for (unsigned alternate = 0; alternate < 2; ++alternate)
    {
        TimRegister *registers[REGISTER_COUNT];
        register_list(alternate ? TIM8 : TIM1, registers);
        for (unsigned index = 0; index < REGISTER_COUNT; ++index)
        {
            result.registers[alternate * REGISTER_COUNT + index] = registers[index]->value;
        }
    }
    for (unsigned index = 0; index < 6; ++index)
    {
        result.channel[index] = handle->ChannelState[index];
    }
    for (unsigned index = 0; index < 4; ++index)
    {
        result.complementary[index] = handle->ChannelNState[index];
    }
    result.state = handle->State;
    result.lock = handle->Lock;
    result.fallback_count = s_fallback_count;
    memcpy(result.fallback_kind, s_fallback_kind, sizeof(result.fallback_kind));
    memcpy(result.fallback_channel, s_fallback_channel, sizeof(result.fallback_channel));
    return result;
}

static void check(const Scenario &scenario, bool cycle)
{
    Result expected = capture(scenario, true, cycle);
    Result actual = capture(scenario, false, cycle);
    assert(actual.enable_result == expected.enable_result);
    assert(s_irq_mask == 0U);
    assert(actual.count == expected.count);
    assert(memcmp(actual.trace, expected.trace, actual.count * sizeof(TraceEvent)) == 0);
    assert(memcmp(actual.registers, expected.registers, sizeof(actual.registers)) == 0);
    assert(memcmp(actual.channel, expected.channel, sizeof(actual.channel)) == 0);
    assert(memcmp(actual.complementary, expected.complementary, sizeof(actual.complementary)) == 0);
    assert(actual.state == expected.state && actual.lock == expected.lock);
    if (!cycle)
    {
        bool fallback = scenario.alternate || scenario.state_variant;
        assert(actual.fallback_count == (fallback ? 6U : 0U));
        for (unsigned index = 0; index < actual.fallback_count; ++index)
        {
            assert(actual.fallback_kind[index] == index / 3);
            assert(actual.fallback_channel[index] == (index % 3) * TIM_CHANNEL_2);
        }
    }
}

static void test_duties(void)
{
    Scenario scenario = {0, 0, 0, 0, 0, 1, 0};
    initialize(scenario, &htim1);
    g_tim1.ARR.value = 7777U;
    const uint32_t ccer = g_tim1.CCER.value, bdtr = g_tim1.BDTR.value;
    const uint32_t control = g_tim1.CR1.value;
    for (unsigned i = 0; i <= 4250U; ++i)
    {
        s_trace_count = 0;
        float duty = (float)i / 4250.0f;
        assert(McuPwm_CommitThree((uint16_t)(duty * 4250U), 1062U, 3187U));
        assert(g_tim1.CCR1.value == (uint16_t)(duty * 4250U));
        assert(g_tim1.CCR2.value == 1062U && g_tim1.CCR3.value == 3187U);
        const uint32_t expected_registers[] = {g_tim1.CR1.identity,
                                               g_tim1.CCR1.identity,
                                               g_tim1.CCR2.identity,
                                               g_tim1.CCR3.identity,
                                               g_tim1.CR1.identity};
        unsigned written = 0U;
        for (unsigned n = 0U; n < s_trace_count; ++n)
        {
            if (s_trace[n].write)
            {
                assert(written < 5U && s_trace[n].identity == expected_registers[written]);
                if (written == 0U)
                {
                    assert(s_trace[n].value == (control | (1U << 1U)));
                }
                else if (written == 4U)
                {
                    assert(s_trace[n].value == control);
                }
                ++written;
            }
        }
        assert(written == 5U && g_tim1.CR1.value == control);
        assert(g_tim1.CCER.value == ccer && g_tim1.BDTR.value == bdtr);
    }
    s_trace_count = 0;
    assert(McuPwm_CommitPhase(BSP_PWM_PHASE_B, 2125U));
    assert(g_tim1.CCR2.value == 2125U);
    assert(s_trace[s_trace_count - 1U].write &&
           s_trace[s_trace_count - 1U].identity == g_tim1.CCR2.identity);
    s_trace_count = 0U;
    assert(!McuPwm_CommitPhase((BspPwmPhase)99, 4250U));
    for (unsigned n = 0U; n < s_trace_count; ++n)
    {
        assert(!s_trace[n].write);
    }
    assert(McuPwm_CommitThree(4250U, 4250U, 4250U));
    assert(g_tim1.CCR1.value == 4250U && g_tim1.CCR2.value == 4250U && g_tim1.CCR3.value == 4250U);
    assert(g_tim1.CCER.value == ccer && g_tim1.BDTR.value == bdtr);
}

/* 比较写入轨迹而非只看最终值，超时不允许触及任何寄存器。 */
static void test_pwm_commit_boundaries(void)
{
    const struct
    {
        uint32_t count;
        bool up, down;
    } cases[] = {{0U, true, false},
                 {256U, true, false},
                 {257U, true, true},
                 {3993U, true, true},
                 {3994U, false, true},
                 {4250U, false, true},
                 {4251U, false, false},
                 {UINT32_MAX, false, false}};
    const uint32_t channels[] = {0U, TIM_CCER_CC1E, 0x555U};
    const uint32_t controls[] = {0U, 1U, 0x11U, 0x13U};
    for (unsigned channel = 0U; channel < 3U; ++channel)
    {
        for (unsigned master = 0U; master < 2U; ++master)
        {
            for (unsigned mode = 0U; mode < 4U; ++mode)
            {
                for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
                {
                    Scenario scenario = {
                        0, 0, channels[channel], master ? TIM_BDTR_MOE : 0U, 0, 0, 0};
                    initialize(scenario, &htim1);
                    g_tim1.CR1.value = controls[mode];
                    g_tim1.CNT.value = cases[i].count;
                    g_tim1.ARR.value = 4250U;
                    g_tim1.CCR1.value = 111U;
                    g_tim1.CCR2.value = 222U;
                    g_tim1.CCR3.value = 333U;
                    bool expected = (channel == 0U && master == 0U) || mode == 0U ||
                                    (mode == 1U ? cases[i].up : cases[i].down);
                    s_trace_count = 0U;
                    assert(McuPwm_CommitThree(12U, 34U, 56U) == expected);
                    unsigned writes = 0U, channel_reads = 0U;
                    for (unsigned n = 0U; n < s_trace_count; ++n)
                    {
                        writes += s_trace[n].write;
                        channel_reads +=
                            !s_trace[n].write && s_trace[n].identity == g_tim1.CCER.identity;
                    }
                    assert(channel_reads == 1U);
                    assert(writes == (expected ? (channel == 0U && master == 0U ? 5U : 3U) : 0U));
                    assert(g_tim1.CCR1.value == (expected ? 12U : 111U));
                    assert(g_tim1.CCR2.value == (expected ? 34U : 222U));
                    assert(g_tim1.CCR3.value == (expected ? 56U : 333U));
                    assert(g_tim1.CR1.value == controls[mode]);
                    assert(g_tim1.CCER.value == channels[channel]);
                    assert(g_tim1.BDTR.value == (master ? TIM_BDTR_MOE : 0U));
                    s_trace_count = 0U;
                    assert(McuPwm_CommitPhase(BSP_PWM_PHASE_B, 78U) == expected);
                    writes = 0U;
                    for (unsigned n = 0U; n < s_trace_count; ++n)
                    {
                        writes += s_trace[n].write;
                    }
                    assert(writes == (expected ? 1U : 0U));
                    assert(g_tim1.CCR2.value == (expected ? 78U : 222U));
                    assert(g_tim1.CR1.value == controls[mode]);
                }
            }
        }
    }
    puts("PASS STM32 merged PWM: 192 timer/output boundary cases, single state read, zero writes "
         "on timeout");
}

int main(void)
{
    test_duties();
    test_pwm_commit_boundaries();
    const uint32_t ccer[] = {0, TIM_CCER_CC4E, 0xA5A5A5A5U, 0xFFFFFFFFU};
    const uint32_t bdtr[] = {0, TIM_BDTR_MOE, 0xA5A55A5AU};
    unsigned cases = 0;
    const unsigned matrix_cases = 2 * 13 * 4 * 3 * 16 * 2 * 3;
    for (unsigned variant = 0; variant < matrix_cases; ++variant)
    {
        unsigned remaining = variant;
        unsigned br = remaining % 3;
        remaining /= 3;
        unsigned cen = remaining % 2;
        remaining /= 2;
        unsigned mode = remaining % 16;
        remaining /= 16;
        unsigned bd = remaining % 3;
        remaining /= 3;
        unsigned cc = remaining % 4;
        remaining /= 4;
        unsigned state = remaining % 13;
        unsigned alternate = remaining / 13;
        uint32_t sms = (mode & 7U) | ((mode & 8U) ? TIM_SMCR_SMS_3 : 0U);
        Scenario scenario = {alternate, state, ccer[cc], bdtr[bd], sms, cen, br};
        check(scenario, false);
        ++cases;
    }
    for (unsigned mode = 0; mode < 16; ++mode)
    {
        uint32_t sms = (mode & 7U) | ((mode & 8U) ? TIM_SMCR_SMS_3 : 0U);
        Scenario scenario = {0, 0, TIM_CCER_CC4E, 0xA5A55A5AU, sms, 1, 0};
        check(scenario, true);
        ++cases;
    }
    printf("PASS %u official-HAL cases; exact MMIO/state traces, fallback and start-stop-start\n",
           cases);
    return 0;
}

/* 初始化由独立采样启动夹具验证。 */
void MX_TIM1_Init(void)
{
}
