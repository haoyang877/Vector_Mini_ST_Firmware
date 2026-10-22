/* 官方HAL函数由Python夹具从原文件提取；比较完整MMIO序列及每次访问时的软件状态。 */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tim.h"
#include "power_stage_hw.h"

TIM_TypeDef g_tim1, g_tim8;
TIM_HandleTypeDef htim1;

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
    TimRegister *items[] = {
        &timer->CR1, &timer->CR2, &timer->SMCR, &timer->CCER, &timer->BDTR, &timer->CNT,
        &timer->PSC, &timer->ARR, &timer->RCR, &timer->CCR1, &timer->CCR2, &timer->CCR3,
        &timer->CCR4, &timer->DIER, &timer->SR, &timer->EGR, &timer->CCMR1, &timer->CCMR2};
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
        HAL_TIM_ChannelStateTypeDef state = scenario.state_variant <= 6
                                               ? HAL_TIM_CHANNEL_STATE_BUSY
                                               : HAL_TIM_CHANNEL_STATE_RESET;
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

static Result capture(const Scenario &scenario, bool reference, bool cycle)
{
    TIM_HandleTypeDef reference_handle;
    TIM_HandleTypeDef *handle = reference ? &reference_handle : &htim1;
    initialize(scenario, handle);
    if (reference)
    {
        reference_start(handle);
        if (cycle)
        {
            reference_stop(handle);
            reference_start(handle);
        }
    }
    else
    {
        power_stage_hw_start();
        if (cycle)
        {
            power_stage_hw_stop();
            power_stage_hw_start();
        }
    }
    Result result = {};
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

int main(void)
{
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
    printf("PASS %u official-HAL cases; exact MMIO/state traces, fallback and start-stop-start\n", cases);
    return 0;
}
