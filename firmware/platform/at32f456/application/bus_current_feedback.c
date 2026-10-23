#include "app_bus_current.h"

#include <math.h>
#include <stddef.h>
#include "critical_hw.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "time_hw.h"

/* 该文件仅由有母线采样硬件的板级APP装配；ADC/BSP及中间层均不依赖APP。
 * PA4为12位单次采样域，不能使用三相四次和域或4mΩ的相电流系数。
 * 每次上电仅在关相时采128个连续、稳定样本；运行中不跟踪零点。 */
#define OFFSET_SAMPLES 128U
#define OFFSET_RANGE_COUNTS 16U
#define OFFSET_CENTER_COUNTS 2048U
#define OFFSET_TOLERANCE_COUNTS 256U
#define RAIL_MARGIN_COUNTS 2U
#define SAMPLE_MAX_AGE_MS 2U
#define FILTER_ALPHA 0.05f
#define BUS_CURRENT_A_PER_COUNT                                                                    \
    (R40_ADC_REFERENCE_V / (float)R40_ADC_MAX_COUNTS / R40_BUS_CURRENT_GAIN / R40_BUS_SHUNT_OHM)

typedef enum
{
    BUS_CURRENT_WAIT_OFFSET = 0,
    BUS_CURRENT_VALID,
    BUS_CURRENT_NO_SAMPLE,
    BUS_CURRENT_RAIL,
    BUS_CURRENT_OFFSET_REJECTED
} BusCurrentStatus;

/* 只读Watch记录；唯一写者为本文件的启动/2kHz任务，不作为参数持久化格式。 */
typedef struct
{
    uint32_t raw_counts, sequence, milliseconds, offset_samples;
    float offset_counts, current_a, filtered_a;
    bool offset_valid, sample_valid;
    BusCurrentStatus status;
} BusCurrentDiagnostic;

volatile BusCurrentDiagnostic g_r40_bus_current_diagnostic;
static uint32_t s_offset_sum, s_offset_min, s_offset_max;
static bool s_sequence_seen;

static void reset_offset_window(void)
{
    s_offset_sum = 0U;
    s_offset_min = R40_ADC_MAX_COUNTS;
    s_offset_max = 0U;
    g_r40_bus_current_diagnostic.offset_samples = 0U;
}

void AppBusCurrent_Init(void)
{
    g_r40_bus_current_diagnostic =
        (BusCurrentDiagnostic){.offset_counts = NAN, .current_a = NAN, .filtered_a = NAN};
    s_sequence_seen = false;
    reset_offset_window();
}

static bool collect_offset(uint32_t counts)
{
    if (!power_stage_hw_phases_disabled())
    {
        reset_offset_window();
        return false;
    }
    if (counts < OFFSET_CENTER_COUNTS - OFFSET_TOLERANCE_COUNTS ||
        counts > OFFSET_CENTER_COUNTS + OFFSET_TOLERANCE_COUNTS)
    {
        reset_offset_window();
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_OFFSET_REJECTED;
        return false;
    }
    if (counts < s_offset_min)
    {
        s_offset_min = counts;
    }
    if (counts > s_offset_max)
    {
        s_offset_max = counts;
    }
    if (s_offset_max - s_offset_min > OFFSET_RANGE_COUNTS)
    {
        reset_offset_window();
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_OFFSET_REJECTED;
        return false;
    }
    s_offset_sum += counts;
    ++g_r40_bus_current_diagnostic.offset_samples;
    if (g_r40_bus_current_diagnostic.offset_samples < OFFSET_SAMPLES)
    {
        return false;
    }
    g_r40_bus_current_diagnostic.offset_counts = (float)s_offset_sum / (float)OFFSET_SAMPLES;
    g_r40_bus_current_diagnostic.offset_valid = true;
    return true;
}

void AppBusCurrent_Sample(void)
{
    R40AuxSamples sample;
    if (!R40_AdcReadAux(&sample))
    {
        g_r40_bus_current_diagnostic.sample_valid = false;
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_NO_SAMPLE;
        if (!g_r40_bus_current_diagnostic.offset_valid)
        {
            reset_offset_window();
        }
        return;
    }
    uint32_t now = time_hw_now_ms();
    uint32_t advance = sample.sequence - g_r40_bus_current_diagnostic.sequence;
    if (s_sequence_seen && advance == 0U)
    {
        if ((uint32_t)(now - g_r40_bus_current_diagnostic.milliseconds) > SAMPLE_MAX_AGE_MS)
        {
            g_r40_bus_current_diagnostic.sample_valid = false;
            g_r40_bus_current_diagnostic.status = BUS_CURRENT_NO_SAMPLE;
            if (!g_r40_bus_current_diagnostic.offset_valid)
            {
                reset_offset_window();
            }
        }
        return;
    }
    bool consecutive = s_sequence_seen && advance == 1U;
    s_sequence_seen = true;
    g_r40_bus_current_diagnostic.sequence = sample.sequence;
    g_r40_bus_current_diagnostic.milliseconds = now;
    g_r40_bus_current_diagnostic.raw_counts = sample.bus_current_counts;
    if (sample.bus_current_counts <= RAIL_MARGIN_COUNTS ||
        sample.bus_current_counts >= R40_ADC_MAX_COUNTS - RAIL_MARGIN_COUNTS)
    {
        g_r40_bus_current_diagnostic.sample_valid = false;
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_RAIL;
        if (!g_r40_bus_current_diagnostic.offset_valid)
        {
            reset_offset_window();
        }
        return;
    }
    if (!g_r40_bus_current_diagnostic.offset_valid)
    {
        if (!consecutive)
        {
            reset_offset_window();
        }
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_WAIT_OFFSET;
        if (!collect_offset(sample.bus_current_counts))
        {
            return;
        }
    }
    float current_a =
        ((float)sample.bus_current_counts - g_r40_bus_current_diagnostic.offset_counts) *
        BUS_CURRENT_A_PER_COUNT;
    /* 缺帧或恢复后重置滤波，不把丢失的样本虚构成连续输入；重复帧不推进滤波。 */
    float filtered_a =
        g_r40_bus_current_diagnostic.sample_valid && consecutive
            ? g_r40_bus_current_diagnostic.filtered_a +
                  FILTER_ALPHA * (current_a - g_r40_bus_current_diagnostic.filtered_a)
            : current_a;
    g_r40_bus_current_diagnostic.current_a = current_a;
    g_r40_bus_current_diagnostic.filtered_a = filtered_a;
    g_r40_bus_current_diagnostic.sample_valid = true;
    g_r40_bus_current_diagnostic.status = BUS_CURRENT_VALID;
}

void AppBusCurrent_Apply(float *current_a, uint8_t *valid_bits)
{
    if (current_a == NULL || valid_bits == NULL)
    {
        return;
    }
    uint32_t key = critical_hw_enter();
    /* 同时核对底层故障/新鲜度和装配结果年龄，不能因前台重复查询而刷新样本。 */
    R40AuxSamples sample;
    bool valid = g_r40_bus_current_diagnostic.offset_valid &&
                 g_r40_bus_current_diagnostic.sample_valid &&
                 (uint32_t)(time_hw_now_ms() - g_r40_bus_current_diagnostic.milliseconds) <=
                     SAMPLE_MAX_AGE_MS &&
                 R40_AdcReadAux(&sample);
    *current_a = valid ? g_r40_bus_current_diagnostic.filtered_a : NAN;
    *valid_bits = (*valid_bits & (uint8_t)~0x40U) | (valid ? 0x40U : 0U);
    critical_hw_exit(key);
}
