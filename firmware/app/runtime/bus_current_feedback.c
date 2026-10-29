#include "bus_current_feedback.h"

#include <math.h>
#include <stddef.h>
#include "bsp_system.h"
#include "bsp_config.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "control_config.h"

/* 独立母线电流测量；只依赖采样/关相/时间契约，不处理APP或协议有效位。
 * PA4为12位单次采样域，不能使用三相四次和域或4mΩ的相电流系数。
 * 每次上电仅在关相时采128个连续、稳定样本；运行中不跟踪零点。 */
#define OFFSET_SAMPLES 128U
#define OFFSET_RANGE_COUNTS 16U
#define OFFSET_CENTER_COUNTS 2048U
#define OFFSET_TOLERANCE_COUNTS 256U
#define RAIL_MARGIN_COUNTS 2U
#define SAMPLE_MAX_AGE_MS 2U
#define FILTER_ALPHA 0.05f

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
    s_offset_min = BUS_CURRENT_ADC_MAX_COUNTS;
    s_offset_max = 0U;
    g_r40_bus_current_diagnostic.offset_samples = 0U;
}

void BusCurrentFeedback_Init(void)
{
    g_r40_bus_current_diagnostic =
        (BusCurrentDiagnostic){.offset_counts = NAN, .current_a = NAN, .filtered_a = NAN};
    s_sequence_seen = false;
    reset_offset_window();
}

static bool collect_offset(uint32_t counts)
{
    if (!BspPwm_ArePhasesDisabled())
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
    if (BspAdc_SetBusCurrentOffset(g_r40_bus_current_diagnostic.offset_counts) != BSP_OK)
    {
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_OFFSET_REJECTED;
        return false;
    }
    g_r40_bus_current_diagnostic.offset_valid = true;
    return true;
}

void BusCurrentFeedback_Sample(void)
{
    if (!BSP_ENABLED_BUS_CURRENT)
    {
        return;
    }
    BspAdcRawCurrent sample;
    if (BspAdc_ReadRawCurrent(&sample) != BSP_OK ||
        (sample.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) == 0U)
    {
        g_r40_bus_current_diagnostic.sample_valid = false;
        g_r40_bus_current_diagnostic.status = BUS_CURRENT_NO_SAMPLE;
        if (!g_r40_bus_current_diagnostic.offset_valid)
        {
            reset_offset_window();
        }
        return;
    }
    uint32_t now = BspSystem_TimeMs();
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
    const uint32_t expected_advance = FOC_FREQ / SUPERVISOR_FREQ;
    bool consecutive =
        s_sequence_seen && advance >= expected_advance - 1U && advance <= expected_advance + 1U;
    s_sequence_seen = true;
    g_r40_bus_current_diagnostic.sequence = sample.sequence;
    g_r40_bus_current_diagnostic.milliseconds = now;
    g_r40_bus_current_diagnostic.raw_counts = sample.bus_current_counts;
    if (sample.bus_current_counts <= RAIL_MARGIN_COUNTS ||
        sample.bus_current_counts >= BUS_CURRENT_ADC_MAX_COUNTS - RAIL_MARGIN_COUNTS)
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
    BspAdcFastSample physical;
    if (BspAdc_ReadFast(&physical) != BSP_OK ||
        (physical.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) == 0U)
    {
        g_r40_bus_current_diagnostic.sample_valid = false;
        return;
    }
    float current_a = physical.bus_current_a;
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

bool BusCurrentFeedback_Read(float *current_a)
{
    if (current_a == NULL)
    {
        return false;
    }
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t key = BspSystem_EnterCritical();
        uint32_t sequence = g_r40_bus_current_diagnostic.sequence;
        uint32_t milliseconds = g_r40_bus_current_diagnostic.milliseconds;
        float filtered_a = g_r40_bus_current_diagnostic.filtered_a;
        bool offset_valid = g_r40_bus_current_diagnostic.offset_valid;
        bool sample_valid = g_r40_bus_current_diagnostic.sample_valid;
        BspSystem_ExitCritical(key);

        /* ADC快照和换算在锁外执行，不能阻塞20kHz采样中断。 */
        BspAdcRawCurrent sample;
        bool healthy = BspAdc_ReadRawCurrent(&sample) == BSP_OK &&
                       (sample.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) != 0U;
        key = BspSystem_EnterCritical();
        bool unchanged = sequence == g_r40_bus_current_diagnostic.sequence &&
                         milliseconds == g_r40_bus_current_diagnostic.milliseconds &&
                         offset_valid == g_r40_bus_current_diagnostic.offset_valid &&
                         sample_valid == g_r40_bus_current_diagnostic.sample_valid;
        bool fresh = (uint32_t)(BspSystem_TimeMs() - milliseconds) <= SAMPLE_MAX_AGE_MS;
        BspSystem_ExitCritical(key);
        if (unchanged)
        {
            bool valid = offset_valid && sample_valid && fresh && healthy;
            *current_a = valid ? filtered_a : NAN;
            return valid;
        }
    }
    /* 连续两次被生产者更新时保守返回失效，不无限重试或刷新样本年龄。 */
    *current_a = NAN;
    return false;
}
