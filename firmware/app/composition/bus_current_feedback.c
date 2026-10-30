#include "bus_current_feedback.h"

#include <math.h>
#include <stddef.h>
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "application_config.h"

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

volatile BusCurrentDiagnostic g_at32_bus_current_diagnostic;
static uint32_t s_offset_sum, s_offset_min, s_offset_max;
static bool s_sample_seen;

static void reset_offset_window(void)
{
    s_offset_sum = 0U;
    s_offset_min = BUS_CURRENT_ADC_MAX_COUNTS;
    s_offset_max = 0U;
    g_at32_bus_current_diagnostic.offset_samples = 0U;
}

static void invalidate_sample(BusCurrentStatus status)
{
    g_at32_bus_current_diagnostic.sample_valid = false;
    g_at32_bus_current_diagnostic.status = status;
    if (!g_at32_bus_current_diagnostic.offset_valid)
    {
        reset_offset_window();
    }
}

void BusCurrentFeedback_Init(void)
{
    g_at32_bus_current_diagnostic =
        (BusCurrentDiagnostic){.offset_counts = NAN, .current_a = NAN, .filtered_a = NAN};
    s_sample_seen = false;
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
        g_at32_bus_current_diagnostic.status = BUS_CURRENT_OFFSET_REJECTED;
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
        g_at32_bus_current_diagnostic.status = BUS_CURRENT_OFFSET_REJECTED;
        return false;
    }
    s_offset_sum += counts;
    ++g_at32_bus_current_diagnostic.offset_samples;
    if (g_at32_bus_current_diagnostic.offset_samples < OFFSET_SAMPLES)
    {
        return false;
    }
    g_at32_bus_current_diagnostic.offset_counts = (float)s_offset_sum / (float)OFFSET_SAMPLES;
    BspAdc_SetBusCurrentOffset(g_at32_bus_current_diagnostic.offset_counts);
    g_at32_bus_current_diagnostic.offset_valid = true;
    return true;
}

void BusCurrentFeedback_Sample(void)
{
    if (!BSP_ENABLED_BUS_CURRENT)
    {
        return;
    }
    uint32_t counts = (uint32_t)BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT);
    uint32_t now = BspSystem_TimeMs();
    bool consecutive =
        s_sample_seen &&
        (uint32_t)(now - g_at32_bus_current_diagnostic.milliseconds) <= SAMPLE_MAX_AGE_MS;
    s_sample_seen = true;
    /* 此序号只记录监督读数次数，不表示ADC触发次数。 */
    ++g_at32_bus_current_diagnostic.sequence;
    g_at32_bus_current_diagnostic.milliseconds = now;
    g_at32_bus_current_diagnostic.raw_counts = counts;
    if (counts <= RAIL_MARGIN_COUNTS || counts >= BUS_CURRENT_ADC_MAX_COUNTS - RAIL_MARGIN_COUNTS)
    {
        invalidate_sample(BUS_CURRENT_RAIL);
        return;
    }
    if (!g_at32_bus_current_diagnostic.offset_valid)
    {
        if (!consecutive)
        {
            reset_offset_window();
        }
        g_at32_bus_current_diagnostic.status = BUS_CURRENT_WAIT_OFFSET;
        if (!collect_offset(counts))
        {
            return;
        }
    }
    float current_a = BspAdc_ReadChannel(BSP_ADC_BUS_CURRENT);
    /* 监督任务间隔过长时重置滤波；ADC只提供数值。 */
    float filtered_a =
        g_at32_bus_current_diagnostic.sample_valid && consecutive
            ? g_at32_bus_current_diagnostic.filtered_a +
                  FILTER_ALPHA * (current_a - g_at32_bus_current_diagnostic.filtered_a)
            : current_a;
    g_at32_bus_current_diagnostic.current_a = current_a;
    g_at32_bus_current_diagnostic.filtered_a = filtered_a;
    g_at32_bus_current_diagnostic.sample_valid = true;
    g_at32_bus_current_diagnostic.status = BUS_CURRENT_VALID;
}

bool BusCurrentFeedback_Read(float *current_a)
{
    if (current_a == NULL)
    {
        return false;
    }
    uint32_t key = BspSystem_EnterControlCritical();
    float filtered_a = g_at32_bus_current_diagnostic.filtered_a;
    bool valid = g_at32_bus_current_diagnostic.offset_valid &&
                 g_at32_bus_current_diagnostic.sample_valid &&
                 (uint32_t)(BspSystem_TimeMs() - g_at32_bus_current_diagnostic.milliseconds) <=
                     SAMPLE_MAX_AGE_MS;
    BspSystem_ExitControlCritical(key);
    *current_a = valid ? filtered_a : NAN;
    return valid;
}
