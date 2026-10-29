/* 共用ADC：快帧发布与换算、慢通道缓存、零偏提交；硬件采集留在MCU实现。 */
#include "bsp_pwm.h"
#include "control_config.h"
#include "bsp_internal.h"
#include "sampling_scale.h"
#include <math.h>
#include <stddef.h>
#include "bsp_system.h"

typedef struct
{
    uint16_t phase_sum[3], vbus_sum, bus_counts;
    uint32_t sequence, sampled_us, sampled_ms;
    float phase_offsets[3], bus_offset;
    bool phase_calibrated, bus_calibrated;
} AdcFastFrame;
static volatile AdcFastFrame s_frame;
static volatile uint32_t s_frame_version;
static bool s_adc_initialized, s_adc_started;
static volatile bool s_fast_seen;

typedef struct
{
    BspAdcRawValue raw;
    float value;
    BspResult physical_result;
    bool seen;
} AdcChannelCache;
/* 快通道已有s_frame，只为慢通道保留物理量缓存。 */
static AdcChannelCache s_channels[BSP_ADC_CHANNEL_COUNT - BSP_ADC_MOTOR_TEMPERATURE];
static const bool s_channel_enabled[BSP_ADC_CHANNEL_COUNT] = {BSP_ENABLED_PHASE_CURRENT,
                                                              BSP_ENABLED_PHASE_CURRENT,
                                                              BSP_ENABLED_PHASE_CURRENT,
                                                              BSP_ENABLED_BUS_VOLTAGE,
                                                              BSP_ENABLED_BUS_CURRENT,
                                                              BSP_ENABLED_MOTOR_TEMPERATURE,
                                                              BSP_ENABLED_MCU_TEMPERATURE,
                                                              BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE,
                                                              BSP_ENABLED_HARDWARE_VERSION};

/* 公共采样入口只使用逻辑通道；寄存器rank与采集模式由配置和后端提供。 */
bool Adc_Init(void)
{
    if (s_adc_initialized)
    {
        return false;
    }
    s_adc_initialized = McuAdc_Init();
    return s_adc_initialized;
}
bool Adc_Start(void)
{
    if (!s_adc_initialized || s_adc_started)
    {
        return false;
    }
    s_adc_started = McuAdc_Start();
    return s_adc_started;
}
bool Adc_CompleteFast(void)
{
    if (!McuAdc_CompleteFast())
    {
        return false;
    }
    /* 仅最高优先级采样ISR发布；低优先级读取者不能抢占本次更新。 */
    static const unsigned ranks[3] = {
        BSP_SAMPLE_PHASE_A_RANK, BSP_SAMPLE_PHASE_B_RANK, BSP_SAMPLE_PHASE_C_RANK};
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        s_frame.phase_sum[phase] = McuAdc_ReadRank(ranks[phase]);
    }
    s_frame.vbus_sum = McuAdc_ReadRank(BSP_SAMPLE_VBUS_RANK);
    s_frame.bus_counts = BSP_ENABLED_BUS_CURRENT ? McuAdc_ReadBusCurrent() : 0U;
    s_frame.sampled_us = BspSystem_TimeUs();
    s_frame.sampled_ms = BspSystem_TimeMs();
    ++s_frame.sequence;
    s_fast_seen = true;
    ++s_frame_version;
    return true;
}

#if BSP_ENABLED_AUX_SAMPLING
static volatile AdcAuxSample s_aux;
static volatile bool s_aux_seen;
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
static volatile AdcTemperatureSample s_temperature;
static volatile bool s_temperature_seen;
#endif
uint32_t AdcFrames_PublishAux(const AdcAuxSample *sample, uint32_t now_ms)
{
#if BSP_ENABLED_AUX_SAMPLING
    uint32_t sequence = s_aux.sequence + 1U;
    s_aux = (AdcAuxSample){sample->motor_temperature_counts, sequence, now_ms};
    s_aux_seen = true;
    return sequence;
#else
    (void)sample;
    (void)now_ms;
    return 0U;
#endif
}
uint32_t AdcFrames_PublishTemperature(const AdcTemperatureSample *sample, uint32_t now_ms)
{
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    uint32_t sequence = s_temperature.sequence + 1U;
    s_temperature = (AdcTemperatureSample){
        sample->mcu_temperature_counts, sample->reference_counts, sequence, now_ms};
    s_temperature_seen = true;
    return sequence;
#else
    (void)sample;
    (void)now_ms;
    return 0U;
#endif
}
void AdcFrames_Invalidate(void)
{
    uint32_t key = BspSystem_EnterCritical();
    s_fast_seen = false;
#if BSP_ENABLED_AUX_SAMPLING
    s_aux_seen = false;
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    s_temperature_seen = false;
#endif
    ++s_frame_version;
    BspSystem_ExitCritical(key);
}
bool Adc_ReadAux(AdcAuxSample *out)
{
    if (out == NULL)
    {
        return false;
    }
#if BSP_ENABLED_AUX_SAMPLING
    uint32_t state = BspSystem_EnterCritical();
    bool valid = Adc_IsHealthy() && s_aux_seen &&
                 (uint32_t)(BspSystem_TimeMs() - s_aux.sampled_ms) <= BSP_AUX_MAX_AGE_MS;
    if (valid)
    {
        *out = s_aux;
    }
    BspSystem_ExitCritical(state);
    return valid;
#else
    return false;
#endif
}
bool Adc_ReadTemperature(AdcTemperatureSample *out)
{
    if (out == NULL)
    {
        return false;
    }
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
#if BSP_TEMPERATURE_SOFTWARE_TRIGGER
    AdcTemperatureSample sample;
    if (!McuAdc_PollTemperature(&sample))
    {
        return false;
    }
    sample.sampled_ms = BspSystem_TimeMs();
    sample.sequence = AdcFrames_PublishTemperature(&sample, sample.sampled_ms);
    *out = sample;
    return true;
#else
    uint32_t state = BspSystem_EnterCritical();
    bool valid =
        Adc_IsHealthy() && s_temperature_seen &&
        (uint32_t)(BspSystem_TimeMs() - s_temperature.sampled_ms) <= BSP_TEMPERATURE_MAX_AGE_MS;
    if (valid)
    {
        *out = s_temperature;
    }
    BspSystem_ExitCritical(state);
    return valid;
#endif
#else
    return false;
#endif
}

/* 最多两次无锁快照，连续冲突时仅第三次复制使用短临界区。 */
static BspResult read_frame(AdcFastFrame *frame)
{
    for (unsigned attempt = 0U;; ++attempt)
    {
        bool locked = attempt == 2U;
        uint32_t key = locked ? BspSystem_EnterCritical() : 0U;
        uint32_t version = s_frame_version;
        BspResult result = !s_adc_started || !s_fast_seen ? BSP_NOT_READY
                           : !Adc_IsHealthy()             ? BSP_IO_ERROR
                                                          : BSP_OK;
        if (result == BSP_OK)
        {
            *frame = s_frame;
            uint32_t now_us = BspSystem_TimeUs();
            if (!locked && version != s_frame_version)
            {
                continue;
            }
            if ((uint32_t)(now_us - frame->sampled_us) > 2U * 1000000U / FOC_FREQ)
            {
                result = BSP_STALE;
            }
        }
        if (locked)
        {
            BspSystem_ExitCritical(key);
        }
        return result;
    }
}

BspResult BspAdc_ReadFast(BspAdcFastSample *sample)
{
    if (sample == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_PHASE_CURRENT && !BSP_ENABLED_BUS_VOLTAGE && !BSP_ENABLED_BUS_CURRENT)
    {
        return BSP_NOT_SUPPORTED;
    }
    AdcFastFrame frame;
    BspResult result = read_frame(&frame);
    if (result != BSP_OK)
    {
        return result;
    }
    /* 校验后直接填充；各字段保持同一帧、原运算顺序和独立校准资格。 */
    bool phase_valid = BSP_ENABLED_PHASE_CURRENT && frame.phase_calibrated;
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        sample->phase_current_a[phase] =
            phase_valid ? -((float)frame.phase_sum[phase] * MOTOR_SENSING_ADC_SUM_TO_COUNTS -
                            frame.phase_offsets[phase]) *
                              MOTOR_SENSING_CURRENT_A_PER_COUNT
                        : NAN;
    }
    sample->bus_voltage_v = BSP_ENABLED_BUS_VOLTAGE
                                ? (float)frame.vbus_sum * MOTOR_SENSING_ADC_SUM_TO_COUNTS *
                                      MOTOR_SENSING_VBUS_V_PER_COUNT
                                : NAN;
    sample->valid_mask = (phase_valid ? BSP_ADC_FAST_PHASE_CURRENT_VALID : 0U) |
                         (BSP_ENABLED_BUS_VOLTAGE ? BSP_ADC_FAST_BUS_VOLTAGE_VALID : 0U);
    sample->bus_current_a = NAN;
#if BSP_ENABLED_BUS_CURRENT
    if (frame.bus_calibrated)
    {
        sample->bus_current_a = ((float)frame.bus_counts - frame.bus_offset) *
                                (BUS_CURRENT_ADC_REFERENCE_V / (float)BUS_CURRENT_ADC_MAX_COUNTS /
                                 BUS_CURRENT_GAIN / BUS_CURRENT_SHUNT_OHM);
        sample->valid_mask |= BSP_ADC_FAST_BUS_CURRENT_VALID;
    }
#endif
    sample->sequence = frame.sequence;
    sample->sampled_us = frame.sampled_us;
    return BSP_OK;
}

BspResult BspAdc_ReadRawCurrent(BspAdcRawCurrent *sample)
{
    if (sample == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_PHASE_CURRENT && !BSP_ENABLED_BUS_CURRENT)
    {
        return BSP_NOT_SUPPORTED;
    }
    AdcFastFrame frame;
    BspResult result = read_frame(&frame);
    if (result != BSP_OK)
    {
        return result;
    }
    BspAdcRawCurrent value = {{NAN, NAN, NAN}, NAN, 0U, frame.sequence};
    if (BSP_ENABLED_PHASE_CURRENT)
    {
        for (unsigned phase = 0U; phase < 3U; ++phase)
        {
            value.phase_counts[phase] =
                (float)frame.phase_sum[phase] * MOTOR_SENSING_ADC_SUM_TO_COUNTS;
        }
        value.valid_mask |= BSP_ADC_FAST_PHASE_CURRENT_VALID;
    }
    if (BSP_ENABLED_BUS_CURRENT)
    {
        value.bus_current_counts = (float)frame.bus_counts;
        value.valid_mask |= BSP_ADC_FAST_BUS_CURRENT_VALID;
    }
    *sample = value;
    return BSP_OK;
}

BspResult BspAdc_SetPhaseOffsets(const float offsets_counts[3])
{
    if (offsets_counts == NULL || !isfinite(offsets_counts[0]) || !isfinite(offsets_counts[1]) ||
        !isfinite(offsets_counts[2]))
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_PHASE_CURRENT)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (!s_adc_initialized)
    {
        return BSP_NOT_READY;
    }
    uint32_t key = BspSystem_EnterCritical();
    if (!BspPwm_ArePhasesDisabled())
    {
        BspSystem_ExitCritical(key);
        return BSP_REJECTED;
    }
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        s_frame.phase_offsets[phase] = offsets_counts[phase];
    }
    s_frame.phase_calibrated = true;
    ++s_frame_version;
    BspSystem_ExitCritical(key);
    return BSP_OK;
}

BspResult BspAdc_SetBusCurrentOffset(float offset_counts)
{
    if (!isfinite(offset_counts))
    {
        return BSP_INVALID_ARGUMENT;
    }
#if BSP_ENABLED_BUS_CURRENT
    if (!s_adc_initialized)
    {
        return BSP_NOT_READY;
    }
    uint32_t key = BspSystem_EnterCritical();
    if (!BspPwm_ArePhasesDisabled())
    {
        BspSystem_ExitCritical(key);
        return BSP_REJECTED;
    }
    s_frame.bus_offset = offset_counts;
    s_frame.bus_calibrated = true;
    ++s_frame_version;
    BspSystem_ExitCritical(key);
    return BSP_OK;
#else
    return BSP_NOT_SUPPORTED;
#endif
}

/* physical为空时只读原码；请求物理值时在同一快照内校准，失败不更新输出。 */
static BspResult read_channel(BspAdcChannel channel, BspAdcRawValue *raw, BspAdcValue *physical)
{
    if ((unsigned)channel >= BSP_ADC_CHANNEL_COUNT)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_channel_enabled[channel])
    {
        return BSP_NOT_SUPPORTED;
    }
    if (channel <= BSP_ADC_BUS_CURRENT)
    {
        AdcFastFrame frame;
        BspResult result = read_frame(&frame);
        if (result != BSP_OK)
        {
            return result;
        }
        float counts = channel <= BSP_ADC_PHASE_CURRENT_C
                           ? (float)frame.phase_sum[channel] * MOTOR_SENSING_ADC_SUM_TO_COUNTS
                       : channel == BSP_ADC_BUS_VOLTAGE
                           ? (float)frame.vbus_sum * MOTOR_SENSING_ADC_SUM_TO_COUNTS
                           : (float)frame.bus_counts;
        if (physical != NULL)
        {
            float value;
            if (channel <= BSP_ADC_PHASE_CURRENT_C)
            {
                if (!frame.phase_calibrated)
                {
                    return BSP_NOT_CALIBRATED;
                }
                value =
                    -(counts - frame.phase_offsets[channel]) * MOTOR_SENSING_CURRENT_A_PER_COUNT;
            }
            else if (channel == BSP_ADC_BUS_VOLTAGE)
            {
                value = counts * MOTOR_SENSING_VBUS_V_PER_COUNT;
            }
            else
            {
#if BSP_ENABLED_BUS_CURRENT
                if (!frame.bus_calibrated)
                {
                    return BSP_NOT_CALIBRATED;
                }
                value = (counts - frame.bus_offset) *
                        (BUS_CURRENT_ADC_REFERENCE_V / (float)BUS_CURRENT_ADC_MAX_COUNTS /
                         BUS_CURRENT_GAIN / BUS_CURRENT_SHUNT_OHM);
#else
                return BSP_NOT_SUPPORTED;
#endif
            }
            *physical = (BspAdcValue){value, frame.sequence, frame.sampled_ms};
        }
        else
        {
            *raw = (BspAdcRawValue){counts, 0.0f, false, frame.sequence, frame.sampled_ms};
        }
        return BSP_OK;
    }
    /* 只复制调用方需要的字段；锁内完成，避免整份缓存的中间副本。 */
    uint32_t key = BspSystem_EnterCritical();
    const AdcChannelCache *value = &s_channels[channel - BSP_ADC_MOTOR_TEMPERATURE];
    BspResult result =
        !s_adc_started || !value->seen ? BSP_NOT_READY
        : !Adc_IsHealthy()             ? BSP_IO_ERROR
        : (uint32_t)(BspSystem_TimeMs() - value->raw.sampled_ms) > BSP_TEMPERATURE_MAX_AGE_MS
            ? BSP_STALE
            : BSP_OK;
    if (result == BSP_OK)
    {
        if (physical != NULL)
        {
            result = value->physical_result;
            if (result == BSP_OK)
            {
                *physical = (BspAdcValue){value->value, value->raw.sequence, value->raw.sampled_ms};
            }
        }
        else
        {
            *raw = value->raw;
        }
    }
    BspSystem_ExitCritical(key);
    return result;
}

BspResult BspAdc_ReadRawChannel(BspAdcChannel channel, BspAdcRawValue *sample)
{
    return sample != NULL ? read_channel(channel, sample, NULL) : BSP_INVALID_ARGUMENT;
}

BspResult BspAdc_ReadChannel(BspAdcChannel channel, BspAdcValue *sample)
{
    return sample != NULL ? read_channel(channel, NULL, sample) : BSP_INVALID_ARGUMENT;
}

#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING || BSP_ENABLED_MOTOR_TEMPERATURE
static void
publish_channel(BspAdcChannel channel, BspAdcRawValue raw, float value, BspResult result)
{
    uint32_t key = BspSystem_EnterCritical();
    s_channels[channel - BSP_ADC_MOTOR_TEMPERATURE] = (AdcChannelCache){raw, value, result, true};
    BspSystem_ExitCritical(key);
}

#endif

BspResult BspAdc_UpdateAuxiliary(void)
{
    if (!BSP_ENABLED_INTERNAL_ANALOG_SAMPLING && !BSP_ENABLED_MOTOR_TEMPERATURE &&
        !BSP_ENABLED_HARDWARE_VERSION)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (!s_adc_started)
    {
        return BSP_NOT_READY;
    }
    if (!Adc_IsHealthy())
    {
        return BSP_IO_ERROR;
    }
    bool updated = false;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    AdcTemperatureSample temperature;
    AdcTemperatureValue value;
    const bool fresh =
        Adc_ReadTemperature(&temperature) &&
        (!s_channels[BSP_ADC_MCU_TEMPERATURE - BSP_ADC_MOTOR_TEMPERATURE].seen ||
         temperature.sequence !=
             s_channels[BSP_ADC_MCU_TEMPERATURE - BSP_ADC_MOTOR_TEMPERATURE].raw.sequence);
    if (fresh)
    {
        McuAdc_ConvertTemperature(&temperature, &value);
    }
#if BSP_TEMPERATURE_SOFTWARE_TRIGGER
    /* 保持换算后、发布前请求下一帧；没有完成帧时也可启动空闲ADC。 */
    McuAdc_ContinueTemperature();
#endif
    if (fresh)
    {
        BspAdcRawValue raw = {(float)temperature.mcu_temperature_counts,
                              (float)temperature.reference_counts,
                              true,
                              temperature.sequence,
                              temperature.sampled_ms};
        publish_channel(BSP_ADC_MCU_TEMPERATURE, raw, value.celsius, value.temperature_result);
        raw.counts = raw.reference_counts;
        raw.reference_counts = 0.0f;
        raw.reference_valid = false;
        publish_channel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, raw, value.vdda_v, value.voltage_result);
        updated = true;
    }
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE
    AdcAuxSample aux;
    if (Adc_ReadAux(&aux) && (!s_channels[0].seen || aux.sequence != s_channels[0].raw.sequence))
    {
        BspAdcRawValue raw = {
            (float)aux.motor_temperature_counts, 0.0f, false, aux.sequence, aux.sampled_ms};
        float celsius = NAN;
        BspResult result = McuAdc_MotorTemperature(aux.motor_temperature_counts, &celsius);
        publish_channel(BSP_ADC_MOTOR_TEMPERATURE, raw, celsius, result);
        updated = true;
    }
#endif
    return updated ? BSP_OK : BSP_NOT_READY;
}
