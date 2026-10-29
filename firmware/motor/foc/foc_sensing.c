#include "app_feature_requirements.h"
#include "foc_sensing.h"

#include <math.h>

#include "bus_voltage_profile.h"
#include "control_config.h"
#include "motor_fault.h"
#include "mcu_temperature.h"
#include "sampling_scale.h"
#include "bsp_adc.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "control_math.h"

/* 母线电压、相电流与MCU温度的判定、滤波与遥测。
 * 原码由采样BSP提供，固定尺度归platform/config；本文件不访问寄存器。 */

#define OVERCURRENT_CONFIRM_CYCLES 5U

/* 零电流时 ADC 中点应落在 2048 附近；越界即认为零点偏置标定失效。 */
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f

/* 监督 ISR 更新、前台快照读取；私有化不改变原有 volatile 访问。 */
static volatile McuTemperatureTelemetry s_mcu_temperature;

/* 不把2048或上次存储零点当作本次上电校正资格。 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              FocState *foc)
{
    *calibration = (CurrentOffsetCalibration){0};
    configuration->device.phase_a_offset_counts = NAN;
    configuration->device.phase_b_offset_counts = NAN;
    configuration->device.phase_c_offset_counts = NAN;
    foc->phase_a_current_a = NAN;
    foc->phase_b_current_a = NAN;
    foc->phase_c_current_a = NAN;
}

/* 保留采样证据和首个故障；失败后不自动恢复上电校正资格。 */
static void fail_current_offset_calibration(CurrentOffsetCalibration *calibration,
                                            MotorControlState *motor_state)
{
    calibration->failed = true;
    MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
}

bool MotorCurrentOffsets_Sample(CurrentOffsetCalibration *calibration,
                                MotorControlState *motor_state)
{
    if (calibration->complete || calibration->failed ||
        calibration->sample_count >= CURRENT_OFFSET_SAMPLE_COUNT)
    {
        return false;
    }
    if (!BspPwm_ArePhasesDisabled() || motor_state->control_mode != MOTOR_CONTROL_NONE)
    {
        fail_current_offset_calibration(calibration, motor_state);
        return false;
    }
    BspAdcRawCurrent frame;
    if (BspAdc_ReadRawCurrent(&frame) != BSP_OK ||
        (frame.valid_mask & BSP_ADC_FAST_PHASE_CURRENT_VALID) == 0U)
    {
        fail_current_offset_calibration(calibration, motor_state);
        return false;
    }
    uint16_t raw[3];
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        if (!isfinite(frame.phase_counts[phase]) || frame.phase_counts[phase] < 0.0f ||
            frame.phase_counts[phase] > CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
        /* 保留原四次和的整数累加域，20k样本不会引入float累计舍入。 */
        raw[phase] = (uint16_t)(frame.phase_counts[phase] * MOTOR_SENSING_ADC_SAMPLE_COUNT);
        if (raw[phase] > MOTOR_SENSING_ADC_SAMPLE_COUNT * CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
    }
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        calibration->phase_sum_raw[phase] += raw[phase];
    }
    return ++calibration->sample_count == CURRENT_OFFSET_SAMPLE_COUNT;
}

bool MotorCurrentOffsets_Finalize(CurrentOffsetCalibration *calibration,
                                  MotorConfiguration *configuration,
                                  MotorControlState *motor_state)
{
    if (calibration->complete || calibration->failed ||
        calibration->sample_count != CURRENT_OFFSET_SAMPLE_COUNT)
    {
        return false;
    }
    float scale = MOTOR_SENSING_ADC_SUM_TO_COUNTS / (float)calibration->sample_count;
    const float offsets[3] = {(float)calibration->phase_sum_raw[0] * scale,
                              (float)calibration->phase_sum_raw[1] * scale,
                              (float)calibration->phase_sum_raw[2] * scale};
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        if (!isfinite(offsets[phase]) || offsets[phase] < CURRENT_OFFSET_MIN_COUNTS ||
            offsets[phase] > CURRENT_OFFSET_MAX_COUNTS)
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
    }
    uint32_t key = BspSystem_EnterCritical();
    if (!BspPwm_ArePhasesDisabled() || motor_state->control_mode != MOTOR_CONTROL_NONE ||
        BspAdc_SetPhaseOffsets(offsets) != BSP_OK)
    {
        fail_current_offset_calibration(calibration, motor_state);
        BspSystem_ExitCritical(key);
        return false;
    }
    configuration->device.phase_a_offset_counts = offsets[0];
    configuration->device.phase_b_offset_counts = offsets[1];
    configuration->device.phase_c_offset_counts = offsets[2];
    calibration->complete = true;
    BspSystem_ExitCritical(key);
    return true;
}

/* 母线电压采样、滤波与过压/欠压确认计数。 */
void Vbus_Update(FocState *foc,
                 MotorControlState *motor_state,
                 bool output_expected,
                 const BspAdcFastSample *sample)
{
    static uint32_t overvoltage_count, undervoltage_count, hard_overvoltage_count;

    foc->bus_voltage_v =
        sample != NULL && (sample->valid_mask & BSP_ADC_FAST_BUS_VOLTAGE_VALID) != 0U
            ? sample->bus_voltage_v
            : NAN;

    UTILS_LP_FAST(foc->bus_voltage_filtered_v, foc->bus_voltage_v, 0.05f);

    if (output_expected)
    {
        /* 首个故障保持锁存；接近硬件电压上限时原始采样绕过低通，
         * 滤波阈值只用于拒绝短时跌落。 */
        if (motor_state->fault_code != MOTOR_FAULT_NONE)
        {
            return;
        }
        if (!isfinite(foc->bus_voltage_v) || !isfinite(foc->bus_voltage_filtered_v))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
            return;
        }
        if (foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
        {
            if (hard_overvoltage_count < BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                ++hard_overvoltage_count;
            }
            if (hard_overvoltage_count >= BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
                return;
            }
        }
        else
        {
            hard_overvoltage_count = 0U;
        }
        if (foc->bus_voltage_filtered_v >= BUS_VOLTAGE_OVERVOLTAGE_V)
        {
            if (overvoltage_count < (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                ++overvoltage_count;
            }
            if (overvoltage_count >= (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
            }
        }
        else
        {
            overvoltage_count = 0U;
        }

        /* 欠压保护。 */
        if (foc->bus_voltage_filtered_v <= BUS_VOLTAGE_UNDERVOLTAGE_V)
        {
            if (undervoltage_count < (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                ++undervoltage_count;
            }
            if (undervoltage_count >= (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_UNDERVOLTAGE);
            }
        }
        else
        {
            undervoltage_count = 0U;
        }
    }
    else
    {
        overvoltage_count = 0U;
        undervoltage_count = 0U;
        hard_overvoltage_count = 0U;
    }
}

/* 三相电流换算、零点偏置有效性与过流确认计数。 */
void Current_Cal(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 const BspAdcFastSample *sample)
{
    static uint8_t overcurrent_count;

    if (!MotorCurrentOffsets_Valid(configuration))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
    }
    else
    {
        if (sample == NULL || (sample->valid_mask & BSP_ADC_FAST_PHASE_CURRENT_VALID) == 0U)
        {
            foc->phase_a_current_a = foc->phase_b_current_a = foc->phase_c_current_a = NAN;
            MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
        }
        else
        {
            foc->phase_a_current_a = sample->phase_current_a[0];
            foc->phase_b_current_a = sample->phase_current_a[1];
            foc->phase_c_current_a = sample->phase_current_a[2];
        }
    }

    if (Math_Abs(foc->phase_a_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        Math_Abs(foc->phase_b_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        Math_Abs(foc->phase_c_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A)
    {
        if (overcurrent_count < OVERCURRENT_CONFIRM_CYCLES)
        {
            overcurrent_count++;
        }
        if (overcurrent_count >= OVERCURRENT_CONFIRM_CYCLES)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_OVERCURRENT);
        }
    }
    else
    {
        overcurrent_count = 0U;
    }
}

/* 内部温度轮询、结果发布、失效超时与过温跳闸。
 * 板级可选策略只影响故障判定，绝不把缺失样本转换成有效温度。 */
void Temperature_Update(FocState *foc, MotorControlState *motor_state)
{
    struct
    {
        bool sample_ready, conversion_ok;
        uint32_t raw_ts, raw_vref;
        float celsius, vdda_mv;
    } poll = {0};
    static bool raw_seen;
    static uint32_t last_raw_sequence;
    bool good = false;

    /* 软件触发的 ADC1 序列在两次 2 kHz 监督调用之间完成（序列约 30.7 µs）：端口在
     * JEOS 就绪时读取两个 rank、清标志并请求下一次序列。ADC2/PWM 电流采样与
     * 20 kHz 中断不受影响。 */
    (void)BspAdc_UpdateAuxiliary();
    BspAdcRawValue raw;
    if (BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK &&
        (!raw_seen || raw.sequence != last_raw_sequence))
    {
        raw_seen = true;
        last_raw_sequence = raw.sequence;
        poll.sample_ready = true;
        poll.raw_ts = (uint32_t)raw.counts;
        poll.raw_vref = raw.reference_valid ? (uint32_t)raw.reference_counts : 0U;
        BspAdcValue value, supply;
        poll.conversion_ok = BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_OK;
        if (poll.conversion_ok)
        {
            poll.celsius = value.value;
        }
        poll.vdda_mv = BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &supply) == BSP_OK
                           ? supply.value * 1000.0f
                           : NAN;
    }
    if (poll.sample_ready)
    {
        s_mcu_temperature.raw_ts = poll.raw_ts;
        s_mcu_temperature.raw_vref = poll.raw_vref;
        good = poll.conversion_ok;
        if (good)
        {
            /* 0.01 @2 kHz 与原 1 kHz 的 0.02 等效，保持约 50 ms 时间常数。 */
            foc->temperature_c =
                s_mcu_temperature.valid
                    ? foc->temperature_c + 0.01f * (poll.celsius - foc->temperature_c)
                    : poll.celsius;
            s_mcu_temperature.raw_celsius = poll.celsius;
            s_mcu_temperature.vdda_mv = poll.vdda_mv;
            s_mcu_temperature.valid = 1U;
            s_mcu_temperature.missed_ticks = 0U;
            ++s_mcu_temperature.sample_count;
            /* 90°C 比后缀 6 器件的 105°C 结温上限低 15°C，为校准公差与测量
             * 误差留余量；不覆盖绕组/MOSFET 温度。 */
            if (BSP_REQUIRED_MCU_TEMPERATURE && poll.celsius >= MCU_TEMPERATURE_TRIP_C &&
                motor_state->fault_code == MOTOR_FAULT_NONE)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERTEMPERATURE);
            }
        }
        else
        {
            s_mcu_temperature.valid = 0U;
            foc->temperature_c = NAN;
        }
    }
    if (!good)
    {
        if (s_mcu_temperature.missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            ++s_mcu_temperature.missed_ticks;
        }
        if (!s_mcu_temperature.valid ||
            s_mcu_temperature.missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            s_mcu_temperature.valid = 0U;
            foc->temperature_c = NAN;
        }
        if (BSP_REQUIRED_MCU_TEMPERATURE &&
            s_mcu_temperature.missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS &&
            motor_state->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_TEMPERATURE_SENSOR);
        }
    }
}

bool MotorTemperature_IsFresh(void)
{
    return s_mcu_temperature.valid != 0U &&
           s_mcu_temperature.missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS;
}

bool MotorCurrentOffsets_Valid(const MotorConfiguration *configuration)
{
    return !(!isfinite(configuration->device.phase_a_offset_counts) ||
             !isfinite(configuration->device.phase_b_offset_counts) ||
             !isfinite(configuration->device.phase_c_offset_counts) ||
             configuration->device.phase_a_offset_counts < CURRENT_OFFSET_MIN_COUNTS ||
             configuration->device.phase_a_offset_counts > CURRENT_OFFSET_MAX_COUNTS ||
             configuration->device.phase_b_offset_counts < CURRENT_OFFSET_MIN_COUNTS ||
             configuration->device.phase_b_offset_counts > CURRENT_OFFSET_MAX_COUNTS ||
             configuration->device.phase_c_offset_counts < CURRENT_OFFSET_MIN_COUNTS ||
             configuration->device.phase_c_offset_counts > CURRENT_OFFSET_MAX_COUNTS);
}
