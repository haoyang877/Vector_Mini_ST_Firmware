#include "motor_sensing.h"

#include <math.h>

#include "application_config.h"
#include "motor_fault.h"
#include "bsp_adc.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "control_math.h"

/* 正常电流测量与独立的INIT零偏校正。 */
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f

/* 不把2048或上次存储零点当作本次上电校正资格。 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              FocState *foc,
                              bool bus_current_enabled)
{
    *calibration = (CurrentOffsetCalibration){0};
    calibration->bus_current_enabled = bus_current_enabled;
    configuration->device.phase_a_offset_counts = NAN;
    configuration->device.phase_b_offset_counts = NAN;
    configuration->device.phase_c_offset_counts = NAN;
    foc->phase_current_a = (ABCVect){NAN, NAN, NAN};
    foc->bus_current_a = NAN;
    foc->bus_current_filtered_a = NAN;
}

/* 保留采样证据和首个故障；失败后不自动恢复上电校正资格。 */
static void fail_current_offset_calibration(CurrentOffsetCalibration *calibration,
                                            MotorControlState *motor_state)
{
    calibration->failed = true;
    MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
}

/* 闭区间比较同时拒绝NaN和无穷，不另设校准容差。 */
static bool current_offset_valid(float counts)
{
    return counts >= CURRENT_OFFSET_MIN_COUNTS && counts <= CURRENT_OFFSET_MAX_COUNTS;
}

bool MotorCurrentOffsets_Calibrate(CurrentOffsetCalibration *calibration,
                                   MotorConfiguration *configuration,
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
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        float counts = BspAdc_ReadRawChannel((BspAdcChannel)(BSP_ADC_PHASE_CURRENT_A + phase));
        if (!(counts >= 0.0f && counts <= CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS))
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
        /* 恢复硬件累加整数域，避免逐拍浮点求和。失败后的部分累计不会发布。 */
        calibration->phase_sum_raw[phase] += (uint32_t)(counts * MOTOR_SENSING_ADC_SAMPLE_COUNT);
    }
    if (calibration->bus_current_enabled)
    {
        float counts = BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT);
        if (!(counts >= 0.0f && counts <= CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS))
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
        /* 母线为12位单次采样，不能套用三相的硬件累加倍率。 */
        calibration->bus_sum_raw += (uint32_t)counts;
    }
    ++calibration->sample_count;
    if (calibration->sample_count != CURRENT_OFFSET_SAMPLE_COUNT)
    {
        return false;
    }
    const float scale = BSP_ADC_SUM_TO_COUNTS / (float)CURRENT_OFFSET_SAMPLE_COUNT;
    const float offsets[3] = {(float)calibration->phase_sum_raw[0] * scale,
                              (float)calibration->phase_sum_raw[1] * scale,
                              (float)calibration->phase_sum_raw[2] * scale};
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        if (!current_offset_valid(offsets[phase]))
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
    }
    float bus_offset_counts = (float)calibration->bus_sum_raw / (float)CURRENT_OFFSET_SAMPLE_COUNT;
    if (calibration->bus_current_enabled && !current_offset_valid(bus_offset_counts))
    {
        fail_current_offset_calibration(calibration, motor_state);
        return false;
    }
    uint32_t key = BspSystem_EnterCritical();
    if (!BspPwm_ArePhasesDisabled() || motor_state->control_mode != MOTOR_CONTROL_NONE)
    {
        fail_current_offset_calibration(calibration, motor_state);
        BspSystem_ExitCritical(key);
        return false;
    }
    BspAdc_SetPhaseOffsets(offsets);
    if (calibration->bus_current_enabled)
    {
        BspAdc_SetBusCurrentOffset(bus_offset_counts);
    }
    configuration->device.phase_a_offset_counts = offsets[0];
    configuration->device.phase_b_offset_counts = offsets[1];
    configuration->device.phase_c_offset_counts = offsets[2];
    calibration->complete = true;
    BspSystem_ExitCritical(key);
    return true;
}

/* 依次读取三相和母线电流；零偏换算由BSP负责，母线失效后首个有效值重建滤波基准。 */
void MotorCurrent_Update(FocState *foc)
{
    foc->phase_current_a.a = BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_A);
    foc->phase_current_a.b = BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_B);
    foc->phase_current_a.c = BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_C);

    float bus_current_a = BspAdc_ReadChannel(BSP_ADC_BUS_CURRENT);
    foc->bus_current_a = bus_current_a;
    if (!isfinite(bus_current_a) || !isfinite(foc->bus_current_filtered_a))
    {
        foc->bus_current_filtered_a = bus_current_a;
        return;
    }
    UTILS_LP_FAST(foc->bus_current_filtered_a, foc->bus_current_a, foc->bus_current_filter_alpha);
}

bool MotorCurrentOffsets_IsValid(const MotorConfiguration *configuration)
{
    return current_offset_valid(configuration->device.phase_a_offset_counts) &&
           current_offset_valid(configuration->device.phase_b_offset_counts) &&
           current_offset_valid(configuration->device.phase_c_offset_counts);
}
