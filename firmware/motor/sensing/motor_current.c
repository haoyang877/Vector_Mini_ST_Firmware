#include "application_config.h"
#include "motor_sensing.h"

#include <math.h>

#include "application_config.h"
#include "motor_fault.h"
#include "bsp_adc.h"
#include "bsp_system.h"
#include "bsp_pwm.h"

/* 相电流与上电零偏。 */
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f

/* 不把2048或上次存储零点当作本次上电校正资格。 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              MotorMeasurement *measurement)
{
    *calibration = (CurrentOffsetCalibration){0};
    configuration->device.phase_a_offset_counts = NAN;
    configuration->device.phase_b_offset_counts = NAN;
    configuration->device.phase_c_offset_counts = NAN;
    measurement->phase_a_current_a = NAN;
    measurement->phase_b_current_a = NAN;
    measurement->phase_c_current_a = NAN;
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
    uint16_t raw[3];
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        float counts = BspAdc_ReadRawChannel((BspAdcChannel)(BSP_ADC_PHASE_CURRENT_A + phase));
        if (!isfinite(counts) || counts < 0.0f || counts > CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            fail_current_offset_calibration(calibration, motor_state);
            return false;
        }
        /* 保留原四次和的整数累加域，20k样本不会引入float累计舍入。 */
        raw[phase] = (uint16_t)(counts * MOTOR_SENSING_ADC_SAMPLE_COUNT);
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
    float scale = BSP_ADC_SUM_TO_COUNTS / (float)calibration->sample_count;
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
    if (!BspPwm_ArePhasesDisabled() || motor_state->control_mode != MOTOR_CONTROL_NONE)
    {
        fail_current_offset_calibration(calibration, motor_state);
        BspSystem_ExitCritical(key);
        return false;
    }
    BspAdc_SetPhaseOffsets(offsets);
    configuration->device.phase_a_offset_counts = offsets[0];
    configuration->device.phase_b_offset_counts = offsets[1];
    configuration->device.phase_c_offset_counts = offsets[2];
    calibration->complete = true;
    BspSystem_ExitCritical(key);
    return true;
}

/* 三相电流搬运；不做零偏、过流或缺失判定。 */
void MotorCurrent_Update(MotorMeasurement *measurement, const float phase_current_a[3])
{
    measurement->phase_a_current_a = phase_current_a[0];
    measurement->phase_b_current_a = phase_current_a[1];
    measurement->phase_c_current_a = phase_current_a[2];
}

bool MotorCurrentOffsets_IsValid(const MotorConfiguration *configuration)
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
