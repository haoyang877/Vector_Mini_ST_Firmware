#include "application_config.h"
#include "motor_sensing.h"

#include <math.h>

#include "application_config.h"
#include "motor_fault.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "control_math.h"

/* 相电流与上电零偏。 */
#define OVERCURRENT_CONFIRM_CYCLES 5U
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

/* 三相电流换算、零点偏置有效性与过流确认计数。 */
void MotorCurrent_Update(const MotorConfiguration *configuration,
                         MotorMeasurement *measurement,
                         MotorControlState *motor_state,
                         const BspAdcFastSample *sample)
{
    static uint8_t overcurrent_count;

    if (!MotorCurrentOffsets_IsValid(configuration))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
    }
    else
    {
        if (sample == NULL || (sample->valid_mask & BSP_ADC_FAST_PHASE_CURRENT_VALID) == 0U)
        {
            measurement->phase_a_current_a = measurement->phase_b_current_a =
                measurement->phase_c_current_a = NAN;
            MotorFault_Report(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
        }
        else
        {
            measurement->phase_a_current_a = sample->phase_current_a[0];
            measurement->phase_b_current_a = sample->phase_current_a[1];
            measurement->phase_c_current_a = sample->phase_current_a[2];
        }
    }

    if (Math_Abs(measurement->phase_a_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        Math_Abs(measurement->phase_b_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        Math_Abs(measurement->phase_c_current_a) > MOTOR_SENSING_OVERCURRENT_TRIP_A)
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
