#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "virtual_platform.h"
#include "inverter_model.h"
#include "motor_axis.h"
#include "motor_state_machine.h"
#include "parameter_record.h"
#include "control_config.h"
#include "motor_profile_defaults.h"
#include "board_profile_defaults.h"

#define RPM_PER_RAD_S (60.0f / 6.283185307f)
static float s_peak_phase_a;
static FILE *s_trace;

/* 与控制器独立的最小物理定律检查，不调用FOC坐标变换。 */
static bool check_models(void)
{
    float duty[3] = {0.25f, 0.75f, 0.75f}, voltage[3];
    if (!InverterModel_Apply(duty, 33.0f, true, voltage) || fabsf(voltage[0] - 11.0f) > 0.0001f ||
        fabsf(voltage[0] + voltage[1] + voltage[2]) > 0.0001f)
    {
        return false;
    }
    VirtualMotorConfig config = {.phase_resistance_ohm = 2.0f,
                                 .d_inductance_h = 0.01f,
                                 .q_inductance_h = 0.01f,
                                 .flux_wb = 0.02f,
                                 .pole_pairs = 4U,
                                 .inertia_kg_m2 = 0.01f};
    MotorModelState rotor = {0};
    const float rl_voltage[3] = {10.0f, -5.0f, -5.0f};
    for (unsigned step = 0U; step < 10000U; ++step)
    {
        if (!MotorModel_Step(&config, &rotor, rl_voltage, true, 0.000001f))
        {
            return false;
        }
    }
    float expected_a = 5.0f * (1.0f - expf(-2.0f));
    if (fabsf(rotor.id_a - expected_a) > 0.0005f || fabsf(rotor.velocity_rad_s) > 0.00001f)
    {
        return false;
    }
    rotor = (MotorModelState){.velocity_rad_s = 10.0f};
    if (!InverterModel_Apply(duty, 33.0f, false, voltage) || voltage[0] != 0.0f ||
        !MotorModel_Step(&config, &rotor, voltage, false, 0.001f) ||
        rotor.velocity_rad_s != 10.0f || fabs(rotor.angle_rad - 0.01) > 0.000001)
    {
        return false;
    }
    rotor = (MotorModelState){.velocity_rad_s = 10.0f};
    if (!MotorModel_Step(&config, &rotor, voltage, true, 0.00001f) || rotor.iq_a >= 0.0f)
    {
        return false;
    }
    rotor = (MotorModelState){.id_a = 2.0f};
    for (unsigned step = 0U; step < 1000U; ++step)
    {
        if (!MotorModel_Step(&config, &rotor, voltage, false, 0.000001f))
        {
            return false;
        }
    }
    return fabsf(rotor.id_a - 2.0f * expf(-0.2f)) < 0.0001f;
}

static bool advance(unsigned ticks)
{
    if (!VirtualPlatform_Advance(ticks))
    {
        fputs("model/sampling/scheduling failed\n", stderr);
        return false;
    }
    VirtualSnapshot snapshot = VirtualPlatform_Read();
    MotorStatus status;
    uint8_t valid;
    MotorMaintenanceKind maintenance;
    MotorControlProgress progress;
    MotorState state = MotorAxis_ReadStatus(&status, &valid, &maintenance, &progress);
    s_peak_phase_a = snapshot.peak_phase_current_a;
    if (s_trace != NULL)
    {
        fprintf(s_trace,
                "%llu,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%u\n",
                (unsigned long long)snapshot.time_us,
                (unsigned)state,
                (unsigned)status.fault,
                (double)(status.speed_target * RPM_PER_RAD_S),
                (double)(snapshot.rotor.velocity_rad_s * RPM_PER_RAD_S),
                (double)(status.speed_feedback_rad_s * RPM_PER_RAD_S),
                (double)status.current_reference,
                (double)status.current_feedback,
                snapshot.phases_disabled ? 1U : 0U);
    }
    if (status.fault != 0U || snapshot.sample_saturated || s_peak_phase_a > 6.5f)
    {
        fprintf(stderr,
                "fault=%u state=%u model_rpm=%.6f peak_phase_a=%.6f\n",
                (unsigned)status.fault,
                (unsigned)state,
                (double)(snapshot.rotor.velocity_rad_s * RPM_PER_RAD_S),
                (double)s_peak_phase_a);
        return false;
    }
    return true;
}

static bool target(float rpm, uint16_t sequence)
{
    const MotorControlCommand command = {.mode = MOTOR_CONTROL_SPEED,
                                         .speed_rad_s = rpm / RPM_PER_RAD_S,
                                         .source_node = 2U,
                                         .sequence = sequence};
    MotorControlResult result = MotorStateMachine_SubmitControl(&command);
    if (result != MOTOR_CONTROL_ACCEPTED)
    {
        fprintf(stderr, "target rejected: %u\n", (unsigned)result);
        return false;
    }
    return true;
}

static bool speed_segment(float rpm, uint16_t sequence)
{
    if (!target(rpm, sequence))
    {
        return false;
    }
    double sum = 0.0, squared = 0.0, feedback_sum = 0.0;
    unsigned samples = 0U;
    for (unsigned tick = 0U; tick < FOC_FREQ * 2U; tick += 10U)
    {
        if (!advance(10U) || MotorStateMachine_GetState() != MOTOR_STATE_RUNNING)
        {
            return false;
        }
        if (tick >= FOC_FREQ * 3U / 2U)
        {
            VirtualSnapshot snapshot = VirtualPlatform_Read();
            MotorStatus status;
            uint8_t valid;
            MotorMaintenanceKind maintenance;
            MotorControlProgress progress;
            (void)MotorAxis_ReadStatus(&status, &valid, &maintenance, &progress);
            if (progress.state != MOTOR_CONTROL_PROGRESS_APPLIED ||
                progress.result != MOTOR_CONTROL_ACCEPTED || progress.source_node != 2U ||
                progress.sequence != sequence ||
                fabsf(status.speed_target * RPM_PER_RAD_S - rpm) > 0.01f)
            {
                return false;
            }
            double actual = snapshot.rotor.velocity_rad_s * RPM_PER_RAD_S;
            sum += actual;
            squared += actual * actual;
            feedback_sum += status.speed_feedback_rad_s * RPM_PER_RAD_S;
            ++samples;
        }
    }
    double mean = sum / samples;
    double deviation = sqrt(fmax(0.0, squared / samples - mean * mean));
    double feedback_mean = feedback_sum / samples;
    bool success = fabs(mean - rpm) <= fmax(2.0, fabs(rpm) * 0.05) && deviation <= 2.0 &&
                   fabs(mean - feedback_mean) <= 2.0;
    printf("{\"segment_rpm\":%.1f,\"mean_rpm\":%.6f,\"std_rpm\":%.6f,"
           "\"feedback_rpm\":%.6f,\"success\":%s}\n",
           (double)rpm,
           mean,
           deviation,
           feedback_mean,
           success ? "true" : "false");
    return success;
}

int main(int argc, char **argv)
{
    unsigned substeps = argc == 2 ? (unsigned)strtoul(argv[1], NULL, 10) : 10U;
    if (!check_models())
    {
        fputs("independent model checks failed\n", stderr);
        return 1;
    }
    /* 保留生产默认电机/硬件/应用参数；只确认配置和理想传感器已知标定。 */
    MotorConfiguration controller;
    InterfaceParam record;
    Param_Return_Default(&controller);
    controller.configuration_confirmed = true;
    controller.device.encoder = (EncoderCalibration){.calib_flag = ENC_CALIB_ALL};
    Param_Upload(&record, &controller);
    FILE *seed = fopen("parameter_seed.bin", "wb");
    if (seed == NULL)
    {
        return 1;
    }
    bool seed_written = fwrite(&record, 1U, sizeof(record), seed) == sizeof(record);
    if (fclose(seed) != 0 || !seed_written)
    {
        return 1;
    }
    const VirtualConfig config = {
        .bus_voltage_v = 33.0f,
        .temperature_c = 25.0f,
        .phase_offset_counts = {PARAM_HW_CURRENT_OFFSET_A_COUNTS,
                                PARAM_HW_CURRENT_OFFSET_B_COUNTS,
                                PARAM_HW_CURRENT_OFFSET_C_COUNTS},
        .motor_connected = true,
        .integration_substeps = substeps,
        .motor = {.phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM,
                  .d_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H,
                  .q_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H,
                  .flux_wb = PARAM_MOTOR_FLUX_WB,
                  .pole_pairs = PARAM_MOTOR_POLE_PAIRS,
                  .inertia_kg_m2 = 0.002f,
                  .damping_nm_per_rad_s = 0.002f}};
    printf("{\"fixture\":\"synthetic_ideal_encoder\",\"independent_model_checks\":true,"
           "\"schema\":%u,\"record_crc32\":%u,"
           "\"model_R_ohm\":%.6f,\"model_Ld_h\":%.9f,\"model_Lq_h\":%.9f,"
           "\"model_flux_wb\":%.9f,\"model_pole_pairs\":%u,\"model_J_kg_m2\":%.6f,"
           "\"model_B_nm_per_rad_s\":%.6f,\"bus_voltage_v\":%.1f,"
           "\"mechanical_parameters_measured\":false,\"hardware_profile\":%u,"
           "\"motor_profile\":%u,\"shunt_milliohm\":%u,\"amplifier_gain\":%.1f,"
           "\"adc_bits\":%u,\"adc_reference_v\":%.1f,\"adc_sample_count\":%u,"
           "\"controller_speed_limit_rpm\":%.6f,\"controller_current_limit_a\":%.1f}\n",
           (unsigned)record.schema_version,
           record.record_crc32,
           (double)config.motor.phase_resistance_ohm,
           (double)config.motor.d_inductance_h,
           (double)config.motor.q_inductance_h,
           (double)config.motor.flux_wb,
           config.motor.pole_pairs,
           (double)config.motor.inertia_kg_m2,
           (double)config.motor.damping_nm_per_rad_s,
           (double)config.bus_voltage_v,
           (unsigned)FOC_ACTIVE_HW_PROFILE,
           (unsigned)FOC_ACTIVE_MOTOR_PROFILE,
           (unsigned)CURRENT_SENSE_SHUNT_MILLIOHM,
           (double)CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN,
           (unsigned)CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS,
           (double)CURRENT_SENSE_PROFILE_ADC_REFERENCE_V,
           (unsigned)MOTOR_SENSING_ADC_SAMPLE_COUNT,
           (double)(controller.application.speed_limit_rad_s * RPM_PER_RAD_S),
           (double)controller.application.current_limit_a);
    if (!VirtualPlatform_LoadParameters(&record, sizeof(record)) || !VirtualPlatform_Init(&config))
    {
        fputs("fixture initialization failed\n", stderr);
        return 1;
    }
    s_trace = fopen("speed.csv", "w");
    if (s_trace == NULL)
    {
        return 1;
    }
    fputs(
        "time_us,state,fault,target_rpm,true_rpm,feedback_rpm,iq_target_a,iq_feedback_a,disabled\n",
        s_trace);
    bool success = advance(FOC_FREQ * 3U / 2U);
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    const MotorControlStartRequest request = {
        .mode = MOTOR_CONTROL_SPEED, .source_node = 2U, .sequence = 10U};
    success = success && MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
              !VirtualPlatform_Read().power_was_enabled && readiness.start_blockers == 0U &&
              MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED &&
              advance(FOC_FREQ / 10U) && MotorStateMachine_GetState() == MOTOR_STATE_RUNNING;
    const float speeds[] = {10.0f, 20.0f, 30.0f, 20.0f, 0.0f};
    for (unsigned index = 0U; success && index < 5U; ++index)
    {
        success = speed_segment(speeds[index], (uint16_t)(11U + index));
    }
    success = success && target(20.0f, 16U) && advance(FOC_FREQ);
    float before_stop_rpm = VirtualPlatform_Read().rotor.velocity_rad_s * RPM_PER_RAD_S;
    /* 无论此前是否失败，都提出原STOP；失败绝不伪装完成或自动重启。 */
    bool stop_accepted = MotorStateMachine_RequestIdentifiedStop(2U, 20U);
    bool stopped = advance(FOC_FREQ / 20U);
    VirtualSnapshot final = VirtualPlatform_Read();
    MotorStatus status;
    uint8_t valid;
    MotorMaintenanceKind maintenance;
    MotorControlProgress progress;
    (void)MotorAxis_ReadStatus(&status, &valid, &maintenance, &progress);
    success = success && stop_accepted && stopped &&
              MotorStateMachine_GetState() == MOTOR_STATE_IDLE && final.phases_disabled &&
              final.power_was_enabled && status.mode == MOTOR_CONTROL_NONE &&
              status.speed_target == 0.0f && status.current_reference == 0.0f &&
              fabsf(final.rotor.iq_a) < 0.01f && fabsf(final.rotor.id_a) < 0.01f &&
              before_stop_rpm > 10.0f && final.rotor.velocity_rad_s > 0.0f;
    printf("{\"success\":%s,\"substeps\":%u,\"peak_phase_a\":%.6f,"
           "\"before_stop_rpm\":%.6f,\"after_stop_rpm\":%.6f,\"phases_disabled\":%s}\n",
           success ? "true" : "false",
           substeps,
           (double)s_peak_phase_a,
           (double)before_stop_rpm,
           (double)(final.rotor.velocity_rad_s * RPM_PER_RAD_S),
           final.phases_disabled ? "true" : "false");
    if (fclose(s_trace) != 0)
    {
        success = false;
    }
    return success ? 0 : 1;
}
