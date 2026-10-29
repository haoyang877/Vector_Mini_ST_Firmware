#include "app_feature_requirements.h"
/* 生产温度/准入/恢复函数的定向回归；模拟测量事实，不连接硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "angle_feedback.h"
#include "bus_voltage_profile.h"
#include "foc_sensing.h"
#include "mcu_temperature.h"
#include "bsp_internal.h"
#include "motor_fault.h"
#include "motor_readiness.h"
#include "sampling_scale.h"

#include "temperature_api_fixture.h"

static McuTemperatureTelemetry s_mcu_temperature;
static TestTemperaturePoll s_poll;
static MotorConfiguration s_configuration;
static FocState s_foc;
static MotorControlState s_motor;
static EncoderState s_encoder;
static bool s_encoder_online = true;
static struct
{
    FocState *foc;
    MotorConfiguration *configuration;
    EncoderState *rotor_encoder;
    bool fast_samples_fresh;
} s_context = {&s_foc, &s_configuration, &s_encoder, true};

static void TestTemperature_Poll(TestTemperaturePoll *result)
{
    *result = s_poll;
}

bool Encoder_IsOnline(const EncoderState *encoder)
{
    (void)encoder;
    return s_encoder_online;
}

uint16_t Encoder_GetBadFrameStreak(const EncoderState *encoder)
{
    (void)encoder;
    return s_encoder_online ? 0U : 1U;
}

/* 由测试入口从当前生产文件提取函数，避免复制决策实现。 */
#include "mcu_temperature_policy_subject.inc"

static bool recovered(MotorFaultCode fault)
{
    MotorFaultRecovery recovery = {0};
    return MotorFault_SourceRecovered(&s_configuration,
                                      &recovery,
                                      &s_motor,
                                      &s_foc,
                                      &s_encoder,
                                      fault,
                                      true,
                                      true,
                                      MotorTemperature_IsFresh());
}

int main(void)
{
    s_foc.bus_voltage_v = s_foc.bus_voltage_filtered_v = 29.0f;
    s_configuration.device.phase_a_offset_counts = 2048.0f;
    s_configuration.device.phase_b_offset_counts = 2048.0f;
    s_configuration.device.phase_c_offset_counts = 2048.0f;

    for (unsigned tick = 0U; tick < MCU_TEMPERATURE_TIMEOUT_TICKS + 5U; ++tick)
    {
        Temperature_Update(&s_foc, &s_motor);
    }
    assert(!MotorTemperature_IsFresh() && isnan(s_foc.temperature_c));
    assert(s_motor.fault_code ==
           (BSP_REQUIRED_MCU_TEMPERATURE ? MOTOR_FAULT_TEMPERATURE_SENSOR : MOTOR_FAULT_NONE));
    assert(state_inputs_valid() == !BSP_REQUIRED_MCU_TEMPERATURE);
    uint32_t checks = read_self_test_measurements();
    assert(!(checks & MOTOR_SELF_TEST_TEMPERATURE_VALID));
    assert(recovered(MOTOR_FAULT_TEMPERATURE_SENSOR) == !BSP_REQUIRED_MCU_TEMPERATURE);
    assert(recovered(MOTOR_FAULT_OVERTEMPERATURE) == !BSP_REQUIRED_MCU_TEMPERATURE);

    assert(self_test_checks_complete(MOTOR_SELF_TEST_ALL));
    assert(self_test_checks_complete(MOTOR_SELF_TEST_ALL & ~MOTOR_SELF_TEST_TEMPERATURE_VALID) ==
           !BSP_REQUIRED_MCU_TEMPERATURE);
    for (uint32_t bit = 1U; bit <= MOTOR_SELF_TEST_RESOURCES_RELEASED; bit <<= 1U)
    {
        if (bit != MOTOR_SELF_TEST_TEMPERATURE_VALID)
        {
            assert(!self_test_checks_complete(MOTOR_SELF_TEST_ALL & ~bit));
        }
    }
    assert(!self_test_checks_complete(MOTOR_SELF_TEST_ALL | 256U));
    s_foc.phase_b_current_a = NAN;
    assert(!state_inputs_valid());
    s_foc.phase_b_current_a = 0.0f;
    s_foc.bus_voltage_v = NAN;
    assert(!state_inputs_valid());
    s_foc.bus_voltage_v = 29.0f;
    s_context.fast_samples_fresh = false;
    assert(!state_inputs_valid());
    s_context.fast_samples_fresh = true;
    s_encoder_online = false;
    assert(!(read_self_test_measurements() & MOTOR_SELF_TEST_ENCODER_VALID));
    assert(!recovered(MOTOR_FAULT_ENCODER));
    s_encoder_online = true;
    s_configuration.device.phase_a_offset_counts = NAN;
    assert(!(read_self_test_measurements() & MOTOR_SELF_TEST_CURRENTS_VALID));
    assert(!recovered(MOTOR_FAULT_CURRENT_OFFSET));
    s_configuration.device.phase_a_offset_counts = 2048.0f;
    s_foc.bus_voltage_filtered_v = 20.0f;
    assert(!recovered(MOTOR_FAULT_UNDERVOLTAGE));
    s_foc.bus_voltage_filtered_v = 29.0f;
    assert(recovered(MOTOR_FAULT_UNDERVOLTAGE));
    assert(!recovered(MOTOR_FAULT_OVERCURRENT));
    assert(!recovered(MOTOR_FAULT_WINDING_THERMAL));

    /* 可选策略不自动清除旧故障，仍需所有者处理显式CLEAR。 */
    s_motor.fault_code = MOTOR_FAULT_TEMPERATURE_SENSOR;
    Temperature_Update(&s_foc, &s_motor);
    assert(s_motor.fault_code == MOTOR_FAULT_TEMPERATURE_SENSOR);
    s_motor.fault_code = MOTOR_FAULT_NONE;
    s_poll = (TestTemperaturePoll){
        .sample_ready = true, .conversion_ok = true, .celsius = 100.0f, .vdda_mv = 3300.0f};
    Temperature_Update(&s_foc, &s_motor);
    assert(MotorTemperature_IsFresh() && s_foc.temperature_c == 100.0f);
    assert(s_motor.fault_code ==
           (BSP_REQUIRED_MCU_TEMPERATURE ? MOTOR_FAULT_OVERTEMPERATURE : MOTOR_FAULT_NONE));
    assert(!(read_self_test_measurements() & MOTOR_SELF_TEST_TEMPERATURE_VALID));
    s_motor.fault_code = MOTOR_FAULT_ENCODER;
    Temperature_Update(&s_foc, &s_motor);
    assert(s_motor.fault_code == MOTOR_FAULT_ENCODER);

    memset(&s_mcu_temperature, 0, sizeof(s_mcu_temperature));
    s_poll.celsius = 25.0f;
    Temperature_Update(&s_foc, &s_motor);
    assert(MotorTemperature_IsFresh() && s_foc.temperature_c == 25.0f);
    assert(read_self_test_measurements() & MOTOR_SELF_TEST_TEMPERATURE_VALID);
    assert(state_inputs_valid());
    puts(BSP_REQUIRED_MCU_TEMPERATURE
             ? "PASS required MCU temperature: timeout/hot/recovery/validity preserved"
             : "PASS optional MCU temperature: invalid telemetry, truthful self-test, other guards "
               "retained");
    return 0;
}
