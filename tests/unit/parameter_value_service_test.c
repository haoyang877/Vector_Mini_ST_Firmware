/* 参数夹具跟随真实服务：单位编码、失败清零与拒绝无副作用，不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "parameter_value_service.h"
#include "application_profile_defaults.h"

static void read_units_and_identity(void)
{
    MotorControlState motor = {0};
    EncoderState encoder = {0};
    ParameterValue value;

    motor.device.node_id = 3U;
    motor.application.speed_ceiling_rad_s = 20.0f;
    motor.application.speed_limit_rad_s = 20.0f;
    motor.application.current_limit_a = 1.25f;
    MotorControlState before = motor;
    assert(ParameterValue_Read(&motor, &encoder, 7U, 1U, &value) == PARAMETER_OK);
    assert(value.value_type == PARAMETER_VALUE_I32 && value.raw_value == 20000U);
    assert(value.parameter_id == 1U && value.revision == 7U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 2U, &value) == PARAMETER_OK);
    assert(value.raw_value == 1250U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 0U, &value) == PARAMETER_OK);
    assert(value.value_type == PARAMETER_VALUE_U32 && value.raw_value == 7U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, &value) == PARAMETER_OK);
    assert(value.raw_value == 3U);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
}

static void read_errors_clear_output(void)
{
    MotorControlState motor = {0};
    EncoderState encoder = {0};
    ParameterValue value;
    const ParameterValue empty = {0};
    const float invalid_values[] = {NAN, INFINITY, 3.0e9f};

    for (unsigned index = 0U; index < sizeof(invalid_values) / sizeof(invalid_values[0]); ++index)
    {
        motor.application.speed_limit_rad_s = invalid_values[index];
        memset(&value, 0x5A, sizeof(value));
        assert(ParameterValue_Read(&motor, &encoder, 7U, 1U, &value) == PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&value, &empty, sizeof(value)) == 0);
    }
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(&motor, &encoder, 7U, 0xFFU, &value) == PARAMETER_UNSUPPORTED);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(NULL, &encoder, 7U, 1U, &value) == PARAMETER_INVALID_ARGUMENT);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(&motor, NULL, 7U, 1U, &value) == PARAMETER_INVALID_ARGUMENT);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 1U, NULL) == PARAMETER_INVALID_ARGUMENT);
}

static void write_rejection_preserves_configuration(void)
{
    MotorControlState motor = {0};
    ParameterWriteRequest request = {
        .parameter_id = 2U,
        .value_type = PARAMETER_VALUE_U32,
        .raw_value = 1000U,
    };

    motor.application.speed_limit_rad_s = 2.0f;
    motor.application.current_limit_a = 1.25f;
    motor.model.motor_flux_wb = 0.01f;
    MotorControlState before = motor;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_BAD_TYPE);
    request.value_type = PARAMETER_VALUE_I32;
    request.raw_value = 0U;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    request.raw_value = UINT32_MAX;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    request.parameter_id = 6U;
    request.raw_value = 1000U;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_READ_ONLY);
    request.parameter_id = 0xFFU;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_UNSUPPORTED);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    request.parameter_id = 2U;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
    before.application.current_limit_a = 1.0f;
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
}

static void configuration_confirmation(void)
{
    MotorControlState motor = {.application.axis_profile_valid = true,
                               .model.motor_pole_pairs = 21,
                               .model.motor_phase_resistance_ohm = 1.0f,
                               .model.motor_d_inductance_h = 0.001f,
                               .model.motor_q_inductance_h = 0.001f,
                               .model.motor_flux_wb = 0.01f,
                               .application.current_limit_a = 1.0f,
                               .model.calibration_current_a = 1.0f,
                               .application.speed_limit_rad_s = 1.0f,
                               .application.speed_acceleration_rad_s2 = 1.0f,
                               .application.speed_deceleration_rad_s2 = 1.0f};
    EncoderState encoder = {0};
    ParameterWriteRequest request = {
        .parameter_id = 0x4DU, .value_type = PARAMETER_VALUE_U32, .raw_value = 1U};
    ParameterValue value;
    MotorWindingThermal_Default(&motor.model.thermal);
    motor.application.speed_feedforward = (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    assert(ParameterValue_ConfigurationValid(&motor));
    assert(ParameterValue_Read(&motor, &encoder, 1U, 0x4DU, &value) == PARAMETER_OK);
    assert(value.raw_value == 0U && value.access == 3U);
    request.raw_value = 2U;
    MotorControlState before = motor;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    request.raw_value = 1U;
    motor.model.motor_flux_wb = NAN;
    before = motor;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    motor.model.motor_flux_wb = 0.01f;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
    assert(motor.configuration_confirmed && encoder.calib_flag == 0U);
    assert(ParameterValue_Read(&motor, &encoder, 2U, 0x4EU, &value) == PARAMETER_OK);
    assert(value.raw_value == MOTOR_CONFIGURATION_ACTIVE && value.access == 1U);
    request.raw_value = 0U;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
    assert(!motor.configuration_confirmed);
}

/* 地址仅属于设备；应用包络已经装配，在线值服务不再据地址选择轮子策略。 */
static void device_address_does_not_select_application(void)
{
    MotorControlState motor = {0};
    EncoderState encoder = {0};
    motor.application.speed_ceiling_rad_s = 20.0f;
    motor.application.speed_limit_rad_s = 2.0f;
    motor.application.position_maximum_speed_rad_s = 1.0f;
    motor.device.node_id = 3U;
    ParameterWriteRequest request = {
        .parameter_id = 1U, .value_type = PARAMETER_VALUE_I32, .raw_value = 15000U};
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
    assert(motor.application.speed_limit_rad_s == 15.0f);
    MotorApplicationConfig application = motor.application;
    motor.device.node_id = 99U;
    ParameterValue value;
    assert(ParameterValue_Read(&motor, &encoder, 1U, 3U, &value) == PARAMETER_OK);
    assert(value.raw_value == 99U);
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
    assert(memcmp(&application, &motor.application, sizeof(application)) == 0);
    request.raw_value = 21000U;
    MotorControlState before = motor;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    motor.application.speed_ceiling_rad_s = NAN;
    request.raw_value = 1000U;
    before = motor;
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
}

/* 整个候选配置通过校验后才能写入，拒绝不得部分改变其他字段或确认状态。 */
static void thermal_and_feedforward_values(void)
{
    const struct
    {
        uint8_t id;
        uint32_t raw, invalid_raw;
    } values[] = {{0x60U, 2500U, 10000U},
                  {0x61U, 6500U, 2000U},
                  {0x62U, 70000U, 0U},
                  {0x63U, 850000U, 600000U},
                  {0x64U, 650000U, 900000U},
                  {0x65U, 950000U, 800000U},
                  {0x66U, 450000U, 700000U},
                  {0x69U, 50000U, 200001U},
                  {0x6AU, 15000U, 0U},
                  {0x6BU, 150000U, 2000001U},
                  {0x6CU, 1000U, 10001U}};
    MotorControlState motor = {.configuration_confirmed = true, .device.node_id = 7U};
    EncoderState encoder = {0};
    MotorWindingThermal_Default(&motor.model.thermal);
    motor.application.speed_feedforward = (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    for (unsigned index = 0U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        ParameterWriteRequest request = {.parameter_id = values[index].id,
                                         .value_type = PARAMETER_VALUE_I32,
                                         .raw_value = values[index].raw};
        assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OK);
        ParameterValue readback;
        assert(ParameterValue_Read(&motor, &encoder, 9U, request.parameter_id, &readback) ==
               PARAMETER_OK);
        assert(readback.raw_value == request.raw_value && readback.revision == 9U);
        assert(readback.value_type == PARAMETER_VALUE_I32 && readback.access == 3U);
        MotorControlState before = motor;
        request.raw_value = values[index].invalid_raw;
        assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&before, &motor, sizeof(motor)) == 0);
        request.raw_value = UINT32_MAX;
        assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&before, &motor, sizeof(motor)) == 0);
        request.value_type = PARAMETER_VALUE_U32;
        request.raw_value = values[index].raw;
        assert(ParameterValue_Apply(&motor, &request) == PARAMETER_BAD_TYPE);
        assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    }
    assert(motor.model.thermal.continuous_vector_a == 2.5f && motor.model.thermal.peak_a == 6.5f);
    assert(motor.model.thermal.heating_tau_s == 70.0f);
    assert(motor.application.speed_feedforward.gain_a_per_rad_s2 == 0.05f);
    assert(motor.application.speed_feedforward.acceleration_limit_rad_s2 == 15.0f);
    assert(motor.application.speed_feedforward.jerk_limit_rad_s3 == 150.0f);
    assert(motor.application.speed_feedforward.current_limit_a == 1.0f);
    assert(motor.configuration_confirmed && motor.device.node_id == 7U);

    ParameterWriteRequest enable = {
        .parameter_id = 0x68U, .value_type = PARAMETER_VALUE_U32, .raw_value = 1U};
    assert(ParameterValue_Apply(&motor, &enable) == PARAMETER_OK);
    ParameterValue readback;
    assert(ParameterValue_Read(&motor, &encoder, 10U, 0x68U, &readback) == PARAMETER_OK);
    assert(readback.raw_value == 1U && readback.value_type == PARAMETER_VALUE_U32);
    MotorControlState before = motor;
    enable.raw_value = 2U;
    assert(ParameterValue_Apply(&motor, &enable) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    enable.raw_value = 0U;
    enable.value_type = PARAMETER_VALUE_I32;
    assert(ParameterValue_Apply(&motor, &enable) == PARAMETER_BAD_TYPE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);

    /* 合法单字段也不能绕过其他字段损坏、热阈值顺序和前馈完整性校验。 */
    motor.model.thermal.derating_exit_heat = 0.99f;
    before = motor;
    ParameterWriteRequest request = {
        .parameter_id = 0x60U, .value_type = PARAMETER_VALUE_I32, .raw_value = 3000U};
    assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    for (unsigned index = 7U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        motor.application.speed_feedforward =
            (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
        if (values[index].id == 0x6AU)
        {
            motor.application.speed_feedforward.jerk_limit_rad_s3 = NAN;
        }
        else
        {
            motor.application.speed_feedforward.acceleration_limit_rad_s2 = NAN;
        }
        before = motor;
        request.parameter_id = values[index].id;
        request.raw_value = values[index].raw;
        assert(ParameterValue_Apply(&motor, &request) == PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    }
    enable.value_type = PARAMETER_VALUE_U32;
    before = motor;
    assert(ParameterValue_Apply(&motor, &enable) == PARAMETER_OUT_OF_RANGE);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    motor.application.speed_feedforward = (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    assert(ParameterValue_Apply(&motor, &enable) == PARAMETER_OK);
    assert(!motor.application.speed_feedforward.enabled);
}

int main(void)
{
    device_address_does_not_select_application();
    configuration_confirmation();
    read_units_and_identity();
    read_errors_clear_output();
    write_rejection_preserves_configuration();
    thermal_and_feedforward_values();
    puts("PASS parameter value service");
    return 0;
}
