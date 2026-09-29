/* 参数夹具跟随真实服务：单位编码、失败清零与拒绝无副作用，不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "parameter_value_service.h"

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

int main(void)
{
    device_address_does_not_select_application();
    configuration_confirmation();
    read_units_and_identity();
    read_errors_clear_output();
    write_rejection_preserves_configuration();
    puts("PASS parameter value service");
    return 0;
}
