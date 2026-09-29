/* 参数夹具跟随真实服务：单位编码、失败清零与拒绝无副作用，不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "parameter_value_service.h"

static void read_units_and_identity(void)
{
    MotorControl_TypeDef motor = {0};
    Encoder_TypeDef encoder = {0};
    ParameterValue value;

    motor.speed_limit = 20.0f;
    motor.current_limit = 1.25f;
    MotorControl_TypeDef before = motor;
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 1U, &value) == PARAMETER_OK);
    assert(value.value_type == PARAMETER_VALUE_I32 && value.raw_value == 20000U);
    assert(value.parameter_id == 1U && value.revision == 7U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 2U, &value) == PARAMETER_OK);
    assert(value.raw_value == 1250U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 0U, &value) == PARAMETER_OK);
    assert(value.value_type == PARAMETER_VALUE_U32 && value.raw_value == 7U);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 3U, &value) == PARAMETER_OK);
    assert(value.raw_value == 3U);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
}

static void read_errors_clear_output(void)
{
    MotorControl_TypeDef motor = {0};
    Encoder_TypeDef encoder = {0};
    ParameterValue value;
    const ParameterValue empty = {0};
    const float invalid_values[] = {NAN, INFINITY, 3.0e9f};

    for (unsigned index = 0U; index < sizeof(invalid_values) / sizeof(invalid_values[0]); ++index)
    {
        motor.speed_limit = invalid_values[index];
        memset(&value, 0x5A, sizeof(value));
        assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 1U, &value) ==
               PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&value, &empty, sizeof(value)) == 0);
    }
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 0xFFU, &value) ==
           PARAMETER_UNSUPPORTED);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(NULL, &encoder, 7U, 3U, 1U, &value) ==
           PARAMETER_INVALID_ARGUMENT);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    memset(&value, 0x5A, sizeof(value));
    assert(ParameterValue_Read(&motor, NULL, 7U, 3U, 1U, &value) ==
           PARAMETER_INVALID_ARGUMENT);
    assert(memcmp(&value, &empty, sizeof(value)) == 0);
    assert(ParameterValue_Read(&motor, &encoder, 7U, 3U, 1U, NULL) ==
           PARAMETER_INVALID_ARGUMENT);
}

static void write_rejection_preserves_configuration(void)
{
    MotorControl_TypeDef motor = {0};
    ParameterWriteRequest request = {
        .parameter_id = 2U,
        .value_type = PARAMETER_VALUE_U32,
        .raw_value = 1000U,
    };

    motor.speed_limit = 2.0f;
    motor.current_limit = 1.25f;
    motor.motor_flux = 0.01f;
    MotorControl_TypeDef before = motor;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_BAD_TYPE);
    request.value_type = PARAMETER_VALUE_I32;
    request.raw_value = 0U;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_OUT_OF_RANGE);
    request.raw_value = UINT32_MAX;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_OUT_OF_RANGE);
    request.parameter_id = 6U;
    request.raw_value = 1000U;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_READ_ONLY);
    request.parameter_id = 0xFFU;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_UNSUPPORTED);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    request.parameter_id = 2U;
    assert(ParameterValue_Apply(&motor, 3U, &request) == PARAMETER_OK);
    before.current_limit = 1.0f;
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
}

int main(void)
{
    read_units_and_identity();
    read_errors_clear_output();
    write_rejection_preserves_configuration();
    puts("PASS parameter value service");
    return 0;
}
