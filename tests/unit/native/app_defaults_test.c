/* 使用生产默认值装配和配置校验，检查两板首次上电参数可通过自检。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "parameter_record.h"
#include "parameter_values.h"
#include "application_profile_defaults.h"
#include "motor_profile_defaults.h"
#include "device_profile_defaults.h"

#include "app_defaults_subject.inc"

int main(void)
{
    MotorConfiguration configuration = {0};
    Param_Return_Default(&configuration);
    assert(configuration.application.current_limit_a == EXPECTED_DEFAULT_A);
    assert(configuration.application.current_limit_a <= CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A);
    assert(ParameterValue_ConfigurationValid(&configuration));
    assert(!configuration.configuration_confirmed);
    assert(configuration.device.encoder.calib_flag == 0U);
    configuration.application.current_limit_a = CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A + 1.0f;
    assert(!ParameterValue_ConfigurationValid(&configuration));
    puts("PASS real APP defaults satisfy board current limit without bypassing configuration "
         "guards");
    return 0;
}
