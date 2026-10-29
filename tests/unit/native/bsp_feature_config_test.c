/* 独立包含所有公共硬件契约；能力描述必须与实际编译配置一一对应。 */
#include <assert.h>
#include "app_feature_requirements.h"
#include "bsp_internal.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_can.h"

int main(void)
{
    assert(!BSP_ENABLED_BUS_CURRENT || BSP_SUPPORTED_BUS_CURRENT);
    assert(!BSP_ENABLED_MOTOR_TEMPERATURE || BSP_SUPPORTED_MOTOR_TEMPERATURE);
    assert(!BSP_ENABLED_OUTPUT_SINCOS || BSP_SUPPORTED_OUTPUT_SINCOS);
    assert(BSP_SUPPORTED_BUS_CURRENT == (BSP_BOARD == 2));
#ifdef EXPECT_TEMPERATURE_ENABLED
    assert(BSP_ENABLED_MCU_TEMPERATURE == EXPECT_TEMPERATURE_ENABLED);
    assert(BSP_REQUIRED_MCU_TEMPERATURE == EXPECT_TEMPERATURE_REQUIRED);
    assert(BSP_ENABLED_CAN == EXPECT_CAN_ENABLED);
    assert(BSP_REQUIRED_CAN == EXPECT_CAN_ENABLED);
#endif
    return 0;
}
