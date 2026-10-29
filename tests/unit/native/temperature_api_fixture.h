/* 仅供应用温度保护测试的输入模型；实际采样链路另由真实公共ADC集成测试覆盖。 */
typedef struct
{
    bool sample_ready, conversion_ok;
    uint32_t raw_ts, raw_vref;
    float celsius, vdda_mv;
} TestTemperaturePoll;
/**
 * @brief 为应用保护回归提供一拍测试输入，不连接硬件。
 * @param result 本拍样本和换算状态的调用方输出。
 * @return 无返回值，测试输入状态写入result。
 * @note 仅供离线夹具定义，不是生产BSP接口。
 */
static void TestTemperature_Poll(TestTemperaturePoll *result);
#include "bsp_adc.h"
static TestTemperaturePoll test_temperature_poll;
static uint32_t test_temperature_sequence;
BspResult BspAdc_UpdateAuxiliary(void)
{
    test_temperature_poll = (TestTemperaturePoll){0};
    TestTemperature_Poll(&test_temperature_poll);
    if (test_temperature_poll.sample_ready)
    {
        ++test_temperature_sequence;
        return BSP_OK;
    }
    return BSP_NOT_READY;
}
BspResult BspAdc_ReadRawChannel(BspAdcChannel channel, BspAdcRawValue *raw)
{
    if (channel != BSP_ADC_MCU_TEMPERATURE || !test_temperature_poll.sample_ready)
    {
        return BSP_NOT_READY;
    }
    *raw = (BspAdcRawValue){(float)test_temperature_poll.raw_ts,
                            (float)test_temperature_poll.raw_vref,
                            true,
                            test_temperature_sequence,
                            0U};
    return BSP_OK;
}
BspResult BspAdc_ReadChannel(BspAdcChannel channel, BspAdcValue *value)
{
    if (!test_temperature_poll.conversion_ok)
    {
        return BSP_NOT_CALIBRATED;
    }
    *value = (BspAdcValue){channel == BSP_ADC_ANALOG_SUPPLY_VOLTAGE
                               ? test_temperature_poll.vdda_mv * .001f
                               : test_temperature_poll.celsius,
                           test_temperature_sequence,
                           0U};
    return BSP_OK;
}
