#include "motor_sensing.h"

#include "bsp_adc.h"
#include "control_math.h"

/* 母线电压采样与低通；不做过压/欠压判定。 */
void MotorVoltage_Update(FocState *foc)
{
    foc->bus_voltage_v = BspAdc_ReadChannel(BSP_ADC_BUS_VOLTAGE);
    UTILS_LP_FAST(foc->bus_voltage_filtered_v, foc->bus_voltage_v, foc->bus_voltage_filter_alpha);
}
