#include "motor_sensing.h"

#include "control_math.h"

/* 母线电压搬运与低通；不做过压/欠压判定。 */
void MotorVoltage_Update(MotorMeasurement *measurement, float bus_voltage_v)
{
    measurement->bus_voltage_v = bus_voltage_v;
    UTILS_LP_FAST(measurement->bus_voltage_filtered_v, measurement->bus_voltage_v, 0.05f);
}
