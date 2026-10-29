#ifndef MOTOR_MEASUREMENT_H
#define MOTOR_MEASUREMENT_H

/**
 * @brief Axis持有的采样值；母线电压V、相电流A、MCU温度°C。
 * @note 电流/电压由20kHz快拍更新，温度由2kHz监督拍更新；不是同步快照。
 *       采样失效、零偏及新鲜度的处理由各采样接口负责。
 */
typedef struct
{
    float bus_voltage_v, bus_voltage_filtered_v;
    float phase_a_current_a, phase_b_current_a, phase_c_current_a;
    float temperature_c;
} MotorMeasurement;

#endif
