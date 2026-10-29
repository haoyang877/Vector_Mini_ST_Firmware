#ifndef MCU_TEMPERATURE_H
#define MCU_TEMPERATURE_H

/* 共用监督策略默认值，未经热箱验证；芯片换算由各板温度BSP负责。 */
#define MCU_TEMPERATURE_TRIP_C 90.0f
#define MCU_TEMPERATURE_TIMEOUT_MS 100U

#endif
