#ifndef BUS_VOLTAGE_PROFILE_H
#define BUS_VOLTAGE_PROFILE_H

/* 8S conventional Li-ion/NMC: 8 * 4.2 V = 33.6 V fully charged.
 * These are measured bus thresholds, not individual-cell/BMS protection.
 * Keep all thresholds below the board's documented 35 V damage boundary
 * and the nominal 36.3 V ADC full scale. No automatic fault recovery.
 */
#define BUS_VOLTAGE_UNDERVOLTAGE_V       24.0f
#define BUS_VOLTAGE_OVERVOLTAGE_V        34.0f
#define BUS_VOLTAGE_HARD_OVERVOLTAGE_V   34.5f
#define BUS_VOLTAGE_ENABLE_MIN_V        25.6f
#define BUS_VOLTAGE_ENABLE_MAX_V        33.8f
#define BUS_VOLTAGE_UNDERVOLTAGE_MS      100U
#define BUS_VOLTAGE_OVERVOLTAGE_MS       2U
#define BUS_VOLTAGE_HARD_CONFIRM_CYCLES  3U

#endif
