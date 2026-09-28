#ifndef BUS_VOLTAGE_PROFILE_H
#define BUS_VOLTAGE_PROFILE_H

/* 8S conventional Li-ion/NMC: 8 * 4.2 V = 33.6 V fully charged.
 * These are measured bus thresholds, not individual-cell/BMS protection.
 *
 * EXPERIMENTAL BRANCH codex/ovp-36v: the two overvoltage thresholds are
 * raised to 36.0 V / 36.2 V, above the board's documented 35 V damage
 * boundary and close to the nominal 36.3 V ADC full scale. Bench experiment
 * only: do not release or treat as a board voltage rating. Undervoltage and
 * enable thresholds are unchanged. No automatic fault recovery.
 */
#define BUS_VOLTAGE_UNDERVOLTAGE_V       24.0f
#define BUS_VOLTAGE_OVERVOLTAGE_V        36.0f
#define BUS_VOLTAGE_HARD_OVERVOLTAGE_V   36.2f
#define BUS_VOLTAGE_ENABLE_MIN_V        25.6f
#define BUS_VOLTAGE_ENABLE_MAX_V        33.8f
#define BUS_VOLTAGE_UNDERVOLTAGE_MS      100U
#define BUS_VOLTAGE_OVERVOLTAGE_MS       2U
#define BUS_VOLTAGE_HARD_CONFIRM_CYCLES  3U

#endif
