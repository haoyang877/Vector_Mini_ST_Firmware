#ifndef DOMAIN_MEASUREMENT_MODEL_H
#define DOMAIN_MEASUREMENT_MODEL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MEASUREMENT_FAULT_NONE = 0U,
    MEASUREMENT_FAULT_OVER_VOLTAGE = 1U << 0,
    MEASUREMENT_FAULT_UNDER_VOLTAGE = 1U << 1,
    MEASUREMENT_FAULT_CURRENT_OFFSET = 1U << 2,
    MEASUREMENT_FAULT_OVER_CURRENT = 1U << 3,
    MEASUREMENT_FAULT_HIGH_TEMPERATURE = 1U << 4
} MeasurementFaultFlags;

typedef struct
{
    uint16_t phase_a_adc;
    uint16_t phase_b_adc;
    uint16_t phase_c_adc;
    uint16_t bus_voltage_adc;
    uint16_t temperature_adc;
    bool protection_is_active;
} MeasurementModelInput;

typedef struct
{
    int16_t phase_a_offset_adc;
    int16_t phase_b_offset_adc;
    int16_t phase_c_offset_adc;
    int16_t minimum_valid_offset_adc;
    int16_t maximum_valid_offset_adc;
    float current_a_per_count;
    float bus_voltage_v_per_count;
    float overcurrent_trip_a;
    float overvoltage_trip_v;
    float undervoltage_trip_v;
    float thermistor_series_resistance_kohm;
    float thermistor_nominal_resistance_kohm;
	float thermistor_beta_k;
	float thermistor_nominal_temperature_c;
	float maximum_temperature_c;
	bool temperature_protection_enabled;
	uint16_t overcurrent_confirm_cycles;
    uint16_t voltage_confirm_cycles;
    uint16_t temperature_sample_divider;
} MeasurementModelConfig;

typedef struct
{
    float phase_a_current_a;
    float phase_b_current_a;
    float phase_c_current_a;
    float bus_voltage_v;
    float bus_voltage_filtered_v;
    float temperature_c;
    MeasurementFaultFlags faults;
} MeasurementModelOutput;

typedef struct
{
    MeasurementModelOutput output;
    uint16_t overvoltage_count;
    uint16_t undervoltage_count;
    uint16_t overcurrent_count;
    uint16_t temperature_count;
} MeasurementModelContext;

void MeasurementModel_Reset(MeasurementModelContext *context);
bool MeasurementModel_Update(MeasurementModelContext *context,
    const MeasurementModelConfig *config, const MeasurementModelInput *input,
    MeasurementModelOutput *output);

#endif
