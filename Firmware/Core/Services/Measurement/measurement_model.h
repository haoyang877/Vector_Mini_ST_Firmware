#ifndef CORE_SERVICES_MEASUREMENT_MEASUREMENT_MODEL_H
#define CORE_SERVICES_MEASUREMENT_MEASUREMENT_MODEL_H

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
    float temperature_c;
    bool temperature_valid;
    bool protection_is_active;
} MeasurementModelInput;

typedef struct
{
    int16_t phase_a_offset_adc;
    int16_t phase_b_offset_adc;
    int16_t phase_c_offset_adc;
    const uint16_t *minimum_valid_offset_adc;
    const uint16_t *maximum_valid_offset_adc;
    const float *current_a_per_count;
    float bus_voltage_v_per_count;
    float bus_voltage_filter_alpha;
    float overcurrent_trip_a;
    float overvoltage_trip_v;
    float undervoltage_trip_v;
	float maximum_temperature_c;
	bool temperature_protection_enabled;
	bool temperature_invalid_is_fault;
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
    MeasurementModelConfig config;
    MeasurementModelOutput output;
    uint16_t overvoltage_count;
    uint16_t undervoltage_count;
    uint16_t overcurrent_count;
    uint16_t temperature_count;
	bool temperature_valid;
	bool temperature_sampled;
    bool is_configured;
} MeasurementModelContext;

void MeasurementModel_Reset(MeasurementModelContext *context);
bool MeasurementModel_Configure(MeasurementModelContext *context,
    const MeasurementModelConfig *config);
bool MeasurementModel_Update(MeasurementModelContext *context,
    const MeasurementModelInput *input, MeasurementModelOutput *output);

#endif
