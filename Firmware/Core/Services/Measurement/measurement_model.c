#include "measurement_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float Measurement_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static bool Measurement_ConfigIsValid(const MeasurementModelConfig *config)
{
    uint8_t channel;

    if (config == NULL || config->minimum_valid_offset_adc == NULL ||
        config->maximum_valid_offset_adc == NULL ||
        config->current_a_per_count == NULL)
        return false;
    for (channel = 0U; channel < 3U; channel++)
    {
        if (config->minimum_valid_offset_adc[channel] >
                config->maximum_valid_offset_adc[channel] ||
            !isfinite(config->current_a_per_count[channel]) ||
            config->current_a_per_count[channel] <= 0.0f)
            return false;
    }
    return
        isfinite(config->bus_voltage_v_per_count) && config->bus_voltage_v_per_count > 0.0f &&
        isfinite(config->bus_voltage_filter_alpha) &&
        config->bus_voltage_filter_alpha > 0.0f &&
        config->bus_voltage_filter_alpha <= 1.0f &&
        isfinite(config->overcurrent_trip_a) && config->overcurrent_trip_a > 0.0f &&
        isfinite(config->overvoltage_trip_v) &&
        isfinite(config->undervoltage_trip_v) &&
        config->overvoltage_trip_v > config->undervoltage_trip_v &&
        isfinite(config->maximum_temperature_c) &&
        config->overcurrent_confirm_cycles > 0U &&
        config->voltage_confirm_cycles > 0U &&
        config->temperature_sample_divider > 0U;
}

static bool Measurement_OffsetsAreValid(const MeasurementModelConfig *config)
{
    return config->phase_a_offset_adc >= config->minimum_valid_offset_adc[0] &&
        config->phase_a_offset_adc <= config->maximum_valid_offset_adc[0] &&
        config->phase_b_offset_adc >= config->minimum_valid_offset_adc[1] &&
        config->phase_b_offset_adc <= config->maximum_valid_offset_adc[1] &&
        config->phase_c_offset_adc >= config->minimum_valid_offset_adc[2] &&
        config->phase_c_offset_adc <= config->maximum_valid_offset_adc[2];
}

void MeasurementModel_Reset(MeasurementModelContext *context)
{
    if (context != NULL)
        memset(context, 0, sizeof(*context));
}

bool MeasurementModel_Configure(MeasurementModelContext *context,
    const MeasurementModelConfig *config)
{
    if (context == NULL || !Measurement_ConfigIsValid(config))
        return false;
    context->config = *config;
    context->is_configured = true;
    return true;
}

bool MeasurementModel_Update(MeasurementModelContext *context,
    const MeasurementModelInput *input, MeasurementModelOutput *output)
{
    bool has_overcurrent;
    const MeasurementModelConfig *config;

    if (context == NULL || input == NULL || output == NULL ||
        !context->is_configured)
        return false;
    config = &context->config;

    context->output.faults = MEASUREMENT_FAULT_NONE;
    context->output.bus_voltage_v = input->bus_voltage_adc *
        config->bus_voltage_v_per_count;
    context->output.bus_voltage_filtered_v += config->bus_voltage_filter_alpha *
        (context->output.bus_voltage_v - context->output.bus_voltage_filtered_v);

    if (input->protection_is_active)
    {
        if (context->output.bus_voltage_filtered_v > config->overvoltage_trip_v)
        {
            if (context->overvoltage_count < config->voltage_confirm_cycles)
                context->overvoltage_count++;
            if (context->overvoltage_count >= config->voltage_confirm_cycles)
                context->output.faults = (MeasurementFaultFlags)
                    (context->output.faults | MEASUREMENT_FAULT_OVER_VOLTAGE);
        }
        else
            context->overvoltage_count = 0U;

        if (context->output.bus_voltage_filtered_v < config->undervoltage_trip_v)
        {
            if (++context->undervoltage_count >= config->voltage_confirm_cycles)
            {
                context->output.faults = (MeasurementFaultFlags)
                    (context->output.faults | MEASUREMENT_FAULT_UNDER_VOLTAGE);
                context->undervoltage_count = 0U;
            }
        }
        else
            context->undervoltage_count = 0U;
    }

    if (!Measurement_OffsetsAreValid(config))
    {
        context->output.faults = (MeasurementFaultFlags)
            (context->output.faults | MEASUREMENT_FAULT_CURRENT_OFFSET);
    }
    else
    {
        context->output.phase_a_current_a =
            -((int16_t)input->phase_a_adc - config->phase_a_offset_adc) *
            config->current_a_per_count[0];
        context->output.phase_b_current_a =
            -((int16_t)input->phase_b_adc - config->phase_b_offset_adc) *
            config->current_a_per_count[1];
        context->output.phase_c_current_a =
            -((int16_t)input->phase_c_adc - config->phase_c_offset_adc) *
            config->current_a_per_count[2];
    }

    has_overcurrent =
        Measurement_Abs(context->output.phase_a_current_a) > config->overcurrent_trip_a ||
        Measurement_Abs(context->output.phase_b_current_a) > config->overcurrent_trip_a ||
        Measurement_Abs(context->output.phase_c_current_a) > config->overcurrent_trip_a;
    if (has_overcurrent)
    {
        if (context->overcurrent_count < config->overcurrent_confirm_cycles)
            context->overcurrent_count++;
        if (context->overcurrent_count >= config->overcurrent_confirm_cycles)
            context->output.faults = (MeasurementFaultFlags)
                (context->output.faults | MEASUREMENT_FAULT_OVER_CURRENT);
    }
    else
        context->overcurrent_count = 0U;

    if (++context->temperature_count >= config->temperature_sample_divider)
    {
		context->temperature_count = 0U;
		context->temperature_sampled = true;
		context->temperature_valid = input->temperature_valid &&
			isfinite(input->temperature_c);
		if (context->temperature_valid)
			context->output.temperature_c = input->temperature_c;
    }

	if (config->temperature_protection_enabled &&
		context->temperature_sampled &&
		((!context->temperature_valid &&
		  config->temperature_invalid_is_fault) ||
		 context->output.temperature_c >= config->maximum_temperature_c))
        context->output.faults = (MeasurementFaultFlags)
            (context->output.faults | MEASUREMENT_FAULT_HIGH_TEMPERATURE);

    *output = context->output;
    return true;
}
