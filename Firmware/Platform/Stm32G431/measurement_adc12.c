#include "measurement_adc12.h"

#include "adc.h"

static bool MeasurementAdc12_ReadRawSample(void *context,
	MeasurementRawSample *sample)
{
	(void)context;
	if (sample == 0)
		return false;

	sample->phase_a_adc = (uint16_t)ADC2->JDR1;
	sample->phase_b_adc = (uint16_t)ADC2->JDR2;
	sample->phase_c_adc = (uint16_t)ADC2->JDR3;
	sample->bus_voltage_adc = (uint16_t)ADC2->JDR4;
	sample->temperature_adc = (uint16_t)ADC1->JDR1;
	return true;
}

MeasurementPort MeasurementAdc12_CreatePort(void)
{
	MeasurementPort port;

	port.context = 0;
	port.read_raw_sample = MeasurementAdc12_ReadRawSample;
	return port;
}
