#ifndef __FOC_SENSING_H__
#define __FOC_SENSING_H__

#include "main.h"
#include "foc_algorithm.h"
#include "data_type.h"

void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Current_Cal(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Temperature_Update(FOC_TypeDef *FOC);

typedef struct {
    uint32_t raw_ts, raw_vref, valid, missed_ms, sample_count;
    float vdda_mv, raw_celsius;
} McuTemperatureTelemetry;
extern volatile McuTemperatureTelemetry McuTemperature;

#endif
