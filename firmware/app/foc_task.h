#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include "main.h"
#include <stdbool.h>

void MotorControl_Init(void);
/** Startup readiness for board glue; querying does not enable the motor. */
bool MotorControl_IsConfigurationValid(void);
void FOC20kHzIRQHandler(void);
/** Run temperature conversion/protection from the existing 1 kHz supervisor. */
void FOC1kHzSupervisor(void);

#endif
