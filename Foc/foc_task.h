#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include "main.h"

void MotorControl_Init(void);
void FOC20kHzIRQHandler(void);
/** Run temperature conversion/protection from the existing 1 kHz supervisor. */
void FOC1kHzSupervisor(void);

#endif
