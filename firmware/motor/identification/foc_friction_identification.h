#ifndef __FOC_FRICTION_IDENTIFICATION_H__
#define __FOC_FRICTION_IDENTIFICATION_H__

#include "data_type.h"
#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "friction_identification.h"

void FocFrictionIdentification_Init(void);
bool FocFrictionIdentification_Start(MotorControl_TypeDef *motor,
	Encoder_TypeDef *encoder, PI_Controller_TypeDef *speed_controller);
void FocFrictionIdentification_Abort(MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller);
void FocFrictionIdentification_Task(FOC_TypeDef *foc, MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller, Encoder_TypeDef *encoder);
bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor);
FrictionIdentificationState_TypeDef FocFrictionIdentification_GetState(void);
FrictionIdentificationReason_TypeDef FocFrictionIdentification_GetReason(void);
uint32_t FocFrictionIdentification_GetPointIndex(void);
float FocFrictionIdentification_GetProgressPercent(void);
uint32_t FocFrictionIdentification_GetSampleCount(void);
const FrictionIdentificationResult_TypeDef *FocFrictionIdentification_GetResult(void);
const FrictionIdentificationSample_TypeDef *FocFrictionIdentification_GetSamples(void);

#endif
