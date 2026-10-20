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
/**
 * @brief 执行摩擦辨识一步，温度判据使用上层提供的统一保护状态。
 * @param foc 电流、母线和MCU温度反馈。
 * @param motor 运行参数与模式状态。
 * @param speed_controller 辨识使用的速度控制器。
 * @param encoder 当前编码器反馈。
 * @param temperature_protection_enabled true检查MCU过温，false旁路温度判据。
 * @note 仍执行电压、编码器、限流及其他故障检查。
 */
void FocFrictionIdentification_Task(FOC_TypeDef *foc, MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller, Encoder_TypeDef *encoder,
    bool temperature_protection_enabled);
bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor);
FrictionIdentificationState_TypeDef FocFrictionIdentification_GetState(void);
FrictionIdentificationReason_TypeDef FocFrictionIdentification_GetReason(void);
uint32_t FocFrictionIdentification_GetPointIndex(void);
float FocFrictionIdentification_GetProgressPercent(void);
uint32_t FocFrictionIdentification_GetSampleCount(void);
const FrictionIdentificationResult_TypeDef *FocFrictionIdentification_GetResult(void);
const FrictionIdentificationSample_TypeDef *FocFrictionIdentification_GetSamples(void);

#endif
