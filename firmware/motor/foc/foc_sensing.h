#ifndef __FOC_SENSING_H__
#define __FOC_SENSING_H__

#include "main.h"
#include "foc_algorithm.h"
#include "data_type.h"

void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
/**
 * @brief 换算并按采样扇区重构相电流，同时保留原始三相过流检查。
 * @param FOC 接收当拍电流反馈，单位 A。
 * @param MotorControl 提供 ADC 零偏和运行模式。
 * @note 仅在完整 ADC 序列结束后、本拍 PWM 写入前调用；零偏校准不重构。
 */
void Current_Cal(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
void Temperature_Update(FOC_TypeDef *FOC);

#endif
