#ifndef __FOC_ERRHANDLE_H__
#define __FOC_ERRHANDLE_H__

#include "data_type.h"

ModeNow_TypeDef Get_ModeNow(void);
void Set_ModeNow(ModeNow_TypeDef tModeNow);
ErrorNow_TypeDef Get_ErroNow(void);
void Set_ErrorNow(ErrorNow_TypeDef tErrorNow);
void Clear_RunningData(void);
bool ModeSwitch_Handle(ModeNow_TypeDef mode_set);
void Detect_Mode_Error_Change(void);
bool Return_Mode_Error_Change(void);
void Clear_Mode_Error_Change(void);
void Stop_PWM_Generate(void);
void Stop_PWM_Generate(void);
void Start_PWM_Generate(void);

#endif
