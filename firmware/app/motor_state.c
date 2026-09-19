#include "motor_state.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_friction_identification.h"
#include "foc_pid.h"
#include "foc_run.h"
#include "foc_run_state.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"

/* 电机与控制对象实例：全部定义在此，其他模块只持有引用。 */
MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef OnBoard_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

/* 上一周期已提交的模式与故障，供迁移判断和变化上报使用。 */
ModeNow_TypeDef ModeLast = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

bool MotorControl_IsConfigurationValid(void)
{
    return MotorControl.axis_profile_valid;
}

void MotorControl_Init(void)
{
    FocRunState_Init();
    Encoder_ParamInit(&OnBoard_Encoder);

    Fluxobserver_ParamInit(&Fluxobserver);
    SensorlessStartup_Reset(&SensorlessStartup);
    FOC_CurrentController_Reset(&FOC);
    PI_Controller_Reset(&PI_Speed);

    MotorControl.pos_error_window = 0.001f;
    MotorControl.pos_vel_filtered = 0.0f;
    Task_Position_Mode_Reset();
    FocFrictionIdentification_Init();

    /* 未配置轴记录的关节在启动后保持禁用。 */
    MotorControl.ModeNow = MotorControl.axis_profile_valid ? Calib_CurrentOffset : Motor_Disable;
    if (!MotorControl.axis_profile_valid)
    {
        Set_ErrorNow(MotorParam_Error);
    }
}
