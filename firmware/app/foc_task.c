#include "foc_task.h"

#include "angle_feedback.h"
#include "fast_loop_profile.h"
#include "foc_mode_dispatch.h"
#include "foc_run.h"
#include "foc_run_state.h"
#include "foc_sensing.h"
#include "foc_sensorless.h"
#include "motor_state.h"
#include "rtt_telemetry.h"

/* 20 kHz 快速环入口：本文件只保留周期顺序与阶段划分，
 * 采样、命令分发、模式执行、运行状态与遥测细节分别在 app 各模块内。 */

void FOC1kHzSupervisor(void)
{
    /* MCU 温度与 VREFINT 换算属于慢速监督任务。 */
    Temperature_Update(&FOC);
}

void FOC20kHzIRQHandler(void)
{
    MotorWorkOutcome_TypeDef work_outcome;
    bool encoder_sample_started;

    FAST_PROFILE_BEGIN(FAST_PROFILE_EMPTY);
    FAST_PROFILE_END(FAST_PROFILE_EMPTY);
    FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER_REQUEST);
    encoder_sample_started = Encoder_BeginSample();
    FAST_PROFILE_END(FAST_PROFILE_ENCODER_REQUEST);
    FAST_PROFILE_BEGIN(FAST_PROFILE_SENSING);
    Vbus_Update(&FOC, &MotorControl);

    Current_Cal(&FOC, &MotorControl);
    FAST_PROFILE_END(FAST_PROFILE_SENSING);

    FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER);
    Encoder_CompleteSample(&MotorControl, &OnBoard_Encoder, encoder_sample_started);
    FAST_PROFILE_END(FAST_PROFILE_ENCODER);
    FAST_PROFILE_BEGIN(FAST_PROFILE_COMMANDS);
    /* 命令分发完成后再选择观测器，覆盖首次模式 3 中断。 */
    if (MotorControl.ModeNow != Position_Mode)
    {
        Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);
    }
    FocRunState_CheckFastFaults();
    FAST_PROFILE_END(FAST_PROFILE_COMMANDS);
    MotorOuterLoop_FastTick(&MotorControl, &PI_Speed, &OnBoard_Encoder);
    work_outcome = FocMode_Dispatch();

    FAST_PROFILE_BEGIN(FAST_PROFILE_POST_CONTROL);
    FocRunState_Tick(work_outcome);
    FAST_PROFILE_END(FAST_PROFILE_POST_CONTROL);

    FAST_PROFILE_BEGIN(FAST_PROFILE_TELEMETRY);
    RTT_Sampling();
    FAST_PROFILE_END(FAST_PROFILE_TELEMETRY);
}
