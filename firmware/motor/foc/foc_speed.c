#include "foc_speed.h"

#include "control_config.h"

/* 速度环实现：参考斜坡、速度 PI 更新与顺序任务入口。
 * 斜坡由速度模式与无感启动共用；顺序入口供摩擦辨识与外部调用。 */

/* 速度参考斜坡：把 speedShadow 按加减速限制推进到 speedRef。
 * 速度模式与无感启动共用；两者对时基、单位与所有权的要求一致。 */
void MotorControl_UpdateSpeedRamp(MotorControl_TypeDef *MotorControl)
{
    MotorControl->isUseSpeedRamp = MotorControl->speedAcc > 0.0f && MotorControl->speedDec > 0.0f;

    if (MotorControl->isUseSpeedRamp)
    {
        if (MotorControl->speedRef > MotorControl->speedShadow)
        {
            MotorControl->speedShadow += MotorControl->speedAcc * Speed_Ts;
            if (MotorControl->speedShadow > MotorControl->speedRef)
            {
                MotorControl->speedShadow = MotorControl->speedRef;
            }
        }
        else if (MotorControl->speedRef < MotorControl->speedShadow)
        {
            MotorControl->speedShadow -= MotorControl->speedDec * Speed_Ts;
            if (MotorControl->speedShadow < MotorControl->speedRef)
            {
                MotorControl->speedShadow = MotorControl->speedRef;
            }
        }
    }
    else
    {
        MotorControl->speedShadow = MotorControl->speedRef;
    }
}

/**
 * @brief  速度模式控制更新：斜坡推进速度参考并生成速度环输出。
 * @param  MotorControl 电机控制状态指针，读写 speedShadow、idRef 与 iqRef。
 * @param  controller 速度 PI 控制器指针。
 * @param  vel_mech 机械角速度反馈，单位 rad/s。
 * @note 在快速环上下文调用；归一化输出按 current_limit 缩放为电流参考。
 */
void SpeedMode_UpdateControl(MotorControl_TypeDef *MotorControl,
                             PI_Controller_TypeDef *controller,
                             float vel_mech)
{
    MotorControl_UpdateSpeedRamp(MotorControl);

    PI_Controller_Configure(
        controller, MotorControl->speed_Kp, MotorControl->speed_Ki, Speed_Ts, -1.0f, 1.0f);
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = PI_Controller_Run(controller, MotorControl->speedShadow, vel_mech) *
                          MotorControl->current_limit;
}

/**
 * @brief  顺序速度模式任务核心：分频执行速度环并保持电流闭环。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  theta_elec 电角度，单位 rad。
 * @param  vel_elec 电角速度，单位 rad/s。
 * @param  vel_mech 机械角速度反馈，单位 rad/s。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 */
void SpeedMode_Run(FOC_TypeDef *FOC,
                   MotorControl_TypeDef *MotorControl,
                   PI_Controller_TypeDef *controller,
                   float theta_elec,
                   float vel_elec,
                   float vel_mech)
{
    static unsigned speedloop_count;

    if (++speedloop_count >= SPEED_LOOP_DIVIDER)
    {
        SpeedMode_UpdateControl(MotorControl, controller, vel_mech);
        speedloop_count = 0U;
    }
    FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}
