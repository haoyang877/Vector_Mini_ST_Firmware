#include "can_status_source.h"

#include <math.h>
#include "data_type.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_sensing.h"
#include "foc_errhandle.h"

/* 遥测采集实现：只读电机与控制状态，前台按需组装快照。
 * 不再由快速环发布，也不在中断中读取。 */

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef OnBoard_Encoder;

void CanStatus_BuildSnapshot(MotorStatus *sample)
{
    sample->fault = (uint16_t)MotorControl.ErrorNow;
    sample->mode = (uint16_t)MotorControl.ModeNow;
    sample->position_target = MotorControl.posRef;
    sample->position_feedback = Encoder_GetMecPos(&OnBoard_Encoder);
    sample->speed_target = MotorControl.speedRef;
    sample->speed_feedback =
        (MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Position_Impedance_Mode)
            ? MotorControl.pos_vel_filtered
            : Encoder_GetMecVel(&OnBoard_Encoder);
    sample->current_reference = MotorControl.iqRef;
    sample->current_feedback = FOC.Iq;
    sample->temperature = FOC.temp;
    sample->bus_voltage = FOC.Vbus_filt;
    sample->bus_current = FOC.Ibus_filt;
    sample->position_planned = NAN;
    sample->speed_planned = NAN;
    if (MotorControl.ModeNow == Position_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.pos_trajectory_speed_rad_s;
    }
    else if (MotorControl.ModeNow == Position_Impedance_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.speedShadow;
    }
    else if (MotorControl.ModeNow == Speed_Mode)
    {
        sample->speed_planned = MotorControl.speedShadow;
    }
}

bool CanStatus_HeartbeatArmed(void)
{
    return MotorControl.ModeNow == Current_Mode || MotorControl.ModeNow == Speed_Mode ||
           MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Position_Impedance_Mode;
}
