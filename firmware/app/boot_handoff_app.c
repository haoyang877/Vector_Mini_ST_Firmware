/* 组成层连接控制所有者、纯交接状态机和硬件端口。 */
#include "boot_handoff_app.h"
#include "boot_handoff.h"
#include "boot_hw.h"
#include "comm_hw.h"
#include "foc_errhandle.h"
#include "foc_task.h"
#include "motor_hw.h"
#include "time_hw.h"

extern MotorControl_TypeDef MotorControl;
extern ModeNow_TypeDef ModeLast;

void BootHandoff_AppPoll(void)
{
    uint8_t reply[4] = {0};
    BootAction action;
    if (!BootHandoff_Pending())
    {
        return;
    }
    MotorControl_LockMaintenance();
    motor_hw_outer_barrier();
    action = BootHandoff_Poll(time_hw_now_ms(),
                              MotorControl.ModeNow == Motor_Disable && ModeLast == Motor_Disable,
                              motor_hw_phase_outputs_disabled(), comm_hw_can_reply_sent());
    if (action == BOOT_STOP)
    {
        (void)ModeSwitch_Handle(Motor_Disable);
    }
    else if (action == BOOT_REPLY_OK || action == BOOT_REPLY_FAILED)
    {
        reply[3] = action == BOOT_REPLY_OK ? 0 : 4;
        if (comm_hw_can_try_send_reply((uint16_t)((BootHandoff_Node() << 8) | 0x66), reply, 4))
        {
            BootHandoff_ReplyQueued(time_hw_now_ms());
        }
    }
    else if (action == BOOT_RESET)
    {
        boot_hw_reset_to_loader(BootHandoff_Node(), time_hw_now_ms());
    }
}
