#include "app.h"
#include "motor_axis.h"
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "interrupt_map.h"
#include "SEGGER_RTT.h"

/* RTT仅由ADC中断写入；主机未读取或缓冲满时丢整帧，不等待。 */
static char s_rtt_buffer[4096];

static void startup_failed(void)
{
    BspPwm_Disable();
    for (;;)
    {
        /* 保持关相，不喂继承或应用看门狗。 */
    }
}

/** @brief 在BSP确认监督事件后执行应用慢环。 */
void BspApp_OnControlTick(void)
{
    App_SupervisorTick();
}

/** @brief 确认并清除ADC完成事件后直接执行轴快环，完成后推进健康计数。 */
void MCU_ADC_IRQ_HANDLER(void)
{
    if (BspAdc_CompleteFast())
    {
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
        ++g_app_fast_samples;
        MotorAxisTrace trace = MotorAxis_ReadTrace();
        (void)SEGGER_RTT_WriteSkipNoLock(1U, &trace, sizeof(trace));
    }
}

/** @brief 将监督定时器向量交给本板BSP。 */
void MCU_CONTROL_IRQ_HANDLER(void)
{
    BspSystem_HandleControlInterrupt();
}

/** @brief 将1ms时基向量交给本板BSP。 */
void MCU_SYSTICK_IRQ_HANDLER(void)
{
    BspSystem_HandleTickInterrupt();
}

int main(void)
{
    if (Bsp_Init() != BSP_OK)
    {
        startup_failed();
    }
    App_Init();
    (void)SEGGER_RTT_ConfigUpBuffer(1U,
                                    "JScope_f4f4f4f4f4f4",
                                    s_rtt_buffer,
                                    sizeof(s_rtt_buffer),
                                    SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    if (Bsp_Start() != BSP_OK)
    {
        startup_failed();
    }
    App_Start();
    for (;;)
    {
        App_Process();
    }
}
