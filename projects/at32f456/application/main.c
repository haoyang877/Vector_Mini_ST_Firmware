#include "app.h"
#include "application_config.h"
#include "motor_axis.h"
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "interrupt_map.h"

#if OUTER_LOOP_DIVIDER != SUPERVISOR_DIVIDER
/* ADC 为单一派发者；PendSV 完成后才允许派发下一拍。 */
static volatile bool s_outer_outstanding;

/** @brief 同优先级的监督与外环串行执行；ADC 可以抢占此处理。 */
void PendSV_Handler(void)
{
    MotorAxis_OuterTick();
    s_outer_outstanding = false;
}
#endif

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
    RTT_Sampling();
}

/** @brief 确认并清除ADC完成事件后直接执行轴快环，完成后推进健康计数。 */
void MCU_ADC_IRQ_HANDLER(void)
{
    if (BspAdc_CompleteFast())
    {
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
        ++g_app_fast_samples;
#if OUTER_LOOP_DIVIDER != SUPERVISOR_DIVIDER
        static unsigned outer_ticks;
        if (++outer_ticks == OUTER_LOOP_DIVIDER)
        {
            outer_ticks = 0U;
            if (s_outer_outstanding)
            {
                MotorAxis_OuterOverrun();
            }
            else
            {
                s_outer_outstanding = true;
                BspSystem_PendOuterTick();
            }
        }
#endif
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
    RTT_Init();
    BspSystem_StartCycleCounter();
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
