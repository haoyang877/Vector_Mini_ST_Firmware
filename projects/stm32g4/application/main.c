#include "app.h"
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "interrupt_map.h"

/* 该板中断向量只服务本机应用。 */
AppContext g_app;

void BspApp_OnCanReceive(void)
{
    Protocol_RxInterrupt(&g_app.endpoint);
}

void BspApp_OnFastSample(void)
{
    MotorAxis_SampleFast(&g_app.axis);
    MotorAxis_ControlFast(&g_app.axis);
    ++g_app.fast_samples;
}

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
    App_SupervisorTick(&g_app.axis, &g_app.supervisor_ticks);
}

/** @brief 将ADC向量交给本板BSP，保留完成确认与回调顺序。 */
void MCU_ADC_IRQ_HANDLER(void)
{
    BspAdc_HandleInterrupt();
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
    App_Init(&g_app);
    if (Bsp_Start() != BSP_OK)
    {
        startup_failed();
    }
    App_Start(&g_app);
    for (;;)
    {
        App_Process(&g_app);
    }
}
