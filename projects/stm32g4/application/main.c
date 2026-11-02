#include "app.h"
#include "bsp.h"
#include "bsp_pwm.h"

static void startup_failed(void)
{
    BspPwm_Disable();
    for (;;)
    {
        /* 保持关相，不喂继承或应用看门狗。 */
    }
}

/**
 * @brief ADC采样完成后执行应用快环。
 * @note 由MCU在ADC中断上下文调用。
 */
void BspApp_OnFastSample(void)
{
    App_FastTick();
}

int main(void)
{
    if (Bsp_Init() != BSP_OK)
    {
        startup_failed();
    }
    App_Init();
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
