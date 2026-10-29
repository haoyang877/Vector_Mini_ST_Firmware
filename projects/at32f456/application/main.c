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
