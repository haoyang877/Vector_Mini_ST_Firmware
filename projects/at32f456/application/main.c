#include "app.h"
#include "hardware_check.h"
#include "application_config.h"
#include "motor_axis.h"
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "interrupt_map.h"

/* 该板中断向量只服务本机应用。 */
AppContext g_app;

#if defined(AT32_KEIL_CALIBRATION) && AT32_KEIL_CALIBRATION
/* Watch 单次写入：1 请求 LUT 标定，2 请求停机。前台消费后清零。 */
volatile uint32_t s_keil_calibration_command;
#endif

/* RTT 只读观测板级实例；不复制反馈。 */
const MotorAxisFeedback MotorAxis_Feedback = {.linear_hall = &g_app.axis.linear_hall};

void BspApp_OnCanReceive(void)
{
    Protocol_RxInterrupt(&g_app.endpoint);
}

#if OUTER_LOOP_DIVIDER != SUPERVISOR_DIVIDER
/* ADC 为单一派发者；PendSV 完成后才允许派发下一拍。 */
static volatile bool s_outer_outstanding;

/** @brief 同优先级的监督与外环串行执行；ADC 可以抢占此处理。 */
void PendSV_Handler(void)
{
    MotorAxis_OuterTick(&g_app.axis);
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
    MotorAxis_SupervisorTick(&g_app.axis);
    RTT_Sampling();
}

/** @brief 确认并清除ADC完成事件后直接执行轴快环，完成后推进健康计数。 */
void MCU_ADC_IRQ_HANDLER(void)
{
    if (BspAdc_CompleteFast())
    {
        MotorAxis_SampleFast(&g_app.axis);
        MotorAxis_ControlFast(&g_app.axis);
        if (g_hardware_check_capture)
        {
            HardwareCheck_CaptureFast(&g_app.axis);
        }

#if OUTER_LOOP_DIVIDER != SUPERVISOR_DIVIDER
        static unsigned outer_ticks;
        if (++outer_ticks == OUTER_LOOP_DIVIDER)
        {
            outer_ticks = 0U;
            if (s_outer_outstanding)
            {
                MotorAxis_OuterOverrun(&g_app.axis);
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

static void update_status_lights(void)
{
    static uint32_t last_poll_ms;
    static uint32_t pattern_start_ms;
    static MotorState previous_state = MOTOR_STATE_SELF_TEST;
    static bool previous_check;

    uint32_t now_ms = BspSystem_TimeMs();
    if (now_ms - last_poll_ms < 50U)
    {
        return;
    }
    last_poll_ms = now_ms;
    MotorStatus status;
    if (!MotorStateMachine_ReadStatus(&g_app.axis.state_machine, &status))
    {
        return;
    }
    MotorState state = status.fault != MOTOR_FAULT_NONE ? MOTOR_STATE_FAULT : status.state;
    bool checking = state == MOTOR_STATE_IDLE && g_hardware_check_capture;
    if (state != previous_state || checking != previous_check)
    {
        pattern_start_ms = now_ms;
        previous_state = state;
        previous_check = checking;
    }

    uint32_t elapsed_ms = now_ms - pattern_start_ms;
    bool blue = false;
    switch (state)
    {
    case MOTOR_STATE_CALIBRATION:
    {
        uint32_t phase_ms = elapsed_ms % 1000U;
        blue = phase_ms < 100U || (phase_ms >= 200U && phase_ms < 300U);
        break;
    }
    case MOTOR_STATE_RUNNING:
        blue = true;
        break;
    case MOTOR_STATE_IDLE:
        blue = elapsed_ms % (checking ? 200U : 2000U) < 100U;
        break;
    default:
        break;
    }
    BspSystem_WriteStatusLights(state == MOTOR_STATE_FAULT, blue);
}

int main(void)
{
    if (Bsp_Init() != BSP_OK)
    {
        startup_failed();
    }
    g_app.endpoint.hardware_check = HardwareCheck_Bind(&g_app.axis);
    App_Init(&g_app);
    RTT_Init();
    BspSystem_StartCycleCounter();
    if (Bsp_Start() != BSP_OK)
    {
        startup_failed();
    }
    App_Start(&g_app);
    for (;;)
    {
#if defined(AT32_KEIL_CALIBRATION) && AT32_KEIL_CALIBRATION
        uint32_t command = s_keil_calibration_command;
        if (command != 0U)
        {
            s_keil_calibration_command = 0U;
            if (command == 1U)
            {
                (void)MotorStateMachine_RequestCalibration(&g_app.axis.state_machine,
                                                           MOTOR_CALIBRATION_ENCODER_LINEARIZATION);
            }
            else if (command == 3U)
            {
                (void)MotorStateMachine_RequestCalibration(&g_app.axis.state_machine,
                                                           MOTOR_CALIBRATION_ELECTRICAL_ZERO);
            }
            else if (command == 2U)
            {
                MotorStateMachine_RequestStop(&g_app.axis.state_machine);
            }
        }
#endif
        App_Process(&g_app);
        HardwareCheck_Process(&g_app.axis);
        update_status_lights();
    }
}
