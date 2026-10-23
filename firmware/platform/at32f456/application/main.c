#include "app.h"
#include <math.h>
#include "app_bus_current.h"
#include "app_telemetry.h"
#include "motor_axis.h"
#include "motor_state_machine.h"
#include "at32f45x.h"
#include "at32f45x_debug.h"
#include "at32f45x_wdt.h"
#include "board_hw.h"
#include "boot_hw.h"
#include "comm_hw.h"
#include "critical_hw.h"
#include "fast_loop_profile.h"
#include "flux_observer.h"
#include "motor_sensing.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "r40_can.h"
#include "r40_indicator.h"
#include "r40_platform.h"
#include "r40_power_window.h"
#include "time_hw.h"
#include "yg_protocol_canfd_irq.h"

#if !defined(R40_APP_PHASE_LOCK) || R40_APP_PHASE_LOCK != 1
#error "Shared R40 APP requires permanent phase lock"
#endif

/* 完整共享APP装配；验证窗口只限制功率，不另建电机算法或写参数自测。 */
typedef struct
{
    uint32_t begin, magic, version, phase_lock, milliseconds;
    uint32_t main_iterations, fast_samples, supervisor_ticks, adc_healthy, max_fast_cycles;
    uint32_t fatal_code, motor_state, motor_fault, motor_mode, valid_bits;
    float bus_v, position_rad, speed_rad_s;
    uint32_t raw_a, raw_b, raw_c, rejected_starts;
    uint32_t uid[3], led_output, cctrl, brk, gpioa, gpiob, can_healthy, end;
} R40AppSnapshot;

volatile R40AppSnapshot g_r40_app_snapshot;
/* 仅0..4指示灯诊断，不提供功率控制；0恢复APP指示。 */
volatile uint32_t g_r40_app_led_test;
/* 复位保留诊断不参与控制；上电后重新建档，WDT/软件复位后保留最近异常现场。 */
typedef struct
{
    uint32_t magic, boot_count, reset_flags, last_fatal;
    uint32_t last_uptime_ms, adc_fault, cfsr, hfsr;
    uint32_t mmfar, bfar, msp, fast_samples;
    uint32_t main_iterations, max_fast_cycles, can_rx_irqs, fault_count;
    uint32_t stack_used_bytes, stack_size_bytes;
    uint32_t max_critical_cycles, max_critical_caller, adc_stall_context[4];
} R40ResetDiagnostic;
_Static_assert(sizeof(R40ResetDiagnostic) <= 128U, "Reset diagnostic exceeds reserved RAM");
volatile R40ResetDiagnostic g_r40_reset_diagnostic __attribute__((section(".bss.r40_noinit")));
extern volatile uint32_t g_r40_rejected_power_starts;
/* 仅平台诊断读取现有Watch对象，不修改控制所有者。 */
extern MotorConfiguration s_motor_configuration;
extern MotorControlState s_motor_state;
extern FocState s_foc_state;
extern EncoderState s_rotor_encoder;
extern volatile uint32_t g_r40_adc_fault_reason;
extern volatile uint32_t g_r40_adc_stall_context[4];
extern volatile uint32_t g_r40_max_critical_cycles, g_r40_max_critical_caller;
static volatile uint32_t s_fast_samples, s_supervisor_ticks, s_max_fast_cycles, s_fatal;
static uint32_t s_main_iterations, s_snapshot_sequence;
extern uint32_t __r40_stack_bottom, __r40_stack_top;

static bool power_contract_valid(void)
{
    uint32_t channels = TMR1->cctrl & 0x5550U;
    bool enabled = (TMR1->brk & 0x8000U) != 0U;
    if (channels == 0U && !enabled)
    {
        return true;
    }
    return false;
}

static void initialize_stack_watermark(void)
{
    /* 仅填充当前SP下方未使用区；保留128字节给此函数及编译器临时栈帧。 */
    uintptr_t stop = __get_MSP() - 128U;
    for (volatile uint32_t *word = &__r40_stack_bottom; (uintptr_t)word < stop; ++word)
    {
        *word = 0xA55A39C3U;
    }
    g_r40_reset_diagnostic.stack_size_bytes =
        (uintptr_t)&__r40_stack_top - (uintptr_t)&__r40_stack_bottom;
}

static void measure_stack_watermark(void)
{
    volatile uint32_t *word = &__r40_stack_bottom;
    while ((uintptr_t)word < (uintptr_t)&__r40_stack_top && *word == 0xA55A39C3U)
    {
        ++word;
    }
    g_r40_reset_diagnostic.stack_used_bytes = (uintptr_t)&__r40_stack_top - (uintptr_t)word;
}

static void halt_disabled(uint32_t reason)
{
    __disable_irq();
    power_stage_hw_stop();
    s_fatal = reason;
    g_r40_reset_diagnostic.last_fatal = reason;
    g_r40_reset_diagnostic.last_uptime_ms = time_hw_now_ms();
    g_r40_reset_diagnostic.adc_fault = g_r40_adc_fault_reason;
    g_r40_reset_diagnostic.cfsr = SCB->CFSR;
    g_r40_reset_diagnostic.hfsr = SCB->HFSR;
    g_r40_reset_diagnostic.mmfar = SCB->MMFAR;
    g_r40_reset_diagnostic.bfar = SCB->BFAR;
    g_r40_reset_diagnostic.msp = __get_MSP();
    g_r40_reset_diagnostic.fast_samples = s_fast_samples;
    g_r40_reset_diagnostic.main_iterations = s_main_iterations;
    g_r40_reset_diagnostic.max_fast_cycles = s_max_fast_cycles;
    g_r40_reset_diagnostic.max_critical_cycles = g_r40_max_critical_cycles;
    g_r40_reset_diagnostic.max_critical_caller = g_r40_max_critical_caller;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        g_r40_reset_diagnostic.adc_stall_context[i] = g_r40_adc_stall_context[i];
    }
    ++g_r40_reset_diagnostic.fault_count;
    __DSB();
    for (;;)
    {
        __WFI();
    }
}

void ADC1_2_IRQHandler(void)
{
    uint32_t start = DWT->CYCCNT;
    if (!R40_AdcComplete())
    {
        if (!R40_AdcHealthy())
        {
            halt_disabled(1U);
        }
        return;
    }
    App_FastTick();
    ++s_fast_samples;
    R40AppTelemetry_Capture(s_fast_samples,
                            &s_foc_state,
                            &s_motor_state,
                            &s_rotor_encoder,
                            MotorStateMachine_GetState(),
                            s_motor_configuration.device.encoder.calib_flag,
                            MotorStateMachine_GetMaintenanceKind());
    uint32_t elapsed = DWT->CYCCNT - start;
    if (elapsed > s_max_fast_cycles)
    {
        s_max_fast_cycles = elapsed;
    }
    if (elapsed >= 9000U || !power_contract_valid())
    {
        halt_disabled(2U);
    }
}

void TMR6_DAC_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR6, TMR_OVF_FLAG))
    {
        tmr_flag_clear(TMR6, TMR_OVF_FLAG);
        AppBusCurrent_Sample();
        App_SupervisorTick();
        ++s_supervisor_ticks;
    }
}

void CAN1_RX_IRQHandler(void)
{
    ++g_r40_reset_diagnostic.can_rx_irqs;
    /* 复用既有有界收包入口；只入队，协议解析仍由APP前台执行。 */
    can_flag_clear(CAN1, CAN_RIF_FLAG);
    YgProtocolCanfd_RxIrqHandler();
    /* RIF是事件标志；超过三帧的积压另挂一次中断，不在单次ISR无界排空。 */
    if (can_rxbuf_status_get(CAN1) != CAN_RXBUF_STATUS_EMPTY)
    {
        NVIC_SetPendingIRQ(CAN1_RX_IRQn);
    }
}

void NMI_Handler(void)
{
    halt_disabled(3U);
}
void HardFault_Handler(void)
{
    halt_disabled(4U);
}
void MemManage_Handler(void)
{
    halt_disabled(5U);
}
void BusFault_Handler(void)
{
    halt_disabled(6U);
}
void UsageFault_Handler(void)
{
    halt_disabled(7U);
}

static bool watchdog_init(void)
{
    DEBUGMCU->apb1_frz &= ~(1U << 12U);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
    uint32_t remaining = 200000U;
    while (crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U)
    {
        return false;
    }
    WDT->cmd = 0x5555U;
    WDT->div = 0U;
    WDT->rld = 511U;
    WDT->win = 4095U;
    remaining = 200000U;
    while (WDT->sts != 0U && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U || WDT->div != 0U || WDT->rld != 511U)
    {
        return false;
    }
    WDT->cmd = 0xAAAAU;
    WDT->cmd = 0xCCCCU;
    return true;
}

static void publish(void)
{
    MotorStatus status;
    uint8_t valid;
    MotorMaintenanceKind maintenance;
    MotorControlProgress progress;
    MotorState state = MotorAxis_ReadStatus(&status, &valid, &maintenance, &progress);
    AppBusCurrent_Apply(&status.bus_current, &valid);
    uint8_t uid[BOARD_HW_UID_BYTES];
    if (!board_hw_read_uid(uid))
    {
        halt_disabled(8U);
    }
    uint32_t key = critical_hw_enter();
    uint32_t sequence = s_snapshot_sequence + 2U;
    g_r40_app_snapshot.begin = sequence - 1U;
    g_r40_app_snapshot.magic = 0x52415050U;
    g_r40_app_snapshot.version = 1U;
    g_r40_app_snapshot.phase_lock = R40_APP_PHASE_LOCK;
    g_r40_app_snapshot.milliseconds = time_hw_now_ms();
    g_r40_app_snapshot.main_iterations = s_main_iterations;
    g_r40_app_snapshot.fast_samples = s_fast_samples;
    g_r40_app_snapshot.supervisor_ticks = s_supervisor_ticks;
    g_r40_app_snapshot.adc_healthy = R40_AdcHealthy();
    g_r40_app_snapshot.max_fast_cycles = s_max_fast_cycles;
    g_r40_app_snapshot.fatal_code = s_fatal;
    g_r40_app_snapshot.motor_state = state;
    g_r40_app_snapshot.motor_fault = status.fault;
    g_r40_app_snapshot.motor_mode = status.mode;
    g_r40_app_snapshot.valid_bits = valid;
    g_r40_app_snapshot.bus_v = status.bus_voltage;
    g_r40_app_snapshot.position_rad = status.position_feedback;
    g_r40_app_snapshot.speed_rad_s = status.speed_feedback_rad_s;
    g_r40_app_snapshot.raw_a = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_A);
    g_r40_app_snapshot.raw_b = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_B);
    g_r40_app_snapshot.raw_c = motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_C);
    g_r40_app_snapshot.rejected_starts = g_r40_rejected_power_starts;
    for (unsigned word = 0U; word < 3U; ++word)
    {
        g_r40_app_snapshot.uid[word] =
            (uint32_t)uid[word * 4U] | ((uint32_t)uid[word * 4U + 1U] << 8U) |
            ((uint32_t)uid[word * 4U + 2U] << 16U) | ((uint32_t)uid[word * 4U + 3U] << 24U);
    }
    g_r40_app_snapshot.led_output = GPIOC->odt;
    g_r40_app_snapshot.cctrl = TMR1->cctrl;
    g_r40_app_snapshot.brk = TMR1->brk;
    g_r40_app_snapshot.gpioa = GPIOA->odt;
    g_r40_app_snapshot.gpiob = GPIOB->odt;
    g_r40_app_snapshot.can_healthy = comm_hw_can_is_healthy();
    g_r40_app_snapshot.end = sequence;
    __DMB();
    g_r40_app_snapshot.begin = sequence;
    s_snapshot_sequence = sequence;
    critical_hw_exit(key);
}

int main(void)
{
    uint32_t reset_flags = CRM->ctrlsts & 0xFC000000U;
    if (g_r40_reset_diagnostic.magic != 0x52445231U || (reset_flags & (1U << 27U)))
    {
        g_r40_reset_diagnostic = (R40ResetDiagnostic){.magic = 0x52445231U};
    }
    ++g_r40_reset_diagnostic.boot_count;
    g_r40_reset_diagnostic.reset_flags = reset_flags;
    CRM->ctrlsts_bit.rstfc = TRUE;
    initialize_stack_watermark();
    (void)BootHw_Init();
    AppBusCurrent_Init();
    if (!R40_PlatformInit() || !R40_CanInit())
    {
        halt_disabled(9U);
    }
    R40_IndicatorInit();
    R40AppTelemetry_Init();
    App_Init();
    /* APP完成端点和业务绑定后才放开接收；独立轮询台架不使用此中断。 */
    NVIC_SetPriority(CAN1_RX_IRQn, 3U);
    NVIC_ClearPendingIRQ(CAN1_RX_IRQn);
    can_interrupt_enable(CAN1, CAN_RIE_INT, TRUE);
    NVIC_EnableIRQ(CAN1_RX_IRQn);
    if (!watchdog_init())
    {
        halt_disabled(10U);
    }
    uint32_t last_ms = time_hw_now_ms(), last_fast = s_fast_samples, last_slow = s_supervisor_ticks;
    uint32_t last_led = 0U, last_stack_ms = 0U;
    for (;;)
    {
        App_Process();
        R40AppTelemetry_Service();
        ++s_main_iterations;
        if (!R40_AdcHealthy() || !power_contract_valid())
        {
            halt_disabled(11U);
        }
        if (g_r40_app_led_test != last_led)
        {
            if (R40_IndicatorDiagnostic(g_r40_app_led_test))
            {
                last_led = g_r40_app_led_test;
            }
        }
        uint32_t now = time_hw_now_ms();
        if ((uint32_t)(now - last_stack_ms) >= 1000U)
        {
            measure_stack_watermark();
            last_stack_ms = now;
        }
        if ((uint32_t)(now - last_ms) >= 10U)
        {
            if (s_fast_samples == last_fast || s_supervisor_ticks == last_slow)
            {
                halt_disabled(12U);
            }
            last_fast = s_fast_samples;
            last_slow = s_supervisor_ticks;
            last_ms = now;
            WDT->cmd = 0xAAAAU;
            publish();
        }
    }
}
