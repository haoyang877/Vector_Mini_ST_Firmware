#include "encoder_protocol.h"
#include "app_feature_requirements.h"
#include "bsp_config.h"
#include "at32f45x.h"
#include "at32f45x_debug.h"
#include "at32f45x_wdt.h"
#include "bsp_system.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "sampling_scale.h"
#include "openloop_model.h"
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
#include "closedloop_bench.h"
static R40ClBench s_closedloop;
volatile struct
{
    uint32_t begin;
    R40ClTelemetry value;
    uint32_t end;
} g_closedloop_snapshot;
/* 独立链接区不由C运行库清零；WDT复位后只读保留，下一次正常复位才清理。 */
__attribute__((section(".bss.R40_RETAIN"))) volatile struct
{
    uint32_t magic, stage, milliseconds, fast_samples, main_sample, max_cycles;
    uint32_t model_state, model_fault, adc_healthy, adc1_flags, adc2_flags, adc_fault_reason;
} g_closedloop_fault_record;
static bool s_record_fault;
#endif
#include "bsp_pwm.h"
#include "bsp_internal.h"
#include "bsp_internal.h"
#include "../../../firmware/platform/config/boards/at32f456.h"

#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
#include "openloop_keil_control.h"
#include "openloop_telemetry.h"
static R40KeilControl s_keil_control;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
#error "Keil open-loop switch cannot control the closed-loop target"
#endif
#endif

#ifndef R40_OPENLOOP_POWER_ENABLED
#define R40_OPENLOOP_POWER_ENABLED 0
#endif
#if R40_OPENLOOP_POWER_ENABLED != 0 && R40_OPENLOOP_POWER_ENABLED != 1
#error "Open-loop power selector must be zero or one"
#endif
/* 独立验证目标；默认编译不含启动功率的调用。命令不兼容产品协议，也不访问CAN或Flash。 */
volatile struct
{
    uint32_t magic, version, command, inverse, nonce, completed_nonce, result;
    uint32_t heartbeat, heartbeat_inverse;
} g_openloop_command = {.magic = 0x4F4C3430U, .version = 1U};

volatile struct
{
    uint32_t begin, milliseconds, state, fault, calibration_samples, running_samples;
    uint32_t encoder_samples, encoder_errors, angle, max_cycles, power_build, reset_cause;
    uint32_t cctrl, brk, gpioa, gpiob, wdt_div, wdt_reload, debug_freeze;
    uint32_t raw_a, raw_b, raw_c, raw_vbus;
    float offset_a, offset_b, offset_c, current_a, current_b, current_c;
    float peak_current, bus_v, duty_a, duty_b, duty_c;
    uint32_t end;
} g_openloop_snapshot;

/* 与主快照使用同一发布序号；母线电流保留ADC原码，零偏在关相基线中确定。 */
volatile struct
{
    uint32_t begin, milliseconds, sequence, bus_current_counts, valid, end;
} g_openloop_aux_snapshot;

static R40OlModel s_model;
static uint16_t s_raw[4];
static volatile uint32_t s_fast_samples, s_encoder_samples, s_encoder_errors, s_max_cycles;
static volatile uint16_t s_angle;
static uint32_t s_reset_cause, s_nonce, s_heartbeat, s_last_feed_sample, s_last_published;
static bool s_output_on, s_hold_snapshot, s_wdt_test;

#if defined(R40_OPENLOOP_CAPTURE) && R40_OPENLOOP_CAPTURE
/* 单次开环板内1kHz原码记录；只在RUNNING写入，关相后读取，不占用控制接口。 */
volatile uint16_t g_openloop_capture[10000][4];
volatile struct
{
    uint32_t magic, version, sample_hz, electrical_millihz, phase_mv;
    uint32_t count, fast_samples, maximum_step_counts, jumps_over_128_counts, zero_steps;
    uint32_t first_angle, last_angle;
} g_openloop_capture_info = {0x43415031U,
                             1U,
                             1000U,
                             (uint32_t)(R40_OL_ELECTRICAL_HZ * 1000.0f),
                             (uint32_t)(R40_OL_VOLTAGE_V * 1000.0f)};

static void capture_openloop(const uint16_t raw[4])
{
    if (s_model.state != R40_OL_RUNNING || s_model.running_samples == 0U)
    {
        return;
    }
    if (g_openloop_capture_info.fast_samples == 0U)
    {
        g_openloop_capture_info.first_angle = s_angle;
    }
    else
    {
        /* 最短路径差分自然消除65535到0的正常回绕，不把回绕误判为断点。 */
        int32_t step = (int16_t)(uint16_t)(s_angle - g_openloop_capture_info.last_angle);
        uint32_t magnitude = (uint32_t)(step < 0 ? -step : step);
        if (magnitude > g_openloop_capture_info.maximum_step_counts)
        {
            g_openloop_capture_info.maximum_step_counts = magnitude;
        }
        g_openloop_capture_info.jumps_over_128_counts += magnitude > 128U ? 1U : 0U;
        g_openloop_capture_info.zero_steps += magnitude == 0U ? 1U : 0U;
    }
    g_openloop_capture_info.last_angle = s_angle;
    ++g_openloop_capture_info.fast_samples;
    if ((s_model.running_samples - 1U) % 20U == 0U && g_openloop_capture_info.count < 10000U)
    {
        uint32_t index = g_openloop_capture_info.count;
        g_openloop_capture[index][0] = s_angle;
        for (unsigned phase = 0U; phase < 3U; ++phase)
        {
            g_openloop_capture[index][phase + 1U] = raw[phase];
        }
        ++g_openloop_capture_info.count;
    }
}
#endif

static void record_fault_stage(uint32_t stage)
{
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    if (s_record_fault)
    {
        g_closedloop_fault_record.stage = stage;
        if (stage == 0U || stage == 4U || stage >= 0xA0U)
        {
            g_closedloop_fault_record.milliseconds = BspSystem_TimeMs();
            g_closedloop_fault_record.fast_samples = s_fast_samples;
            g_closedloop_fault_record.max_cycles = s_max_cycles;
            g_closedloop_fault_record.model_state = s_model.state;
            g_closedloop_fault_record.model_fault = s_model.fault;
        }
    }
#else
    (void)stage;
#endif
}

static void stop_outputs(void)
{
    BspPwm_Disable();
    s_output_on = false;
    BspPwm_SetDuty(0.5f, 0.5f, 0.5f);
}

static void fatal_stop(void)
{
    __disable_irq();
    stop_outputs();
    record_fault_stage(0xF0U);
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    if (s_record_fault)
    {
        g_closedloop_fault_record.adc_healthy = Adc_IsHealthy();
        g_closedloop_fault_record.adc1_flags = ADC1->sts;
        g_closedloop_fault_record.adc2_flags = ADC2->sts;
        g_closedloop_fault_record.adc_fault_reason = Adc_IsHealthy() ? 0U : 1U;
    }
#endif
    for (;;)
    {
        __WFI();
    }
}

static bool watchdog_init(void)
{
    DEBUGMCU->apb1_frz &= ~DEBUG_WDT_PAUSE;
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
    WDT->cmd = WDT_CMD_UNLOCK;
    WDT->div = WDT_CLK_DIV_4;
    WDT->rld = 511U;
    WDT->win = 4095U;
    remaining = 200000U;
    while (WDT->sts != 0U && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U || WDT->div != WDT_CLK_DIV_4 || WDT->rld != 511U ||
        (DEBUGMCU->apb1_frz & DEBUG_WDT_PAUSE) != 0U)
    {
        return false;
    }
    WDT->cmd = WDT_CMD_RELOAD;
    WDT->cmd = WDT_CMD_ENABLE;
    return true;
}

void ADC1_2_IRQHandler(void)
{
    uint32_t began = DWT->CYCCNT;
    record_fault_stage(1U);
    bool adc_ok = Adc_CompleteFast();
    if (!adc_ok)
    {
        R40Ol_Stop(&s_model, R40_OL_ADC_ERROR);
        record_fault_stage(0xA0U);
        stop_outputs();
        return;
    }
    ++s_fast_samples;
    record_fault_stage(2U);
    uint16_t raw[4];
    BspAdcRawCurrent current;
    BspAdcRawValue voltage;
    if (BspAdc_ReadRawCurrent(&current) != BSP_OK ||
        BspAdc_ReadRawChannel(BSP_ADC_BUS_VOLTAGE, &voltage) != BSP_OK)
    {
        R40Ol_Stop(&s_model, R40_OL_ADC_ERROR);
        stop_outputs();
        return;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        raw[i] = (uint16_t)(current.phase_counts[i] * R40_ADC_ACCUMULATIONS);
    }
    raw[3] = (uint16_t)(voltage.counts * R40_ADC_ACCUMULATIONS);
    if (s_model.state != R40_OL_FAULT)
    {
        for (unsigned i = 0U; i < 4U; ++i)
        {
            s_raw[i] = raw[i];
        }
    }
    bool began_encoder = BspEncoder_Begin() == BSP_OK;
    uint8_t stage = began_encoder ? 1U : 0U;
    if (began_encoder)
    {
        (void)BspEncoder_BeginReadback();
    }
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    /* SPI响应在途时只做无I/O的电流/时限检查；本拍帧验证完成前不提交PWM。 */
    R40Ol_Step(&s_model, raw, Adc_IsHealthy(), true, BspSystem_TimeMs());
    uint16_t angle;
    bool encoder_ok = R40Cl_ReadEncoder(&s_closedloop, stage, &angle);
    if (encoder_ok)
    {
        s_angle = angle;
#else
    BspEncoderSample sample;
    bool encoder_ok = stage != 0U && BspEncoder_Complete(&sample) == BSP_OK;
    if (encoder_ok)
    {
        s_angle = sample.angle_counts;
#endif
        ++s_encoder_samples;
    }
    else
    {
        ++s_encoder_errors;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
        R40Ol_Stop(&s_model, R40_OL_ENCODER_ERROR);
#endif
    }
    bool gate_ok = s_output_on ? ((TMR1->cctrl & 0x5550U) == 0x5550U && (TMR1->brk & 0x8000U) != 0U)
                               : (BspPwm_ArePhasesDisabled() && (TMR1->brk & 0x8000U) == 0U);
    if (!gate_ok || (DEBUGMCU->apb1_frz & DEBUG_WDT_PAUSE) != 0U)
    {
        R40Ol_Stop(&s_model, R40_OL_OUTPUT_ERROR);
    }
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    R40Cl_Step(&s_closedloop, &s_model, s_angle);
    record_fault_stage(3U);
#else
    R40Ol_Step(&s_model, raw, Adc_IsHealthy(), encoder_ok, BspSystem_TimeMs());
#endif
#if defined(R40_OPENLOOP_CAPTURE) && R40_OPENLOOP_CAPTURE
    capture_openloop(raw);
#endif
    if (s_model.state == R40_OL_RUNNING)
    {
#if !defined(R40_CLOSEDLOOP_TEST) || !R40_CLOSEDLOOP_TEST
        BspPwm_SetDuty(s_model.duty[0], s_model.duty[1], s_model.duty[2]);
#endif
#if R40_OPENLOOP_POWER_ENABLED
        if (!s_output_on)
        {
            BspPwm_Enable();
            s_output_on = true;
        }
#endif
    }
    else if (s_output_on || s_model.state == R40_OL_DONE || s_model.state == R40_OL_FAULT)
    {
        stop_outputs();
    }
    uint32_t elapsed = DWT->CYCCNT - began;
    if (elapsed > s_max_cycles)
    {
        s_max_cycles = elapsed;
    }
    if (elapsed >= 9000U)
    {
        R40Ol_Stop(&s_model, R40_OL_TIMING_ERROR);
        stop_outputs();
    }
    record_fault_stage(4U);
}

void TMR6_DAC_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR6, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR6, TMR_OVF_FLAG);
    }
}
void NMI_Handler(void)
{
    fatal_stop();
}
void HardFault_Handler(void)
{
    fatal_stop();
}
void MemManage_Handler(void)
{
    fatal_stop();
}
void BusFault_Handler(void)
{
    fatal_stop();
}
void UsageFault_Handler(void)
{
    fatal_stop();
}

static void command_poll(void)
{
    uint32_t key = BspSystem_EnterCritical();
    uint32_t command = g_openloop_command.command;
    uint32_t nonce = g_openloop_command.nonce;
    uint32_t heartbeat = g_openloop_command.heartbeat;
    if (g_openloop_command.magic == 0x4F4C3430U && g_openloop_command.version == 1U)
    {
        if (heartbeat != s_heartbeat && g_openloop_command.heartbeat_inverse == ~heartbeat)
        {
            s_heartbeat = heartbeat;
            R40Ol_Heartbeat(&s_model, BspSystem_TimeMs());
        }
        if (command != 0U && nonce != 0U && nonce != s_nonce &&
            g_openloop_command.inverse == ~command)
        {
            s_nonce = nonce;
            g_openloop_command.result = 2U;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
            if (command >= 10U && command <= 13U)
            {
                if (!BspPwm_ArePhasesDisabled() ||
                    !R40Cl_Begin(&s_closedloop, &s_model, command - 9U, BspSystem_TimeMs()))
                {
                    g_openloop_command.result = 4U;
                }
            }
#else
            if (command == 1U)
            {
                if (!BspPwm_ArePhasesDisabled() || !R40Ol_Begin(&s_model, BspSystem_TimeMs()))
                {
                    g_openloop_command.result = 4U;
                }
            }
#endif
            else if (command == 2U)
            {
                R40Ol_Stop(&s_model, R40_OL_USER_STOP);
                stop_outputs();
            }
            else if (command == 3U || command == 4U)
            {
                s_hold_snapshot = command == 3U;
            }
            else if (command == 5U && !R40_OPENLOOP_POWER_ENABLED &&
                     s_model.state != R40_OL_RUNNING && s_model.state != R40_OL_ARMING &&
                     BspPwm_ArePhasesDisabled())
            {
                s_wdt_test = true;
            }
            else
            {
                g_openloop_command.result = 4U;
            }
            g_openloop_command.completed_nonce = nonce;
        }
    }
    BspSystem_ExitCritical(key);
}

static void publish(void)
{
    uint32_t now = BspSystem_TimeMs();
    if (s_hold_snapshot || now == s_last_published)
    {
        return;
    }
    s_last_published = now;
    uint32_t key = BspSystem_EnterCritical();
    AdcAuxSample aux = {0};
    bool aux_valid = Adc_ReadAux(&aux);
    uint32_t version = (g_openloop_snapshot.begin + 2U) & ~1U;
    g_openloop_snapshot.begin = version | 1U;
    g_openloop_snapshot.milliseconds = now;
    g_openloop_snapshot.state = s_model.state;
    g_openloop_snapshot.fault = s_model.fault;
    g_openloop_snapshot.calibration_samples = s_model.calibration_samples;
    g_openloop_snapshot.running_samples = s_model.running_samples;
    g_openloop_snapshot.encoder_samples = s_encoder_samples;
    g_openloop_snapshot.encoder_errors = s_encoder_errors;
    g_openloop_snapshot.angle = s_angle;
    g_openloop_snapshot.max_cycles = s_max_cycles;
    g_openloop_snapshot.power_build = R40_OPENLOOP_POWER_ENABLED;
    g_openloop_snapshot.reset_cause = s_reset_cause;
    g_openloop_snapshot.cctrl = TMR1->cctrl;
    g_openloop_snapshot.brk = TMR1->brk;
    g_openloop_snapshot.gpioa = GPIOA->idt;
    g_openloop_snapshot.gpiob = GPIOB->idt;
    g_openloop_snapshot.wdt_div = WDT->div;
    g_openloop_snapshot.wdt_reload = WDT->rld;
    g_openloop_snapshot.debug_freeze = DEBUGMCU->apb1_frz;
    g_openloop_snapshot.raw_a = s_raw[0];
    g_openloop_snapshot.raw_b = s_raw[1];
    g_openloop_snapshot.raw_c = s_raw[2];
    g_openloop_snapshot.raw_vbus = s_raw[3];
    g_openloop_snapshot.offset_a = s_model.offsets[0];
    g_openloop_snapshot.offset_b = s_model.offsets[1];
    g_openloop_snapshot.offset_c = s_model.offsets[2];
    g_openloop_snapshot.current_a = s_model.current_a[0];
    g_openloop_snapshot.current_b = s_model.current_a[1];
    g_openloop_snapshot.current_c = s_model.current_a[2];
    g_openloop_snapshot.peak_current = s_model.peak_current_a;
    g_openloop_snapshot.bus_v = s_model.bus_v;
    g_openloop_snapshot.duty_a = s_model.duty[0];
    g_openloop_snapshot.duty_b = s_model.duty[1];
    g_openloop_snapshot.duty_c = s_model.duty[2];
    g_openloop_aux_snapshot.begin = version | 1U;
    g_openloop_aux_snapshot.milliseconds = now;
    g_openloop_aux_snapshot.sequence = aux.sequence;
    BspAdcRawValue fast_bus_raw = {0};
    (void)BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT, &fast_bus_raw);
    g_openloop_aux_snapshot.bus_current_counts = fast_bus_raw.counts;
    g_openloop_aux_snapshot.valid = aux_valid ? 1U : 0U;
    g_openloop_aux_snapshot.end = version;
    g_openloop_aux_snapshot.begin = version;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    g_closedloop_snapshot.begin = version | 1U;
    g_closedloop_snapshot.value = s_closedloop.telemetry;
    g_closedloop_snapshot.end = version;
    g_closedloop_snapshot.begin = version;
#endif
    g_openloop_snapshot.end = version;
    g_openloop_snapshot.begin = version;
    BspSystem_ExitCritical(key);
}

#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
static void keil_poll(void)
{
    uint32_t key = BspSystem_EnterCritical();
    if (g_openloop_enable != 0U && !s_keil_control.active && !BspPwm_ArePhasesDisabled())
    {
        R40Ol_Stop(&s_model, R40_OL_OUTPUT_ERROR);
        g_openloop_enable = 0U;
        stop_outputs();
    }
    else if (R40KeilControl_Poll(&s_keil_control, &s_model, BspSystem_TimeMs()))
    {
        stop_outputs();
    }
    BspSystem_ExitCritical(key);
    static uint32_t previous_slot;
    uint32_t slot = s_fast_samples / R40_RTT_FAST_TICKS;
    if (slot == previous_slot)
    {
        return;
    }
    previous_slot = slot;
    key = BspSystem_EnterCritical();
    R40OpenLoopTelemetrySample sample = {s_fast_samples,
                                         s_model.state,
                                         s_model.fault,
                                         s_model.running_samples,
                                         s_encoder_errors,
                                         s_angle,
                                         0U,
                                         0U,
                                         0U,
                                         0U,
                                         0U,
                                         s_model.current_a[0],
                                         s_model.current_a[1],
                                         s_model.current_a[2],
                                         s_model.bus_v};
    BspSystem_ExitCritical(key);
    /* 缓冲区写入和格式化只在前台执行，不占用ADC中断或临界区。 */
    R40OpenLoopTelemetry_Publish(&sample);
}
#endif

int main(void)
{
    if (!McuSystem_Init())
    {
        fatal_stop();
    }
    s_reset_cause = CRM->ctrlsts;
    crm_flag_clear(CRM_ALL_RESET_FLAG);
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    s_record_fault = (s_reset_cause & (1U << 29U)) == 0U;
    if (s_record_fault)
    {
        g_closedloop_fault_record.magic = 0x434C4631U;
        record_fault_stage(0U);
    }
#endif
    R40Ol_Init(&s_model);
#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
    R40KeilControl_Init(&s_keil_control);
    R40OpenLoopTelemetry_Init();
#endif
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    R40Cl_Init(&s_closedloop);
#endif
    if ((s_reset_cause & (1U << 29U)) != 0U)
    {
        R40Ol_Stop(&s_model, R40_OL_TIMING_ERROR);
    }
    if (!watchdog_init() || !Adc_Init())
    {
        fatal_stop();
    }
#if !defined(R40_CLOSEDLOOP_TEST) || !R40_CLOSEDLOOP_TEST
    (void)Encoder_Init();
#endif
    if (!Adc_Start() || !McuSystem_Start())
    {
        fatal_stop();
    }
    for (;;)
    {
        if (!Adc_IsHealthy())
        {
            fatal_stop();
        }
        command_poll();
#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
        keil_poll();
#endif
        publish();
        /* 前台和快采样均前进才喂狗；CPU暂停/前台卡死/采样停滞均不能保持输出。 */
        uint32_t sample = s_fast_samples;
        if (!s_wdt_test && sample != s_last_feed_sample)
        {
            s_last_feed_sample = sample;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
            if (s_record_fault)
            {
                g_closedloop_fault_record.main_sample = sample;
            }
#endif
            WDT->cmd = WDT_CMD_RELOAD;
        }
        __WFI();
    }
}
