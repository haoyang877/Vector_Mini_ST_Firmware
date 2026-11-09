#include "bsp_adc.h"
#include "bsp_system.h"
#include "../../../firmware/platform/bsp/bsp_internal.h"
#include "application_config.h"
#include "at32f45x.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_flash.h"
#include "bsp_pwm.h"
#include "bsp_can.h"
#include "../../../firmware/platform/bsp/bsp_internal.h"
#include "at32f456_bench.h"
#include <math.h>
#include <string.h>

/* 仅用于底层启动/链接验收的独立镜像，未连接APP或电机控制。
 * 本目标接入ADC、CAN、MT6835，但没有完整APP及功率启动调用，不作为可用APP发布。 */
static volatile uint32_t s_supervisor_ticks;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_encoder_samples;
static volatile uint32_t s_encoder_errors;
static volatile uint16_t s_encoder_angle;
static volatile AdcMotorTemperatureSample s_motor_temperature;
static volatile uint32_t s_motor_temperature_samples;
static uint32_t s_parameter_first_word;
static volatile uint32_t s_fast_samples, s_fast_max_cycles, s_adc_revision;
static volatile uint16_t s_angle_min = UINT16_MAX, s_angle_max;

static void halt_disabled(void)
{
    __disable_irq();
    BspPwm_Disable();
    for (;;)
    {
        __WFI();
    }
}

/** @brief 独立台架继续使用MCU提供的1ms时基。 */
void SysTick_Handler(void)
{
    BspSystem_HandleTickInterrupt();
}

void ADC1_2_IRQHandler(void)
{
    uint32_t begin = DWT->CYCCNT;
    ++s_adc_revision;
    if (BspAdc_CompleteFast())
    {
        ++s_fast_samples;
        BspEncoder_Begin();
        s_encoder_angle = (uint16_t)BspEncoder_ReadAngle(BSP_ENCODER_16BIT);
        ++s_encoder_samples;
        if (s_encoder_angle < s_angle_min)
        {
            s_angle_min = s_encoder_angle;
        }
        if (s_encoder_angle > s_angle_max)
        {
            s_angle_max = s_encoder_angle;
        }
    }
    uint32_t elapsed = DWT->CYCCNT - begin;
    if (elapsed > s_fast_max_cycles)
    {
        s_fast_max_cycles = elapsed;
    }
}

void TMR6_DAC_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR6, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR6, TMR_OVF_FLAG);
        ++s_supervisor_ticks;
    }
}

void NMI_Handler(void)
{
    halt_disabled();
}

void HardFault_Handler(void)
{
    halt_disabled();
}

void MemManage_Handler(void)
{
    halt_disabled();
}

void BusFault_Handler(void)
{
    halt_disabled();
}

void UsageFault_Handler(void)
{
    halt_disabled();
}

int main(void)
{
    McuSystem_Init();
    (void)Flash_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    McuAdc_Init();
    McuCan_Init();
    BspPwm_SetDuty(0.5f, 0.5f, 0.5f);
    if (BspFlash_Read(&s_parameter_first_word, sizeof(s_parameter_first_word)) != BSP_OK)
    {
        halt_disabled();
    }
    McuEncoder_Init();
    Adc_Start();
    McuSystem_Start();
    BspCan_Start();
    for (;;)
    {
        if (!BspPwm_ArePhasesDisabled())
        {
            halt_disabled();
        }
        BspCanFrame frame;
        if (BspCan_Receive(&frame))
        {
            ++s_received_frames;
            AT32_BenchCanReceive(&frame);
        }
        uint32_t revision = s_adc_revision;
        BspSystem_MemoryBarrier();
        float raw_counts = BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE);
        float celsius = BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE);
        /* 诊断序号记录前台读数次数，ADC接口只返回数值。 */
        AdcMotorTemperatureSample temperature = {
            ++s_motor_temperature_samples,
            isfinite(raw_counts) ? (uint16_t)raw_counts : 0U,
            isfinite(celsius) ? ADC_MOTOR_TEMPERATURE_VALID : ADC_MOTOR_TEMPERATURE_UNAVAILABLE,
            NAN,
            celsius};
        s_motor_temperature = temperature;
        AT32BenchSnapshot sample = {.supervisor_ticks = s_supervisor_ticks,
                                    .fast_samples = s_fast_samples,
                                    .encoder_samples = s_encoder_samples,
                                    .encoder_errors = s_encoder_errors,
                                    .fast_max_cycles = s_fast_max_cycles,
                                    .angle = s_encoder_angle,
                                    .angle_min = s_angle_min,
                                    .angle_max = s_angle_max,
                                    .ntc_sequence = temperature.sequence,
                                    .ntc_raw = temperature.raw_counts,
                                    .ntc_status = temperature.status};
        memcpy(&sample.ntc_ohm_bits, &temperature.resistance_ohm, sizeof(uint32_t));
        memcpy(&sample.ntc_celsius_bits, &temperature.celsius, sizeof(uint32_t));
        AT32_BenchPublish(&sample, revision, &s_adc_revision);
        if (AT32_BenchPoll())
        {
            halt_disabled();
        }
        __WFI();
    }
}
