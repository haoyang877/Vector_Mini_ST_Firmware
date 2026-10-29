#include "bsp_internal.h"
#include "encoder_protocol.h"
#include "app_feature_requirements.h"
#include "at32f45x.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_flash.h"
#include "bsp_pwm.h"
#include "bsp_can.h"
#include "bsp_internal.h"
#include "r40_bench.h"
#include <math.h>

/* 仅用于底层启动/链接验收的独立镜像，未连接APP或电机控制。
 * 本目标接入ADC、CAN、MT6835，但没有完整APP及功率启动调用，不作为可用APP发布。 */
static volatile uint32_t s_supervisor_ticks;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_encoder_samples;
static volatile uint32_t s_encoder_errors;
static volatile BspEncoderSample s_encoder_last;
static volatile AdcMotorTemperatureSample s_motor_temperature;
static volatile uint32_t s_motor_temperature_samples;
static uint32_t s_parameter_first_word;
static volatile uint32_t s_fast_samples, s_fast_max_cycles;
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

void ADC1_2_IRQHandler(void)
{
    uint32_t begin = DWT->CYCCNT;
    if (Adc_CompleteFast())
    {
        ++s_fast_samples;
        bool started = BspEncoder_Begin() == BSP_OK;
        uint8_t stage = started ? 1U : 0U;
        if (started)
        {
            (void)BspEncoder_BeginReadback();
        }
        BspEncoderSample sample = {0};
        if (stage != 0U && BspEncoder_Complete(&sample) == BSP_OK)
        {
            s_encoder_last = sample;
            ++s_encoder_samples;
            if (sample.angle_counts < s_angle_min)
            {
                s_angle_min = sample.angle_counts;
            }
            if (sample.angle_counts > s_angle_max)
            {
                s_angle_max = sample.angle_counts;
            }
        }
        else
        {
            ++s_encoder_errors;
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
    if (!McuSystem_Init() || !Flash_Init() || !Adc_Init() || !Can_Init())
    {
        halt_disabled();
    }
    BspPwm_SetDuty(0.5f, 0.5f, 0.5f);
    if (BspFlash_Read(&s_parameter_first_word, sizeof(s_parameter_first_word)) != BSP_OK)
    {
        halt_disabled();
    }
    (void)Encoder_Init();
    if (!Adc_Start() || !McuSystem_Start())
    {
        halt_disabled();
    }
    BspCan_Start();
    for (;;)
    {
        if (!BspPwm_ArePhasesDisabled() || !Adc_IsHealthy())
        {
            halt_disabled();
        }
        BspCanFrame frame;
        if (BspCan_Receive(&frame) == BSP_OK)
        {
            ++s_received_frames;
            R40_BenchCanReceive(&frame);
        }
        (void)BspAdc_UpdateAuxiliary();
        BspAdcRawValue raw;
        BspAdcValue value;
        AdcMotorTemperatureSample temperature = {
            0U, 0U, ADC_MOTOR_TEMPERATURE_UNAVAILABLE, NAN, NAN};
        if (BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE, &raw) == BSP_OK)
        {
            temperature.sequence = raw.sequence;
            temperature.raw_counts = (uint16_t)raw.counts;
            if (BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE, &value) == BSP_OK)
            {
                temperature.status = ADC_MOTOR_TEMPERATURE_VALID;
                temperature.celsius = value.value;
            }
            if (temperature.sequence != s_motor_temperature.sequence)
            {
                ++s_motor_temperature_samples;
            }
        }
        s_motor_temperature = temperature;
        R40_BenchPublish(s_supervisor_ticks,
                         s_fast_samples,
                         s_encoder_samples,
                         s_encoder_errors,
                         s_fast_max_cycles,
                         s_encoder_last.angle_counts,
                         s_angle_min,
                         s_angle_max,
                         &temperature);
        if (R40_BenchPoll())
        {
            halt_disabled();
        }
        __WFI();
    }
}
