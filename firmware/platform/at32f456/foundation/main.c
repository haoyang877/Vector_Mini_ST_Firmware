#include "at32f45x.h"
#include "comm_hw.h"
#include "encoder_sensor.h"
#include "motor_hw.h"
#include "param_store.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "r40_can.h"
#include "r40_platform.h"
#include "r40_bench.h"
#include "../ports/motor_temperature_at32f456.h"

/* 仅用于底层启动/链接验收的独立镜像，未连接APP或电机控制。
 * 本目标接入ADC、CAN、MT6835，但没有完整APP及功率启动调用，不作为可用APP发布。 */
static volatile uint32_t s_supervisor_ticks;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_encoder_samples;
static volatile uint32_t s_encoder_errors;
static volatile EncoderSensorSample s_encoder_last;
static volatile R40MotorTemperatureSample s_motor_temperature;
static volatile uint32_t s_motor_temperature_samples;
static uint32_t s_parameter_first_word;
static volatile uint32_t s_fast_samples, s_fast_max_cycles;
static volatile uint16_t s_angle_min = UINT16_MAX, s_angle_max;

static void halt_disabled(void)
{
    __disable_irq();
    power_stage_hw_stop();
    for (;;)
    {
        __WFI();
    }
}

void ADC1_2_IRQHandler(void)
{
    uint32_t begin = DWT->CYCCNT;
    if (R40_AdcComplete())
    {
        ++s_fast_samples;
        bool started = encoder_sensor_begin();
        uint8_t stage = encoder_sensor_begin_readback(started);
        EncoderSensorSample sample = {0};
        if (encoder_sensor_complete(stage, &sample) == ENCODER_SENSOR_OK)
        {
            s_encoder_last = sample;
            ++s_encoder_samples;
            if (sample.angle_q15 < s_angle_min)
            {
                s_angle_min = sample.angle_q15;
            }
            if (sample.angle_q15 > s_angle_max)
            {
                s_angle_max = sample.angle_q15;
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
    if (!R40_PlatformInit() || !R40_AdcInit() || !R40_CanInit())
    {
        halt_disabled();
    }
    motor_hw_pwm_set_duty(0.5f, 0.5f, 0.5f);
    if (!param_store_read(&s_parameter_first_word, sizeof(s_parameter_first_word)))
    {
        halt_disabled();
    }
    encoder_sensor_init();
    if (!R40_AdcStart() || !R40_TimingStart())
    {
        halt_disabled();
    }
    comm_hw_can_start_fd();
    for (;;)
    {
        if (!power_stage_hw_phases_disabled() || !R40_AdcHealthy())
        {
            halt_disabled();
        }
        CommHwCanFrame frame;
        if (comm_hw_can_receive(&frame))
        {
            ++s_received_frames;
            R40_BenchCanReceive(&frame);
        }
        R40MotorTemperatureSample temperature;
        if (R40_MotorTemperaturePoll(&temperature))
        {
            ++s_motor_temperature_samples;
        }
        s_motor_temperature = temperature;
        R40_BenchPublish(s_supervisor_ticks,
                         s_fast_samples,
                         s_encoder_samples,
                         s_encoder_errors,
                         s_fast_max_cycles,
                         s_encoder_last.angle_q15,
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
