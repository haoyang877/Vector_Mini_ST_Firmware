#include "board_hw.h"

#include "adc.h"
#include "delay.h"
#include "tim.h"

/* 板级启动序列：顺序、条件与时序与迁移前的 app 组合层逐字一致。
 * 本文件是"顺序即契约"的落点：只搬运调用位置，不改变任何寄存器写入、
 * 延时、条件判断或中断使能顺序。 */

void board_hw_start(bool motor_phases_enabled)
{
    /* 1 µs 延迟基准；主频按 SystemCoreClock 推导（170 MHz）。 */
    delay_init((uint16_t)(SystemCoreClock / 1000000U));

    /* ADC 内部校准：保持先 ADC1 后 ADC2 的顺序。 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 仅配置有效时启动三相互补 PWM；CH4 仍为未配置关节保留采样时钟。 */
    if (motor_phases_enabled)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);
    }

    /* CH4 PWM 触发 ADC 转换。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 启动 ADC1/ADC2 注入采样。 */
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    /* 四个注入通道就绪后才派发控制 tick：JEOC 逐 rank，可能暴露混合周期样本。 */
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOS);

    /* 启动 TIM7 2 kHz 监督中断（节拍契约见 control_config.h，周期覆写见 MX_TIM7_Init）。 */
    HAL_TIM_Base_Start_IT(&htim7);
}
