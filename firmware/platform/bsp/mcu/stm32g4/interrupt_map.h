/**
 * @file interrupt_map.h
 * @brief STM32正式APP的逻辑中断入口与启动向量映射。
 * @note 只映射符号；优先级、外设配置和事件确认归MCU BSP。
 */
#ifndef STM32G4_INTERRUPT_MAP_H
#define STM32G4_INTERRUPT_MAP_H

#define MCU_ADC_IRQ_HANDLER ADC1_2_IRQHandler
#define MCU_CONTROL_IRQ_HANDLER TIM7_IRQHandler
#define MCU_SYSTICK_IRQ_HANDLER SysTick_Handler

#endif
