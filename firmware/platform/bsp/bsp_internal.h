/**
 * @file bsp_internal.h
 * @brief BSP内部跨文件声明；按功能分段，不对应用公开。
 * @note 不放函数实现、可变对象定义或板级配置值；MCU专用状态留在对应实现。
 */
#ifndef BSP_INTERNAL_H
#define BSP_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "include/bsp.h"

/* 启动交接只由MCU启动编排调用。 */
/**
 * @brief 最早启动阶段识别Loader APP布局并设置向量表。
 * @return 成功建立Loader交接为true，独立APP或不支持为false。
 * @note HAL初始化及开放中断前调用一次，应用只查询BootHw_IsAvailable。
 */
bool BootHw_Init(void);

/* 单相输出仅用于底层台架，不属于正式应用接口。 */
/** @brief 台架使用的逻辑相序，硬件通道由MCU实现映射。 */
typedef enum
{
    BSP_PWM_PHASE_A,
    BSP_PWM_PHASE_B,
    BSP_PWM_PHASE_C
} BspPwmPhase;
/**
 * @brief 直接设置台架指定相的占空比。
 * @param phase A、B或C逻辑相。
 * @param duty 调用方保证为0..1的有限值。
 */
void BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty);

/* 两板共用初始化声明，工程选择对应MCU实现。 */
/** @brief 初始化ADC、采样触发和片内校准，保持外部采样未启动。 */
void McuAdc_Init(void);
/** @brief 初始化三相PWM引脚和定时器，保持相输出关闭。 */
void McuPwm_Init(void);
/** @brief 初始化编码器GPIO和SPI/DMA，不发起传输。 */
void McuEncoder_Init(void);
/** @brief 初始化CAN引脚、位时序和过滤器，控制器尚未启动。 */
void McuCan_Init(void);

/* ADC硬件操作由各MCU实现；原码和物理量通过公开数值接口读取。 */
/** @brief 开放ADC触发，启动对应采样通道，不维护软件状态。 */
void Adc_Start(void);
/** @brief 关闭采样触发并等待当前转换停止；Flash事务调用，不维护恢复状态。 */
void Adc_Stop(void);
/**
 * @brief 确认本拍硬件ADC完成事件。
 * @return 本拍完成为true，否则false。
 * @note ADC中断调用，按本板IRQ顺序清除硬件完成位。
 */
bool Adc_CompleteFast(void);
/* Flash初始化 */
/**
 * @brief 在芯片准备完成后建立固定存储区的访问资格。
 * @return 本次首次初始化成功为true，重复调用或已锁存故障为false。
 * @note BSP生命周期或专用底层台架调用；不擦写记录。
 */
bool Flash_Init(void);

/* AT32启动编排；测试入口与完整APP复用初始化顺序。 */
/** @brief 准备系统时钟、PWM和监督定时器。
 * @return 首次初始化成功返回true，否则false。
 */
bool McuSystem_Init(void);
/** @brief 开放时基和采样载波，不修改六路输出位。
 * @return 首次启动成功返回true，否则false。
 */
bool McuSystem_Start(void);
#endif /* BSP_INTERNAL_H */
