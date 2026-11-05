/**
 * @file bsp_adc.h
 * @brief 两板共用的ADC数值读取与零偏设置接口，读取、换算与零偏由各MCU实现。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 板上模拟信号；三相原码归一化到12位单次计数。 */
    typedef enum
    {
        BSP_ADC_PHASE_CURRENT_A,
        BSP_ADC_PHASE_CURRENT_B,
        BSP_ADC_PHASE_CURRENT_C,
        BSP_ADC_BUS_VOLTAGE,
        BSP_ADC_BUS_CURRENT,
        BSP_ADC_MOTOR_TEMPERATURE,
        BSP_ADC_MCU_TEMPERATURE,
        BSP_ADC_ANALOG_SUPPLY_VOLTAGE,
        BSP_ADC_LINEAR_HALL_SIN,
        BSP_ADC_LINEAR_HALL_COS
    } BspAdcChannel;

    /**
     * @brief 确认AT32两组快采样均已完成，并清除完成位。
     * @return 两路所需采样都完成时返回true；无主事件或辅ADC未完成时返回false。
     * @note 仅AT32 ADC中断及独立台架调用。辅ADC未完成时仍清除主事件，不执行快环。
     */
    bool BspAdc_CompleteFast(void);

    /**
     * @brief 处理STM32共享ADC中断并在快采样完成时通知应用。
     * @note 仅由APP映射的ADC向量调用；保持各板清标志、HAL派发和回调的原有顺序。
     *       不开放中断，不改变采样配置；应用回调仍在当前ADC中断上下文执行。
     */
    void BspAdc_HandleInterrupt(void);

    /**
     * @brief 直接读取通道原码。
     * @param channel 板上模拟信号；供电电压通道返回内部VREF原码。
     * @return sin/cos和电机NTC为原始12位ADC码；其他通道为等效12位计数，保留四次累加的小数；未启用的通道返回NAN。
     * @note 直接读ADC结果寄存器；不返回状态，不缓存或等待。
     *       多次读取不保证同拍，也不判断是否已有首帧。
     */
    float BspAdc_ReadRawChannel(BspAdcChannel channel);

    /**
     * @brief 直接读取并换算通道数值。
     * @param channel 相/母线电流单位A，母线/供电电压单位V，温度单位摄氏度。
     * @return 物理量；sin/cos仅提供原码，此入口返回NAN；未启用、缺参数或公式无定义时返回NAN。
     * @note 不返回状态、有效位、序号或时间戳，不屏蔽中断，不做采样保护。
     */
    float BspAdc_ReadChannel(BspAdcChannel channel);

    /**
     * @brief 直接保存三相零偏。
     * @param offsets_counts 三个等效12位零偏计数，不保留指针。
     * @note 依次写入，不校验、不屏蔽中断。
     */
    void BspAdc_SetPhaseOffsets(const float offsets_counts[3]);

    /**
     * @brief 直接保存母线电流零偏。
     * @param offset_counts 等效12位零偏计数。
     */
    void BspAdc_SetBusCurrentOffset(float offset_counts);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
