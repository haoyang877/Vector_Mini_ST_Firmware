/**
 * @file bsp_adc.h
 * @brief 两板共用的ADC数值读取与零偏设置接口，读取、换算与零偏由各MCU实现。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

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
        BSP_ADC_ANALOG_SUPPLY_VOLTAGE
    } BspAdcChannel;

    /**
     * @brief 直接读取通道原码。
     * @param channel 板上模拟信号；供电电压通道返回内部VREF原码。
     * @return 等效12位计数，保留四次累加的小数；未启用的通道返回NAN。
     * @note 直接读寄存器或DMA已完成的半缓冲；不返回状态，不缓存或等待。
     *       多次读取不保证同拍，也不判断是否已有首帧。
     */
    float BspAdc_ReadRawChannel(BspAdcChannel channel);

    /**
     * @brief 直接读取并换算通道数值。
     * @param channel 相/母线电流单位A，母线/供电电压单位V，温度单位摄氏度。
     * @return 物理量；通道未启用、缺换算参数或公式无定义时返回NAN。
     * @note 不返回状态、有效位、序号或时间戳，不屏蔽中断，不做采样保护。
     */
    float BspAdc_ReadChannel(BspAdcChannel channel);

    /**
     * @brief 读取一份新完成的电机NTC采样并确认完成标志。
     * @param celsius 非空输出，单位摄氏度；坏采样返回NAN。
     * @return 有新完成采样返回true；未安装或尚无新采样返回false并保持输出。
     * @note 监督单读者调用，不等待、不屏蔽中断；与MCU温度独立。
     */
    bool BspAdc_ReadMotorTemperature(float *celsius);

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
