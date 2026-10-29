/**
 * @file bsp_led.h
 * @brief 模式、故障及颜色指示。
 * @note 闪烁和呼吸状态由共用实现持有，MCU只驱动实际输出。
 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 指示颜色，支持范围及原有降级映射由板配置确定。 */
    typedef enum
    {
        BSP_LED_OFF = 0,
        BSP_LED_RED,
        BSP_LED_GREEN,
        BSP_LED_BLUE,
        BSP_LED_YELLOW,
        BSP_LED_PURPLE,
        BSP_LED_CYAN,
        BSP_LED_WHITE
    } BspLedColor;

    /** @brief 本次显示码的类型；由应用选定，BSP不判断电机故障。 */
    typedef enum
    {
        BSP_LED_MODE = 0,
        BSP_LED_FAULT
    } BspLedCodeChannel;

    /**
     * @brief 设置模式或故障通道的闪烁码。
     * @param channel 本次选中的模式或故障码类型，不代表两份独立排队的显示请求。
     * @param blink_count 沿用现有闪烁码编号和0值语义，不重新编码。
     * @return 接受为BSP_OK；通道或非既有显示码非法为INVALID_ARGUMENT；
     *   未初始化为NOT_READY；未提供为NOT_SUPPORTED。
     * @note 原应用状态更新上下文有界调用，不等待闪烁周期；不参与电机准入。
     * @note 应用唯一指示所有者先选择故障或模式；本函数替换当前码，不缓存被覆盖的模式码。
     * @note 相同通道和码重复设置不重启；变更在下一5Hz显示拍从熄灭起始重新计数。
     * @note 0表示停用闪烁；故障解除后由应用提交当前模式，BSP不自行恢复旧码。
     * @note 共用灯时非零码覆盖颜色且熄灭间隔不透出颜色；独立灯带可并行显示颜色。
     *   板配置只描述输出是否共用及颜色映射；本函数不改变呼吸相位。
     */
    BspResult BspLed_SetCode(BspLedCodeChannel channel, uint8_t blink_count);

    /**
     * @brief 设置支持的指示颜色或灯带目标色。
     * @param color 有效颜色枚举，允许使用板配置明确约定的原有降级映射。
     * @return 接受为BSP_OK；枚举非法为INVALID_ARGUMENT；功能或颜色不支持为NOT_SUPPORTED；
     *   未初始化为NOT_READY。
     * @note 原监督所有者有界设置目标，重复设置和切色均不重置或推进呼吸相位；
     *   BSP按原20Hz节拍推进一次呼吸，DMA在原提交点刷新，与调用次数无关。
     * @note 共用灯的颜色在非零闪烁码期间被遮蔽，但呼吸继续推进；码清零后显示当前相位。
     *   独立灯带不受码影响；OFF只关闭颜色输出，不清除闪烁码。
     * @note OK表示接受目标，不保证当前可见；拒绝不修改目标、相位或显示。
     *   只允许板配置列出的真实颜色或显式降级映射，不伪造硬件颜色能力。
     */
    BspResult BspLed_SetColor(BspLedColor color);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */
