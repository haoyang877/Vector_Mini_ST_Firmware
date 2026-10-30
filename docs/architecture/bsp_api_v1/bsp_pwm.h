/**
 * @file bsp_pwm.h
 * @brief 两板共用的PWM接口；各MCU直接完成占空比换算、硬件读写和启停。
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 将三相占空比换算为比较计数并直接写入。
     * @param phase_a A相占空比，调用方提供0..1的有限值。
     * @param phase_b B相占空比，调用方提供0..1的有限值。
     * @param phase_c C相占空比，调用方提供0..1的有限值。
     * @note 三次独立写入，不校验、限幅、屏蔽中断或检查提交窗口。
     */
    void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c);

    /**
     * @brief 直接打开六路通道和主输出位。
     * @note 不检查许可或读回结果，不维护软件状态。
     */
    void BspPwm_Enable(void);

    /** @brief 直接关闭主输出和六路通道位，保留比较值与计数器。 */
    void BspPwm_Disable(void);

    /**
     * @brief 直接读取六路相通道位是否全关。
     * @return 六路位全为零时为true，否则false。
     * @note 只读寄存器，不返回状态结构或事件序号，不检查主输出位。
     */
    bool BspPwm_ArePhasesDisabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
