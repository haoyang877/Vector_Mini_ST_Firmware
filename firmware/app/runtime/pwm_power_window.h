#ifndef PWM_POWER_WINDOW_H
#define PWM_POWER_WINDOW_H

#include <stdbool.h>
#include <stdint.h>

/* 仅限时APP使用；窗口只约束功率许可，不实现电机模式或标定算法。 */
typedef enum
{
    PWM_POWER_DISARMED,
    PWM_POWER_ARMED,
    PWM_POWER_ACTIVE,
    PWM_POWER_DONE,
    PWM_POWER_EXPIRED
} PwmPowerWindowState;

typedef struct
{
    PwmPowerWindowState state;
    uint32_t duration_ms, armed_ms, started_ms, fast_ticks, disable_sequence;
} PwmPowerWindow;

extern volatile PwmPowerWindow g_pwm_power_window;

/** @brief 上电清除许可；仅启动时调用，不允许用来运行中重装窗口。 */
void PwmPowerWindow_Init(void);
/**
 * @brief 为本次启动装载唯一窗口，本函数不会启相。
 * @param duration_ms 最大功率时间，200..45000ms。
 * @param now_ms 单调毫秒。
 * @return 本次上电尚未装载且时间有效时为true。
 * @note 调用方持有临界区并确认APP空闲、实际关相；窗口5秒未使用则过期。
 */
bool PwmPowerWindow_Arm(uint32_t duration_ms, uint32_t now_ms);
/**
 * @brief 检查端口启相许可；首次成功时开始计时，重复调用不能续期。
 * @param now_ms 单调毫秒。
 * @return 窗口已装载且未过期时为true。
 * @note 仅应用功率所有者在临界区调用；通过BSP快照检查关相序号，不操作寄存器。
 */
bool PwmPowerWindow_Start(uint32_t now_ms);
/** @brief 实际停相后消费活动窗口；已过期状态保留。 */
void PwmPowerWindow_Stop(void);
/**
 * @brief 不可逆撤销本次窗口，用于上层发现验证边界被突破时关断。
 * @note 调用方随后立即关相；不允许用本函数重新装载窗口。
 */
void PwmPowerWindow_Abort(void);
/**
 * @brief 每个20kHz快拍检查毫秒和独立拍数上限。
 * @param now_ms 单调毫秒。
 * @return 窗口过期返回false，调用方必须立即关相。
 * @note 仅快速ISR调用一次；不调用电机状态机、不访问硬件。
 */
bool PwmPowerWindow_Tick(uint32_t now_ms);

#endif
