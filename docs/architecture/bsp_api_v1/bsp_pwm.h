/**
 * @file bsp_pwm.h
 * @brief 三相PWM、功率启停及实际输出状态。
 * @note MCU实现保持原比较换算和采样触发，异常路径统一关相。
 * @note 本模块不屏蔽中断。快环为不可抢占的唯一写入者；前台功率操作和一致快照由调用方保证互斥。
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 三相逻辑相序，与具体定时器通道无关。 */
    typedef enum
    {
        BSP_PWM_PHASE_A = 0,
        BSP_PWM_PHASE_B,
        BSP_PWM_PHASE_C
    } BspPwmPhase;

    /** @brief 六路通道及总输出位的状态，不表示引脚电压实测。 */
    typedef enum
    {
        BSP_PWM_OFF = 0,   /**< 通道和总输出位均关闭。 */
        BSP_PWM_ON,        /**< 通道和总输出位均启用。 */
        BSP_PWM_INCOMPLETE /**< 其它不完整组合，不作为可运行状态。 */
    } BspPwmState;

    /** @brief 一致的输出快照，关相序号用于发现两次查询之间发生的关闭。 */
    typedef struct
    {
        BspPwmState state;         /**< 六路通道和总输出位的硬件读回状态。 */
        uint32_t disable_sequence; /**< 每次统一关相路径推进，自然回绕，读取不推进。 */
    } BspPwmStatus;

    /**
     * @brief 写入三相比较预装载值，成功时不切换输出使能。
     * @param phase_a A相有限占空比，0..1。
     * @param phase_b B相有限占空比，0..1。
     * @param phase_c C相有限占空比，0..1。
     * @return 三相完整暂存为BSP_OK；任一非法为INVALID_ARGUMENT且三相均不写；
     *   错过安全提交窗口为TIMEOUT且执行统一关相；未初始化为NOT_READY；未提供为NOT_SUPPORTED。
     * @note 唯一快环所有者在本拍约定提交窗口调用；关相前台仅在与快环写入互斥时预置。
     * @note 三相在同一个既定硬件更新事件生效，不能跨事件出现新旧比较值混用；
     *   不可抢占并不冻结定时器，BSP须在首个写入前确认剩余窗口覆盖最坏写入预算。
     * @note 完全关相时允许跨计数边界预装载；写三相期间屏蔽影子更新，恢复后统一锁存。
     *   仅开启或不完整输出受上述提交时限约束，预装载本身不启相。
     * @note 保留原换算/舍入和目标生效事件，不自动限幅、不改采样通道，不强制更新或重置计数器。
     * @note 占空比换算使用初始化时缓存的周期，运行中不允许修改PWM周期。
     * @note OK表示暂存成功，不表示已生效；TIMEOUT使输出关闭并推进关相序号，应用走原故障路径。
     */
    BspResult BspPwm_SetDuty(float phase_a, float phase_b, float phase_c);

    /**
     * @brief 只写指定相的比较预装载值。
     * @param phase 有效A/B/C逻辑相。
     * @param duty 有限占空比，0..1。
     * @return 接受为BSP_OK；非法参数为INVALID_ARGUMENT；未初始化为NOT_READY；
     *   未提供为NOT_SUPPORTED；错过提交窗口为TIMEOUT并执行统一关相。
     * @note 仅编译选定的单相台架所有者调用，与SetDuty互斥；其它相及输出许可不变。
     * @note 在该台架原提交窗口写入，于下一约定更新事件生效；错过窗口返回TIMEOUT并统一关相。
     * @note 完全关相时只写一个预装载寄存器，不适用带电提交期限，也不会启相。
     * @note 不是运行中三相控制的替代接口；非法值不写硬件，不改变采样触发或强制更新。
     */
    BspResult BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty);

    /**
     * @brief 由唯一应用功率所有者申请启相并核对硬件结果。
     * @return 完整启用为BSP_OK；硬件准入拒绝为REJECTED；未初始化为NOT_READY；
     *   未提供为NOT_SUPPORTED；部分开启或读回不符为IO_ERROR。
     * @note 仅既有应用状态机功率所有者调用；
     *   窗口检查、GetStatus和Enable在同一短临界区，见EXECUTION.md。
     * @note BSP校验永久关相及硬件故障锁，不管理测试时长；不得从其它应用模块绕过功率所有者调用。
     * @note 不改占空比、不重试。存在PWM硬件时，失败返回前由BSP执行统一Disable并核对关闭，
     *   推进关相序号；应用只处理许可消耗和故障，不承担首次硬件收尾。
     * @note 关闭仍不完整时保持硬件故障锁并进入既有致命故障路径；不得报告OFF或允许再次Enable。
     */
    BspResult BspPwm_Enable(void);

    /**
     * @brief 立即关闭输出并记录一次关相事件。
     * @note 前台/故障ISR沿用既有安全路径，调用方保证操作不交叠；关闭六路并保留比较值和采样时基。
     * @note 每次调用关闭硬件后推进disable_sequence，即使调用前已关相；不清硬件故障。
     * @note BSP内部故障和硬件保护ISR也必须通过此路径记录事件。
     * @note 实际硬件关闭先于应用处理；
     *   应用活动窗口一旦观察到序号改变便消费，下一次Enable前必须复核。
     */
    void BspPwm_Disable(void);

    /**
     * @brief 按原准入语义只检查六路通道是否全关。
     * @return 通道位全部关闭为true，否则false。
     * @note 前台/ISR有界只读，不混入总输出位或采样通道，不表示门极电压已实测。
     * @note 不得仅因未初始化返回true；有硬件时以通道读回为准，无硬件时为true。
     */
    bool BspPwm_ArePhasesDisabled(void);

    /**
     * @brief 单次读取实际输出状态及关相事件序号。
     * @return 一致的BspPwmStatus；无PWM硬件时state为OFF、disable_sequence为0。
     * @note Init后有界只读，不屏蔽中断、不重试、不清除事件、不改变输出。
     * @note 快环直接读取；前台需要状态和序号一致时，调用方须与启停操作互斥。
     * @note 活动窗口保存启相时的序号；后续不相等即说明发生关闭，不能只比较当前ON/OFF。
     * @note 两次检查间不能跨越2^32次关相事件；窗口时限内遵守此约束。
     */
    BspPwmStatus BspPwm_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
