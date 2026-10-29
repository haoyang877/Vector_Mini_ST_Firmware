/**
 * @file bsp_system.h
 * @brief 系统时间、临界区、身份、复位及看门狗。
 * @note 按已评审BSP契约分阶段迁移；仅使用当前已接入的功能。
 */
#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BSP_SYSTEM_UID_BYTES 12U
#define BSP_SYSTEM_IDENTITY_BYTES 64U

    /**
     * @brief 读取单调递增的毫秒时间。
     * @return 毫秒计数，按2^32自然回绕。
     * @note Init成功后前台/ISR有界调用；差值使用uint32_t无符号减法。
     */
    uint32_t BspSystem_TimeMs(void);

    /**
     * @brief 读取单调递增的微秒时间。
     * @return 微秒计数，按2^32自然回绕。
     * @note Init成功后前台/ISR有界调用；不用于跨回绕区间的绝对排序。
     * @note 不代替原周期级快环测量；分辨率及读取开销按板验证。
     */
    uint32_t BspSystem_TimeUs(void);

    /**
     * @brief 读取当前实际配置的内核频率。
     * @return Init成功后的内核频率，Hz。
     * @note 前台/ISR只读，不修改时钟，不把晶振频率当作内核频率。
     */
    uint32_t BspSystem_CoreClockHz(void);

    /**
     * @brief 保存并屏蔽可屏蔽中断。
     * @return 必须原样传给ExitCritical的原中断屏蔽状态。
     * @note 允许嵌套，同一上下文严格后进先出配对；不屏蔽NMI/HardFault。
     * @note 普通调用只保护短状态访问，不包住日志或通信等待，不跨任务保存；
     *   Flash完整屏蔽由BspFlash_Write内部单独管理。
     */
    uint32_t BspSystem_EnterCritical(void);

    /**
     * @brief 恢复对应临界区进入前的中断屏蔽状态。
     * @param saved_state 最近一次未退出的EnterCritical返回值。
     * @note 同一上下文配对退出，不无条件开中断，不新增隐含跨核同步保证。
     */
    void BspSystem_ExitCritical(uint32_t saved_state);

    /**
     * @brief 保护前台与低频监督的控制状态，不屏蔽快采样和载波中断。
     * @return 必须原样交给ExitControlCritical的原优先级屏蔽状态。
     * @note 仅屏蔽逻辑优先级2及以下紧急程度的中断；不修改PRIMASK。
     *   快环仍可抢占，不能用于保护快环拥有的数据或实现等待；同上下文嵌套配对。
     */
    uint32_t BspSystem_EnterControlCritical(void);

    /**
     * @brief 恢复控制临界区原有优先级屏蔽状态。
     * @param saved_state 对应EnterControlCritical返回值。
     */
    void BspSystem_ExitControlCritical(uint32_t saved_state);

    /**
     * @brief 发布或读取跨中断邮箱前建立内存访问顺序，不屏蔽中断。
     * @note 不提供互斥；邮箱必须仍遵循单写者、完整载荷先于发布标志的约定。
     */
    void BspSystem_MemoryBarrier(void);

    /**
     * @brief 不屏蔽中断地比较并替换一个32位共享状态。
     * @param address 4字节对齐且指向SRAM的有效共享字。
     * @param expected 允许替换的原值。
     * @param desired 匹配时写入的新值。
     * @return 原值匹配且写入成功为true，原值不符为false。
     * @note 使用排他访问；快环不等待低优先级所有者，前台被抢占可重试。
     */
    bool
    BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired);

    /**
     * @brief 读取12字节芯片唯一标识。
     * @param uid 调用方至少12字节可写缓冲，不保存指针。
     * @return 完成为BSP_OK；空指针为BSP_INVALID_ARGUMENT。
     * @note Init成功后前台调用，按三个小端32位字排列，不等同于板卡版本。
     */
    BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES]);

    /**
     * @brief 只读复制固定区域的生产身份记录。
     * @param record 调用方至少64字节可写缓冲，不保存指针。
     * @return 完成为BSP_OK；空指针为INVALID_ARGUMENT；无区域为NOT_SUPPORTED；读失败为IO_ERROR。
     * @note Init成功后前台调用；不解释CRC、不生成默认身份、不写回；失败后缓冲不可用。
     */
    BspResult BspSystem_ReadIdentity(uint8_t record[BSP_SYSTEM_IDENTITY_BYTES]);

    /**
     * @brief 请求芯片系统复位。
     * @note 正常不返回；前台所有者先完成停机及关相确认，不自动授权或恢复运动。
     * @note 进入Loader使用BootHw_EnterLoader；本接口不代写Loader邮箱。
     */
    void BspSystem_Reset(void);

    /**
     * @brief 按目标配置启动应用看门狗。
     * @return 启动为BSP_OK；未启用为NOT_SUPPORTED；未初始化或已启动为NOT_READY。
     * @note 启动前台一次，保留原时点和重装参数；不得重配Loader继承的看门狗。
     */
    BspResult BspWatchdog_Start(void);

    /**
     * @brief 重装载应用已启动的看门狗。
     * @note 仅健康判定所有者满足全部健康条件后调用；不判断业务健康、不隐式启动。
     * @note 未启用或未启动时不操作硬件；Loader继承看门狗使用BootHw_Watchdog。
     */
    void BspWatchdog_Feed(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTEM_H */
