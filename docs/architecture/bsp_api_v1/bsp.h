/**
 * @file bsp.h
 * @brief 公共结果、启动生命周期及静态应用事件。
 * @note 按已评审BSP契约分阶段迁移；仅使用当前已接入的功能。
 */
#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 调用结果；不直接作为线上协议或持久记录序列化。 */
    typedef enum
    {
        BSP_OK = 0,           /**< 本次完成或请求被接受；具体完成语义见函数。 */
        BSP_NOT_SUPPORTED,    /**< 硬件或目标未提供此功能。 */
        BSP_NOT_READY,        /**< 生命周期条件不满足，或当前没有结果。 */
        BSP_REJECTED,         /**< 安全准入拒绝。 */
        BSP_BUSY,             /**< 资源已被占用，本次未取得所有权。 */
        BSP_INVALID_ARGUMENT, /**< 指针、长度、枚举或数值非法。 */
        BSP_TIMEOUT,          /**< 操作超过规定的提交或完成期限。 */
        BSP_STALE,            /**< 最近样本超过有效年龄。 */
        BSP_NOT_CALIBRATED,   /**< 缺少输出物理量所需的校准。 */
        BSP_CRC_ERROR,        /**< 实际执行的CRC校验失败。 */
        BSP_FRAME_ERROR,      /**< 帧格式或事务阶段错误。 */
        BSP_SENSOR_FAULT,     /**< 器件报告故障。 */
        BSP_OVERFLOW,         /**< 接收或缓冲溢出，发生数据丢失。 */
        BSP_IO_ERROR          /**< 其它硬件操作或读回失败。 */
    } BspResult;

    /**
     * @brief 初始化已启用的硬件并建立安全初始状态。
     * @return 完成为BSP_OK；重复调用为BSP_NOT_READY；硬件失败返回对应错误或进入既有致命路径。
     * @note 启动前台一次，可包含原有有界校准等待；不从ISR调用。
     * @note 先执行BootHw早期交接再进行时钟/外设准备；
     *   不启相、不开放应用控制事件、不写参数或启动应用看门狗。
     * @note 失败保持关相并锁存初始化失败，不能继续Start或在同一次启动中重试Init。
     */
    BspResult Bsp_Init(void);

    /**
     * @brief 按原顺序开放采样触发及监督节拍。
     * @return 成功为BSP_OK；Init未完成或已经启动为BSP_NOT_READY；硬件失败返回对应错误。
     * @note 启动前台一次，应用对象及必需的静态事件实现已就绪。
     * @note 不启相；CAN和看门狗由各自Start按原目标时点开放。
     */
    BspResult Bsp_Start(void);

    /**
     * @brief 处理BSP刚发布的完整快采样帧。
     * @note 应用提供静态链接实现，BSP在原ADC快中断位置调用；不是注册接口。
     * @note BSP先完成校验及必要硬件处理；应用有界执行，不阻塞、打印或写Flash。
     */
    void BspApp_OnFastSample(void);

    /**
     * @brief 处理当前目标的一次监督控制节拍。
     * @note 应用提供实现，BSP在原监督IRQ调用，保留目标频率和优先级。
     * @note 处理必须有界；不在此无条件喂狗以掩盖前台停止，不执行Flash或阻塞通信。
     */
    void BspApp_OnControlTick(void);

    /**
     * @brief 有界地将已接收的CAN帧移入应用队列。
     * @note 中断接收目标由应用提供实现，在RX上下文通过BspCan_Receive取帧；
     *   轮询目标不启用该事件。
     * @note 不执行协议命令或Flash操作；积压重挂及硬件标志确认归BSP，不用空默认实现吞帧。
     */
    void BspApp_OnCanReceive(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
