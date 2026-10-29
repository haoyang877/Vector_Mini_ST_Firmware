/**
 * @file bsp_uart.h
 * @brief 字节收发、独立TX终态与明确RX丢弃边界；尚未绑定硬件。
 * @note 两板尚未绑定UART资源；共用禁用实现不触碰硬件。
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 链路状态；错误优先，具体TX结果由TakeTxResult消费。 */
    typedef enum
    {
        BSP_UART_DISABLED = 0,
        BSP_UART_READY,
        BSP_UART_TX_BUSY,
        BSP_UART_ERROR
    } BspUartState;

    /** @brief 一次已接受发送的终态；不引入票据或应用排队。 */
    typedef enum
    {
        BSP_UART_TX_OK = 0, /**< 最后停止位已发送且无该次发送错误。 */
        BSP_UART_TX_ERROR,  /**< 硬件发送失败，不自动重发。 */
        BSP_UART_TX_TIMEOUT /**< 截止前未确认完整发送，收尾后报告，不等同于对端未收到。 */
    } BspUartTxResult;

    /**
     * @brief 非阻塞接受一整段待发送字节。
     * @param data 调用期间稳定的输入，零长度允许NULL。
     * @param length 字节数，不超过配置的单次发送缓冲容量。
     * @return 整段复制并接受为BSP_OK；超容量/非零空指针为INVALID_ARGUMENT；
     *   活动发送或未消费终态为BUSY；未初始化为NOT_READY；未启用为NOT_SUPPORTED；
     *   硬件拒绝或已锁存发送禁止为IO_ERROR。
     * @note 前台单发送所有者；不部分接受，不保存指针，返回OK后可复用输入。
     * @note 每次接受的非零发送产生且只产生一次终态；拒绝不产生终态。
     *   零长度为无硬件操作且不产生终态。
     * @note 发送失败仅收尾本次事务；下一次请求重新检查硬件，仍不可用则拒绝，不重置或盲目重发。
     * @note 非零发送接受时记录微秒起点；期限使用目标配置BSP_UART_TX_TIMEOUT_US，
     *   必须覆盖最大单次长度、帧格式、波特率及服务裕量，不由调用方逐次传入。
     */
    BspResult BspUart_TryWrite(const uint8_t *data, size_t length);

    /**
     * @brief 消费一次已接受发送的真实完成或失败结果。
     * @param result 仅取到终态时更新的输出，不保存指针。
     * @return 取得为BSP_OK，发送成败看result；尚无终态为NOT_READY；空指针为INVALID_ARGUMENT；
     *   未启用为NOT_SUPPORTED。
     * @note 前台同一发送所有者调用；DMA搬完不算成功，只有UART发送完成标志确认最后停止位结束。
     * @note 正常收尾后消费终态才释放发送槽，不清RX错误；活动发送未收尾不允许下一次发送。
     * @note 本函数也是有界超时推进点；所有者在每个前台服务拍调用，即使未见完成中断。
     * @note 截止前确认UART传输完成才为TX_OK；达到期限仍无确认则停止本次DMA/发送中断，
     *   确认TX静止后发布一次TX_TIMEOUT；截止后才观察到完成且无及时完成记录也按超时处理。
     * @note 若无法确认TX静止，返回一次TX_ERROR并锁存发送禁止至复位；消费记录不解除禁止。
     *   晚到IRQ不能覆盖已发布结果或归到下一笔；收尾不清RX缓存或重置整个UART。
     */
    BspResult BspUart_TakeTxResult(BspUartTxResult *result);

    /**
     * @brief 非阻塞消费字节或一次接收错误。
     * @param data 调用方可写缓冲，零容量允许NULL。
     * @param capacity 缓冲字节容量。
     * @param received 必须非NULL，有效指针入口置0，成功时为实际复制字节数。
     * @return 取得字节为BSP_OK；无数据或未初始化为NOT_READY；参数非法为INVALID_ARGUMENT；
     *   未启用为NOT_SUPPORTED；溢出为OVERFLOW；线路帧/校验错为FRAME_ERROR；
     *   其它接收错误为IO_ERROR。
     * @note 前台单接收所有者，可少于capacity；
     *   失败received为0，输出不作为有效数据，不影响TX完成记录。
     * @note RX错误锁存期间不再接纳字节；
     *   非零Read在同一短临界区丢弃全部RX缓存并消费一次错误，然后重新接纳字节。
     * @note 返回错误后的新字节由上层协议重新同步；BSP不判断业务帧边界，不保证下一字节就是帧头。
     * @note 已就绪时零容量成功且不消费缓存或错误；
     *   多个RX错误按OVERFLOW、FRAME_ERROR、IO_ERROR优先返回。
     */
    BspResult BspUart_Read(uint8_t *data, size_t capacity, size_t *received);

    /**
     * @brief 只读查询UART是否运行、忙或有未消费错误。
     * @return 未启用/未初始化为DISABLED；RX锁存错误、未消费TX失败/超时或发送禁止为ERROR；
     *   否则活动发送为TX_BUSY，其余为READY。
     * @note 前台有界只读，不清错；
     *   READY不替代TakeTxResult，未消费成功记录仍会使下一次发送返回BUSY。
     * @note TX错误记录通过TakeTxResult消费，RX错误通过非零Read消费，二者互不清除。
     *   TX禁止锁存不能由读取状态或消费结果解除。
     */
    BspUartState BspUart_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
