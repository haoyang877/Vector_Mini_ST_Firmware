/**
 * @file bsp_can.h
 * @brief 两板共用CAN报文、直接收发及硬件忙/完成读取接口。
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief CAN报文数据，不直接序列化此结构。 */
    typedef struct
    {
        uint32_t identifier; /**< 标准ID为11位，扩展ID为29位。 */
        bool extended, remote, fd, bitrate_switch;
        uint8_t length; /**< 字节数：0..8、12、16、20、24、32、48或64。 */
        uint8_t data[64];
    } BspCanFrame;

    /**
     * @brief 在 CAN 启动前选择设备保存的位时序。
     * @param profile 参数服务校验过的档位；固定速率板只支持零。
     * @return 已设置支持的档位返回 true。
     * @note 仅在初始化的配置窗口内调用，不切换运行中的总线。
     */
    bool BspCan_ConfigureBitrate(uint8_t profile);

    /**
     * @brief 开放已配置的CAN及接收中断。
     * @note 应用接收对象就绪后调用；AT32轮询台架不启用接收中断。
     */
    void BspCan_Start(void);

    /**
     * @brief 向空闲硬件发送槽写入一帧并请求发送。
     * @param frame 调用方保证ID、格式和DLC对应长度合法，不保留指针。
     * @return 硬件忙返回false；已写入并请求发送返回true，不表示已上总线。
     * @note 单一发送者；不加锁、不检查参数、不建立票据、不重试。
     */
    bool BspCan_TrySend(const BspCanFrame *frame);

    /**
     * @brief 从硬件FIFO取出一帧。
     * @param frame 非空输出；数据帧更新length个字节，远程帧只更新头字段。
     * @return FIFO空返回false，已取出返回true。
     * @note 单一接收者；不做格式保护、溢出锁存或协议处理。
     */
    bool BspCan_Receive(BspCanFrame *frame);

    /**
     * @brief 直接读取硬件发送请求是否仍挂起。
     * @return 任一请求仍挂起为true，否则false。
     */
    bool BspCan_IsTransmitBusy(void);

    /**
     * @brief 直接读取最近一帧的硬件发送完成标志。
     * @return 发送槽已空闲且硬件记录最近一帧发送成功为true，否则false。
     * @note 不消费标志；成功提交下一帧后查询对应新帧，没有软件完成队列。
     */
    bool BspCan_IsTransmitComplete(void);

#ifdef __cplusplus
}
#endif
#endif /* BSP_CAN_H */
