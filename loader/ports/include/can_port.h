#ifndef LOADER_CAN_PORT_H
#define LOADER_CAN_PORT_H
#include <stdbool.h>
#include <stdint.h>
/** 端口只接纳扩展数据帧、FD+BRS；长度为真实DLC字节数。 */
typedef struct
{
    uint32_t identifier;
    uint8_t bytes;
    uint8_t data[64];
} LoaderCanFrame;
typedef struct
{
    uint64_t ticket;
    bool success;
} LoaderCanTxEvent;
/**
 * @brief 冷启动配置固定1M/5M CAN-FD及本机/广播过滤器。
 * @param node 本机0..254节点。
 * @return 配置且进入正常模式返回true。
 * @note 每次真实启动只调用一次；失联不自动恢复旧发送票据。
 */
bool LoaderCan_Init(uint8_t node);
/** @brief 读取一帧并重启被有界ISR暂停的接收。 @param frame 输出副本。 @return 有数据返回true。 */
bool LoaderCan_Receive(LoaderCanFrame *frame);
/**
 * @brief 复制完整物理帧到独占硬件槽。
 * @param frame 20/24/32/48/64B帧，填充由通信层生成。
 * @param ticket 完整64位激活票据；普通回复为0。
 * @return 入队返回true，不代表发送成功。
 */
bool LoaderCan_Send(const LoaderCanFrame *frame, uint64_t ticket);
/**
 * @brief 前台消费一个精确硬件完成事件。
 * @param event 输出票据和值，不依赖稍后可能复用的槽索引。
 * @return 有事件返回true；仅对应TXBTO且无取消/错误才成功。
 */
bool LoaderCan_TxEvent(LoaderCanTxEvent *event);
/** @brief 回读链路可用性；不自动清错误。 @return 正常运行且无bus-off返回true。 */
bool LoaderCan_Ready(void);
/** @brief 取消所有发送并停止CAN外设中断，不恢复旧会话。 */
void LoaderCan_Shutdown(void);
#endif
