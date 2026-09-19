#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "comm_hw.h"

/* CAN 传输机制层：本人持有波特率运行态，负责启停编排、帧级收帧过滤与应答发送重试。
 * 所有硬件动作都经 platform/api 的 comm_hw 契约，本层不含 HAL 类型或句柄。 */

/**
 * @brief 初始化 CAN 传输：按节点启动接收通道，并把当前波特率置为 1000 kbps。
 * @param node 本节点号，调用方保证取值 0..7。
 * @note 启动失败由移植层按致命处理，本函数不返回。
 */
void CanTransport_Init(uint8_t node);
/**
 * @brief 检测波特率设置变化并按需切换外设波特率。
 * @note 每个 2 kHz 监督 tick 调用一次；设置未变化时完全不访问硬件。
 */
void CAN_BaudRateSwitching(void);
/**
 * @brief 读取当前生效的 CAN 波特率设置。
 * @return 波特率设置，单位 kbps。
 */
uint32_t CanTransport_Baudrate(void);
/**
 * @brief 写入 CAN 波特率设置，由后续的切换调用生效。
 * @param kbps 目标波特率，单位 kbps。
 * @note 只更新设置存储，不立即访问硬件。
 */
void CanTransport_SetBaudrate(uint32_t kbps);
/**
 * @brief 取出一帧已完成接收的报文并做帧级过滤。
 * @param frame 调用方提供的输出缓冲区；通过过滤时写入完整帧。
 * @return 取到并通过过滤返回 true；无帧或过滤失败返回 false。
 * @note 丢弃扩展帧、远程帧、长度不是 2/4 的帧、标识符越界帧以及状态流 ID 区间的帧。
 */
bool CanTransport_ReceiveFrame(CommHwCanFrame *frame);
/**
 * @brief 以最多 5 次尝试提交一帧控制应答。
 * @param identifier 标准帧标识符，只使用低 11 位。
 * @param data 只读负载缓冲区，长度由 length 给出。
 * @param length 负载字节数，取 2 或 4。
 * @note 前台调用；仅重试到上限后放弃，不等待、不阻塞、不保存指针。
 */
void CanTransport_SendReply(uint32_t identifier, const uint8_t *data, uint8_t length);
/**
 * @brief 仅在发送队列空闲时尝试提交一帧状态流。
 * @param identifier 标准帧标识符，只使用低 11 位。
 * @param data 只读负载缓冲区；函数返回后不保存指针。
 * @param length 负载字节数，必须为 48。
 * @return 成功入队返回 true；参数非法或队列忙返回 false。
 * @note 不等待、不重试，为高优先级应答保留发送空间。
 */
bool CanTransport_TrySendStatus(uint16_t identifier, const uint8_t *data, size_t length);

/* ---- 命令/应答定长环：中断只入队，2 kHz 服务统一排空 ----
 * RX 环为单生产者（CAN 接收中断）/单消费者（2 kHz 服务）；TX 环的入队与出队
 * 都在 2 kHz 服务内，属单上下文。满时一律丢弃最新并计数，不等待、不分配。 */

#define CAN_RX_RING_CAPACITY 8U
#define CAN_TX_RING_CAPACITY 8U

/** @brief 已解码的待派发命令；param_id 只保留线路参数 ID 的低 8 位。 */
typedef struct
{
    uint8_t param_id;
    float data;
} CanQueuedCommand_TypeDef;

/** @brief 已编码的待发应答；data 已按线路编码的字节序排好。 */
typedef struct
{
    uint8_t param_id;
    uint8_t data[4];
    uint8_t length;
} CanTxReply_TypeDef;

/**
 * @brief 命令入队：把一帧已解码命令交给 2 kHz 服务派发。
 * @param param_id 线路参数 ID 低 8 位。
 * @param data 已按线路编码还原的 SI 值。
 * @return 入队成功返回 true；环满返回 false 并递增 RX 丢帧计数。
 * @note 仅 CAN 接收中断调用；单生产者写入，不等待、不分配。
 */
bool CanTransport_PushRxCommand(uint8_t param_id, float data);
/**
 * @brief 命令出队：取出一条待派发命令。
 * @param command 调用方持有的输出缓冲区；取出成功时写入。
 * @return 取到返回 true；环空返回 false。
 * @note 仅 2 kHz 服务调用；单消费者读取。
 */
bool CanTransport_PopRxCommand(CanQueuedCommand_TypeDef *command);
/**
 * @brief 应答入队：把一帧已编码应答排入发送队列。
 * @param reply 待入队应答，指针只在调用期间有效。
 * @return 入队成功返回 true；环满返回 false 并递增 TX 丢帧计数。
 * @note 仅 2 kHz 派发上下文调用；FIFO 顺序保序。
 */
bool CanTransport_PushTxReply(const CanTxReply_TypeDef *reply);
/**
 * @brief 应答出队：取出一帧待发应答。
 * @param reply 调用方持有的输出缓冲区；取出成功时写入。
 * @return 取到返回 true；队列空返回 false。
 * @note 仅 2 kHz 服务调用。
 */
bool CanTransport_PopTxReply(CanTxReply_TypeDef *reply);
/**
 * @brief 查询是否仍有待发应答。
 * @return 有待发应答返回 true，否则返回 false。
 * @note 2 kHz 服务用于"应答优先于状态流"判定。
 */
bool CanTransport_TxPending(void);
/**
 * @brief 读取 RX 环丢帧计数。
 * @return 自启动以来的累计丢帧数；只增不减。
 */
uint32_t CanTransport_GetRxDropCount(void);
/**
 * @brief 读取 TX 环丢帧计数。
 * @return 自启动以来的累计丢帧数；只增不减。
 */
uint32_t CanTransport_GetTxDropCount(void);
#endif
