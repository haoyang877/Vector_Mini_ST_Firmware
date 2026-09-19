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
 * @note 每个 1 kHz 任务调用一次；设置未变化时完全不访问硬件。
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
#endif
