#ifndef CAN_MOTOR_STATUS_H
#define CAN_MOTOR_STATUS_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "motor_status.h"

#define CAN_MOTOR_STATUS_COMMAND 0x64U
#define CAN_MOTOR_STATUS_REPLY 0x65U
#define CAN_MOTOR_STATUS_ID_BASE 0x7F0U
#define CAN_MOTOR_STATUS_SIZE 48U
/** Offset 38: extension revision 1 provides signed milliamp Ibus at offset 36.
 * Revision 0 is legacy padding and must not be interpreted as measured zero. */
#define CAN_MOTOR_STATUS_EXTENSION_REVISION 1U

/**
 * @brief 初始化状态流配置：默认停止发送，并把待用频率设为 20 Hz。
 * @note 必须在接收任何状态流命令之前调用；不发送报文，也不访问 Flash。
 */
void CanMotorStatus_Init(void);
/**
 * @brief 解析并应用状态流命令：0 停止，1 启动/恢复，10..200 设置 Hz 后启动。
 * @param command CAN 参数协议传入的浮点命令；设置频率时必须是范围内整数。
 * @return 命令合法且已应用返回 true；非法值返回 false 且保持原配置。
 * @note 可由接收 ISR 调用，只修改 RAM，不访问 Flash，也不直接发送报文。
 */
bool CanMotorStatus_Configure(float command);
/**
 * @brief 查询当前有效的状态流发送频率。
 * @return 已启用时返回 10..200 Hz；停止状态返回 0。
 * @note ISR 与前台均可读取。
 */
uint16_t CanMotorStatus_Rate(void);
/**
 * @brief 把状态快照编码为固定 48 字节的大端协议负载。
 * @param sample 待编码的只读状态快照，单位语义由 MotorStatus 定义。
 * @param data 调用方持有的输出缓冲区，成功时写入 CAN_MOTOR_STATUS_SIZE 字节。
 * @param capacity data 可写容量，必须不小于 CAN_MOTOR_STATUS_SIZE。
 * @return 编码成功返回 true；空指针或容量不足返回 false。
 * @note 纯函数式编码，不读取时钟、不修改调度状态，也不执行硬件 I/O。
 */
bool CanMotorStatus_Encode(const MotorStatus *sample, uint8_t *data, size_t capacity);
/**
 * @brief 在前台生成一帧到期状态；错过的多个周期合并为一帧最新数据。
 * @param now_ms 当前单调毫秒计数，允许按 2^32 回绕。
 * @param node 电机节点号，用于生成状态帧标识符。
 * @param identifier 输出标准 CAN 标识符；失败时保持内容不变。
 * @param data 调用方持有的状态负载输出缓冲区。
 * @param capacity data 可写容量，必须不小于 CAN_MOTOR_STATUS_SIZE。
 * @return 有新鲜且到期的状态帧时返回 true；未到期、无样本或参数非法返回 false。
 * @note 调用方只尝试提交一次且不得等待；总线拥塞时允许丢弃本帧。
 */
bool CanMotorStatus_Prepare(
    uint32_t now_ms, uint8_t node, uint16_t *identifier, uint8_t *data, size_t capacity);
#endif
