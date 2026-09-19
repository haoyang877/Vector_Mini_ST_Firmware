#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct
{
    uint32_t identifier;
    uint8_t length;
    bool extended, remote;
    uint8_t data[64];
} CommHwCanFrame;

/**
 * @brief 非阻塞地取出一帧已完成接收的 CAN/CAN FD 报文。
 * @param frame 调用方提供的输出缓冲区；成功时写入完整帧，失败时保持内容不变。
 * @return 取到一帧返回 true；当前无完整帧返回 false。
 * @note 不等待、不重试，也不保存 frame 指针；调用方继续拥有缓冲区。
 */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/**
 * @brief 仅在发送队列空闲时提交低优先级 CAN FD 状态帧。
 * @param identifier 标准帧标识符，只使用低 11 位。
 * @param data 只读负载缓冲区；函数返回后不保存指针。
 * @param length 负载字节数，合法范围为 0..64。
 * @return 成功入队返回 true；参数非法或队列忙返回 false。
 * @note 函数不等待、不重试，为后续高优先级应答保留发送空间。
 */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
/**
 * @brief 初始化并启动 CAN 接收通道：配置本节点范围滤波、启动外设并使能接收中断。
 * @param node 本节点号；滤波范围固定为 [node<<8, node<<8+0xFF]，调用方保证取值 0..7。
 * @note 失败按致命处理：移植层直接进入平台错误处理，本函数不返回。
 */
void comm_hw_can_start(uint8_t node);
/**
 * @brief 切换 CAN 传输波特率并重启外设。
 * @param kbps 目标波特率，单位 kbps；调用方保证不为 0。
 * @note kbps 不大于 1000 时数据段与仲裁段同取 10000/kbps 分频，否则仲裁段固定 10 分频；
 *       失败按致命处理，本函数不返回。
 */
void comm_hw_can_set_baudrate(uint32_t kbps);
/**
 * @brief 单次非阻塞提交一帧控制应答帧。
 * @param identifier 标准帧标识符，只使用低 11 位。
 * @param data 只读负载缓冲区；函数返回后不保存指针，调用方保证非空。
 * @param length 负载字节数，取 2 或 4，与对应 DLC 编码同值。
 * @return 成功进入发送队列返回 true；队列忙或提交失败返回 false。
 * @note 不等待、不重试；重试次数与退避由调用方决定。
 */
bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, uint8_t length);
#endif
