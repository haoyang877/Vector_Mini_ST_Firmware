#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t identifier;
    uint8_t length;
    bool extended, remote;
    uint8_t data[64];
} CommHwCanFrame;

/**
 * @brief 配置节点接收范围并启动 CAN 接收通知。
 * @param node 节点号，范围 0～7。
 * @note 仅板级启动调用，外设基础时序须已初始化；硬件失败沿用致命 Error_Handler。
 */
void comm_hw_can_start(uint8_t node);
/**
 * @brief 停止 CAN 并按指定数据段速率重新配置、启动。
 * @param bitrate_kbps 数据段速率，单位 kbit/s；调用者须提供协议允许的档位。
 * @note 不超过 1000 时仲裁段同速，否则仲裁段固定 1000；不改变过滤器与通知。
 * 沿用 100 ms 监督调用和 HAL 同步等待行为，硬件失败进入致命 Error_Handler。
 */
void comm_hw_can_set_bitrate(uint32_t bitrate_kbps);

/** Pop one received frame into a full-size bounded buffer, without waiting. */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/** Submit optional CAN FD telemetry only when no transmission is queued.
 * Never retry/wait; reserves room for subsequent priority replies. */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
/** @brief 空队列中提交2/4/8/16字节参数或维护回复。
 * @param identifier 标准CAN ID。
 * @param data 只读回复字节。
 * @param length 2、4、8或16字节。
 * @return 入队成功为true。
 * @note 不等待；成功后用sent接口确认本次实际发送。
 */
bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, size_t length);
/** @brief 检查维护回复已完成且无待发送消息。
 * @return 实际完成为true。
 */
bool comm_hw_can_reply_sent(void);
/** @brief 更新标准帧节点范围过滤器，失败时保留原过滤器。
 * @param node 节点号，范围 0～7。
 * @return 硬件过滤器更新成功返回 true。
 * @note 仅 CAN RX 中断在电机完全停机后调用；不停止外设、不等待、不屏蔽 ADC。
 * 调用者仅在成功后发布软件节点号，同一 CAN 中断不可重入。
 */
bool comm_hw_can_set_node_filter(uint8_t node);
#endif
