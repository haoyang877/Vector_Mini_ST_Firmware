#ifndef YG_PROTOCOL_CANFD_H
#define YG_PROTOCOL_CANFD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "bsp_can.h"
#include "yg_protocol_common.h"

#define YG_PROTOCOL_TRANSFER_DATA_SIZE 64U

/** @brief 不含 HAL 类型的 CAN 帧描述；不作为串口或以太网通用帧。 */
typedef struct
{
    uint32_t identifier;
    uint8_t length;
    bool extended;
    bool remote;
    bool fd;
    bool bitrate_switch;
    uint8_t data[YG_PROTOCOL_TRANSFER_DATA_SIZE];
} yg_protocol_transfer_frame_t;

/** @brief 固定容量SPSC帧队列；唯一生产者发布produced，唯一消费者发布consumed。 */
typedef struct
{
    yg_protocol_transfer_frame_t *storage;
    size_t capacity;
    size_t head;
    size_t tail;
    volatile uint32_t produced;
    volatile uint32_t consumed;
    uint32_t dropped;
} yg_protocol_transfer_queue_t;

/**
 * @brief 初始化传输帧队列。
 * @param queue 输出队列对象。
 * @param storage 调用方提供的固定帧数组。
 * @param capacity 数组容量，不能为零。
 * @return 成功返回 true；参数错误返回 false。
 * @note 不分配内存；storage 在队列生命周期内必须保持有效。
 */
bool yg_protocol_transfer_queue_init(yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *storage,
                                     size_t capacity);

/**
 * @brief 把一帧放入传输队列。
 * @param queue 已初始化队列。
 * @param frame 待入队帧，函数复制完整内容。
 * @return 入队成功返回 true；队列满或参数非法返回 false。
 * @note 不等待、不重试；队列满时丢弃最新帧并增加 dropped。允许唯一ISR生产者与唯一前台消费者并发；禁止多个生产者/消费者。
 */
bool yg_protocol_transfer_queue_push(yg_protocol_transfer_queue_t *queue,
                                     const yg_protocol_transfer_frame_t *frame);

/**
 * @brief 从传输队列取出一帧。
 * @param queue 已初始化队列。
 * @param frame 输出帧缓冲区。
 * @return 取到帧返回 true；队列为空或参数非法返回 false。
 * @note 函数复制完整帧，不保存调用方输出指针。
 */
bool yg_protocol_transfer_queue_pop(yg_protocol_transfer_queue_t *queue,
                                    yg_protocol_transfer_frame_t *frame);

/**
 * @brief 释放队首槽位，不再次复制已经读取的帧。
 * @param queue 已初始化队列。
 * @return 成功消费返回true；空队列或非法参数返回false。
 * @note 消费者复制完成后发布消费序号；peek至本调用期间只能有一个消费者。
 */
bool yg_protocol_transfer_queue_discard(yg_protocol_transfer_queue_t *queue);

/**
 * @brief 查询队列当前帧数量。
 * @param queue 已初始化队列。
 * @return 当前排队帧数；非法参数返回零。
 */
size_t yg_protocol_transfer_queue_count(const yg_protocol_transfer_queue_t *queue);

/**
 * @brief 查询队列累计丢弃帧数。
 * @param queue 已初始化队列。
 * @return 自初始化以来因队列满丢弃的帧数；非法参数返回零。
 */
uint32_t yg_protocol_transfer_queue_dropped(const yg_protocol_transfer_queue_t *queue);

/**
 * @brief 复制队首帧但不出队，供发送成功后再提交消费。
 * @param queue 已初始化队列；调用方保证访问串行化。
 * @param frame 输出副本，失败时保持不变。
 * @return 有帧返回 true；队列空或参数非法返回 false。
 * @note peek 到 pop 期间不得有其他消费者，防止发送和出队错配。
 * @note 单消费者可在不释放队首槽位期间复制；唯一ISR生产者只能追加或满队列丢新帧，
 *   不得重置队列。随后直接discard发布消费，不屏蔽中断。
 */
bool yg_protocol_transfer_queue_peek(const yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *frame);

/**
 * @brief 将单个应用帧编码为最小 DLC 长度的扩展 CAN FD 帧。
 * @param message 输入消息片段，payload 最大 46 字节。
 * @param priority CAN 仲裁优先级，范围 0..7。
 * @param frame 输出完整帧；失败时保持不变。
 * @return 编码结果。
 * @note 不访问硬件；FD/BRS 置位，CRC 之后的 DLC 填充置零；不能在 ISR 中计算 CRC。
 */
yg_protocol_result_t yg_protocol_canfd_pack(const yg_protocol_message_t *message,
                                            uint8_t priority,
                                            yg_protocol_transfer_frame_t *frame);

/**
 * @brief 校验 CAN 元数据、地址与填充并解码一个应用帧。
 * @param frame 输入扩展 CAN FD 帧，length 为实际 DLC 字节数。
 * @param local_node 本机节点；不能为广播地址。
 * @param message 输出借用视图；失败时不变，成功后 payload 指向 frame 的数据。
 * @return 解码结果；CRC 校验通过不代表消息已被业务服务接受。
 * @note 接受本机及广播目的地；ID 地址必须匹配应用头。仅通信任务调用。
 */
yg_protocol_result_t yg_protocol_canfd_unpack(const yg_protocol_transfer_frame_t *frame,
                                              uint8_t local_node,
                                              yg_protocol_message_t *message);

/**
 * @brief 将平台收到的 CAN FD 帧复制到 RX 队列，不读取硬件 FIFO。
 * @param queue 调用方拥有的已初始化 RX 队列。
 * @param received 由唯一硬件接收入口提供的帧；函数返回后不保存其指针。
 * @return 入队结果；CRC、地址匹配与业务准入留给后续 unpack 和服务处理。
 * @note 只做有界复制和元数据检查；队列仅允许一个接收生产者和一个前台消费者，初始化不得并发。
 */
yg_protocol_result_t yg_protocol_canfd_receive(yg_protocol_transfer_queue_t *queue,
                                               const BspCanFrame *received);

/**
 * @brief 尝试发送 TX 队首的一个 CAN FD 帧，成功才出队。
 * @param queue 已初始化 TX 队列，内容由 pack 生成。
 * @return 空队列、格式错误或硬件忙返回对应结果，失败时保留队首。
 * @note 单上下文调用；一次调用最多一次平台发送，无等待、无循环重试。
 */
yg_protocol_result_t yg_protocol_canfd_send_one(yg_protocol_transfer_queue_t *queue);

#endif
