#ifndef YG_PROTOCOL_TRANSFER_H
#define YG_PROTOCOL_TRANSFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

/** @brief 固定容量帧队列；支持一个生产者与一个消费者跨 ISR/任务并发。 */
typedef struct
{
    yg_protocol_transfer_frame_t *storage;
    size_t capacity;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
    volatile uint32_t dropped;
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
 * @note 不等待、不重试；队列满时丢弃最新帧并增加 dropped。队列只能有一个生产者和一个消费者。
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
 */
bool yg_protocol_transfer_queue_peek(const yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *frame);

#endif
