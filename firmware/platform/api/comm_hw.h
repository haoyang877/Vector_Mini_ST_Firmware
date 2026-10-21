#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "../../../shared/can_deployment.h"

/** @brief 一帧完整接收结果；调用者拥有数据，端口不保留缓冲区。 */
typedef struct {
    uint32_t identifier; /**< 接收的标准或扩展标识符，不包含格式标志位。 */
    uint8_t length;      /**< 实际载荷字节数，范围 0～64。 */
    bool extended;      /**< true 为扩展帧。 */
    bool remote;        /**< true 为远程请求，不得作为参数命令。 */
    bool fd;            /**< true 为 CAN FD，false 为经典 CAN。 */
    bool brs;           /**< FD 数据段速率切换标志；经典帧必须为 false。 */
    uint8_t data[64];    /**< 完整载荷副本，生命周期由调用者管理。 */
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
 * @note 不超过 1000 时仲裁段同速，否则仲裁段固定 1000；经典部署拒绝大于 1000。
 * 重初始化后重新安装当前节点过滤器、部署帧格式及通知。
 * 沿用 100 ms 监督调用和 HAL 同步等待行为，硬件失败进入致命 Error_Handler。
 */
void comm_hw_can_set_bitrate(uint32_t bitrate_kbps);

/**
 * @brief 非阻塞取出一帧完整载荷及 FD/BRS 格式。
 * @param[out] frame 接收缓冲区；NULL 时不读取硬件。
 * @return 取帧成功且 DLC 合法返回 true；无帧、HAL 失败或非法 DLC 返回 false。
 * @note 不保留调用者缓冲区，不发送任何报文。
 */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/** Submit optional CAN FD telemetry only when no transmission is queued.
 * Never retry/wait; reserves room for subsequent priority replies. */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
/**
 * @brief 在空发送队列中按请求冻结的格式提交参数回复。
 * @param[in] identifier 标准 CAN ID，范围 0～0x7FF。
 * @param[in] data 至少 length 字节的载荷；函数返回后不再使用该指针。
 * @param[in] length 允许 2、4、8 字节，FD 另允许 16 字节。
 * @param[in] fd true 发送 CAN FD，false 发送经典 CAN。
 * @param[in] brs FD 速率切换标志；经典帧必须为 false。
 * @return HAL 入队成功返回 true；参数非法、队列忙或 HAL 失败返回 false。
 * @note 不等待、不重试；成功入队不代表已送达，维护流程仍须查询 sent 接口。
 */
bool comm_hw_can_try_send_reply_format(uint16_t identifier, const uint8_t *data,
    size_t length, bool fd, bool brs);
/**
 * @brief 使用历史 FD/BRS 格式提交维护回复，保持 BOOT 接口兼容。
 * @param[in] identifier 标准 CAN ID，范围 0～0x7FF。
 * @param[in] data 至少 length 字节的载荷；函数返回后不再使用该指针。
 * @param[in] length 允许 2、4、8 或 16 字节。
 * @return HAL 入队成功返回 true；参数非法、队列忙或 HAL 失败返回 false。
 * @note 不等待、不重试；实际发送完成仍须查询 sent 接口。
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
