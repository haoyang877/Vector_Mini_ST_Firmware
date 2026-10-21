#ifndef LOADER_LINK_H
#define LOADER_LINK_H
#include <stdbool.h>
#include <stdint.h>
/** @brief 原始CAN-FD帧，不暴露HAL类型。 */
typedef struct
{
    uint16_t id;
    uint8_t len;
    uint8_t data[64];
} ldr_rx_frame_t;
/** @brief FD 部署初始化1M/1M目标过滤；经典部署保持端口静默。
 * @param node 0～7节点。
 * @return 初始化成功或经典静默部署为true。
 */
bool loader_link_init(uint8_t node);
/** @brief 前台取一帧，不阻塞。
 * @param out 输出帧。
 * @return 有帧为true；经典静默部署始终false。
 */
bool loader_link_rx_pop(ldr_rx_frame_t *out);
/** @brief 非阻塞提交回复；发送完成需另行查询。
 * @param data 回复字节。
 * @param len 合法FD字节数。
 * @return 实际入队成功为true；经典静默部署始终false。
 * @note 旧回复挂起超过100 ms时请求取消并返回false；必须重试原请求，取消不等于发送完成。
 */
bool loader_link_send(const uint8_t *data, uint8_t len);
/** @brief 检查本次队列发送是否全部完成。
 * @return 有完成标志且无待发送消息为true。
 */
bool loader_link_sent(void);
/** @brief 前台关闭CAN中断与外设。 */
void loader_link_shutdown(void);
#endif
