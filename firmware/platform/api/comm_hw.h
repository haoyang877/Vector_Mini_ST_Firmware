#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t identifier;
    uint8_t length;
    bool extended, remote;
    uint8_t data[64];
    bool fd;
    bool bitrate_switch;
} CommHwCanFrame;

/**
 * @brief 单次非阻塞提交一帧 CAN FD 扩展数据帧。
 * @param frame 调用方拥有的帧；必须是 29 位扩展 ID、FD 数据帧和 BRS，length 为 DLC 对应的实际字节数。
 * @return 成功提交返回 true；格式非法或 HAL 忙/失败返回 false。
 * @note 不保存指针，不启动外设或改变滤波与位时序；单次硬件提交防中断抢占，调用方保留失败帧。
 */
bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame);

/**
 * @brief 非阻塞地取出一帧已完成接收的 CAN FD 报文。
 * @param frame 调用方提供的输出缓冲区；成功时写入完整帧，失败时保持内容不变。
 * @return 取到一帧返回 true；当前无完整帧返回 false。
 * @note 不等待、不重试，也不保存 frame 指针；调用方继续拥有缓冲区。
 */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/**
 * @brief 初始化并启动 CAN FD 接收通道。
 * @note 硬件接受扩展数据帧，FD/BRS 和地址由上层校验。失败进入平台错误处理。
 *       只允许在外设初始化后调用一次，不在此处更改 1M/5M 位时序。
 */
void comm_hw_can_start_fd(void);

/**
 * @brief 检查 bus-off 并重新启动 CAN FD 控制器。
 * @return 执行了恢复返回 true，健康时返回 false。
 * @note 前台每秒限频调用；不更改滤波、位时序或协议队列，失败进入平台错误处理。
 */
bool comm_hw_can_service_bus_off(void);
#endif
