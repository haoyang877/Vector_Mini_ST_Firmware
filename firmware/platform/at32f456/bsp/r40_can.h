#ifndef R40_CAN_H
#define R40_CAN_H

#include <stdbool.h>

/**
 * @brief 配置CAN1时钟、引脚、固定1M/5M时序及扩展数据帧过滤，保持控制器复位。
 * @return 时钟与配置读回匹配返回true，否则false。
 * @note 仅前台初始化一次；随后由既有comm_hw_can_start_fd启动，不发送报文。
 */
bool R40_CanInit(void);

#endif
