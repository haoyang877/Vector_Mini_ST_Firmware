#ifndef __RTT_TELEMETRY_H__
#define __RTT_TELEMETRY_H__

#include <stdbool.h>

/**
 * @brief 采样一帧 4 通道 RTT 遥测：机械位置、机械转速、Iq 指令与 Iq 反馈。
 * @note 仅由 20 kHz 快速中断调用；非阻塞、有界，不修改控制状态或运行模式。
 *       帧布局固定为 8 字节小端 int16，换算与量程见 rtt_telemetry.c。
 */
void RTT_Sampling(void);

#endif
