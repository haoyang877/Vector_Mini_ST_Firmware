#ifndef PROJECT_SEGGER_RTT_CONF_H
#define PROJECT_SEGGER_RTT_CONF_H

/* 公用RTT项目配置：当前调用方均由前台单写，ISR只写本地采样队列。
 * 通道名和专用缓冲区由ConfigUpBuffer传入；不等待主机或屏蔽采样中断。 */
#define SEGGER_RTT_LOCK()
#define SEGGER_RTT_UNLOCK()
#define RTT_USE_ASM 0
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS 2
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 1
#define BUFFER_SIZE_UP 1024
#define BUFFER_SIZE_DOWN 16
#define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP

#endif
