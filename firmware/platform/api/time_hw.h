#ifndef TIME_HW_H
#define TIME_HW_H
#include <stdint.h>
/**
 * @brief 读取单调递增的毫秒时钟，计数按 2^32 自然回绕。
 * @return 当前毫秒计数；调用方必须使用无符号差值处理回绕。
 * @note 前台和 ISR 均可调用，不阻塞且不修改硬件状态。
 */
uint32_t time_hw_now_ms(void);
#endif
