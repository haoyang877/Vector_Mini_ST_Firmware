#ifndef R40_PLATFORM_H
#define R40_PLATFORM_H

#include <stdbool.h>

/**
 * @brief 将六路门极输入置低、编码器CS置高，再配置时钟及停止状态的定时器。
 * @return 初始化成功返回true，时钟或关相核对失败返回false。
 * @note 仅从复位后的前台调用一次；失败时不得继续启动调度或功率输出。
 */
bool R40_PlatformInit(void);

/**
 * @brief 启动1ms时基、20kHz中心对齐计数器和2kHz监督定时器。
 * @return 前置状态满足且时基启动成功返回true，否则返回false。
 * @note 必须先成功初始化；保持六路输出关闭，不启用ADC或调用App_FastTick。
 */
bool R40_TimingStart(void);

#endif
