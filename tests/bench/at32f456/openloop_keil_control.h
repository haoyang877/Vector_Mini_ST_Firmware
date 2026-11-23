#ifndef AT32_OPENLOOP_KEIL_CONTROL_H
#define AT32_OPENLOOP_KEIL_CONTROL_H

#include "openloop_model.h"

/* Watch仅写此开关；默认0，每次复位最多一次试转，完成或故障自动清零。 */
extern volatile uint32_t g_openloop_enable;
extern volatile uint32_t g_openloop_rejected_starts;

typedef struct
{
    volatile bool active;
} AT32KeilControl;

/** @brief 初始化Keil台架开关，不改变模型或功率输出。
 * @param control 快拍维护的开关状态。
 */
void AT32KeilControl_Init(AT32KeilControl *control);

/** @brief 在关相就绪时消费一次开关，运行期间由存活前台刷新租期。
 * @param control 快拍维护的开关状态。
 * @param model 原开环保护模型，保留电流、总时限及故障锁存。
 * @param requested 前台本次采到的开关值。
 * @param now_ms 前台登记的毫秒时刻，过期时不会延长租期。
 * @return 需要调用方立即关闭输出时返回true，否则false。
 * @note ADC 拍内串行推进；无硬件访问，不清除DONE或FAULT，不替代WDT。
 */
bool AT32KeilControl_Poll(AT32KeilControl *control,
                          AT32OlModel *model,
                          uint32_t requested,
                          uint32_t now_ms);

#endif
