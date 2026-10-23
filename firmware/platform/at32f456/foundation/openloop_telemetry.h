#ifndef R40_OPENLOOP_TELEMETRY_H
#define R40_OPENLOOP_TELEMETRY_H

#include <stdint.h>

/* 20kHz快采样每10拍发布一次，对应2kHz；主入口与发布器共用同一时隙。 */
#define R40_RTT_FAST_TICKS 10U

/* 仅台架前台使用的采样值，快照来源由入口在短临界区中复制。 */
typedef struct
{
    uint32_t fast_samples, state, fault, running_samples, encoder_errors;
    uint32_t angle_counts, sin_raw, cos_raw, sincos_sequence, sincos_age_ms, sincos_valid;
    float ia_a, ib_a, ic_a, bus_v;
} R40OpenLoopTelemetrySample;

/* Watch读取对象；begin/end相同偶数才是一致快照，角度单位度，速度单位rpm。 */
typedef struct
{
    uint32_t begin, state, fault, elapsed_ms, encoder_errors, angle_counts;
    uint32_t sin_raw, cos_raw, sincos_sequence, sincos_age_ms, sincos_valid;
    float ia_a, ib_a, ic_a, angle_deg, speed_rpm, bus_v;
    uint32_t rtt_sent, rtt_dropped, end;
} R40OpenLoopDebug;

extern volatile R40OpenLoopDebug g_openloop_debug;

/** @brief 初始化仅输出的非阻塞RTT通道及台架观察量。
 * @note 仅在关相启动期间由前台调用，不读取RTT输入，也不使能功率。
 */
void R40OpenLoopTelemetry_Init(void);

/** @brief 按2kHz时隙发布至Watch和RTT；RTT满时丢整帧，不等待主机。
 * @param sample 已一致复制的当前采样，单位由结构字段名给出。
 * @note 仅前台调用，禁止在ISR调用；RTT没有控制电机的入口。
 */
void R40OpenLoopTelemetry_Publish(const R40OpenLoopTelemetrySample *sample);

#endif
