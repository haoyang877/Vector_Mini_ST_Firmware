#ifndef R40_BENCH_H
#define R40_BENCH_H

#include <stdbool.h>
#include <stdint.h>
#include "comm_hw.h"
#include "../ports/motor_temperature_at32f456.h"

/* 只在关相验证镜像中使用的调试内存布局，不属于产品协议或公共BSP。 */
#define R40_BENCH_MAGIC 0x52343042U
#define R40_BENCH_CAN_REQUEST 0x1FFF0001U
#define R40_BENCH_CAN_RESPONSE 0x1FFF0002U

typedef struct
{
    uint32_t magic, version, command, command_inverse, nonce, completed_nonce, result, detail;
    uint32_t arguments[4];
    uint32_t values[16];
} R40BenchMailbox;

typedef struct
{
    uint32_t begin, milliseconds, supervisor_ticks, fast_samples, encoder_samples, encoder_errors;
    uint32_t fast_max_cycles, angle, angle_min, angle_max;
    uint32_t cctrl, brk, gpioa_input, gpiob_input;
    uint32_t phase_a, phase_b, phase_c, vbus, bus_current;
    uint32_t ntc_sequence, ntc_raw, ntc_status, ntc_ohm_bits, ntc_celsius_bits;
    uint32_t can_rx, can_tx_ok, can_tx_failed, can_ctrl, core_hz, end;
} R40BenchSnapshot;

extern volatile R40BenchMailbox g_r40_bench;
extern volatile R40BenchSnapshot g_r40_snapshot;

/** @brief 处理关相诊断请求；命令1测试PWM比较值，2写空白参数区，3核对并擦回空白。
 * @return 存储测试或安全检查结束返回true，调用者必须进入关相停止状态。
 * @note 只由验证前台调用；存储命令另需arguments[0]=0x53544F52，默认不写Flash。
 */
bool R40_BenchPoll(void);
/** @brief 接收带R40B标记的诊断帧，仅回显数据，不解释控制指令。
 * @param frame 已通过公共CAN端口解码的输入帧。
 * @note 前台调用，繁忙时丢弃额外测试帧；不会启动功率输出。
 */
void R40_BenchCanReceive(const CommHwCanFrame *frame);
/** @brief 发布一致性诊断快照。
 * @param supervisor_ticks 监督中断累计次数。
 * @param fast_samples 快采样累计次数。
 * @param encoder_samples 编码器有效样本次数。
 * @param encoder_errors 编码器错误次数。
 * @param max_cycles 快采样中断最大执行周期。
 * @param angle 最近有效编码器角度，单圈65536计数。
 * @param angle_min 本次启动有效角度最小值。
 * @param angle_max 本次启动有效角度最大值。
 * @param temperature 私有电机温度诊断值。
 * @note 前台调用，短临界区复制；主机仅接受begin=end且为非零偶数的快照。
 */
void R40_BenchPublish(uint32_t supervisor_ticks,
                      uint32_t fast_samples,
                      uint32_t encoder_samples,
                      uint32_t encoder_errors,
                      uint32_t max_cycles,
                      uint32_t angle,
                      uint32_t angle_min,
                      uint32_t angle_max,
                      const R40MotorTemperatureSample *temperature);

#endif
