#ifndef AT32_BENCH_H
#define AT32_BENCH_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"

typedef enum
{
    ADC_MOTOR_TEMPERATURE_UNAVAILABLE = 0,
    ADC_MOTOR_TEMPERATURE_DISABLED,
    ADC_MOTOR_TEMPERATURE_VALID,
    ADC_MOTOR_TEMPERATURE_OPEN,
    ADC_MOTOR_TEMPERATURE_SHORT,
    ADC_MOTOR_TEMPERATURE_OUT_OF_RANGE,
    ADC_MOTOR_TEMPERATURE_BAD_CONFIG
} AdcMotorTemperatureStatus;

typedef struct
{
    uint32_t sequence;
    uint16_t raw_counts;
    AdcMotorTemperatureStatus status;
    float resistance_ohm;
    float celsius; /* 只有VALID时有效；其余状态为NaN。 */
} AdcMotorTemperatureSample;

/* 只在关相验证镜像中使用的调试内存布局，不属于产品协议或公共BSP。 */
#define AT32_BENCH_MAGIC 0x52343042U
#define AT32_BENCH_CAN_REQUEST 0x1FFF0001U
#define AT32_BENCH_CAN_RESPONSE 0x1FFF0002U

typedef struct
{
    uint32_t magic, version, command, command_inverse, nonce, completed_nonce, result, detail;
    uint32_t arguments[4];
    uint32_t values[16];
} AT32BenchMailbox;

typedef struct
{
    uint32_t begin, milliseconds, supervisor_ticks, fast_samples, encoder_samples, encoder_errors;
    uint32_t fast_max_cycles, angle, angle_min, angle_max;
    uint32_t cctrl, brk, gpioa_input, gpiob_input;
    uint32_t phase_a, phase_b, phase_c, vbus, bus_current;
    uint32_t ntc_sequence, ntc_raw, ntc_status, ntc_ohm_bits, ntc_celsius_bits;
    uint32_t can_rx, can_tx_ok, can_tx_failed, can_ctrl, core_hz, end;
} AT32BenchSnapshot;

extern volatile AT32BenchMailbox g_at32_bench;
extern volatile AT32BenchSnapshot g_at32_snapshot;

/** @brief 处理关相诊断请求；命令1测试PWM比较值，2写空白参数区，3核对并擦回空白。
 * @return 存储测试或安全检查结束返回true，调用者必须进入关相停止状态。
 * @note 只由验证前台调用；存储命令另需arguments[0]=0x53544F52，默认不写Flash。
 */
bool AT32_BenchPoll(void);
/** @brief 接收带固定四字节标记的诊断帧，仅回显数据，不解释控制指令。
 * @param frame 已通过公共CAN端口解码的输入帧。
 * @note 前台调用，繁忙时丢弃额外测试帧；不会启动功率输出。
 */
void AT32_BenchCanReceive(const BspCanFrame *frame);
/** @brief 发布同一 ADC 拍内取得的台架诊断数据。
 * @param sample 前台已采集的计数与温度，本函数补齐板级诊断字段。
 * @param revision 开始采集前的 ADC 中断序号。
 * @param adc_revision 本板 ADC 中断更新的序号。
 * @note 仅前台调用，采集被 ADC 抢占时保留上一帧。
 */
void AT32_BenchPublish(AT32BenchSnapshot *sample,
                       uint32_t revision,
                       const volatile uint32_t *adc_revision);

#endif
