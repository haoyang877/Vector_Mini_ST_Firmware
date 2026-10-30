#ifndef AT32_APP_TELEMETRY_H
#define AT32_APP_TELEMETRY_H

#include <stdint.h>
#include "encoder_feedback.h"
#include "foc.h"
#include "motor_types.h"

/* 可选RTT诊断只读帧；历史符号保留以兼容Watch/采集脚本。
 * 平台验证只读帧；角速度均为电机轴rad/s，电流为瞬时幅值A。
 * timestamp_us按20kHz采样拍计数，uint32自然回绕；不构成产品协议ABI。 */
typedef struct
{
    uint32_t timestamp_us, sequence;
    float ia_a, ib_a, ic_a, id_reference_a, id_a, iq_reference_a, iq_a;
    float vd_v, vq_v, electrical_angle_rad, speed_reference_rad_s, speed_rad_s;
    uint32_t raw_q15, linearized_q15, state, fault, calibration_flags;
    uint32_t reserved_encoder; /* 保留原调试帧布局，固定为0。 */
    uint32_t maintenance_kind;
    float bus_v;
} AT32AppTelemetryFrame;

typedef struct
{
    uint32_t begin;
    AT32AppTelemetryFrame sample;
    uint32_t queued, queue_dropped, sent, rtt_dropped, end;
} AT32AppTelemetryDiagnostic;

extern volatile AT32AppTelemetryDiagnostic g_at32_app_telemetry;

/**
 * @brief 初始化仅输出的2kHz诊断通道及单生产者队列。
 * @note 前台启动时、中断采样前调用一次；不处理命令、不启相。
 */
void AT32AppTelemetry_Init(void);

/**
 * @brief 每10个真实快拍复制一帧，不调用RTT、不等待前台。
 * @param fast_samples 已完成的20kHz快拍序号。
 * @param foc 快环所有者的本拍电流与电压，只读。
 * @param measurement Axis持有的只读测量对象；电流A、电压V、温度°C。
 * @param motor 快环控制目标和故障，只读。
 * @param encoder 本拍角度与速度，只读。
 * @param state 当前生命周期值。
 * @param calibration_flags 活动配置的标定事实位。
 * @param maintenance_kind 当前维护种类。
 * @note 仅由快速采样末尾调用；满队列丢整帧并计数，不改任何控制对象。
 */
void AT32AppTelemetry_Capture(uint32_t fast_samples,
                              const FocState *foc,
                              const MotorMeasurement *measurement,
                              const MotorControlState *motor,
                              const EncoderState *encoder,
                              uint32_t state,
                              uint32_t calibration_flags,
                              uint32_t maintenance_kind);

/**
 * @brief 前台最多发送四帧；RTT满时丢整帧，不延迟控制与看门狗。
 * @note 单消费者，不能从ISR调用；没有RTT控制输入。
 */
void AT32AppTelemetry_Service(void);

#endif
