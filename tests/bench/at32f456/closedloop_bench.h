#ifndef AT32_CLOSEDLOOP_BENCH_H
#define AT32_CLOSEDLOOP_BENCH_H

#include "openloop_model.h"
#include "foc.h"
#include "encoder_feedback.h"

/* 独立台架装配：复用原有FOC/速度算法，不接入产品命令、参数持久化或输出轴闭环。 */
typedef struct
{
    uint32_t mode, passed_mask, failure, elapsed_ms, control_samples, quality_samples;
    int32_t direction;
    float id_reference_a, iq_reference_a, id_a, iq_a, vd_v, vq_v;
    float phase_rad, velocity_rad_s, zero_q15, tracking_mae_a, tracking_max_a;
    float calibration_travel_counts, calibration_return_counts, phase_sum_a;
    uint32_t speed_samples;
    float speed_mae_rpm;
} AT32ClTelemetry;

typedef struct
{
    MotorConfiguration configuration;
    MotorControlState motor;
    FocState foc;
    PiController speed_pi;
    EncoderState encoder;
    AT32ClTelemetry telemetry;
    int32_t position_counts, last_angle, stage_start_counts;
    int64_t angle_sums[3];
    uint32_t angle_samples[3];
    float error_sum_a;
    float speed_error_sum_rpm;
    float speed_reference_rpm;
    bool angle_initialized;
} AT32ClBench;

/** @brief 初始化闭环台架，固定低电流参考与3A瞬时相电流停机边界。
 * @param bench 调用方独占的台架状态。
 * @note 仅在平台初始化后调用，会初始化编码器端口，不使能功率。
 */
void AT32Cl_Init(AT32ClBench *bench);
/**
 * @brief 消费已完成的DMA角度并更新方向校正和速度PLL。
 * @param bench 台架配置和编码器运行对象。
 * @return 原始单圈角度，0..65535。
 * @note 调用前由台架执行BspEncoder_Begin，不提供采集错误状态。
 */
uint16_t AT32Cl_ReadEncoder(AT32ClBench *bench);
/** @brief 按顺序请求定向电流、整圈角度校准、Vq开环加Id闭环、速度四个独立阶段。
 * @param bench 调用方独占的台架状态。
 * @param guard 原开环入口的校零、采样、租期与停机保护状态。
 * @param mode 阶段编号1..4，每阶段仅允许成功执行一次。
 * @param now_ms 当前单调毫秒。
 * @return 已通过前序阶段且保护允许本次开始时返回true。
 * @note 故障后不重启；阶段完成不自动开始下一阶段。
 */
bool AT32Cl_Begin(AT32ClBench *bench, AT32OlModel *guard, uint32_t mode, uint32_t now_ms);
/** @brief 消费保护后的一拍采样，运行原有FOC并记录校准与跟踪证据。
 * @param bench 调用方独占的台架状态。
 * @param guard 已完成本拍原始采样保护的状态。
 * @param angle_q15 MT6835整圈16位角度。
 * @note 20kHz调用；仅RUNNING时调用原FOC写PWM，输出使能仍由入口管理。
 */
void AT32Cl_Step(AT32ClBench *bench, AT32OlModel *guard, uint16_t angle_q15);

#endif
