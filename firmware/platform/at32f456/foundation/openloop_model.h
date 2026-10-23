#ifndef R40_OPENLOOP_MODEL_H
#define R40_OPENLOOP_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/* 首次试转固定小信号配置；开环电压不构成电流调节，超限只能立即停机。 */
#ifndef R40_OL_VOLTAGE_V
#define R40_OL_VOLTAGE_V 0.20f
#endif
#ifndef R40_OL_ELECTRICAL_HZ
#define R40_OL_ELECTRICAL_HZ 2.0f
#endif
#ifndef R40_OL_DURATION_MS
#define R40_OL_DURATION_MS 1000U
#endif
#if R40_OL_DURATION_MS < 400U || R40_OL_DURATION_MS > 10000U
#error "Open-loop bench duration must remain within 400..10000 ms"
#endif
#define R40_OL_SAMPLE_LIMIT (R40_OL_DURATION_MS * 20U)
#define R40_OL_RAMP_MS 200U
#ifndef R40_OL_CURRENT_LIMIT_A
#define R40_OL_CURRENT_LIMIT_A 0.60f
#endif
#define R40_OL_LEASE_MS 250U
#define R40_OL_SETTLE_SAMPLES 20000U
#define R40_OL_CALIBRATION_SAMPLES 2048U

typedef enum
{
    R40_OL_CALIBRATING,
    R40_OL_READY,
    R40_OL_RUNNING,
    R40_OL_DONE,
    R40_OL_FAULT,
    R40_OL_ARMING
} R40OlState;

typedef enum
{
    R40_OL_OK,
    R40_OL_ADC_ERROR,
    R40_OL_ENCODER_ERROR,
    R40_OL_OFFSET_ERROR,
    R40_OL_OVERCURRENT,
    R40_OL_BUS_VOLTAGE,
    R40_OL_LEASE_EXPIRED,
    R40_OL_OUTPUT_ERROR,
    R40_OL_TIMING_ERROR,
    R40_OL_USER_STOP
} R40OlFault;

typedef struct
{
    R40OlState state;
    R40OlFault fault;
    uint32_t settling_samples, calibration_samples, quiet_samples, running_samples;
    uint32_t started_ms, heartbeat_ms;
    uint32_t sums[3];
    uint16_t low[3], high[3];
    float offsets[3], current_a[3], peak_current_a, bus_v, phase_rad, duty[3];
} R40OlModel;

/** @brief 初始化首次开环试转模型，开始关相零偏采样。
 * @param model 调用者独占的模型状态。
 * @note 无硬件操作；不能作为功率使能授权。
 */
void R40Ol_Init(R40OlModel *model);
/** @brief 显式请求一次固定参数的试转计算，完成后必须复位才能再次开始。
 * @param model 已完成零偏校验的模型。
 * @param now_ms 当前单调毫秒计数。
 * @return 仅READY状态接受请求并等待连续10ms零流稳定，接受返回true，否则false。
 */
bool R40Ol_Begin(R40OlModel *model, uint32_t now_ms);
/** @brief 更新主机试转心跳租期。
 * @param model 调用者独占的模型。
 * @param now_ms 当前单调毫秒计数。
 */
void R40Ol_Heartbeat(R40OlModel *model, uint32_t now_ms);
/** @brief 锁存故障并把三相计算占空比恢复中点。
 * @param model 调用者独占的模型。
 * @param fault 首个停机原因。
 */
void R40Ol_Stop(R40OlModel *model, R40OlFault fault);
/** @brief 消费一拍20kHz真实采样并计算三相正弦电压占空比。
 * @param model 调用者独占的模型。
 * @param raw 三相电流四次累加计数及母线四次累加计数，长度4。
 * @param adc_ok 本拍采样完整且健康。
 * @param encoder_ok 本拍编码器CRC及状态校验通过。
 * @param now_ms 当前单调毫秒计数。
 * @note 角度不参与闭环；电流只作为超限停机条件，不作PI调节。无硬件操作。
 */
void R40Ol_Step(
    R40OlModel *model, const uint16_t raw[4], bool adc_ok, bool encoder_ok, uint32_t now_ms);

#endif
