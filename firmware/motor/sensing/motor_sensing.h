#ifndef MOTOR_SENSING_H
#define MOTOR_SENSING_H

#include "motor_config.h"
#include "application_config.h"
#include "foc.h"
#include "motor_types.h"

/** @brief 上电零偏采样状态；Axis持有实例，采样模块通过显式对象独占推进。 */
typedef struct
{
    volatile uint32_t phase_sum_raw[3];
    volatile uint32_t bus_sum_raw;
    volatile uint32_t sample_count;
    volatile bool complete, failed;
    bool bus_current_enabled;
} CurrentOffsetCalibration;

/**
 * @brief 清除本次上电零偏资格及电流测量，不沿用存储零偏。
 * @param calibration Axis持有的校正状态，调用后清零。
 * @param configuration 活动配置，三相零偏设为NAN。
 * @param foc 测量对象，三相及母线电流设为NAN。
 * @param bus_current_enabled 硬件是否提供母线电流通道；提供时必须与三相一起校正。
 * @note 仅上电关相初始化；不写硬件、参数修订或Flash。
 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              FocState *foc,
                              bool bus_current_enabled);

/**
 * @brief 累计三相及已启用的母线电流原码，2048拍后校验并一次发布全部零偏。
 * @param calibration Axis持有的校正状态，完成或失败后不再读写ADC零偏。
 * @param configuration 活动配置，成功时更新三相零偏，单位counts；母线零偏仅写BSP。
 * @param motor_state 当前控制模式及故障所有者，非关相、原码或均值越界时锁存零偏故障。
 * @return 本拍首次完成并发布返回true，其余返回false；失败状态由calibration报告。
 * @note 仅由INIT状态的20kHz快拍调用；不启相、不重试、不写Flash。
 *       三相累加域与母线单次域分别求均值，在短临界区复核关相后统一发布。
 */
bool MotorCurrentOffsets_Calibrate(CurrentOffsetCalibration *calibration,
                                   MotorConfiguration *configuration,
                                   MotorControlState *motor_state);

/**
 * @brief 读取BSP母线电压并更新低通滤波，单位V。
 * @param foc 测量对象；bus_voltage_filter_alpha由调用方设置为[0,1]。
 * @note 快速环调用；读取本拍已完成采样的通道，不启动转换，不判定过压/欠压或上报故障。
 */
void MotorVoltage_Update(FocState *foc);

/**
 * @brief 读取BSP三相及母线电流并更新母线低通，单位A。
 * @param foc 调用方持有的测量对象；bus_current_filter_alpha由调用方设置为[0,1]。
 * @note 校零完成后在快速环依次读取A/B/C/母线通道，不启动转换，不校零或上报故障。
 *       缺失或未校零由BSP返回NAN；母线首次有效值直接建立滤波基准，
 *       非有限输入使滤波值同步失效，后续有效输入重新建立基准。
 */
void MotorCurrent_Update(FocState *foc);

/**
 * @brief  更新 MCU 温度换算与遥测。
 * @param  foc 测量对象，写入温度字段。
 * @param  motor_state 电机故障所有者；采样失效时锁存故障。
 * @note 2 kHz 监督任务调用，不得在 20 kHz 快速环调用。
 */
void McuTemperature_Update(FocState *foc, MotorControlState *motor_state);

/* missed_ticks 以监督 tick（2 kHz，0.5 ms）计数；valid 为换算有效性标志。 */
typedef struct
{
    uint32_t raw_ts, raw_vref, valid, missed_ticks, sample_count;
    float vdda_mv, raw_celsius;
} McuTemperatureTelemetry;
/**
 * @brief 读取温度采样是否有效且未超时。
 * @return 监督拥有的温度来源有效且未超时返回 true。
 * @note 只读标量查询，不访问 ADC、不修改采样状态。
 */
bool McuTemperature_IsFresh(void);

/**
 * @brief 按既有 ADC 零偏范围验证三相偏置。
 * @param configuration 所属活动配置，只读，偏置单位为 ADC counts。
 * @return 三个偏置均有限且位于原保护范围时返回 true。
 * @note 无 I/O，不改变阈值或配置。
 */
bool MotorCurrentOffsets_IsValid(const MotorConfiguration *configuration);
#endif
