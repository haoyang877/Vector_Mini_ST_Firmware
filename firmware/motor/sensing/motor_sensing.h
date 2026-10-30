#ifndef MOTOR_SENSING_H
#define MOTOR_SENSING_H

#include "motor_config.h"
#include "application_config.h"
#include "application_config.h"
#include "motor_measurement.h"
#include "motor_types.h"

/** @brief 上电零偏采样状态；Axis持有实例，采样模块通过显式对象独占推进。 */
typedef struct
{
    volatile uint32_t phase_sum_raw[3];
    volatile uint32_t sample_count;
    volatile bool complete, failed;
} CurrentOffsetCalibration;

/**
 * @brief 清除本次上电零偏资格及电流测量，不沿用存储零偏。
 * @param calibration Axis持有的校正状态，调用后清零。
 * @param configuration 活动配置，三相零偏设为NAN。
 * @param measurement 测量对象，三相电流设为NAN。
 * @note 仅上电关相初始化；不写硬件、参数修订或Flash。
 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              MotorMeasurement *measurement);

/**
 * @brief 读取并累计一拍原始三相采样，满窗口后冻结累加结果。
 * @param calibration Axis显式借用的校正状态；窗口已满、完成或失败后不继续累计。
 * @param motor_state 当前模式及故障所有者；越界或非关相时锁存既有零偏故障。
 * @return 本拍首次集满窗口返回true；其余情况返回false，不代表零偏已发布。
 * @note 仅新鲜20kHz采样拍调用；非阻塞，不启相、不重试、不更新修订或写Flash。
 *       不做一次性浮点计算或发布，由监督任务调用Finalize；发布前电流保持无效。
 */
bool MotorCurrentOffsets_Sample(CurrentOffsetCalibration *calibration,
                                MotorControlState *motor_state);

/**
 * @brief 在监督任务计算并原子发布已集满窗口的三相零偏。
 * @param calibration Axis持有的累加结果；仅成功发布后置complete。
 * @param configuration 活动配置，成功时写入三相零偏，单位counts。
 * @param motor_state 模式及故障所有者；失去关相条件或校验失败时锁存原故障。
 * @return 本次首次成功发布返回true，未就绪、已完成或失败返回false。
 * @note 仅2kHz监督调用，浮点计算可被快中断抢占；短临界区只执行最终校验与发布。
 *       调用者成功后推进参数修订；下一快拍换算电流，不修改原采样窗口或保护阈值。
 */
bool MotorCurrentOffsets_Finalize(CurrentOffsetCalibration *calibration,
                                  MotorConfiguration *configuration,
                                  MotorControlState *motor_state);

/**
 * @brief 将母线电压搬运到测量对象并更新低通滤波。
 * @param measurement 测量对象。
 * @param bus_voltage_v 本拍母线电压，V。
 * @note 快速环调用；不做过压/欠压判定，不上报故障。
 */
void MotorVoltage_Update(MotorMeasurement *measurement, float bus_voltage_v);

/**
 * @brief 将三相电流搬运到测量对象。
 * @param measurement 测量对象。
 * @param phase_current_a 三相电流，A，不保留指针。
 * @note 快速环调用；不做零偏或过流校验。
 */
void MotorCurrent_Update(MotorMeasurement *measurement, const float phase_current_a[3]);

/**
 * @brief  更新 MCU 温度换算与遥测。
 * @param  measurement 测量对象，写入温度字段。
 * @param  motor_state 电机故障所有者；采样失效时锁存故障。
 * @note 2 kHz 监督任务调用，不得在 20 kHz 快速环调用。
 */
void McuTemperature_Update(MotorMeasurement *measurement, MotorControlState *motor_state);

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
