#ifndef FOC_SENSING_H
#define FOC_SENSING_H

#include "motor_config.h"
#include "control_config.h"
#include "current_sense_limits.h"
#include "foc_algorithm.h"
#include "motor_types.h"
#include "bsp_adc.h"

/* 温度失效门限：监督 tick 数（2 kHz，0.5 ms/tick），保持 MCU_TEMPERATURE_TIMEOUT_MS 契约。 */
#define MCU_TEMPERATURE_TIMEOUT_TICKS (MCU_TEMPERATURE_TIMEOUT_MS * SUPERVISOR_TICKS_PER_MS)

/* 沿用已退役上电零点任务的1秒采样窗口，不恢复其其它标定模式。 */
#define MOTOR_CURRENT_OFFSET_SAMPLE_TIME_MS 1000U

/* JEOS每帧推进一次；20kHz下累计1秒，最大四次和的总和仍可由uint32_t容纳。 */
#define CURRENT_OFFSET_SAMPLE_COUNT (FOC_FREQ * MOTOR_CURRENT_OFFSET_SAMPLE_TIME_MS / 1000U)

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
 * @param foc 采样运行对象，三相电流设为NAN。
 * @note 仅上电关相初始化；不写硬件、参数修订或Flash。
 */
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              FocState *foc);

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
 * @brief  更新母线电压采样与滤波值。
 * @param  foc foc 状态指针，写入 bus_voltage_v 相关字段。
 * @param  motor_state 电机控制状态指针，写入电压故障。
 * @param output_expected 所有者确认需要保护输出；关闭确认前保持 true。
 * @param sample 本拍只读快帧；NULL表示采样失败，电压按原无效路径处理。
 * @note 快速环周期调用；禁用状态下采样用于复位计数器。
 */
void Vbus_Update(FocState *foc,
                 MotorControlState *motor_state,
                 bool output_expected,
                 const BspAdcFastSample *sample);

/**
 * @brief  完成相电流采样换算与偏置补偿。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针，写入 phase_a_current_a/phase_b_current_a/phase_c_current_a。
 * @param  motor_state 电机控制状态指针。
 * @param sample 与电压处理共用的本拍只读快帧；NULL按原采样失败路径处理。
 * @note 快速环周期调用；换算系数来自平台感测契约。
 */
void Current_Cal(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor_state,
                 const BspAdcFastSample *sample);

/**
 * @brief  更新 MCU 温度换算与遥测。
 * @param  foc foc 状态指针，写入温度字段。
 * @param  motor_state 电机故障所有者；采样失效时锁存故障。
 * @note 2 kHz 监督任务调用，不得在 20 kHz 快速环调用。
 */
void Temperature_Update(FocState *foc, MotorControlState *motor_state);

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
bool MotorTemperature_IsFresh(void);

/**
 * @brief 按既有 ADC 零偏范围验证三相偏置。
 * @param configuration 所属活动配置，只读，偏置单位为 ADC counts。
 * @return 三个偏置均有限且位于原保护范围时返回 true。
 * @note 无 I/O，不改变阈值或配置。
 */
bool MotorCurrentOffsets_Valid(const MotorConfiguration *configuration);
#endif
