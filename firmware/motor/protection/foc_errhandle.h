#ifndef FOC_ERRHANDLE_H
#define FOC_ERRHANDLE_H
#include "data_type.h"
#include "foc_algorithm.h"
#include "angle_feedback.h"

typedef struct
{
    uint16_t overcurrent_ticks;
} MotorFaultRecovery;
/**
 * @brief 向所属电机对象报告故障，保留首个控制故障。
 * @param motor 调用方拥有的电机状态；NULL 不产生写入。
 * @param error 非零故障编号；No_Error 不清除锁存。
 * @note 控制上下文调用；不启动、不清错，调用方随后执行快速关断。
 */
void MotorFault_Report(MotorControl_TypeDef *motor, ErrorNow_TypeDef error);
/**
 * @brief 推进所属故障恢复的驻留证据。
 * @param recovery APP 独占的恢复计数。
 * @param foc 本次三相电流测量，A。
 * @param error 当前锁存故障。
 * @note 2 kHz 监督调用，保留原计数和阈值。
 */
void MotorFault_UpdateRecovery(MotorFaultRecovery *recovery,
                               const FOC_TypeDef *foc,
                               ErrorNow_TypeDef error);
/**
 * @brief 按源恢复矩阵确认显式 CLEAR 的条件。
 * @param recovery 所属驻留证据，只读。
 * @param motor 有效配置与偏置。
 * @param foc 电压 V、温度 °C 的稳定测量。
 * @param encoder 所属反馈健康。
 * @param error 需要确认的原始故障编号。
 * @param samples_fresh 本次监督已获得新的有效快环测量。
 * @param maintenance_released APP 已确认维护借用和工作区释放。
 * @param temperature_fresh 温度来源有效且未超时。
 * @return 源已恢复或终止型失败已释放且可显式重试时返回 true。
 * @note 无 I/O，不清错；模式准入仍独立检查全部校准标志。
 */
bool MotorFault_SourceRecovered(const MotorFaultRecovery *recovery,
                                const MotorControl_TypeDef *motor,
                                const FOC_TypeDef *foc,
                                const Encoder_TypeDef *encoder,
                                ErrorNow_TypeDef error,
                                bool samples_fresh,
                                bool maintenance_released,
                                bool temperature_fresh);
#endif
