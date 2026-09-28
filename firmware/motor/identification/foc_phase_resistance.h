#ifndef __FOC_PHASE_RESISTANCE_H__
#define __FOC_PHASE_RESISTANCE_H__

#include "foc_algorithm.h"
#include "phase_resistance.h"
#include "motor_task_result.h"

/**
 * @file foc_phase_resistance.h
 * @brief 相电阻纯核心与电流环之间的适配边界。
 *
 * 功能说明：本文件是相电阻算法和现有 FOC 控制之间的唯一适配层。
 * - 从每电机参数 profile 读取双电流测试配置，并受 MotorControl 电流上限保护；
 * - 将核心模块给出的 Id/Iq/电角度命令交给 FOC_Current() 执行；
 * - 将本周期电流与上一 PWM 周期实际输出的调制电压配对并回传核心；
 * - 在间隔、完成或失败时复位电流环并释放 PWM；
 * - 将六方向双电流拟合后的 A/B/C 相阻和不平衡率写回 MotorControl。
 * - 本模块不访问编码器或位置环；不同电机通过参数 profile 调整测试电流和时序。
 *
 * 调度层每个 FOC 周期调用 PhaseResistanceMode_Run()；函数返回运行、完成
 * 或具体失败状态。APP 模式分发将其转换为结果，运行所有者提交模式与故障。
 * 本模块不处理 RTT 或任何通信。
 */

typedef enum
{
    PHASE_RESISTANCE_MODE_RUNNING = 0,
    PHASE_RESISTANCE_MODE_DONE,
    PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT,
    PHASE_RESISTANCE_MODE_INVALID_RESULT,
    PHASE_RESISTANCE_MODE_UNDER_VOLTAGE,
    PHASE_RESISTANCE_MODE_OVER_VOLTAGE
} PhaseResistanceModeStatus_TypeDef;

/**
 * 与辨识核心输入一致的电流电压值；适配层拥有状态，查询只复制诊断快照。
 */
typedef struct
{
    float electrical_angle;
    float id_ref;
    float id;
    float iq;
    float vd;
    float vq;
    float current_magnitude;
    float parallel_voltage;
    float vbus;
} PhaseResistanceModeTelemetry_TypeDef;

/**
 * @brief 执行一次相电阻辨识周期。
 * @param foc 电流环状态，写入本次调制命令。
 * @param motor 电机状态，保存已验证辨识结果。
 * @return 运行、完成或具体失败原因。
 * @note 20 kHz 调度；模式提交由 APP 所有者负责。
 */
MotorTaskResult PhaseResistanceMode_Run(FOC_TypeDef *foc, MotorControl_TypeDef *motor);

/**
 * @brief 复制辨识使用的最近电流电压样本。
 * @param telemetry 调用方快照空间，不保留指针。
 * @return 任务活动且样本可用时返回 true。
 */
bool PhaseResistanceMode_GetTelemetry(PhaseResistanceModeTelemetry_TypeDef *telemetry);

/**
 * @brief 停止辨识输出并归还未完成工作状态。
 * @param foc 待复位的电流环状态。
 * @param motor 待清零的电流命令所有者。
 * @note APP 取消路径调用；停相失败仍由 APP 守卫判定。
 */
void PhaseResistanceMode_Cancel(FOC_TypeDef *foc, MotorControl_TypeDef *motor);

/**
 * @brief 确认辨识任务没有持有活动工作状态。
 * @return 本地任务释放后返回 true。
 * @note 运行所有者取消后查询，不执行 I/O。
 */
bool PhaseResistanceMode_IsReleased(void);
#endif
