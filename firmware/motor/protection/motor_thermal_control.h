#ifndef MOTOR_THERMAL_CONTROL_H
#define MOTOR_THERMAL_CONTROL_H

#include "motor_thermal_config.h"
#include "foc_algorithm.h"

/* APP_ENABLED_WINDING_THERMAL_MODEL默认开启。限时AT32台架显式关闭时不累计热量、
 * 不执行热降额/热更新看门狗，CurrentLimit仍返回有效配置限流，Vq瞬时限流仍执行。
 * 关闭时GetDiagnostics返回false且不写输出，不能把未运行的热模型当作健康遥测。 */

/** @brief 快速环独占的热历史与更新证据；不属于参数或通信布局。 */
typedef struct
{
    MotorThermalState model;
    uint32_t last_update_ms;
    uint32_t updates;
    bool initialized;
} MotorThermalDiagnostics;

/**
 * @brief 上电初始化热模型一次，重复调用保留已有热历史。
 * @param config 所属电机的热配置，必须有效。
 * @param now_ms 单调毫秒时钟，允许自然回绕。
 * @return 模型已初始化返回true；配置无效时返回false。
 * @note 仅上电关相初始化；MCU复位不保留热历史。
 */
bool MotorThermalControl_Init(const MotorWindingThermalConfig *config, uint32_t now_ms);
/**
 * @brief 关相时同步应用有效配置，保留并保守换算热历史和锁存。
 * @param config 完整候选配置，单位A、s或热比例。
 * @param outputs_disabled 已取得三相功率实际关闭的证据。
 * @return 重配成功返回true；失败时模型及旧配置不变。
 * @note 参数所有者在既有停机临界区调用，成功后再发布活动参数；不写Flash。
 */
bool MotorThermalControl_Configure(const MotorWindingThermalConfig *config, bool outputs_disabled);
/**
 * @brief 累计本拍实际三相平方，每20拍更新热模型，失效输入禁止出力。
 * @param ia_a 本拍A相实测电流，A。
 * @param ib_a 本拍B相实测电流，A。
 * @param ic_a 本拍C相实测电流，A。
 * @param outputs_disabled 本拍功率输出实际已关闭。
 * @param now_ms 单调毫秒时钟。
 * @note 仅20kHz快环在上电零偏校正完成后调用；故障后继续更新以估计冷却。
 */
void MotorThermalControl_Sample(
    float ia_a, float ib_a, float ic_a, bool outputs_disabled, uint32_t now_ms);
/**
 * @brief 发布热限流快照并按本拍功率策略检查绕组保护，不操作硬件。
 * @param configuration 只读活动配置，电流上限单位A。
 * @param foc 本拍三相实测电流，单位A。
 * @param motor 所属运行对象；更新热快照，失败仅报告原绕组热故障。
 * @param samples_ready 本次上电电流零偏已完成；false时只发布快照。
 * @param outputs_disabled 本拍实际关相事实，与本拍采样使用相同值。
 * @param output_expected 生命周期要求出力；仅在模型允许运行时查询并传入。
 * @param derating_allowed 当前控制策略允许热降额；开环电压和功率标定为false。
 * @note 20kHz快环在Sample之后调用；不清历史、不改配置、不启停功率。
 */
void MotorThermalControl_UpdateProtection(const MotorConfiguration *configuration,
                                          const FocState *foc,
                                          MotorControlState *motor,
                                          bool samples_ready,
                                          bool outputs_disabled,
                                          bool output_expected,
                                          bool derating_allowed);
/**
 * @brief 监督功率开启时的热采样窗口，丢失更新时发布不可自动恢复的失效事件。
 * @param now_ms 单调毫秒时钟。
 * @param outputs_disabled 当前功率输出实际已关闭。
 * @return 调度证据有效或功率已关闭返回true；超时返回false。
 * @note 2kHz监督调用；调用者必须沿既有故障快车道关相，不得自动恢复试验。
 */
bool MotorThermalControl_CheckWatchdog(uint32_t now_ms, bool outputs_disabled);
/**
 * @brief 核对热模型是否允许启动指定功率策略。
 * @param derating_allowed 电流闭环策略可为true，开环电压/标定必须为false。
 * @return 模型有效、未锁存且策略可用返回true。
 * @note 只读；未初始化返回false，不给离线夹具或真实启动默认放行。
 */
bool MotorThermalControl_CanRun(bool derating_allowed);
/**
 * @brief 检查运行热边界；标定或开环遇到降额时锁存热故障。
 * @param derating_allowed 当前策略是否可以执行动态电流降额。
 * @return 允许继续执行返回true，否则必须走既有故障关断。
 * @note 快上下文调用；不直接操作硬件或改变主状态机。
 */
bool MotorThermalControl_Guard(bool derating_allowed);
/**
 * @brief 计算配置限流与热可用限流的较小值。
 * @param configured_a 应用配置电流上限，A。
 * @return 可用向量幅值A；未初始化、失效或热锁存返回0。
 */
float MotorThermalControl_CurrentLimit(float configured_a);
/**
 * @brief 核对独立热锁存是否已满足显式清除条件。
 * @return 无热锁存或有效模型已冷却至恢复门限返回true。
 * @note 不修改热历史；采样失效必须排障重启，不能通过CLEAR伪造有效样本。
 */
bool MotorThermalControl_CanClearFault(void);
/**
 * @brief 显式清除已恢复的独立热锁存，不清热历史或采样窗。
 * @return 已无热锁存或成功清除返回true。
 * @note 主状态机仅在CLEAR已被接受时、既有临界区内调用。
 */
bool MotorThermalControl_ClearFault(void);
/**
 * @brief 复制所属热模型的只读调试快照。
 * @param output 接收快照，非空。
 * @return 已初始化且输出有效返回true。
 * @note 调用者需要一致快照时须持既有临界区，不允许写回快照清热量。
 */
bool MotorThermalControl_GetDiagnostics(MotorThermalDiagnostics *output);

#endif
