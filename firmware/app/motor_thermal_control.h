/** @file
 * @brief 单电机热保护组合接口；20 kHz快上下文独占所有状态写入。
 */
#ifndef MOTOR_THERMAL_CONTROL_H
#define MOTOR_THERMAL_CONTROL_H
#include "motor_thermal.h"

/** @brief 调试只读状态；不是Flash或通信结构。 */
typedef struct
{
    MotorThermalState model;
    uint32_t last_update_ms;
    uint32_t updates;
    bool initialized;
    bool parameters_confirmed;
    bool watchdog_fault_pending; /* 只读快照补充项，监督事件尚待快环消费。 */
} MotorThermalDiagnostics;

/**
 * @brief 初始化一次；首次上电/软件复位热量为0，重复调用保留运行热量。
 * @param now_ms 单调毫秒时钟，允许uint32回绕。
 * @note 输出关闭时初始化；正常STOP及清故障不得重新初始化，热历史不跨MCU复位保存。
 */
void MotorThermalControl_Init(uint32_t now_ms);

/**
 * @brief 快拍累计电流平方，每20拍更新热状态；缺时钟窗口锁定模型。
 * @param ia_a 实测A相电流，A。
 * @param ib_a 实测B相电流，A。
 * @param ic_a 实测C相电流，A。
 * @param outputs_disabled 六路功率输出是否已关闭且为高阻。
 * @param now_ms 单调毫秒时钟。
 * @note 仅快上下文调用；ADC序列必须完整，不修改电机参数或硬件。
 */
void MotorThermalControl_Sample(
    float ia_a, float ib_a, float ic_a, bool outputs_disabled, uint32_t now_ms);

/**
 * @brief 只读查询距下一次热更新还需的有效三相实采样数，供交接入场错峰。
 * @return 有效模型返回剩余样本数；未初始化、模型无效或配置/时钟事件待处理时返回0。
 * @note 仅快上下文且在本拍 Sample 后调用；刚更新完返回完整窗口大小。
 *       不重置、延后或改变热采样、热更新和保护动作，不能据此跳过 Sample。
 */
uint32_t MotorThermalControl_SamplesUntilUpdate(void);

/**
 * @brief 监督快环缺样；失去采样时发布停机事件，不在监督上下文改热状态。
 * @param now_ms 单调毫秒时钟。
 * @param outputs_disabled 六路输出已高阻关闭时允许维护暂停，冻结热历史。
 * @return 活跃输出仍有新采样时为true；false时调用方须立即关相。
 * @note 1 kHz监督上下文调用；事件由快上下文消费，不能通过清故障绕过。
 */
bool MotorThermalControl_CheckWatchdog(uint32_t now_ms, bool outputs_disabled);

/**
 * @brief 查询模式的热准入，涵盖标定、电压模式和制动。
 * @param mode 当前模式编号。
 * @return 允许该模式时为true；只是热准入，不替代其他保护。
 * @note 只读；通信上下文可查询，快上下文仍需再次执行Guard。
 */
bool MotorThermalControl_CanRunMode(uint32_t mode);

/**
 * @brief 快上下文执行最终热准入，热态禁止的标定/开环会锁存退出。
 * @param mode 本拍将执行的模式。
 * @return 允许继续时为true；false时须清参考、转mode 0并在同一快拍关相。
 */
bool MotorThermalControl_Guard(uint32_t mode);

/**
 * @brief 读取实际可用电流；未放行配置或热锁存时返回0。
 * @param configured_a 既有保存电流上限，A。
 * @return 不超过配置与热边界的电流，A。
 */
float MotorThermalControl_CurrentLimit(float configured_a);

/**
 * @brief 查询有效模型是否已冷却至配置恢复门限以内且参数已放行。
 * @return 具备热清故障资格时为true。
 * @note 查询不清除热量或故障，也不使能输出。
 */
bool MotorThermalControl_CanClearFault(void);

/**
 * @brief 快上下文按资格清热锁存；热量和采样窗口保留。
 * @return 已满足冷却及参数条件时为true；仍需新的运动命令。
 */
bool MotorThermalControl_ClearFault(void);

/**
 * @brief 查询配置放行标志，供区分参数未放行和热保护动作。
 * @return 软件放行开关为1时为true，不证明已完成实机热标定。
 */
bool MotorThermalControl_ParametersConfirmed(void);

/**
 * @brief 复制只读热诊断快照，不暴露可写模型状态。
 * @param output 调用方持有的输出快照。
 * @return 已初始化且输出指针有效时为true。
 * @note 仅快上下文或MCU已暂停时读取，不保证跨ISR原子采集。
 */
bool MotorThermalControl_GetDiagnostics(MotorThermalDiagnostics *output);

/**
 * @brief 启动前加载热配置；空指针选择编译默认值。
 * @param values 按固定参数顺序排列的七个值，热门限为比例；NULL选择默认值。
 * @return 尚未初始化且配置有效时为true；运行中拒绝覆盖。
 * @note 不初始化模型、不清热量；仅启动参数加载调用。
 */
bool MotorThermalControl_LoadParameters(const float *values);

/**
 * @brief 读取已接受的参数；待消费配置也可查询和保存。
 * @param parameter 固定参数索引。
 * @param value 输出A、s或热比例；通信层负责百分数转换。
 * @return 索引和指针有效时为true。
 * @note 只读，不返回可写状态；CAN配置应在停机时逐项读回核验。
 */
bool MotorThermalControl_GetParameter(MotorThermalParameter parameter, float *value);

/**
 * @brief 校验并提交单个热参数，下一关相快拍应用完整配置。
 * @param parameter 固定参数索引。
 * @param value 候选值，单位A、s或热比例。
 * @param mode 请求时电机模式，必须为mode 0。
 * @param outputs_disabled 三相主/互补输出均已高阻关闭。
 * @return 完整配置有效且单槽空闲时为true；拒绝时不修改任何状态。
 * @note 通信上下文预计算指数系数；不清热历史、故障或采样窗口。
 */
bool MotorThermalControl_RequestParameter(MotorThermalParameter parameter,
                                          float value,
                                          uint32_t mode,
                                          bool outputs_disabled);

#endif
