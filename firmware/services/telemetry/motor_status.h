#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H
#include <stdbool.h>
#include <stdint.h>

/** 快速周期末由电机所有者采集的 SI 单位快照。
 * target 字段来自外部命令，planned 字段来自控制器规划输出；current_reference 和
 * current_feedback 表示 q 轴电流而非母线电流或相电流 RMS，bus_current 表示滤波后的
 * 直流母线电流估计值，单位 A。 */
typedef struct
{
    uint16_t fault, mode;
    float position_target, position_feedback;
    float speed_target, speed_feedback;
    float current_reference, current_feedback;
    float position_planned, speed_planned;
    float temperature, bus_voltage;
    float bus_current;
} MotorStatus;

/**
 * @brief 请求快速环发布一个新状态样本；重复请求会合并为一次。
 * @note 仅供单个前台消费者调用，不阻塞、不进行编码或总线访问。
 */
void MotorStatus_Request(void);
/**
 * @brief 查询前台是否正在等待状态样本。
 * @return 存在未完成请求返回 true，否则返回 false。
 * @note 仅允许快速环生产者调用。
 */
bool MotorStatus_IsRequested(void);
/**
 * @brief 在存在请求时发布一个完整状态快照。
 * @param sample 快速环持有的只读样本；函数返回后不保存该指针。
 * @note 仅允许快速环生产者调用，不进行编码、动态分配或硬件 I/O。
 */
void MotorStatus_Publish(const MotorStatus *sample);
/**
 * @brief 由前台消费最近完成的状态快照。
 * @param sample 调用方提供的输出对象；无可用样本时保持内容不变。
 * @return 成功取得样本返回 true；尚未发布完成时返回 false，调用方应在后续主循环重试。
 * @note 成功读取会消费当前样本，本接口只支持单消费者。
 */
bool MotorStatus_Take(MotorStatus *sample);
#endif
