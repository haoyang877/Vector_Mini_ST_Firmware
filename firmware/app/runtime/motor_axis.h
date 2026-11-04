#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H
#include <stdint.h>
#include "motor_state_types.h"

/** @brief 同一快环中的只读波形，按三相电流A、电角度rad、反馈/目标转速rad/s排列。 */
typedef struct
{
    float phase_a_a, phase_b_a, phase_c_a;
    float electrical_angle_rad;
    float speed_rad_s, speed_target_rad_s;
} MotorAxisTrace;

/**
 * @brief 读取本拍电流、编码器角度和机械转速波形，不改变控制状态。
 * @return 六路float快照，电流A、角度rad、机械转速rad/s。
 * @note 仅由ADC快环在采样和控制完成后调用，避免与采样并发。
 */
MotorAxisTrace MotorAxis_ReadTrace(void);

/**
 * @brief 装载参数、初始化并绑定单轴运行对象。
 * @note 仅启动组合层在中断启用前调用一次，不自动使能。
 */
void MotorAxis_Init(void);

/**
 * @brief 读取快环电压、三相/母线电流及编码器，完成零偏校正和反馈估计。
 * @note ADC确认完成后调用；保持编码器传输与计算重叠，必须先于MotorAxis_ControlFast。
 *       零偏未完成时读取三相原码做校正，完成后读取已换算电流。
 */
void MotorAxis_SampleFast(void);

/**
 * @brief 使用本拍采样结果，互斥执行标定或正常控制并发布故障与输出。
 * @note 每拍MotorAxis_SampleFast之后调用一次；不重复读取ADC或发起编码器事务。
 */
void MotorAxis_ControlFast(void);

/**
 * @brief 执行慢估计、外环和生命周期监督。
 * @note 仅2kHz监督上下文调用。
 */
void MotorAxis_SupervisorTick(void);

/**
 * @brief 在关相前台发布已完成的编码器线性化候选。
 * @note 仅主循环调用；发布和原会话复核共用短临界区，不写Flash。
 */
void MotorAxis_ProcessCalibrationPublication(void);

/**
 * @brief 停机前台同步保存当前配置及编码器标定到Flash。
 * @return ACCEPTED表示记录持久化及回读完成；BUSY表示控制未停止或标定未释放，
 *         DENIED表示配置或标定无效，FAULT表示已有故障，PERSIST_FAILED表示存储失败。
 * @note 仅串行前台调用；零偏完成且空闲关相后复用启动记录暂存区，不自动重试。
 *       保存完整schema14记录，BSP独占采样暂停与中断恢复，不自动启相。
 */
MotorControlResult MotorAxis_SaveCalibration(void);

/**
 * @brief 读取启动配置的设备节点地址。
 * @return 节点地址。
 * @note 仅初始化后使用；不公开配置指针。
 */
uint8_t MotorAxis_NodeId(void);
#endif
