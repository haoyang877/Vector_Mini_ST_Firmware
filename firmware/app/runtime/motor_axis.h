#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H
#include <stdint.h>

/**
 * @brief 装载参数、初始化并绑定单轴运行对象。
 * @note 仅启动组合层在中断启用前调用一次，不自动使能。
 */
void MotorAxis_Init(void);

/**
 * @brief 更新采样，互斥执行标定或正常控制并发布本拍结果。
 * @note 20kHz ADC中断调用；编码器传输与计算重叠，所有分支统一发布结果。
 */
void MotorAxis_FastTick(void);

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
 * @brief 读取启动配置的设备节点地址。
 * @return 节点地址。
 * @note 仅初始化后使用；不公开配置指针。
 */
uint8_t MotorAxis_NodeId(void);
#endif
