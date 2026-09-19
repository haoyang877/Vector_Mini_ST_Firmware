#ifndef FOC_COGGING_CALIBRATION_H
#define FOC_COGGING_CALIBRATION_H
#include "data_type.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "cogging_calibration.h"
/* 齿槽功能对外公共接口：本文件承载标定适配、运行补偿适配与台架保护；
 * 补偿纯核心由以下模块提供，并在此汇总导出，使既有调用方
 * （foc_run.c、interface_can.c、main.c）无需改动。 */
#include "cogging_compensation.h"

extern CoggingCalibration CoggingCalib;

/**
 * @brief 检查标定启动前置条件：无边界轴、无故障、编码器在线且已标定、运动量在限。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 允许进入标定模式返回 true；否则返回 false。
 * @note 前台调用；不启动 PWM、不修改任何状态。
 */
bool FocCogging_CanStart(const MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);

/**
 * @brief 标定模式主任务：会话启动、安全闸门、2 kHz 抽点、位置保持与外环整形。
 * @param foc FOC 状态。
 * @param motor 电机控制状态。
 * @param pi 标定专用位置 PI（复用速度环控制器对象）。
 * @param encoder 编码器状态。
 * @return 无返回值。
 * @note 由 20 kHz 电流环上下文调用；失败时关闭 PWM 并切到 Motor_Disable。
 */
void FocCogging_Task(FOC_TypeDef *foc,
                     MotorControl_TypeDef *motor,
                     PI_Controller_TypeDef *pi,
                     Encoder_TypeDef *encoder);

/**
 * @brief 中止标定会话并复位标定步骤；不操作 PWM、不切换模式。
 * @return 无返回值。
 * @note 可从故障处理路径调用。
 */
void FocCogging_Abort(void);

/**
 * @brief 前台服务：协调补偿请求与运行模式，并在标定完成后接管参数保存流程。
 * @return 无返回值。
 * @note 仅前台主循环调用；包含 CRC 与表拷贝等长耗时工作，禁止在中断调用。
 */
void FocCogging_Service(void);

/**
 * @brief 校验已发布表与当前编码器身份、CRC 是否一致。
 * @return 有效返回 true；否则返回 false。
 * @note 包含整表 CRC 复算，禁止在 20 kHz 电流环调用。
 */
bool FocCogging_TableValid(void);

/**
 * @brief 读取标定对外状态，保存阶段统一报告为 COGGING_SAVING。
 * @return 当前标定状态。
 * @note 前台与 CAN 查询上下文只读安全。
 */
CoggingState FocCogging_GetState(void);

/**
 * @brief 通知标定保存结果；失败时把状态标记为 COGGING_SAVE_FAILED。
 * @param success 参数保存是否成功。
 * @return 无返回值。
 * @note 仅前台调用；仅在保存流程挂起时生效。
 */
void FocCogging_SaveResult(bool success);

/**
 * @brief 请求开启或平滑关闭运行补偿；开启需前台完整校验并快照硬件身份。
 * @param enabled true 请求开启；false 平滑关闭并保留 200 ms 渐出。
 * @return 请求被接受返回 true；校验拒绝返回 false 并置 rejected=1。
 * @note 仅前台调用；开启路径包含整表 CRC 复核。
 */
bool FocCogging_SetCompensation(bool enabled);

/**
 * @brief 20 kHz 补偿入口：采集硬件事实并调用补偿核心，返回叠加后的总 Iq。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 叠加补偿并限幅后的总 Iq 指令，单位 A；未启用时为用户指令的限幅结果。
 * @note 仅电流环上下文调用；不修改 motor->iqRef，指令所有权仍属用户。
 */
float FocCogging_Apply(const MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);

/** 台架保护状态：lease_ticks 由测试主机续租，trip 记录最近一次跳闸原因。 */
typedef struct
{
    uint32_t enabled, lease_ticks, trip;
    float speed_limit_rad_s;
} CoggingTorqueGuard;

extern volatile CoggingTorqueGuard TorqueGuard;

/** 同一 ISR 时刻的力矩遥测帧；主机经 TorqueTelemetryState 请求后读取。 */
typedef struct
{
    uint32_t tick;
    float position_rad, velocity_rad_s, command_a, compensation_a, total_a;
    float feedback_a, vbus_v, blend;
} CoggingTorqueFrame;

extern volatile uint32_t TorqueTelemetryState;
extern volatile CoggingTorqueFrame TorqueTelemetry;

/**
 * @brief 台架保护入口：租约倒计时与测速上限判定；跳闸时停 PWM、退出补偿并回 mode 0。
 * @param motor 电机控制状态；跳闸时清零 idRef/iqRef 并切到 Motor_Disable。
 * @param encoder 编码器状态，只读。
 * @return 未跳闸返回 true；跳闸并已停机返回 false。
 * @note 仅电流环上下文调用；默认关闭且不持久化，不是量产主机断连保护。
 *       trip=1 租约到期，trip=2 测速超限或无效；电流包络由 MotorControl.current_limit 保证。
 */
bool FocCogging_TorqueGuard(MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);

/**
 * @brief 力矩遥测：主机请求时冻结同一 ISR 时刻的指令、补偿与反馈。
 * @param foc FOC 状态，只读。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 无返回值。
 * @note 仅电流环上下文调用；未请求时只递增计数，不做拷贝。
 */
void FocCogging_TorqueObserve(const FOC_TypeDef *foc,
                              const MotorControl_TypeDef *motor,
                              const Encoder_TypeDef *encoder);
#endif
