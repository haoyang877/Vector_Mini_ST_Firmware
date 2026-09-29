#ifndef FOC_COGGING_CALIBRATION_H
#define FOC_COGGING_CALIBRATION_H
#include "motor_types.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "cogging_calibration.h"
#include "motor_task_result.h"
/* 齿槽适配边界：表、补偿状态与采样会话私有；调用方传入明确控制对象。 */
#include "cogging_compensation.h"

/**
 * @brief 检查标定启动前置条件：无边界轴、无故障、编码器在线且已标定、运动量在限。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 允许进入标定模式返回 true；否则返回 false。
 * @note 前台调用；不启动 PWM、不修改任何状态。
 */
bool FocCogging_CanStart(const MotorControlState *motor, const EncoderState *encoder);

/**
 * @brief 标定模式主任务：会话启动、安全闸门、2 kHz 抽点、位置保持与外环整形。
 * @param foc foc 状态。
 * @param motor 电机控制状态。
 * @param pi 标定专用位置 PI（复用速度环控制器对象）。
 * @param encoder 编码器状态。
 * @return 零矢量准备完成为 PREPARED，采集结束为 FINALIZE，失败为 FAILED，其余 RUNNING。
 * @note 20 kHz 调用；不启相、不停相、不选择下一任务；所有者按 epoch/guards
 *       授予功率许可，采集结束后关相并认领前台整理。失败只清理功能本地状态。
 */
MotorTaskResult
FocCogging_Task(FocState *foc, MotorControlState *motor, PiController *pi, EncoderState *encoder);

/**
 * @brief 中止标定会话并复位标定步骤；不操作 PWM、不切换模式。
 * @return 无返回值。
 * @note 可从故障处理路径调用。
 */
void FocCogging_Abort(void);

/**
 * @brief 前台消费补偿请求；成表与保存由 APP 维护所有者调度。
 * @param motor 电机状态，认领期间配置稳定。
 * @param encoder 编码器状态，认领期间安装校准稳定。
 * @return 无返回值。
 * @note 仅前台主循环调用；包含 CRC 与表拷贝等长耗时工作，禁止在中断调用。
 */
void FocCogging_Service(MotorControlState *motor, EncoderState *encoder);

/**
 * @brief 校验已发布表与当前编码器身份、CRC 是否一致。
 * @param motor 稳定的电机配置。
 * @param encoder 稳定的编码器安装校准。
 * @return 有效返回 true；否则返回 false。
 * @note 包含整表 CRC 复算，禁止在 20 kHz 电流环调用。
 */
bool FocCogging_TableValid(const MotorControlState *motor, const EncoderState *encoder);

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
 * @param motor 稳定的电机配置，提交时复核模式和故障。
 * @param encoder 稳定的编码器安装校准。
 * @return 请求被接受返回 true；校验拒绝返回 false 并置 rejected=1。
 * @note 仅前台调用；开启路径包含整表 CRC 复核。
 */
bool FocCogging_SetCompensation(bool enabled,
                                const MotorControlState *motor,
                                const EncoderState *encoder);

/**
 * @brief 20 kHz 补偿入口：采集硬件事实并调用补偿核心，返回叠加后的总 iq_a。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 叠加补偿并限幅后的总 iq_a 指令，单位 A；未启用时为用户指令的限幅结果。
 * @note 仅电流环上下文调用；不修改 motor->iq_target_a，指令所有权仍属用户。
 */
float FocCogging_Apply(const MotorControlState *motor, const EncoderState *encoder);

/** 台架保护值快照：lease_ticks 为剩余快速拍，trip 记录最近一次跳闸原因。 */
typedef struct
{
    uint32_t enabled, lease_ticks, trip;
    float speed_limit_rad_s;
} CoggingTorqueGuard;

/** 同一 ISR 时刻的力矩遥测值快照，所有字段由模块复制给调用方。 */
typedef struct
{
    uint32_t tick;
    float position_rad, velocity_rad_s, command_a, compensation_a, total_a;
    float feedback_a, vbus_v, blend;
} CoggingTorqueFrame;

/**
 * @brief 原子设置台架保护租约和测速上限，不修改控制目标或功率使能。
 * @param lease_ticks 租约快速拍数，20 kHz；零表示关闭保护。
 * @param speed_limit_rad_s 开启时必须有限且大于零，单位 rad/s；关闭时忽略。
 * @return 请求有效并已复制返回 true；无效上限返回 false，旧保护保持。
 * @note 前台调用；不清除既有跳闸及挂起 STOP，不持久化，不能代替生产断连保护。
 */
bool FocCogging_SetTorqueGuard(uint32_t lease_ticks, float speed_limit_rad_s);
/**
 * @brief 复制当前台架保护状态，不返回内部可写地址。
 * @param snapshot 调用方值快照空间。
 * @return 参数有效且复制成功返回 true；空指针返回 false。
 * @note 前台短临界区读取；不消耗跳闸，不授权启相。
 */
bool FocCogging_ReadTorqueGuard(CoggingTorqueGuard *snapshot);
/**
 * @brief 请求下一电流快环冻结一帧力矩遥测。
 * @return 空闲且请求已登记返回 true；已有请求或未消费帧返回 false。
 * @note 前台调用；不采样硬件、不修改控制目标，等待真实电流任务发布。
 */
bool FocCogging_RequestTorqueSample(void);
/**
 * @brief 复制并消费已经冻结的同拍力矩遥测。
 * @param sample 调用方帧空间，复制后独立于下一帧。
 * @return 已有完整帧且复制成功返回 true；空指针或尚未发布返回 false。
 * @note 前台短临界区复制36B；失败不消费帧，不返回内部可写地址。
 */
bool FocCogging_ReadTorqueSample(CoggingTorqueFrame *sample);

/**
 * @brief 台架保护入口：租约倒计时与测速上限判定；跳闸时置挂起标志、退出补偿并清零指令。
 * @param motor 电机控制状态；跳闸时清零 id_target_a/iq_target_a。
 * @param encoder 编码器状态，只读。
 * @return 未跳闸返回 true；跳闸返回 false（停相与模式回退由调度层转为结果协议后，
 *         由运行状态机执行）。
 * @note 仅电流环上下文调用；默认关闭且不持久化，不是量产主机断连保护。
 *       trip=1 租约到期，trip=2 测速超限或无效；电流包络由 motor_state.application.current_limit_a 保证。
 *       跳闸标志经 FocCogging_TakeTorqueTrip 由模式调度消费。
 */
bool FocCogging_TorqueGuard(MotorControlState *motor, const EncoderState *encoder);

/**
 * @brief 读取并清除台架保护跳闸挂起标志。
 * @return 有挂起跳闸返回 true（并清除）；否则返回 false。
 * @note 仅模式调度（20 kHz 上下文）调用；标志由 FocCogging_TorqueGuard 置位。
 */
bool FocCogging_TakeTorqueTrip(void);

/**
 * @brief 力矩遥测：主机请求时冻结同一 ISR 时刻的指令、补偿与反馈。
 * @param foc foc 状态，只读。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 无返回值。
 * @note 仅电流环上下文调用；未请求时只递增计数，不做拷贝。
 */
void FocCogging_TorqueObserve(const FocState *foc,
                              const MotorControlState *motor,
                              const EncoderState *encoder);

/**
 * @brief 清空所属齿槽表。
 * @note 仅启动或停机且无前台认领时调用。
 */
void FocCogging_ResetMap(void);
/**
 * @brief 导入已验证齿槽记录。
 * @param record 调用方独占的有效记录，只复制值。
 * @note 仅启动调用。
 */
void FocCogging_ImportMap(const CoggingMapRecord *record);
/**
 * @brief 按当前校准身份导出有效表。
 * @param record 接收记录，失效时清零。
 * @param motor 稳定的电机配置。
 * @param encoder 稳定的编码器配置。
 * @note 停机前台调用，包含 CRC，不返回内部指针。
 */
void FocCogging_ExportMap(CoggingMapRecord *record,
                          const MotorControlState *motor,
                          const EncoderState *encoder);
/**
 * @brief 查询自然完成后的待整理表。
 * @return 需要前台认领时返回 true。
 */
bool FocCogging_NeedsFinalize(void);
/**
 * @brief 确认标定与前台会话已归还。
 * @return 无活动标定、待整理或前台借用时返回 true。
 */
bool FocCogging_IsReleased(void);
/**
 * @brief 认领已结束的齿槽采样会话。
 * @return 认领成功返回 true。
 * @note APP 维护 owner 在短临界区调用；必须由 ReleaseFinalize 归还。
 */
bool FocCogging_BeginFinalize(void);
/**
 * @brief 在调用方候选空间构建完整表。
 * @param record 前台独占候选空间。
 * @param motor 认领期间稳定的配置。
 * @param encoder 认领期间稳定的安装校准。
 * @return 表构建成功返回 true。
 * @note 仅前台调用，CRC 保持中断开启；取消不改写正在读取的样本。
 */
bool FocCogging_BuildCandidate(CoggingMapRecord *record,
                               const MotorControlState *motor,
                               const EncoderState *encoder);
/**
 * @brief 发布已构建的完整候选表。
 * @param record 完整候选，仅复制值。
 * @return 会话未取消且仍由前台认领时返回 true。
 * @note APP 在同一临界区复核原始身份、无故障与关相后调用。
 */
bool FocCogging_PublishCandidate(const CoggingMapRecord *record);
/**
 * @brief 归还前台会话并记录发布结果。
 * @param published 候选是否已提交到 RAM，不表示写入 Flash。
 * @note 候选已释放后在 owner 临界区调用；取消不能撤回已发布 RAM。
 */
void FocCogging_ReleaseFinalize(bool published);
/* 显式 SWD 补偿请求邮箱：1 请求开启，2 请求关闭，0 空闲；CPU 消费，不直接使能功率。 */
/**
 * @brief 提交补偿开关值，由稳定配置借用下的前台服务校验并应用。
 * @param enabled true 请求开启，false 请求平滑关闭；最新请求覆盖未消费请求。
 * @note 前台短临界区只登记请求，不做CRC、不启相、不表示校验已经通过。
 */
void FocCogging_RequestCompensation(bool enabled);
#endif
