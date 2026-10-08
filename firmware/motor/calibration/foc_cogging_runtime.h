#ifndef FOC_COGGING_RUNTIME_H
#define FOC_COGGING_RUNTIME_H

#include "motor_types.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "cogging_compensation.h"

/* 运行适配边界：只拥有活动表、补偿与力矩 guard；不依赖标定过程状态。 */

/**
 * @brief 前台消费补偿请求；成表与保存由 APP 维护所有者调度。
 * @param motor 电机状态，认领期间配置稳定。
 * @param encoder 编码器状态，认领期间安装校准稳定。
 * @return 无返回值。
 * @note 仅前台主循环调用；准入包含整表 CRC，禁止在中断调用。
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
 * @note 仅启动导入或 APP 已验证关相/认领的完整候选发布时调用，不操作功率或 Flash。
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
/* 显式 SWD 补偿请求邮箱：1 请求开启，2 请求关闭，0 空闲；CPU 消费，不直接使能功率。 */
/**
 * @brief 提交补偿开关值，由稳定配置借用下的前台服务校验并应用。
 * @param enabled true 请求开启，false 请求平滑关闭；最新请求覆盖未消费请求。
 * @note 前台短临界区只登记请求，不做CRC、不启相、不表示校验已经通过。
 */
void FocCogging_RequestCompensation(bool enabled);
#endif
