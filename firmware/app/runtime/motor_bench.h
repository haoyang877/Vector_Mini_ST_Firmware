#ifndef MOTOR_BENCH_H
#define MOTOR_BENCH_H

#include "encoder_feedback.h"
#include "foc.h"
#include "flux_observer.h"
#include "yg_protocol_endpoint.h"

/** @brief 仅测试镜像的命令；正式APP不注册此编号。 */
#define MOTOR_BENCH_MESSAGE_TYPE 0x7F00U

/**
 * @brief 绑定测试使用的现有轴对象，默认无运行许可、无待执行测试。
 * @param configuration 活动配置，仅读取，不修改参数或标定。
 * @param foc 本轴FOC及测量对象。
 * @param motor 本轴控制目标。
 * @param encoder 本轴编码器反馈。
 * @param observer 本轴已有磁链观测器，仅用于记录标定参考。
 * @note 仅启动阶段调用；不启动外设、不启相、不分配内存。
 */
void MotorBench_Init(const MotorConfiguration *configuration,
                     FocState *foc,
                     MotorControlState *motor,
                     EncoderState *encoder,
                     FluxObserverState *observer);

/**
 * @brief 快环控制前执行本机时限/限值检查并更新已明确请求的测试目标。
 * @note 20kHz调用，必须在MotorLifecycle_BeginFastTick之前；停止仍经原生命周期立即关相。
 */
void MotorBench_BeforeControl(void);

/**
 * @brief 在控制完成后有界复制一拍数据，触发完成后冻结缓冲。
 * @param outcome 本拍真实控制/标定结果，用于区分正常完成与中止。
 * @note 20kHz调用；最多记录一个32B样本，不发送CAN、不等待主机。
 */
void MotorBench_AfterControl(MotorTaskResult outcome);

/**
 * @brief 记录关相前台的LUT发布结果，区分真正完成与取消。
 * @param succeeded 原会话的LUT是否成功发布。
 * @note 由Axis在已有发布临界区内调用，不采集、不驱动输出。
 */
void MotorBench_CalibrationPublished(bool succeeded);

/**
 * @brief 处理测试镜像的版本化配置、单次激励、采集和只读导出请求。
 * @param context 保留绑定参数，固定NULL。
 * @param request 已通过外层校验的单帧请求，仅主站2可调用。
 * @param result 当前请求的回复缓冲，不保存其地址。
 * @return 已生成明确业务回复返回OK，外层请求无效返回INVALID_FIELD。
 * @note 仅前台；有副作用的重复序号复用原回复，不重复执行；未配置限值时拒绝动力请求。
 */
yg_protocol_result_t MotorBench_Handle(void *context,
                                       const yg_protocol_message_t *request,
                                       yg_protocol_service_result_t *result);

#endif
