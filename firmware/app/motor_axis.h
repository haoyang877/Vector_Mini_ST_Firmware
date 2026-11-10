#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H
#include <stdint.h>
#include "encoder_feedback.h"
#include "linear_hall.h"
#include "output_encoder.h"
#include "foc.h"
#include "application_config.h"
#include "motor_state_machine.h"
#include "motor_calibration.h"
#include "parameter_record.h"
#include "motor_sensing.h"
#include "bsp_flash.h"
#include "parameter_values.h"

/** @brief 上电前绑定该轴独占的 PWM 和参数槽，指针在轴的寿命内有效。 */
typedef struct
{
    MotorPowerOutput power;
    BspResult (*read_record)(void *data, size_t length);
    BspResult (*write_record)(const void *data, size_t length);
} MotorAxisHardware;

/** @brief 一根轴独占的控制、反馈与工作区；由板级静态零初始化。 */
typedef struct
{
    const MotorAxisHardware *hardware;
    MotorConfiguration configuration;
    MotorControlState motor;
    PiController speed_pi;
    EncoderState rotor_encoder;
    FluxObserverState observer;
    SensorlessStartupState startup;
    FocState foc;
    CurrentOffsetCalibration current_offsets;
    LinearHall linear_hall;
    OutputEncoder output_encoder;

    /* 启动装载和停机前台保存串行复用，避免再分配整条记录。 */
    MotorAxisRecord record;
    ParameterPersistence parameters;
#if BSP_ENABLED_LINEAR_HALL
    /* 外环选定读槽后，ADC 只更新另一槽。两槽均在启动时清零。 */
    volatile OutputEncoderCalibSample hall_samples[2];
    volatile uint8_t hall_read_slot, hall_published_slot;
#endif

    MotorOuterLoop outer;
    MotorStateMachine state_machine;
    MotorCalibration calibration;
    volatile McuTemperatureTelemetry temperature;
    MotorTemperatureTelemetry motor_temperature;
    MotorOutputTelemetry output_feedback;
    MotorPositionPlan position_work;
} MotorAxis;

/**
 * @brief 停机前台提交一个有版本约束的参数，重新装配控制器。
 * @param axis 所属轴。
 * @param request 含原始定点值及期望活动版本。
 * @return 校验或提交结果；相同值重试不增加版本。
 */
ParameterStatus MotorAxis_WriteParameter(MotorAxis *axis, const ParameterWriteRequest *request);
/**
 * @brief 显式保存当前整轴记录，成功代表BSP写入和回读均完成。
 * @param axis 所属轴。
 * @param expected_revision 期望活动版本。
 * @return 版本冲突、状态拒绝或实际持久化结果。
 * @note 仅停机前台调用；Flash事务内允许全局临界区。
 */
ParameterStatus MotorAxis_SaveParameters(MotorAxis *axis, uint32_t expected_revision);
/**
 * @brief 读取当前或Flash中已保存的标量。
 * @param axis 所属轴，复用记录暂存区。
 * @param id 参数目录编号。
 * @param saved 为true时读取并验证Flash记录。
 * @param value 成功时填写标量。
 * @return 实际读取或验证结果。
 * @note 仅串行前台调用；不改变活动配置。
 */
ParameterStatus
MotorAxis_ReadParameter(MotorAxis *axis, uint8_t id, bool saved, ParameterValue *value);
/**
 * @brief 恢复控制调参默认值，保留节点、生产身份及全部传感器标定。
 * @param axis 所属轴。
 * @param expected_revision 期望活动版本。
 * @return RAM提交结果；随后由122显式保存。
 * @note 仅关相IDLE前台调用，不恢复电机模型或安装几何。
 */
ParameterStatus MotorAxis_RestoreControlDefaults(MotorAxis *axis, uint32_t expected_revision);

/** @brief AT32 测试通道只读借用本机 Hall 状态。 */
typedef struct
{
    const LinearHall *linear_hall;
} MotorAxisFeedback;

/**
 * @brief 静态绑定的轴反馈，供 APP 在 2 kHz 维护拍读取最近的外环输出。
 * @note 不修改或缓存所指状态；跨中断读取需另外取得一致快照。
 */
extern const MotorAxisFeedback MotorAxis_Feedback;

/**
 * @brief 装载参数、初始化并绑定单轴运行对象。
 * @note 仅启动组合层在中断启用前调用一次。hardware 为空时使用本机 PWM 和参数槽。
 * @param axis 调用方持有的单轴实例。
 */
void MotorAxis_Init(MotorAxis *axis);

/**
 * @brief 读取快环电压、三相/母线电流及编码器，完成零偏校正和反馈估计。
 * @note ADC确认完成后调用；保持编码器传输与计算重叠，必须先于MotorAxis_ControlFast。
 *       零偏未完成时读取三相原码做校正，完成后读取已换算电流。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_SampleFast(MotorAxis *axis);

/**
 * @brief 使用本拍采样结果，互斥执行标定或正常控制并发布故障与输出。
 * @note 每拍MotorAxis_SampleFast之后调用一次；不重复读取ADC或发起编码器事务。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_ControlFast(MotorAxis *axis);

/**
 * @brief 执行 2 kHz 状态机监督及温度维护。
 * @note 仅监督上下文调用。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_SupervisorTick(MotorAxis *axis);

/**
 * @brief 外环执行上下文更新位置估计、速度或位置控制，发布本拍电流参考。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_OuterTick(MotorAxis *axis);

/**
 * @brief 外环超过一个更新周期时锁存时序故障并立即关闭 PWM，仅 ADC 快拍调用。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_OuterOverrun(MotorAxis *axis);

/**
 * @brief 前台承接位置轨迹建表，仅短暂屏蔽外环交接副本。
 * @note 仅主循环调用，建表期间 ADC 与外环可以抢占。
 * @param axis 调用方持有的实例状态。
 */
void MotorAxis_ProcessPositionPlan(MotorAxis *axis);

#endif
