#ifndef VIRTUAL_PLATFORM_H
#define VIRTUAL_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>
#include "comm_hw.h"
#include "motor_model.h"

/** 外设及可选物理对象配置；未连接模型时保留静态启动场景。 */
typedef struct
{
    float bus_voltage_v;
    float temperature_c;
    uint16_t phase_offset_counts[3]; /**< 单次12位ADC零电流计数。 */
    uint16_t encoder_angle_q15;      /**< 沿用现有接口编码，不重解释位宽。 */
    bool motor_connected;
    unsigned integration_substeps; /**< 连接模型时每快拍细分1..100步。 */
    VirtualMotorConfig motor;
} VirtualConfig;

/** 台架只读事实，不提供APP内部对象的写入口。 */
typedef struct
{
    uint64_t time_us;
    uint32_t fast_ticks, supervisor_ticks, tx_frames;
    float duty[3];
    bool phases_disabled;
    bool power_was_enabled;
    MotorModelState rotor;
    float phase_current_a[3];
    float peak_phase_current_a; /**< 所有20kHz模型采样点的相电流幅值峰值。 */
    bool sample_saturated;
} VirtualSnapshot;

/**
 * @brief 初始化一次虚拟平台并调用真实App_Init。
 * @param config 台架输入，复制后不保留地址。
 * @return 参数有效且首次初始化返回true，否则false。
 * @note 单线程；每个进程只启动一次APP，不访问真实硬件。
 */
bool VirtualPlatform_Init(const VirtualConfig *config);
/**
 * @brief 按快环拍数推进虚拟时间，并执行真实APP入口。
 * @param fast_ticks 推进的20kHz拍数。
 * @return 调度、对象和采样有效返回true，否则false。
 * @note 前一拍PWM先推进模型，再接收、快环、监督、前台；静态场景仍禁止启相。
 */
bool VirtualPlatform_Advance(uint32_t fast_ticks);
/**
 * @brief 初始化前装载一份虚拟存储记录，仍由真实参数装载器校验与应用。
 * @param record 只读字节记录，成功时复制。
 * @param size_bytes 记录长度，1..RAW区容量。
 * @return 尚未初始化且输入有效返回true，否则false。
 * @note 不写APP状态；夹具必须明确记录配置确认和理想标定的来源。
 */
bool VirtualPlatform_LoadParameters(const void *record, uint32_t size_bytes);
/**
 * @brief 复制台架事实快照。
 * @return 当前虚拟时间、调用次数和数字输出状态。
 */
VirtualSnapshot VirtualPlatform_Read(void);
/**
 * @brief 向虚拟CAN投递一帧，供下一拍真实接收入口处理。
 * @param frame 调用方帧，成功时复制。
 * @return 队列有空间且输入有效返回true，否则false。
 */
bool VirtualPlatform_Receive(const CommHwCanFrame *frame);
/**
 * @brief 消费一帧真实协议产生的虚拟CAN发送数据。
 * @param frame 调用方输出，成功时写入。
 * @return 取得帧返回true，否则false。
 * @note 消费跟踪帧才产生虚拟发送完成，不等同于物理CAN证据。
 */
bool VirtualPlatform_TakeTx(CommHwCanFrame *frame);

#endif
