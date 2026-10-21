#ifndef PARAMETER_VALUES_H
#define PARAMETER_VALUES_H

#include <stdint.h>
#include "motor_config.h"

/** 内部业务结果，通信层显式映射，不直接作为线上 result 编号。 */
typedef enum
{
    PARAMETER_OK,
    PARAMETER_ACCEPTED,
    PARAMETER_BUSY,
    PARAMETER_INVALID_ARGUMENT,
    PARAMETER_UNSUPPORTED,
    PARAMETER_READ_ONLY,
    PARAMETER_BAD_TYPE,
    PARAMETER_OUT_OF_RANGE,
    PARAMETER_WRONG_STATE,
    PARAMETER_REVISION_CONFLICT,
    PARAMETER_PERSIST_FAILED,
    PARAMETER_CANCELLED,
    PARAMETER_STALE_TICKET,
    PARAMETER_STALE_SEQUENCE,
    PARAMETER_CONFLICTING_RETRY,
    PARAMETER_FAULT
} ParameterStatus;

enum
{
    PARAMETER_VALUE_I32 = 1U,
    PARAMETER_VALUE_U32 = 2U,
    PARAMETER_ACCESS_READ = 1U,
    PARAMETER_ACCESS_WRITE_IDLE = 2U,
    PARAMETER_ACCESS_CALIBRATION = 4U
};

/** 单个活动值；raw_value 保留 i32/u32 位模式，缩放见参数目录 v0.1。 */
typedef struct
{
    uint8_t parameter_id, value_type, access, reserved;
    uint32_t revision, raw_value;
} ParameterValue;

/** 只读请求身份；同一来源与序号只能查询同一个参数。 */
typedef struct
{
    uint8_t parameter_id, source_node;
    uint16_t sequence;
} ParameterReadRequest;

/** 有身份的单标量写请求；同 source/sequence 只能重放同一原始请求。 */
typedef struct
{
    uint8_t parameter_id, value_type, source_node, reserved;
    uint16_t sequence, reserved_sequence;
    uint32_t expected_revision, raw_value;
} ParameterWriteRequest;

/** 持久化请求身份，不携带 schema 对象或 Flash 地址。 */
typedef struct
{
    uint8_t source_node, reserved;
    uint16_t sequence;
    uint32_t expected_revision;
} ParameterSaveRequest;

/**
 * @brief 从稳定活动配置读取一个有明确来源的标量。
 * @param configuration APP 所有者提供的稳定配置，含编码器标定。
 * @param revision 当前活动版本，不是 Flash schema。
 * @param parameter_id 参数目录 v0.1 的单字节 ID。
 * @param value 接收值，失败保持全零，不保留指针。
 * @return 成功、未支持来源、编码越界或错误。
 * @note 纯有界计算，不分配、不执行 CRC 或 I/O；外部必须保证一致读取。
 */
ParameterStatus ParameterValue_Read(const MotorConfiguration *configuration,
                                    uint32_t revision,
                                    uint8_t parameter_id,
                                    ParameterValue *value);
/**
 * @brief 校验并应用具备既有产品范围的停机单标量。
 * @param configuration APP 独占配置，任何失败均保持原值。
 * @param request 原始值请求，仅读取 ID/类型和值；身份与 revision 由 APP 核对。
 * @return 已应用返回 OK；只读、类型或范围错误不修改配置。
 * @note 仅所有者停机独占时调用；耦合模型和未冻结范围的项保持只读。
 */
ParameterStatus ParameterValue_Apply(MotorConfiguration *configuration,
                                     const ParameterWriteRequest *request);

/**
 * @brief 检查当前电机配置的基础数值、轴与控制限值是否可用。
 * @param configuration 所有者提供的稳定配置；空指针无效。
 * @return 关键数值有限、范围有效且轴配置有效返回 true；不要求确认或标定。
 * @note 纯有界计算；不证明参数与实际电机匹配，不访问硬件。
 */
bool ParameterValue_ConfigurationValid(const MotorConfiguration *configuration);

#endif
