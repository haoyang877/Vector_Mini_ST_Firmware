#ifndef YG_PROTOCOL_H
#define YG_PROTOCOL_H

#include "yg_protocol_common.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "bsp_can.h"
#include "bsp_system.h"
#include "loader_entry.h"

/* 线路结果码与内部业务状态分开；数值由既有报文约定固定。 */
typedef enum
{
    PROTOCOL_RESULT_OK = 0,
    PROTOCOL_RESULT_ACCEPTED = 1,
    PROTOCOL_RESULT_UNSUPPORTED = 2,
    PROTOCOL_RESULT_BAD_LENGTH = 3,
    PROTOCOL_RESULT_BAD_FIELD = 4,
    PROTOCOL_RESULT_DENIED = 5,
    PROTOCOL_RESULT_BUSY = 6,
    PROTOCOL_RESULT_FORBIDDEN = 7,
    PROTOCOL_RESULT_FAILED = 8,
    PROTOCOL_RESULT_OUT_OF_RANGE = 11,
    PROTOCOL_RESULT_FAULT = 17,
} ProtocolCode;

typedef struct ProtocolEndpoint ProtocolEndpoint;

/** @brief 命令回复的载荷；地址、序号和 CAN 发送由 yg_protocol.c 统一处理。 */
typedef struct
{
    uint32_t length;
    uint8_t data[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
} ProtocolResponse;

/**
 * @brief 一次命令的请求与回复，由前台处理函数持有。
 * @note request.payload 借用当前接收帧；reply 持有回复字节，处理返回前完成编码入队。
 */
typedef struct
{
    yg_protocol_message_t request;
    ProtocolResponse reply;
} ProtocolExchange;

/** @brief 内部业务执行结果；不是线上错误码，不得直接序列化枚举。 */
typedef enum
{
    PROTOCOL_OK = 0,
    PROTOCOL_ACCEPTED,
    PROTOCOL_UNSUPPORTED,
    PROTOCOL_INVALID_ARGUMENT,
    PROTOCOL_BUSY,
    PROTOCOL_DENIED,
    PROTOCOL_FAILED,
    PROTOCOL_OUT_OF_RANGE,
    PROTOCOL_FAULT
} ProtocolStatus;

/** @brief 电机请求的结果和明细，对应线上 4 字节回复。 */
typedef struct
{
    ProtocolStatus status;
    uint16_t detail;
} ProtocolReply;

#define PROTOCOL_MASTER_ID 2U
#define PROTOCOL_READ_PARAMETER 120U
#define PROTOCOL_WRITE_PARAMETER 121U
#define PROTOCOL_SAVE_PARAMETER 122U

#define PROTOCOL_TRANSFER_DATA_SIZE 64U

#define PROTOCOL_PAGE_REQUEST_SIZE 12U
#define PROTOCOL_INFO_PAGE0_SIZE 32U
#define PROTOCOL_CAPS_PAGE0_SIZE 16U
#define PROTOCOL_DEVICE_STATUS_SIZE 16U
#define PROTOCOL_DEVICE_ROLE_APP 1U
#define PROTOCOL_DEVICE_ROLE_LOADER 2U
#define PROTOCOL_BOOT_STATE_UNAVAILABLE 0U
#define PROTOCOL_BOOT_STATE_CONFIRMED_APP 1U
#define PROTOCOL_BOOT_STATE_TRIAL_APP 2U
#define PROTOCOL_BOOT_STATE_LOADER 3U

/** @brief Type3真实启动身份快照，boot_epoch不能由GET_INFO的u32计数补造。 */
typedef struct
{
    uint64_t boot_epoch;
    uint8_t role;
    uint8_t boot_state;
    bool epoch_valid;
} ProtocolDeviceStatus;

/** @brief GET_INFO page 0 的设备身份和版本字段。 */
typedef struct
{
    uint8_t uid[12];
    uint32_t product_id;
    uint16_t hardware_revision;
    uint16_t boot_api;
    uint32_t application_version;
    uint32_t boot_version;
    uint32_t boot_id;
} ProtocolInfo;

/** @brief GET_CAPS page 0 的设备能力字段。 */
typedef struct
{
    uint32_t features;
    uint16_t supported_modes;
    uint16_t max_full_frame;
    uint16_t max_block_data;
    uint16_t write_alignment;
    uint16_t default_watchdog_ms;
    uint16_t max_watchdog_ms;
} ProtocolCaps;

/* 设备查询与状态反馈命令。 */
#define PROTOCOL_GET_INFO 0x0001U
#define PROTOCOL_GET_CAPS 0x0002U
#define PROTOCOL_GET_DEVICE_STATUS 0x0003U
#define PROTOCOL_GET_MOTOR_STATE 0x006CU
#define PROTOCOL_MOTION_FEEDBACK 0x007CU

#define PROTOCOL_FEEDBACK_SIZE 45U
#define PROTOCOL_FEEDBACK_VALID_MASK 0x0FFFU

/**
 * @brief 状态反馈字段的固定单位值对象；结构体内存布局不是线路布局。
 * @note result/关联序号来自当前请求，valid_bits 的位 0～11 对应十二个测量量。
 *       无效测量由所有者填入对应整数最小值，编码器只写入这些显式值。
 */
typedef struct
{
    uint16_t result;
    uint16_t correlated_seq;
    uint32_t fault_code;
    int32_t reference_position_mrad;
    int32_t actual_position_mrad;
    int32_t reference_speed_mrad_s;
    int32_t actual_speed_mrad_s;
    uint16_t valid_bits;
    int16_t bus_voltage_cV;
    int16_t bus_current_mA;
    int16_t reference_iq_mA;
    int16_t actual_iq_mA;
    int16_t mcu_temperature_centi_c;
    int16_t motor_temperature_centi_c;
    int16_t v_q_mV;
    int16_t v_d_mV;
    uint8_t node_id;
    uint8_t motor_state;
    uint8_t control_mode;
} ProtocolFeedback;

/* 电机命令与载荷长度。 */
#define PROTOCOL_STOP 110U
#define PROTOCOL_ENABLE 101U
#define PROTOCOL_CLEAR_FAULT 107U
#define PROTOCOL_CONTROL 116U
#define PROTOCOL_START 117U
#define PROTOCOL_ENABLE_SIZE 4U
#define PROTOCOL_START_SIZE 4U
#define PROTOCOL_CONTROL_SIZE 20U
#define PROTOCOL_OPEN_LOOP_SIZE 32U
#define PROTOCOL_REPLY_SIZE 4U

/** @brief 单轴控制值对象；位置、速度、电流、电压单位依次为 mrad、mrad/s、mA、mV。 */
typedef struct
{
    uint8_t source_node;
    uint16_t sequence;
    uint8_t mode;
    int32_t position_mrad;
    int32_t speed_mrad_s;
    int32_t iq_mA;
    int32_t v_q_mV;
    int32_t electrical_frequency_mHz;
    int32_t id_mA;
    int32_t vd_mV;
} ProtocolCommand;

/**
 * @brief 电机所有者实现的非阻塞操作，通信在校验后直接调用对应函数。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 所有者填充的结果，必须设置 status。
 * @note 前台串行调用；状态和范围由所有者校验，不能直接把线上值写入控制变量。
 */
typedef void (*ProtocolMotorHandler)(void *context,
                                     const ProtocolCommand *request,
                                     ProtocolReply *reply);

/** @brief 四个电机操作均由 APP 静态提供；不支持的操作直接返回 UNSUPPORTED。 */
typedef struct
{
    void *context;
    ProtocolMotorHandler start;
    ProtocolMotorHandler target;
    ProtocolMotorHandler stop;
    ProtocolMotorHandler clear_fault;
    /** 前台填写线路状态和有效测量。未测字段保留预置哨兵，返回是否有状态样本。 */
    bool (*read_feedback)(void *context, ProtocolFeedback *feedback);
} ProtocolMotor;

#define PROTOCOL_START_JOB 160U
#define PROTOCOL_GET_JOB_STATUS 161U
#define PROTOCOL_CANCEL_JOB 162U
#define PROTOCOL_JOB_REQUEST_SIZE 4U
#define PROTOCOL_JOB_REPLY_SIZE 12U
#define PROTOCOL_JOB_KIND_ENCODER_LUT 1U
#define PROTOCOL_JOB_KIND_ELECTRICAL_ZERO 2U
#define PROTOCOL_JOB_KIND_COGGING 3U
#define PROTOCOL_JOB_KIND_PHASE_RESISTANCE 4U
#define PROTOCOL_JOB_KIND_FRICTION 5U
/* 3～5 保留旧编号，但当前产品不再提供相应任务。 */
#define PROTOCOL_JOB_KIND_MECHANICAL_ZERO 6U
#define PROTOCOL_JOB_KIND_SAVE 7U
#define PROTOCOL_JOB_KIND_DEFAULTS 8U
#define PROTOCOL_JOB_PHASE_NONE 0U
#define PROTOCOL_JOB_PHASE_IN_PROGRESS 1U
#define PROTOCOL_JOB_PHASE_COMPLETED 2U
#define PROTOCOL_JOB_PHASE_CANCEL_REQUESTED 3U
#define PROTOCOL_JOB_PHASE_CANCELLED 4U
#define PROTOCOL_JOB_PHASE_FAILED 5U
#define PROTOCOL_JOB_EFFECT_UNKNOWN 0U
#define PROTOCOL_JOB_EFFECT_UNCOMMITTED 1U
#define PROTOCOL_JOB_EFFECT_COMMITTED 2U
#define PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND 1U
#define PROTOCOL_JOB_DETAIL_STALE_TOKEN 2U
#define PROTOCOL_JOB_DETAIL_WRONG_STATE 3U
#define PROTOCOL_JOB_DETAIL_CONFLICTING_RETRY 4U
#define PROTOCOL_JOB_DETAIL_STALE_SEQUENCE 5U
#define PROTOCOL_JOB_DETAIL_BAD_FIELD 6U
#define PROTOCOL_JOB_DETAIL_OPERATION_FAILED 7U

/** @brief job 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    PROTOCOL_JOB_START = 0,
    PROTOCOL_JOB_CANCEL,
    PROTOCOL_JOB_QUERY,
} ProtocolJobAction;

/** @brief 任务值请求；kind独立于内部模式，job_id隔离取消与查询，不含工作区指针。 */
typedef struct
{
    ProtocolJobAction operation;
    uint16_t kind;
    uint32_t job_id;
    uint16_t sequence;
    uint8_t source_node;
} ProtocolJobRequest;

/** @brief 设备查询页的三个小端字段。 */
typedef struct
{
    uint32_t session_id;
    uint32_t request_id;
    uint16_t page;
} PageRequest;

/** @brief 单条已解析命令的值参数，由命令号选择成员，内部不借用帧内存。 */
typedef union
{
    ProtocolCommand motor;
    ProtocolJobRequest job;
    PageRequest page;
    uint8_t enable;
    uint64_t boot_epoch;
    struct
    {
        uint32_t version;
        uint32_t image_crc32;
    } confirm;
    struct
    {
        uint8_t id;
        uint8_t value_type;
        uint32_t expected_revision;
        uint32_t value;
    } parameter;
} ProtocolCommandArgs;

/**
 * @brief 前台发布最近一条通过权限和载荷检查的命令到 Keil Watch。
 * @param request 已解码的请求，仅复制命令号、地址、序号和标志。
 * @param args 与命令号对应的值参数，无参数的请求传零值。
 * @param endpoint 调用方持有的实例状态。
 */
void Protocol_RecordCommand(ProtocolEndpoint *endpoint,
                            const yg_protocol_message_t *request,
                            ProtocolCommandArgs args);

/** @brief 任务专用结果；phase/effects为固定线路值，不复用参数revision/value字段。 */
typedef struct
{
    ProtocolStatus status;
    uint32_t token;
    uint16_t kind;
    uint16_t detail;
    uint8_t phase;
    uint8_t effects;
} ProtocolJobReply;

/**
 * @brief 业务所有者实现的非阻塞处理入口。
 * @param context 后端拥有的状态，生命周期覆盖调用。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 后端填充的值结果，必须设置 status。
 * @note 前台串行调用；权限、状态和范围由后端校验，不得直接把线上输入写入控制变量。
 */
typedef void (*ProtocolJobHandler)(void *context,
                                   const ProtocolJobRequest *request,
                                   ProtocolJobReply *reply);

/** @brief 由组合层注入的后端绑定；handler 为 NULL 表示该能力未实现。 */
typedef struct
{
    void *context;
    ProtocolJobHandler handler;
} ProtocolJobs;

#define PROTOCOL_BOOT_ENTER 56U
#define PROTOCOL_BOOT_CONFIRM 57U
#define PROTOCOL_BOOT_REQUEST_SIZE 8U
#define PROTOCOL_BOOT_REPLY_SIZE 4U

/* 收发入口：启动时 Init / Start，CAN 中断调用 RxInterrupt，主循环调用 Process */

/**
 * @brief 设置本机节点号；队列、回调和默认状态均已静态初始化，不启动硬件。
 * @param node_id 本机节点号，不能为广播地址。
 * @return 初始化成功返回 true；地址非法或已经初始化返回 false。
 * @note 仅在启动前调用；之后调用 Start，运行中不能重新初始化。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_Init(ProtocolEndpoint *endpoint, uint8_t node_id);

/**
 * @brief 启动 CAN 接收，重复调用不重复启动。
 * @return 已初始化返回 true，否则返回 false。
 * @note 仅在前台启动阶段调用，不启动电机。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_Start(ProtocolEndpoint *endpoint);

/**
 * @brief 查询链路是否已成功初始化。
 * @return 已初始化返回 true，否则返回 false。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_IsReady(ProtocolEndpoint *endpoint);

/**
 * @brief 唯一 CAN FD RX FIFO0 中断入口。
 * @note 单次最多取 3 帧，只复制入队；格式、CRC、分片及业务检查留在前台。
 * @param endpoint 调用方持有的实例状态。
 */
void Protocol_RxInterrupt(ProtocolEndpoint *endpoint);

/**
 * @brief 把硬件接收的帧复制到协议 RX 队列，由前台统一校验。
 * @param frame 硬件端口提供的完整帧，函数返回后不保存指针。
 * @return 帧已复制入队返回 true；未初始化、空指针或队列满返回 false。
 * @note 由唯一CAN RX IRQ生产；前台消费端负责互斥，生产端允许电机ISR抢占。
 *   不得从第二个并发生产者调用；报文格式、CRC 和命令均在前台检查。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_QueueReceivedFrame(ProtocolEndpoint *endpoint, const BspCanFrame *frame);

/**
 * @brief 在主循环后台推进协议收发；未 Start 时不处理。
 * @param now_ms 单调毫秒时间，用于升级回复的发送超时。
 * @note 有界、不等待；一次调用最多处理 8 个输入和 8 个发送帧。116 回复与 124 反馈
 *       成对预留发送空间；队列拥塞时保留 RX 请求，发送重试不重新执行业务。
 *       电机的目标顺序和完成语义由业务所有者决定。不得在 CAN ISR 调用。
 * @param endpoint 调用方持有的实例状态。
 */
void Protocol_Process(ProtocolEndpoint *endpoint, uint32_t now_ms);

/* 设备信息与 Loader 业务绑定 */

/**
 * @brief 设置由产品组合层确认的真实设备身份和已实现能力。
 * @param info 身份和版本值；与capabilities同时为NULL时撤销描述。
 * @param capabilities 真实可用能力值，不以候选Type或编码器存在推断功能。
 * @return 两个输入同时有效或同时为NULL时返回true，否则false。
 * @note 后台串行调用，不与 Protocol_Process 并发；复制现有值缓冲，不保留输入指针。
 *       初始化或撤销后1/2查询回复4B result6/detail0，不提供占位版本或虚构能力。
 *       不访问UID寄存器，不探测硬件；组合层从平台端口和业务所有者收集数据。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_SetInfo(ProtocolEndpoint *endpoint,
                      const ProtocolInfo *info,
                      const ProtocolCaps *capabilities);

/**
 * @brief 发布GET_INFO维护页快照。
 * @param page 1为生产身份与启动状态，2为目标与Flash布局。
 * @param payload 组合层编码的32B只读页，返回后不保留指针。
 * @return 页号为1或2且输入非空返回true。
 * @note 前台串行调用；通信层不访问Flash或参数所有者。
 * @param endpoint 调用方持有的实例状态。
 */
bool Protocol_SetInfoPage(ProtocolEndpoint *endpoint, uint16_t page, const uint8_t payload[32]);

/**
 * @brief 将业务结果映射为线路码，不直接序列化枚举。
 * @param status 业务结果。
 * @return 线路结果码，未知值为内部失败。
 */
static inline ProtocolCode Protocol_StatusToCode(ProtocolStatus status)
{
    switch (status)
    {
    case PROTOCOL_OK:
        return PROTOCOL_RESULT_OK;
    case PROTOCOL_ACCEPTED:
        return PROTOCOL_RESULT_ACCEPTED;
    case PROTOCOL_UNSUPPORTED:
        return PROTOCOL_RESULT_UNSUPPORTED;
    case PROTOCOL_INVALID_ARGUMENT:
        return PROTOCOL_RESULT_BAD_FIELD;
    case PROTOCOL_BUSY:
        return PROTOCOL_RESULT_BUSY;
    case PROTOCOL_DENIED:
        return PROTOCOL_RESULT_DENIED;
    case PROTOCOL_OUT_OF_RANGE:
        return PROTOCOL_RESULT_OUT_OF_RANGE;
    case PROTOCOL_FAULT:
        return PROTOCOL_RESULT_FAULT;
    case PROTOCOL_FAILED:
    default:
        return PROTOCOL_RESULT_FAILED;
    }
}

/**
 * @brief 写入四字节线路结果，供电机和设备命令共用。
 * @param reply 调用方持有的非空回复缓冲。
 * @param code 线路结果码。
 * @param detail 当前命令的细节码。
 */
static inline void Protocol_WriteResult(ProtocolResponse *reply, ProtocolCode code, uint16_t detail)
{
    YgProtocol_WriteU16Le(reply->data, (uint16_t)code);
    YgProtocol_WriteU16Le(reply->data + 2U, detail);
    reply->length = PROTOCOL_REPLY_SIZE;
}

/* 两个实现文件共享的队列与 Loader 接口；仅前台操作 TX 队列。 */

/** @brief 固定容量队列；storage 与 capacity 静态初始化后保持有效。 */
typedef struct
{
    BspCanFrame *storage;
    size_t capacity;
    size_t head;
    size_t tail;
    volatile uint32_t produced;
    volatile uint32_t consumed;
    uint32_t dropped;
} CanQueue;

typedef struct
{
    uint16_t type;
    uint16_t sequence;
    uint8_t source_node;
    uint8_t destination_node;
    uint8_t flags;
    ProtocolCommandArgs args;
} ProtocolReceivedCommand;

typedef struct
{
    ProtocolInfo info;
    uint8_t maintenance_pages[2][32];
    ProtocolCaps capabilities;
    bool device_description_available;
} DeviceMessages;

typedef struct
{
    uint16_t feedback_sequence;
    bool feedback_pending;
    uint8_t feedback_destination;
    ProtocolResponse feedback;
} MotorMessages;

typedef enum
{
    BOOT_TX_IDLE,
    BOOT_TX_QUEUED,
    BOOT_TX_PENDING
} BootTxState;

typedef struct
{
    uint8_t node_id;
    size_t tx_slot;
    BootTxState tx_state;
    uint64_t ticket;
    uint32_t ready_ms;
    uint16_t reply_sequence;
    uint8_t reply_destination;
} BootState;

#define PROTOCOL_QUEUE_CAPACITY 8U

/** @brief 一个 CAN 端点独占的队列、命令、身份、回调与交接状态。 */
struct ProtocolEndpoint
{
    uint8_t node_id;
    bool ready, started;
    CanQueue rx_queue, tx_queue;
    BspCanFrame rx_storage[PROTOCOL_QUEUE_CAPACITY], tx_storage[PROTOCOL_QUEUE_CAPACITY];
    DeviceMessages device;
    MotorMessages messages;
    volatile ProtocolReceivedCommand command;
    BootState boot;
    ProtocolMotor motor;
    ProtocolJobs jobs;
    BootEntryService loader;
    ProtocolDeviceStatus device_status;
};

/**
 * @brief 返回已入队但未消费的帧数。
 * @param queue 有效且已初始化的队列。
 * @return 当前帧数。
 */
static inline size_t queue_count(const CanQueue *queue)
{
    return (uint32_t)(queue->produced - queue->consumed);
}

/**
 * @brief 复制队首帧，不移动读指针。
 * @param queue 有效且已初始化的队列。
 * @param frame 非空输出帧。
 * @return 队列非空并复制成功时为 true。
 */
static inline bool queue_peek(const CanQueue *queue, BspCanFrame *frame)
{
    if (queue->produced == queue->consumed)
    {
        return false;
    }
    BspSystem_MemoryBarrier();
    *frame = queue->storage[queue->tail];
    return true;
}

/**
 * @brief 消费队首帧。
 * @param queue 有效且已初始化的队列。
 * @return 队列非空并消费成功时为 true。
 */
static inline bool queue_drop(CanQueue *queue)
{
    if (queue->produced == queue->consumed)
    {
        return false;
    }
    queue->tail = (queue->tail + 1U) % queue->capacity;
    BspSystem_MemoryBarrier();
    ++queue->consumed;
    return true;
}

/**
 * @brief 取出并消费队首帧。
 * @param queue 有效且已初始化的队列。
 * @param frame 非空输出帧。
 * @return 队列非空并取出成功时为 true。
 */
static inline bool queue_pop(CanQueue *queue, BspCanFrame *frame)
{
    if (!queue_peek(queue, frame))
    {
        return false;
    }
    return queue_drop(queue);
}

/**
 * @brief 处理已经通过通用请求校验的 Loader 命令。
 * @param exchange 已解码的单播请求及由本函数填写的回复。
 * @return 处理结果；静态配置不支持 Loader 时返回 ROUTE_NOT_FOUND。
 * @param endpoint 调用方持有的实例状态。
 */
yg_protocol_result_t ProtocolBoot_HandleCommand(ProtocolEndpoint *endpoint,
                                                ProtocolExchange *exchange);

/**
 * @brief 记录新产生的 READY 回复在发送队列中的准确位置。
 * @param queue 有效的 TX 队列。
 * @param previous_head 处理请求之前的写入位置。
 * @param result 处理请求与回复入队的结果。
 * @param now_ms 本轮前台时间，单位 ms。
 * @param endpoint 调用方持有的实例状态。
 */
void ProtocolBoot_TrackReply(ProtocolEndpoint *endpoint,
                             const CanQueue *queue,
                             size_t previous_head,
                             yg_protocol_result_t result,
                             uint32_t now_ms);

/**
 * @brief 检查交接回复是否实际发出，并清理超时的待发回复。
 * @param queue 有效的 TX 队列。
 * @param now_ms 本轮前台时间，单位 ms。
 * @param endpoint 调用方持有的实例状态。
 */
void ProtocolBoot_Process(ProtocolEndpoint *endpoint, CanQueue *queue, uint32_t now_ms);

/**
 * @brief 提交下一帧；等待交接回复完成时阻止后续帧覆盖硬件完成状态。
 * @param queue 有效的 TX 队列，帧均已通过发送编码。
 * @return 提交结果或队列空、硬件忙。
 * @param endpoint 调用方持有的实例状态。
 */
yg_protocol_result_t ProtocolBoot_SendNext(ProtocolEndpoint *endpoint, CanQueue *queue);

/**
 * @brief 将本机单帧消息复制入 TX 队列，不等待硬件发送。
 * @param message 借用的完整消息，源地址必须为本机。
 * @param priority CAN 优先级，范围 0～7。
 * @return 编码或入队结果；队满时不消费调用方数据。
 * @param endpoint 调用方持有的实例状态。
 */
yg_protocol_result_t Protocol_QueueMessage(ProtocolEndpoint *endpoint,
                                           const yg_protocol_message_t *message,
                                           uint8_t priority);

#endif
