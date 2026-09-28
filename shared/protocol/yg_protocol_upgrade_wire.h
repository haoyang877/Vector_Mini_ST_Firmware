#ifndef YG_PROTOCOL_UPGRADE_WIRE_H
#define YG_PROTOCOL_UPGRADE_WIRE_H

#include "yg_protocol_wire_types.h"

#define YG_PROTOCOL_UPGRADE_TYPE_BEGIN 50U
#define YG_PROTOCOL_UPGRADE_TYPE_WRITE 51U
#define YG_PROTOCOL_UPGRADE_TYPE_FINISH 52U
#define YG_PROTOCOL_UPGRADE_TYPE_STATUS 53U
#define YG_PROTOCOL_UPGRADE_TYPE_ACTIVATE 54U
#define YG_PROTOCOL_UPGRADE_TYPE_ABORT 55U
#define YG_PROTOCOL_UPGRADE_MANIFEST_BYTES 96U
#define YG_PROTOCOL_UPGRADE_MAX_BLOCK_BYTES 128U
#define YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES 8U
#define YG_PROTOCOL_UPGRADE_BINDING_BYTES 12U
#define YG_PROTOCOL_UPGRADE_BEGIN_BYTES 104U
#define YG_PROTOCOL_UPGRADE_WRITE_PREFIX_BYTES 18U
#define YG_PROTOCOL_UPGRADE_MAX_REQUEST_BYTES 146U
#define YG_PROTOCOL_UPGRADE_REPLY_BYTES 40U
#define YG_PROTOCOL_UPGRADE_FLAG_ACTIVATION_PENDING 0x01U
#define YG_PROTOCOL_UPGRADE_FLAG_INTEGRITY_ONLY 0x02U

/* 与既定公司应用结果显式对齐，产品适配不能直接发送内部枚举值。 */
#define YG_PROTOCOL_UPGRADE_RESULT_OK 0U
#define YG_PROTOCOL_UPGRADE_RESULT_ACCEPTED 1U
#define YG_PROTOCOL_UPGRADE_RESULT_UNSUPPORTED 2U
#define YG_PROTOCOL_UPGRADE_RESULT_BAD_LENGTH 3U
#define YG_PROTOCOL_UPGRADE_RESULT_BAD_FIELD 4U
#define YG_PROTOCOL_UPGRADE_RESULT_DENIED 5U
#define YG_PROTOCOL_UPGRADE_RESULT_BUSY 6U
#define YG_PROTOCOL_UPGRADE_RESULT_UNAUTHORIZED 7U
#define YG_PROTOCOL_UPGRADE_RESULT_FAILED 8U
#define YG_PROTOCOL_UPGRADE_RESULT_OUT_OF_RANGE 11U
#define YG_PROTOCOL_UPGRADE_RESULT_FAULT 17U

/* 升级特定原因；0仅表示无进一步原因，不能代替真实存储/摘要错误。 */
#define YG_PROTOCOL_UPGRADE_DETAIL_NONE 0U
#define YG_PROTOCOL_UPGRADE_DETAIL_BAD_FIELD 1U
#define YG_PROTOCOL_UPGRADE_DETAIL_WRONG_BOOT 2U
#define YG_PROTOCOL_UPGRADE_DETAIL_WRONG_SESSION 3U
#define YG_PROTOCOL_UPGRADE_DETAIL_WRONG_STATE 4U
#define YG_PROTOCOL_UPGRADE_DETAIL_IDENTITY 5U
#define YG_PROTOCOL_UPGRADE_DETAIL_RANGE 6U
#define YG_PROTOCOL_UPGRADE_DETAIL_FLASH 7U
#define YG_PROTOCOL_UPGRADE_DETAIL_DATA_CONFLICT 8U
#define YG_PROTOCOL_UPGRADE_DETAIL_DIGEST 9U
#define YG_PROTOCOL_UPGRADE_DETAIL_UNSUPPORTED_AUTH 10U
#define YG_PROTOCOL_UPGRADE_DETAIL_OUTPUTS_OR_LINK 11U
#define YG_PROTOCOL_UPGRADE_DETAIL_TIMEOUT 12U
#define YG_PROTOCOL_UPGRADE_DETAIL_INTERNAL 13U

/** @brief 线路阶段固定值；与Loader内部枚举分别显式映射，不序列化C枚举。 */
#define YG_PROTOCOL_UPGRADE_PHASE_IDLE 0U
#define YG_PROTOCOL_UPGRADE_PHASE_READY 1U
#define YG_PROTOCOL_UPGRADE_PHASE_ERASING 2U
#define YG_PROTOCOL_UPGRADE_PHASE_RECEIVING 3U
#define YG_PROTOCOL_UPGRADE_PHASE_VERIFYING 4U
#define YG_PROTOCOL_UPGRADE_PHASE_VERIFIED 5U
#define YG_PROTOCOL_UPGRADE_PHASE_COMMITTED 6U
#define YG_PROTOCOL_UPGRADE_PHASE_ERROR 7U

/** @brief 已校验的升级值请求；data借用输入载荷，返回后不得由异步业务保留。 */
typedef struct
{
    uint64_t boot_epoch;
    uint32_t session_id;
    uint32_t image_offset_bytes;
    const uint8_t *data;
    uint16_t data_bytes;
    uint16_t message_type;
    uint16_t sequence;
    uint8_t source_node;
    bool discover;
} YgProtocolUpgradeRequest;

/** @brief 升级结果与完整状态值；本地布局不是40B线路布局，不能用memcpy序列化。 */
typedef struct
{
    uint64_t boot_epoch;
    uint32_t session_id;
    uint32_t next_offset_bytes;
    uint32_t image_bytes;
    uint32_t image_token;
    uint32_t image_version;
    uint32_t image_role;
    uint16_t result;
    uint16_t detail;
    uint16_t max_block_bytes;
    uint8_t phase;
    uint8_t flags;
} YgProtocolUpgradeReply;

/**
 * @brief 解码已校验且已完整重组的50～55请求，不执行升级业务。
 * @param message 公司消息视图，载荷及数值封装为小端；96B清单保持原样大端。
 * @param request 成功时写入请求值，失败保持原内容。
 * @return 字段合法返回OK；非法参数、Type、长度、身份或对齐返回对应协议错误。
 * @note 前台调用；不分配、不访问硬件、不解释清单、不验证主站授权。
 *       组合层须先校验CAN ID/头地址、CRC、填充和来源策略。
 *       sequence仅关联当前回复，分片尾序号不能作为跨镜像块的去重键或写权限。
 */
yg_protocol_result_t YgProtocolUpgrade_DecodeRequest(const yg_protocol_message_t *message,
                                                     YgProtocolUpgradeRequest *request);

/**
 * @brief 显式编码40B升级回复，保持公司帧与镜像事务所有者独立。
 * @param reply 真实所有者提供的完整状态，不表示对应回复已物理发送。
 * @param payload 输出缓冲，不得与reply重叠。
 * @param capacity 输出容量，至少40B。
 * @param written 成功时写入40，可为NULL；失败保持原值。
 * @return 编码成功返回OK；非法阶段/标志/容量或参数返回对应协议错误。
 * @note 不处理ACK票据。激活必须把完整u64内部票据绑定到具体回复帧的物理完成事件。
 *       v1仅声明SHA完整性校验；INTEGRITY_ONLY必须设置，不能宣称签名认证。
 */
yg_protocol_result_t YgProtocolUpgrade_EncodeReply(const YgProtocolUpgradeReply *reply,
                                                   uint8_t *payload,
                                                   size_t capacity,
                                                   size_t *written);

#endif
