#ifndef LOADER_UPGRADE_H
#define LOADER_UPGRADE_H

#include "image_transfer.h"

#define LOADER_UPGRADE_LEASE_MS 5000U
#define LOADER_UPGRADE_ACK_MS 500U
#define LOADER_UPGRADE_MAX_BLOCK_BYTES 128U

/** 公司 seq16 只关联回复；此绑定隔离重启、主机与升级事务。 */
typedef struct
{
    uint64_t boot_epoch;
    uint32_t session_id;
    uint8_t source_node;
} LoaderUpgradeBinding;

/** 状态输出不包含指向内部存储的指针；next_offset 仅统计已读回的连续镜像。 */
typedef struct
{
    LoaderUpgradeBinding binding;
    LoaderImagePhase phase;
    LoaderImageResult reason;
    uint32_t next_offset;
    uint32_t image_bytes;
    uint32_t token;
    uint32_t version;
    uint32_t role;
    bool activation_pending;
} LoaderUpgradeStatus;

/** 平台生命周期端口；时钟须包含 Flash 停顿，所有回调均由前台串行调用。 */
typedef struct
{
    void *context;
    uint32_t (*now_ms)(void *context);
    bool (*outputs_off_and_link_ready)(void *context);
    void (*watchdog)(void *context);
    uint32_t (*lock)(void *context);
    void (*unlock)(void *context, uint32_t key);
    /** @brief 复核通过后发布启动复位；成功复位不返回，返回 true 仅用于纯内存模型。 */
    bool (*boot)(void *context);
} LoaderUpgradePorts;

/** 组合层独占；不使用持久启动记录，仅绑定本次会话的几何与端口。 */
typedef struct
{
    LoaderImageTransfer image;
    LoaderImageConfig config;
    LoaderUpgradePorts ports;
    LoaderUpgradeBinding binding;
    uint32_t sessions;
    uint32_t last_activity_ms;
    uint32_t ack_started_ms;
    uint32_t written_bytes;
    uint64_t tickets;
    uint64_t pending_ticket;
    bool initialized;
    bool active;
    bool in_call;
} LoaderUpgrade;

/**
 * @brief 绑定镜像几何与物理端口。
 * @param upgrade 调用方拥有的整个升级状态。
 * @param image 固定镜像配置；复制几何与 Flash 端口，不保留配置指针。
 * @param ports 终生有效的平台回调值。
 * @return 参数有效并初始化成功返回 true。
 * @note 前台初始化一次；不擦写、不启动硬件、不分配持久启动号。
 */
bool LoaderUpgrade_Init(LoaderUpgrade *upgrade,
                        const LoaderImageConfig *image,
                        const LoaderUpgradePorts *ports);

/**
 * @brief 检查新镜像并分配会话，随后由 Poll 执行整槽擦除。
 * @param upgrade 唯一升级状态。
 * @param source_node 通过外层校验的主机节点，禁止广播。
 * @param expected_boot 主机提供的非零不透明会话标识；不再对应持久启动记录。
 * @param manifest 完整 96B 镜像容器，返回后不保留指针。
 * @param status 输出当前状态和分配的 session_id。
 * @return 接受返回 YI_OK；其他主机、未过期不同镜像和错误启动代号拒绝。
 */
LoaderImageResult LoaderUpgrade_Begin(LoaderUpgrade *upgrade,
                                      uint8_t source_node,
                                      uint64_t expected_boot,
                                      const uint8_t manifest[96],
                                      LoaderUpgradeStatus *status);

/**
 * @brief 校验会话后写入一个有界数据块并读回。
 * @param upgrade 独占状态。
 * @param binding 外层解析的主机、启动和会话绑定。
 * @param offset 相对镜像偏移，单位字节且 8B 对齐。
 * @param data 借用的数据，不保留指针。
 * @param bytes 数据长度，8..128B 且为 8 的倍数。
 * @return 数据持久验证返回 YI_OK；不等于整镜像已验证。
 */
LoaderImageResult LoaderUpgrade_Write(LoaderUpgrade *upgrade,
                                      const LoaderUpgradeBinding *binding,
                                      uint32_t offset,
                                      const uint8_t *data,
                                      uint32_t bytes);

/**
 * @brief 启动完整镜像校验，完成前不能激活。
 * @param upgrade 独占状态。
 * @param binding 精确会话绑定。
 * @return 接受返回 YI_OK，其他状态返回拒绝原因。
 */
LoaderImageResult LoaderUpgrade_Finish(LoaderUpgrade *upgrade, const LoaderUpgradeBinding *binding);

/**
 * @brief 拷贝当前状态，合法活动会话的查询可续租。
 * @param upgrade 独占状态。
 * @param binding 活动查询绑定；NULL 为无续租的启动发现。
 * @param status 完整状态输出。
 * @return 成功返回 YI_OK，错误绑定返回 YI_DENIED。
 */
LoaderImageResult LoaderUpgrade_Status(LoaderUpgrade *upgrade,
                                       const LoaderUpgradeBinding *binding,
                                       LoaderUpgradeStatus *status);

/**
 * @brief 为已验证镜像分配精确 ACK 票据，尚不提交或复位。
 * @param upgrade 独占状态。
 * @param binding 精确会话绑定。
 * @param ack_ticket 输出不可截断的 64 位平台发送票据。
 * @return 接受返回 YI_OK；重复请求只返回同一票据，不延长截止。
 * @note 票据必须随对应 54 回复帧进入硬件队列；只有同帧完成事件才能回传。
 *       不再持久提交；成功后由完成事件复核镜像并复位，下次启动现场校验。
 */
LoaderImageResult LoaderUpgrade_Activate(LoaderUpgrade *upgrade,
                                         const LoaderUpgradeBinding *binding,
                                         uint64_t *ack_ticket);

/**
 * @brief 处理对应 ACK 的物理发送完成，再持久提交和发布 START。
 * @param upgrade 独占状态。
 * @param ack_ticket 原样返回的平台完成票据。
 * @param success 对应帧真实成功发送；入队或 FIFO 空不算成功。
 * @return 持久提交且 START 已发布返回 true，其他情况不复位。
 * @note 平台 ISR 只排队事件；本函数仅前台执行，并在最终临界区复核截止与输出。
 */
bool LoaderUpgrade_TxComplete(LoaderUpgrade *upgrade, uint64_t ack_ticket, bool success);

/**
 * @brief 撤销未提交会话，保持功率禁止。
 * @param upgrade 独占状态。
 * @param binding 精确绑定；已撤销的最后会话重复 ABORT 返回成功。
 * @return 撤销返回 YI_OK，已提交或错误绑定拒绝。
 */
LoaderImageResult LoaderUpgrade_Abort(LoaderUpgrade *upgrade, const LoaderUpgradeBinding *binding);

/**
 * @brief 检查租约并推进一页擦除或 64B 摘要。
 * @param upgrade 独占状态。
 * @note 主循环有界调用，不按轮询自动续租。
 */
void LoaderUpgrade_Poll(LoaderUpgrade *upgrade);

/**
 * @brief 链路丢失时撤销当前升级写权限与待发激活票据。
 * @param upgrade 独占状态。
 * @note 不自动恢复会话或使能输出；下一次 BEGIN 要求全擦重传。
 */
void LoaderUpgrade_LinkLost(LoaderUpgrade *upgrade);
#endif
