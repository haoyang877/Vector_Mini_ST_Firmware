#ifndef LOADER_IMAGE_TRANSFER_H
#define LOADER_IMAGE_TRANSFER_H

#include "nv_record.h"
#include "nv_sha256.h"
#include "image_format.h"

/** 镜像阶段由单一前台所有者推进；写失败只能重新建立整包事务。 */
typedef enum
{
    YI_IDLE,
    YI_READY,
    YI_ERASING,
    YI_RECEIVING,
    YI_VERIFYING,
    YI_VERIFIED,
    YI_COMMITTED,
    YI_ERROR
} LoaderImagePhase;
typedef enum
{
    YI_OK = 0,
    YI_ARGUMENT = 1,
    YI_BUSY = 5,
    YI_IDENTITY = 6,
    YI_FLASH = 8,
    YI_CONFLICT = 10,
    YI_DENIED = 13,
    YI_RANGE = 14,
    YI_DIGEST = 15,
    YI_STATE = 16
} LoaderImageResult;
/** RAW11 仅表示存储外形识别；不能授予电机使能，完整标定检查由 APP 所有者执行。 */
typedef enum
{
    YI_PARAMS_RAW11,
    YI_PARAMS_BLANK,
    YI_PARAMS_UNSAFE
} LoaderParameterState;
typedef enum
{
    YI_RESCUE,
    YI_BOOT_NORMAL,
    YI_BOOT_MAINTENANCE_INHIBITED
} LoaderBootDecision;
typedef struct
{
    uint32_t version, token, bytes, role;
    uint8_t sha[32];
} LoaderImageIdentity;
/** 组合层持有终生有效的几何与权限端口；读写不得访问参数区。 */
typedef struct
{
    LoaderFlashIo io;
    LoaderFlashRegion app, record;
    uint32_t layout, mcu, product, hardware, param_abi, ram_start, ram_end, descriptor_offset;
    uint16_t schema;
    void *owner;
    bool (*allowed)(void *owner);
    void (*watchdog)(void *owner);
} LoaderImageConfig;
/** 调用方持有状态，禁止并发调用或修改进行中的清单和配置。 */
typedef struct
{
    const LoaderImageConfig *config;
    uint8_t manifest[96];
    LoaderImageIdentity image;
    LoaderSha256 hash;
    uint32_t manifest_crc, next, erased;
    LoaderImagePhase phase;
    LoaderImageResult reason;
    bool in_call;
} LoaderImageTransfer;

/**
 * @brief 校验固定几何并初始化镜像事务。
 * @param transfer 调用方独占状态。
 * @param config 生命周期覆盖事务的只读配置。
 * @return 几何和端口有效返回 true。
 * @note 前台调用，不擦除 Flash。
 */
bool LoaderImage_Init(LoaderImageTransfer *transfer, const LoaderImageConfig *config);
/**
 * @brief 解析完整清单并检查目标、角色、长度和参数 ABI。
 * @param config 固定目标配置。
 * @param manifest 完整 96B 大端清单。
 * @param identity 成功后写入身份，失败不修改。
 * @return 清单符合本目标返回 true。
 */
bool LoaderImage_DecodeManifest(const LoaderImageConfig *config,
                                const uint8_t manifest[96],
                                LoaderImageIdentity *identity);
/**
 * @brief 接受清单副本，尚不擦除。
 * @param transfer 已初始化的独占事务。
 * @param manifest 完整 96B 清单，返回后不保留指针。
 * @return 接受返回 YI_OK，否则返回明确原因。
 */
LoaderImageResult LoaderImage_Begin(LoaderImageTransfer *transfer, const uint8_t manifest[96]);
/**
 * @brief 请求整槽擦除，实际操作由 Poll 逐页完成。
 * @param transfer 已接收兼容清单的事务。
 * @return 接受返回 YI_OK，阶段不符返回 YI_STATE。
 */
LoaderImageResult LoaderImage_Erase(LoaderImageTransfer *transfer);
/**
 * @brief 写入并逐单元读回，旧偏移只比较，不重复编程。
 * @param transfer 唯一前台事务。
 * @param offset 相对镜像起点的字节偏移，8B 对齐。
 * @param data 本次借用的数据。
 * @param bytes 数据字节数，8..128 且为 8 的倍数。
 * @return 全部落盘返回 YI_OK，错误后 next 仅包含已读回数据。
 * @note 最多 16 次 8B 编程；每次检查权限，失败不在原 ECC 单元补写。
 */
LoaderImageResult LoaderImage_Write(LoaderImageTransfer *transfer,
                                    uint32_t offset,
                                    const uint8_t *data,
                                    uint32_t bytes);
/**
 * @brief 启动整镜像摘要、向量表和内嵌描述符检查。
 * @param transfer 已连续接收全部镜像的事务。
 * @return 接受返回 YI_OK，未收齐返回 YI_STATE。
 */
LoaderImageResult LoaderImage_Verify(LoaderImageTransfer *transfer);
/**
 * @brief 推进一步擦除或摘要校验。
 * @param transfer 独占事务。
 * @return 本步结果，不等于完整事务已提交。
 * @note 每次最多擦除一页或摘要 64B；仅前台调用。
 */
LoaderImageResult LoaderImage_Poll(LoaderImageTransfer *transfer);
/**
 * @brief 撤销 RAM 会话，已擦写的镜像不会回滚。
 * @param transfer 待撤销事务。
 * @note 不擦写 Flash，不使能输出；已提交事务由上层拒绝撤销。
 */
void LoaderImage_Abort(LoaderImageTransfer *transfer);
/**
 * @brief 在精确 ACK 完成后重新验证并发布 trial-ready 记录。
 * @param transfer 已 VERIFIED 的事务。
 * @return 持久记录读回完成返回 YI_OK。
 * @note 唯一升级所有者调用；配置 allowed 每个写单元再次核对权限。
 */
LoaderImageResult LoaderImage_Commit(LoaderImageTransfer *transfer);
/**
 * @brief 读取完整有效的镜像记录。
 * @param config 固定目标配置。
 * @param manifest 成功输出 96B 清单。
 * @param state 输出 1待试启动、2已尝试未确认、3已确认。
 * @return 容器和身份有效返回 true，不代表镜像已经校验。
 */
bool LoaderImage_ReadRecord(const LoaderImageConfig *config, uint8_t manifest[96], uint32_t *state);
/**
 * @brief 检查整个 Flash 镜像、向量和内嵌描述符。
 * @param config 只读目标配置。
 * @param identity 已校验清单的镜像身份。
 * @return 所有校验满足返回 true。
 * @note 有界扫描整个镜像并喂狗，不得在 ISR 调用。
 */
bool LoaderImage_Check(const LoaderImageConfig *config, const LoaderImageIdentity *identity);
/**
 * @brief 准入启动并在首次试启动前持久标记 attempted。
 * @param config 固定目标配置。
 * @param parameters 参数外形状态，不授予电机使能。
 * @param forced_rescue 显式救援请求。
 * @param identity 成功返回实际启动镜像身份。
 * @return 救援、普通启动或受抑制维护启动。
 */
LoaderBootDecision LoaderImage_PrepareBoot(const LoaderImageConfig *config,
                                           LoaderParameterState parameters,
                                           bool forced_rescue,
                                           LoaderImageIdentity *identity);
/**
 * @brief 确认精确绑定的已尝试镜像。
 * @param config 固定目标配置，只允许镜像记录页写入。
 * @param parameters 参数外形状态，完整 APP 健康由调用方验证。
 * @param expected APP 自检且通信可用后提交的精确镜像身份。
 * @return 持久确认完成返回 YI_OK，身份或阶段不符拒绝。
 * @note 仅可信 APP 确认所有者调用，不能由任意主机清除故障启动状态。
 */
LoaderImageResult LoaderImage_Confirm(const LoaderImageConfig *config,
                                      LoaderParameterState parameters,
                                      const LoaderImageIdentity *expected);
#endif
