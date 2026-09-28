#ifndef LOADER_IMAGE_TRANSFER_H
#define LOADER_IMAGE_TRANSFER_H

#include "image_format.h"
#include <stdbool.h>
#include <stdint.h>

/* 独占存储原语只描述端口能力，不授予功率使能；调用方负责生命周期与写边界。 */
typedef struct
{
    void *context;
    uint32_t base, bytes, erase_bytes, program_bytes;
    bool (*read)(void *context, uint32_t address, void *out, uint32_t bytes);
    bool (*erase)(void *context, uint32_t address, uint32_t bytes);
    bool (*program)(void *context, uint32_t address, const void *data, uint32_t bytes);
} LoaderFlashIo;
typedef struct
{
    uint32_t base, bytes;
} LoaderFlashRegion;

/** 镜像阶段由单一前台所有者推进；写失败只能重新建立整包事务。 */
typedef enum
{
    YI_IDLE,
    YI_READY,
    YI_ERASING,
    YI_RECEIVING,
    YI_VERIFYING,
    YI_VERIFIED,
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
/** 镜像声明身份：长度与整包 CRC32 由描述符/清单给出，仅声明完整性，不提供认证。 */
typedef struct
{
    uint32_t version, token, bytes, role, crc32;
} LoaderImageIdentity;
/** 组合层持有终生有效的几何与权限端口；读写不得访问参数区。 */
typedef struct
{
    LoaderFlashIo io;
    LoaderFlashRegion app;
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
    uint32_t manifest_crc, hash, next, erased;
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
 * @param manifest 完整 96B 大端清单，48..51 为整包 CRC32，其后保留必须为零。
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
 * @brief 启动整镜像 CRC32、向量表和内嵌描述符检查。
 * @param transfer 已连续接收全部镜像的事务。
 * @return 接受返回 YI_OK，未收齐返回 YI_STATE。
 */
LoaderImageResult LoaderImage_Verify(LoaderImageTransfer *transfer);
/**
 * @brief 推进一步擦除或 CRC32 校验。
 * @param transfer 独占事务。
 * @return 本步结果，不等于完整事务已可用。
 * @note 每次最多擦除一页或校验 64B；仅前台调用。
 */
LoaderImageResult LoaderImage_Poll(LoaderImageTransfer *transfer);
/**
 * @brief 撤销 RAM 会话，已擦写的镜像不会回滚。
 * @param transfer 待撤销事务。
 * @note 不擦写 Flash，不使能输出；已校验事务由上层拒绝撤销。
 */
void LoaderImage_Abort(LoaderImageTransfer *transfer);
/**
 * @brief 按清单身份整包核对已写入镜像。
 * @param config 只读目标配置。
 * @param identity 清单声明的镜像身份，含长度与 CRC32。
 * @return 向量、长度、CRC32 和内嵌描述符全部满足返回 true。
 * @note 有界扫描并喂狗，不得在 ISR 调用。
 */
bool LoaderImage_Check(const LoaderImageConfig *config, const LoaderImageIdentity *identity);
/**
 * @brief 现场读取 APP 内嵌描述符并整包核对，作为每次跳转前的唯一判据。
 * @param config 只读目标配置。
 * @return 描述符合法且向量、长度、CRC32 全部满足返回 true。
 * @note 不依赖任何持久提交记录；损坏或半写镜像返回 false，调用方留在 Loader。
 */
bool LoaderImage_CheckImage(const LoaderImageConfig *config);
#endif
