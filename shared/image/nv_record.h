#ifndef YG_NV_RECORD_H
#define YG_NV_RECORD_H
#include <stdbool.h>
#include <stdint.h>

/* 独占存储原语不授予功率使能；调用方负责生命周期与写边界，底层不依赖 HAL。 */
typedef enum
{
    NVR_OK,
    NVR_ARGUMENT,
    NVR_GEOMETRY,
    NVR_RANGE,
    NVR_IO,
    NVR_ERASED,
    NVR_INVALID,
    NVR_AMBIGUOUS,
    NVR_INCOMPLETE_PAIR
} LoaderRecordResult;

typedef struct
{
    void *context;
    uint32_t base, bytes, erase_bytes, program_bytes;
    bool (*read)(void *, uint32_t address, void *, uint32_t bytes);
    bool (*erase)(void *, uint32_t address, uint32_t bytes);
    bool (*program)(void *, uint32_t address, const void *, uint32_t bytes);
} LoaderFlashIo;
typedef struct
{
    uint32_t base, bytes;
} LoaderFlashRegion;
typedef struct
{
    uint32_t magic, commit_magic, kind, layout, version, payload_bytes;
} LoaderRecordFormat;
typedef struct
{
    uint32_t sequence, context, crc;
} LoaderRecordInfo;

/**
 * @brief 检查完整镜像记录的 CRC 与最终提交单元。
 * @param io 只读存储端口。
 * @param region 擦除页对齐的记录区。
 * @param format 唯一持久格式。
 * @param info 成功输出记录身份。
 * @return 记录状态；失败不授予任何写权限。
 */
LoaderRecordResult LoaderRecord_Inspect(const LoaderFlashIo *io,
                                        const LoaderFlashRegion *region,
                                        const LoaderRecordFormat *format,
                                        LoaderRecordInfo *info);
/**
 * @brief 校验记录后复制载荷，并对复制结果再次计算 CRC。
 * @param io 串行化的读取端口。
 * @param region 固定记录区。
 * @param format 只读格式。
 * @param out 载荷输出，失败时全部内容不可消费。
 * @param capacity 输出容量，单位字节。
 * @param info 成功返回记录信息。
 * @return 完整有效返回 NVR_OK，否则返回原因。
 */
LoaderRecordResult LoaderRecord_Read(const LoaderFlashIo *io,
                                     const LoaderFlashRegion *region,
                                     const LoaderRecordFormat *format,
                                     void *out,
                                     uint32_t capacity,
                                     LoaderRecordInfo *info);
/**
 * @brief 整页擦除后写入记录，提交单元最后写并读回。
 * @param io 仅授权固定记录页的写端口。
 * @param region 独占记录区。
 * @param format 记录格式。
 * @param sequence 非零镜像 token。
 * @param context 镜像启动状态。
 * @param payload 不可变 RAM 载荷，不得指向将被擦除的 Flash。
 * @return 完整提交返回 NVR_OK，失败不得原位补写。
 * @note 前台独占；失败也可能已写入完整记录，复位后必须重新读取。
 */
LoaderRecordResult LoaderRecord_Replace(const LoaderFlashIo *io,
                                        const LoaderFlashRegion *region,
                                        const LoaderRecordFormat *format,
                                        uint32_t sequence,
                                        uint32_t context,
                                        const void *payload);
#endif
