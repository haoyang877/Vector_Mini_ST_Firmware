#ifndef LOADER_SHA256_H
#define LOADER_SHA256_H
#include <stdbool.h>
#include <stdint.h>
/** 单一所有者的SHA256状态，不动态分配；只验证完整性，不提供身份认证。 */
typedef struct
{
    uint32_t h[8];
    uint64_t bytes;
    uint8_t block[64];
    unsigned used;
    bool finished;
} LoaderSha256;
/** @brief 初始化摘要状态。 @param hash 输出独占状态，NULL不操作。 */
void LoaderSha256_Init(LoaderSha256 *hash);
/**
 * @brief 增量添加数据，检查总位数溢出。
 * @param hash 已初始化且未结束的状态。
 * @param data 借用数据；bytes为0允许NULL。
 * @param bytes 本次字节数。
 * @return 合法更新返回true。
 */
bool LoaderSha256_Update(LoaderSha256 *hash, const void *data, uint32_t bytes);
/**
 * @brief 完成摘要并结束本状态。
 * @param hash 未结束的独占状态。
 * @param digest 输出32B大端摘要。
 * @return 成功返回true；同一状态不可再次结束。
 */
bool LoaderSha256_Finish(LoaderSha256 *hash, uint8_t digest[32]);
/**
 * @brief 计算单个有界缓冲的SHA256。
 * @param data 借用数据。
 * @param bytes 数据长度，单位字节。
 * @param digest 输出32B摘要。
 * @return 完整成功返回true。
 */
bool LoaderSha256_Digest(const void *data, uint32_t bytes, uint8_t digest[32]);
#endif
