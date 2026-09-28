#include "param_store.h"
#include "flash.h"

/* 字节存储能力不依赖电机对象或业务参数服务。 */
bool param_store_read(void *record, uint32_t size_bytes)
{
    return flash_read_param(record, size_bytes);
}
bool param_store_write(const void *record, uint32_t size_bytes)
{
    return flash_write_param(record, size_bytes);
}
