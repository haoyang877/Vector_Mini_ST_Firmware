#include "param_store.h"

#include "flash.h"

/* 参数区端口：薄封装板级参数记录读写；擦写顺序与校验语义保持不变。 */

void param_store_load(void)
{
    flash_read_param();
}

bool param_store_save(void)
{
    return flash_write_param();
}
