#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdbool.h>

/**
 * @brief 从非易失参数区读取记录并装载运行参数。
 * @note 记录无效或校验失败时回退编译期默认值；不写回、不阻塞电机路径。
 *       语义与迁移前的板级参数区读取一致。
 */
void param_store_load(void);

/**
 * @brief 将当前参数写入非易失参数区。
 * @return 写入并校验成功返回 true；失败返回 false，由调用方负责故障上报。
 * @note 只在参数保存流程调用；擦写顺序与校验语义保持不变。
 */
bool param_store_save(void);

#endif
