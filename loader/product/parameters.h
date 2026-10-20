#ifndef LOADER_PARAMETERS_H
#define LOADER_PARAMETERS_H
#include "image_transfer.h"
/**
 * @brief 识别参数存储外形并读取 CAN 节点，不判定完整标定有效性。
 * @param node 成功时输出节点，失败或空白时保持原值。
 * @return 原格式识别、空白或不可读/不可信状态。
 * @note 仅前台只读扫描；生产身份不属于参数扫描范围。
 */
LoaderParameterState LoaderParameters_ReadNode(uint8_t *node);
#endif
