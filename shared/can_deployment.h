#ifndef CAN_DEPLOYMENT_H
#define CAN_DEPLOYMENT_H

/** @brief 部署策略：1 为共线全经典 CAN，0 为隔离总线的历史 FD 兼容版本。
 * @note APP 与 Loader 必须用同一值重编译；不得通过运行时命令切换。
 * 共线版本禁用 FD 遥测和在线维护，Loader 保持 CAN 静默。
 */
#ifndef VECTOR_CAN_CLASSIC_ONLY
#define VECTOR_CAN_CLASSIC_ONLY 1
#endif

#if VECTOR_CAN_CLASSIC_ONLY != 0 && VECTOR_CAN_CLASSIC_ONLY != 1
#error "VECTOR_CAN_CLASSIC_ONLY must be 0 or 1"
#endif

#endif
