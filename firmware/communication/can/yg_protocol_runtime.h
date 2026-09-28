#ifndef YG_PROTOCOL_RUNTIME_H
#define YG_PROTOCOL_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化唯一CAN FD协议的软件状态和冻结节点配置，不启动硬件。
 * @param node_id 本机节点号；不得使用广播地址。
 * @return 协议端点初始化成功返回 true。
 * @note 仅在前台启动阶段调用一次；成功后由APP绑定真实业务，再调用Start。
 *       已成功初始化后再次调用返回 false，不清除队列或重新启动硬件。
 *       参数桥保存的是配置值；启动后修改参数桥不会改变 link 已冻结的地址。
 */
bool YgProtocolRuntime_Init(uint8_t node_id);

/**
 * @brief 在全部业务绑定完成后启动CAN FD硬件接收。
 * @return 软件已初始化且平台启动正常返回时返回true；未初始化或link不可用返回false。
 * @note 仅前台启动阶段调用；已启动时幂等返回true，不重新配置外设或清队列。
 *       平台启动失败由既有平台错误处理负责；本函数不恢复电机、不绑定默认业务。
 */
bool YgProtocolRuntime_Start(void);

/**
 * @brief 推进 CAN FD 协议并按秒限频恢复 bus-off。
 * @param now_ms 单调毫秒时钟，允许 uint32_t 回绕。
 * @note 只在前台调用；未Start不处理；一次最多处理8个输入和8个输出，无等待。
 */
void YgProtocolRuntime_Service(uint32_t now_ms);

#endif
