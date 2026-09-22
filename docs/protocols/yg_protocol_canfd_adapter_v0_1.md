# yg_protocol CAN FD 承载适配契约 v0.2

日期：2026-09-22。状态：CAN FD-only 运行入口和离线链路已接入，电机业务命令仍按服务目录继续扩展。

## 1. 线路基线

- CAN ID：29 位扩展，J1939 风格 priority/R/DP/PF/dst/src，当前 PF=`0xEF`。
- 帧格式：CAN FD、BRS、数据帧；仲裁段 1 Mbit/s，数据段 5 Mbit/s。
- 应用头：公司协议定义的 magic、version、flags、源/目的节点、type、sequence、长度、保留字段和 CRC8；payload 后为 CRC16。
- 字节序：应用字段小端；CAN 控制器 CRC 不重复放入应用 payload。
- 实时控制和位置/速度/电流合并反馈必须单帧完成；管理、诊断、标定表和升级才允许分片。

## 2. 代码边界

```text
FDCAN HAL → comm_hw.h → yg_protocol_canfd_irq → yg_protocol_link
                                      │                  │
                                      │                  ├─ frame_codec / CRC
                                      │                  ├─ fragment / transfer queue
                                      │                  ├─ registry / router
                                      │                  └─ motor / readonly services
                                      ▼
                               电机与参数服务
```

`comm_hw.h` 只描述能力：取一帧、提交一帧、启动 FDCAN。它不解释 type、sequence、payload 或业务错误。`YgProtocolCanfd_RxIrqHandler` 是唯一 RX FIFO0 入口，ISR 只做有限复制和帧元数据筛选。

## 3. 队列和处理上限

- RX 队列 8 个物理帧；TX 队列 40 个物理帧，可覆盖 1767 字节逻辑消息的 39 个分片并保留一个空槽。
- `YgProtocolLink_Service` 每次最多处理 8 个输入和 8 个输出，失败帧保留在队首。
- 队列不分配动态内存；满时丢弃最新项并递增统计。
- 物理帧填充为零，不计入应用 CRC；非法填充、ID、地址、长度、CRC 或 BRS 均拒绝。

## 4. 当前已接入的类型

| 类型 | 方向 | 当前行为 |
| --- | --- | --- |
| GET_INFO | 请求/回复 | 返回设备和版本页 |
| GET_CAPS | 请求/回复 | 返回 CAN FD、帧长和分片能力 |
| GET_MOTOR_STATE | 请求/回复 | 调用应用状态快照；启动后尚未刷新时返回 BUSY |
| MOTOR_STOP | 请求/回复 | 经电机停机服务执行；未绑定服务返回 UNSUPPORTED |
| MOTOR_DISABLE | 请求/回复 | 经电机停机服务执行；未绑定服务返回 UNSUPPORTED |

所有未注册类型都返回路由错误，不自动转到历史标准 CAN 编号。

## 5. 旧协议兼容边界

本版本不兼容旧标准 CAN 参数帧、旧 48 字节状态流、旧心跳超时和旧波特率切换命令。
这些实现、工程条目、测试和主机镜像已删除；上位机必须按本文件的 CAN FD 扩展帧、公司
应用头、CRC、请求/回复和序列号重新建立会话。旧帧即使使用相同节点号也不会被转换或
转发到新服务。

## 6. 后续接入约束

模式、位置/速度/电流目标、参数、标定、升级和多轴同步都必须实现为 `yg_protocol` service handler。handler 接收平台无关值对象，先校验单位、范围、权限、状态和 sequence，再调用电机/参数所有者；执行结果通过统一回复和状态快照确认。禁止在 handler 中访问 HAL、直接操作 FDCAN 或恢复旧协议兼容路径。

协议 ABI 变更必须同步更新版本号、黄金向量、主机镜像、离线测试和本文件。硬件回归先做只读命令，再做停机和状态反馈；释放固件前重新生成当前提交的构建哈希证据。
