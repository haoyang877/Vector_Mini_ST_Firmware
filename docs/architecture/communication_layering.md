# 核心工程通信分层

更新：2026-10-09。适用v1.3.0；APP与Loader共用yg_protocol纯codec。
当前链路为CAN FD：29位扩展ID、FD+BRS、1M/5M；不兼容旧YI、UART或USB服务。

## 帧到业务的路径

```text
平台FDCAN → canfd_irq有界入队 → link前台收发
  → shared/protocol帧校验/分片 → protocol注册/路由
  → composition所有者适配器 → 主状态机/参数事务
状态所有者 → 一致快照 → status_adapter → 编码回复
```

RX ISR只复制输入；不处理业务、Flash或日志。前台有界处理队列与回复。
通信不持有电机对象可写指针、不计算控制算法、不自行打开PWM。

| 入口 | 当前职责 |
| --- | --- |
| [comm_hw.h](../../firmware/platform/api/comm_hw.h) | 硬件收发能力，HAL位于平台ports |
| [CAN接入](../../firmware/communication/can/) | 节点运行态、队列、链路服务 |
| [shared codec](../../shared/protocol/README.md) | CAN ID、公司头、CRC和分片；APP/Loader单份实现 |
| [APP协议](../../firmware/communication/protocol/) | 命令注册及请求/结果值对象 |
| [owner_adapter](../../firmware/app/composition/yg_protocol_owner_adapter.c) | 显式映射启动/清错/维护/参数请求 |
| [status_adapter](../../firmware/app/composition/yg_protocol_status_adapter.c) | 将同一时刻快照编码为状态回复 |
| [board_config](../../firmware/app/composition/board_config.c) | 绑定业务所有者、设备身份及Loader交接能力 |

## 保留的核心线路

| 命令 | 行为与限制 |
| --- | --- |
| 101 ENABLE/DISABLE | 不用enable=1代替普通启动；原低侧制动候选不扩大语义 |
| 107 CLEAR | 空payload，既有4B结果；条件恢复后回IDLE，不启动 |
| 110 STOP | 真实关相/撤销目标；请求受理与关相完成不同 |
| 116 SET_CONTROL | 只更新RUNNING同模式目标，不隐式使能或切模式 |
| 117 START_CONTROL | 已实现候选电流/速度/位置/VQ入口；零目标或当前位置启动，切模式先STOP |
| 120～122 参数 | 标量读取、停机写入及异步保存；revision和真实持久结果明确 |
| 160～162 维护 | LUT、电/机械零位、SAVE、DEFAULTS；原ticket查询/取消 |
| 56/57 升级交接/确认 | 仅配套LoaderApp；停机/资源归还/TX完成后交接，健康满足后确认 |

117线路模式为1/2/3/6，不能直接强转内部枚举。
GET_CAPS仍保守报告supported_modes=0，表示未发布已验收能力；不应据此称源码没有117接线。
同身份重试按有界记录处理；拒绝请求不能改变功率、模式或活动目标。
未提供跨复位电机启动重放保护；不要把Loader升级nonce/session当作电机启动身份。
0x0076运行会话、0x00A6组合计划已退役；齿槽/摩擦/电阻任务不再可执行。

字节权威见[启动v0.2](../protocols/yg_protocol_ordinary_start_wire_v0_2.md)、
[维护v1.2](../protocols/yg_protocol_calibration_wire_v1_2.md)、
[参数目录](../protocols/yg_protocol_parameter_catalog_v0_1.md)、
[升级v3](../protocols/yg_protocol_upgrade_wire_v3.md)。
普通独立APP与LoaderApp的升级能力不同；源码接线和离线检查不能代替实机联调。
