# 电机通信与固件升级协议设计规范

版本：1.2.3 评审稿　　日期：2026 年 9 月 8 日

本版确认仅以 CAN FD 实现本通信协议：29 位扩展 ID，仲裁段 500 kbit/s、数据段 5 Mbit/s、BRS=1。全部控制、反馈、管理及升级报文采用公司统一的 16 B 帧头 + payload + 2 B CRC16。协议逻辑和硬件驱动保持分层，不因当前只实现一种链路而耦合到 MCU SDK。

控制目标和合并运动反馈均为 200 Hz（5 ms）。type=124 的反馈 payload 为 position:i32 + speed:i16 + iq:i16，共 8 B，同一快照；完整报文 26 B，填充后为一个 32 B CAN FD 帧。

当前有效消息 65 种：63 条请求、2 种上报（3 心跳/状态事件、124 合并运动反馈）。120～123 为退出当前协议的历史编号，不实现、不发送、不复用。登记记录仍为 69 项。详见[命令总表](motor_command_catalog.md)及 motor_command_catalog.json。控制命令合并仍是[独立建议稿](motor_command_consolidation_review.md)，尚未采用。

本文件为设计规范，当前固件未据此完成实现、编译和硬件验证。历史经典 CAN、RS485、TCP 适配说明及裸 8 B 实时帧不属于本版实现范围。

## 1 公司帧框架与扩展边界

设计依据为已读取原文的[公司通信协议](https://wcnxj7iyqdkp.feishu.cn/wiki/MM14wIJF7i8irjkx448cHu31nLb)及[消息类型定义](https://wcnxj7iyqdkp.feishu.cn/wiki/DXV4wohO0ioGbskmFAFct8wFngd)。原文核对记录见 company_protocol_source_readback.md。本版按公司 CAN FD 框架实现，不再使用压缩帧头或免应用 CRC 的例外。

| 公司框架要求 | 本版实现约定 |
| --- | --- |
| 固定 16 B 帧头 | magic/version/flags/src/dst/type/seq/len/reserved/CRC8，偏移保持原定义 |
| payload 后 2 B CRC16 | 覆盖帧头与当前 payload；每个单帧/分片均有独立校验 |
| 小端与固定字段 | 不使用大端 float，按显式字段编解码 |
| CAN FD 最多 64 B | 16+46+2=64，单帧 payload≤46 B |
| 实时控制必须单帧 | 目标、停止、保活、同步提交/取消、周期反馈和心跳不分片 |
| 长消息使用公司分片位 | flags[7:6] 标识单/首/中/尾，seq_id 表示片序，len 为当前片 payload 长度 |
| 统一节点及 type | 原节点编号不变；通用 0～99，电机 100～199；type 在公司帧头内 |
| 转发不做字段翻译 | 若中枢转发已编码报文，保留完整报文字节，仅将 src/dst 映射到 CAN ID，不另造电机业务帧 |

公司原文对 ACK 标志、CRC16 名称和头部对齐存在相互冲突的描述。本版明确采用 bit5=ACK_REQ、bit4=RESPONSE，CRC16 参数为 poly=0xBAAD，len 偏移为 10；字段布局保持不变。第 12.1 节采用整条消息重传，明确不实现原文提及的选片补发。这些是需要同步登记的语义澄清，不能声称公司已确认这些勘误或本版新增命令编号。

J1939 仅用于参考 29 位 ID 的寻址与优先级结构，不使用 J1939 TP，不宣称实现 J1939-22 或完整 J1939 协议栈。

## 2 分层与数据路径

```text
CAN FD 控制器 / 收发器
    ↓
BSP 资源绑定 + HAL Port
    ↓ comm_hw（项目硬件能力）
CAN FD Transport（队列、ID、帧收发、截止时间）
    ↓
公司 Protocol（帧头、CRC、固定字段、分片重组、会话）
    ↓
Command Router → mc_command_t / 服务请求对象
    ↓
电机控制 / Safety / 参数 / Telemetry / Loader 升级服务
```

通信协议禁止直接写 PID、FOC、PWM 或硬件寄存器；HAL Port 不承载命令策略、故障策略或升级状态机。Loader 与 APP 是独立固件域；PRODUCT/FACTORY/DEBUG 共用通信核心和命令注册，支持差异通过 GET_CAPS 公布。只实现 CAN FD 不意味着在业务代码中直接引用 FDCAN 控制器或 Vendor HAL。

### 2.1 公共传输接口

| 接口 | 责任 |
| --- | --- |
| transport_send(peer,message,traffic_class,deadline) | 将消息加入有界队列，返回 QUEUED/BUSY/TOO_LARGE/LINK_DOWN |
| transport_on_message(peer,src,dst,type,flags,seq,payload) | 交付完整、校验成功、已完成重组的逻辑消息 |
| transport_get_caps(link) | 读取有效长度上限、已配置速率和时序能力 |
| transport_on_event(peer,event) | 报告链路启动、关闭、bus-off、发送完成或失败 |
| storage_read/write/erase(slot,offset,length) | 升级服务通过受限存储接口操作镜像槽，与通信驱动解耦 |

traffic_class 为 STOP、CONTROL、FEEDBACK、MANAGEMENT、UPDATE，对应第 4 节优先级。QUEUED 只表示数据已复制到队列，不代表已上总线或执行成功。接收 payload 仅在回调期间有效，异步任务复制到有界自有存储。不从通信 ISR 擦写 Flash、等待阻塞发送或运行复杂业务。

### 2.2 控制权与链路恢复

每台设备全局仅允许一个控制者；peer 包含逻辑源地址、CAN FD 接口和本地链路代次。CAN FD 不建立 TCP 式连接，CONNECTED/DISCONNECTED 仅为本地链路状态事件。响应发往请求 src；周期反馈发往当前控制者，无控制者时按产品默认目标配置发送。

bus-off、设备复位或控制会话失效后，清除旧队列与 lease；恢复后读取 boot_id、重新申请控制、预装安全目标并显式使能，不自动续接旧目标。升级恢复重新 OPEN、提交相同镜像清单并查询持久进度，不依赖断线前 RAM 偏移。

同网节点地址必须唯一。中枢若代理上层逻辑节点，必须在同一地址管理规则中占用该逻辑地址；转发公司帧时 CAN SA/DA 与帧头 src/dst 保持一致。协议不实现跨链路字段翻译或自动跨接口升级切换。

## 3 基础编码与节点

所有多字节整数均小端。i8、i16、i32、i64 为二进制补码，u8/u16/u32/u64 为无符号固定宽度整数。字段按表紧密排列，无隐式填充。除另有说明，保留字段发送 0，接收非 0 返回 BAD_FIELD。固定长度消息必须严格匹配长度。

运行期时间间隔使用本地单调时钟和可正确处理计数回绕的差值计算。只有明确允许协商的字段可由远端改变；不通过目标命令直接修改安全限值。表中 Q 后/R 后表示需要额外加公共前缀；写“共 N B”时 N 是包含 Q 或 R 的完整 payload 长度。APPLY_TASK_RESULT 中读取参数版本的方式为 READ_PARAM(param_id=0)，见第 8.4 节。

原文 len 在偏移 10，并非自然 4 字节对齐。实现必须使用显式小端读写或安全 memcpy，不能将收包缓冲强转为未对齐结构体。不得沿用旧 CAN 的大端 float 编码。

| node_id | 节点 |
| --- | --- |
| 0x01 | MT6897 上层平台 |
| 0x02 | RV1126B 下层中枢 |
| 0x03 / 0x04 | 左 / 右底盘电机 |
| 0x05 / 0x06 / 0x07 | ROW / PITCH / YAW 电机，ROW 保留公司原拼写 |
| 0x08 / 0x09 | BMS / 回充控制器 |
| 0xFF | 广播目的地址，禁止作为源地址 |

首版固定地址，地址在设备未使能时离线配置；不实现自动地址声明。0x00、0xFE 和未分配地址不得作为本系统正常节点。设备唯一 UID 与逻辑 node_id 分离；产线维护 UID 到地址映射，检测到重复地址必须禁止电机使能。接入开放 J1939 网络前，另行补齐 NAME、地址声明和网络管理互操作验证。

## 4 CAN FD 标识符与承载分类

采用 J1939 的 29 位 PDU1 ID 布局，静态地址；PS 为目的地址，不属于 PGN，参考 [Linux J1939 文档](https://www.kernel.org/doc/html/latest/networking/j1939.html)。所有业务统一使用 PGN=0x00EF00，消息类型只由公司帧头 type 定义。

| ID 位 | 字段 | 值 |
| --- | --- | --- |
| 28～26 | Priority | 0～7，0 最高 |
| 25 / 24 | R / DP | 均为 0 |
| 23～16 | PF | 0xEF |
| 15～8 | PS/DA | 目的 node_id，FF 为广播 |
| 7～0 | SA | 源 node_id |

```text
CAN_ID = (priority << 26) | (0xEF << 16) | (dst_id << 8) | src_id
```

| 类别 | Priority |
| --- | --- |
| MOTOR_STOP / SYNC_ABORT | 0 |
| 周期目标 / MOTION_KEEPALIVE / SYNC_COMMIT | 2 |
| MOTION_FEEDBACK / 状态故障事件 | 3 |
| 周期 HEARTBEAT | 5 |
| 管理和升级请求/响应/分片 | 6 |

主机 02→电机 03 速度目标 ID=0x08EF0302；电机 03→主机 02 合并反馈 ID=0x0CEF0203；主机 02 广播停止 ID=0x00EFFF02；管理请求 ID=0x18EF0302。上述 ID 对应的数据区全部是公司完整单帧或完整分片，绝不发送裸 payload。

链路仅接受 IDE=1、RTR=0、FDF=1、BRS=1 的本协议报文，R/DP=0、PF=EF，地址与第 3 节匹配。公司帧头 src/dst 必须分别等于 SA/DA，不一致拒绝；命令类型与 Priority 按本表校验。PF=EC/EB、PGN=FF7C、标准帧、远程帧、经典 CAN 帧及非法物理长度均拒绝，不触发业务动作。

## 5 公司完整报文

| 字节偏移 | 类型 | 字段 | 规定 |
| --- | --- | --- | --- |
| 0 | u16 | magic | 0xA55A，线上 5A A5 |
| 2 | u8 | version | 0x01 |
| 3 | u8 | flags | 见下表 |
| 4 / 5 | u8 / u8 | src_id / dst_id | 与接收路径绑定地址一致；CAN 上须与 SA/DA 相符 |
| 6 | u16 | type | 同一请求与响应使用相同 type |
| 8 | u16 | seq_id | 未分片事务序号；重传不变；分片片序见第 12.1 节 |
| 10 | u32 | len | 当前 payload 字节数 |
| 14 | u8 | reserved | 0 |
| 15 | u8 | header_crc8 | 覆盖字节 0～14，不含自身 |
| 16 | byte[len] | payload | 按 type 定义 |
| 16+len | u16 | frame_crc16 | 覆盖完整帧头含 CRC8 和 payload，小端存储 |

整帧长度严格等于 18+len。CRC8：width=8，poly=0x9B，init=0，xorout=0，refin=false，refout=false。CRC16：width=16，poly=0xBAAD，init=0xFFFF，xorout=0，refin=false，refout=false。算法采用逐字节先异或到寄存器高位、逐位左移、移出最高位为 1 时异或多项式，无额外末尾补零。本 CRC16 是公司自定义参数，不称 CRC16 CCITT。

| flags 位 | 定义 | 本版本使用 |
| --- | --- | --- |
| 7～6 | 公司分片 | 00 单帧、01 首片、10 中片、11 尾片；仅 CAN FD 启用 |
| 5 | ACK_REQ | 请求需业务应答为 1；响应必须为 0 |
| 4 | RESPONSE | 请求 0，业务响应 1 |
| 3 | REPORT_REQ | 仅设置上报周期请求使用 1 |
| 2 | RETRY | 原请求重发为 1；seq 和 payload 不变 |
| 1～0 | 保留 | 0 |

flags 0x20 表示需应答请求，0x24 表示重传请求，0x10 表示响应，0x28 表示需应答的上报配置请求。响应不再触发应答。该定义消除原文 bit5 同时描述“请求确认”和“ACK”的歧义，必须在公司主协议同步勘误。

CAN FD 使用第 12.1 节公司分片。电机端点暂保留逻辑 payload 上限 1767 B，具体设备可进一步缩小；此值是有界内存配置上限，不再与 J1939 TP 绑定。该上限不改变单帧 46 B 限制，也不要求为每种消息预分配最大缓冲。

## 6 收包与业务应答边界

接收流程：校验 CAN FD 格式及地址 → 检查最小物理长度 → 校验 magic/version/保留位/头 CRC8 → 校验当前 len≤46 与 DLC/填充 → 校验 CRC16 → 校验 type、优先级和方向 → 单帧交付或进入有界重组 → Command Router。

实时消息只允许 flags[7:6]=00；长管理报文按第 12.1 节重组，完整后才执行一次业务。广播仅允许 MOTOR_STOP、SYNC_COMMIT、SYNC_ABORT，均不请求业务 ACK；其他广播业务请求拒绝。

CAN 硬件 ACK 仅表示物理接收，不代表应用校验或执行成功。需要确认的命令等待同 type、关联 request_id 的 RESPONSE，结果见 R.result；ACCEPTED 表示长操作开始，完成需查询。升级 WRITE 的 OK 必须在 Flash 写入和读回成功后返回。

## 7 请求响应和能力发现

除紧凑实时消息与心跳外，管理命令使用以下公共前缀。字段顺序即线上顺序，消息表中的 Q、R 必须展开后编码。

| 前缀 | 布局 | 大小 |
| --- | --- | --- |
| Q | session_id:u32, request_id:u32 | 8 B |
| R | session_id:u32, request_id:u32, result:u16, detail:u16 | 12 B |

session_id=0 用于无会话只读请求和申请会话。控制会话与升级会话使用非 0 标识，由设备分配，绑定源地址、连接、设备 boot_id 和会话类型；不是鉴权凭证。request_id 在会话内递增，0 不使用，回绕前重建会话。无会话请求每个对端仅允许一条在途。

写操作按 (boot_id, peer, session_id, type, request_id) 去重。相同键和相同 payload 返回原结果；相同键但 payload 不同返回 CONFLICT。每个活动会话至少保留最近一条请求及结果，之前的 request_id 返回 STALE_REQUEST，不执行。首版停等，不允许并发提交后依靠无限结果缓存。复位后旧会话无效，升级通过镜像身份和持久化检查点恢复。

业务错误响应只带 R；成功响应带 R 和表中附加数据。ACCEPTED 表示长操作已开始，此响应只带 R，detail=0；使用 UPDATE_QUERY 或设备状态查询最终结果。只有 result=OK 才代表该条命令要求的操作已经完成。接收 CRC 错误不回复，防止基于不可信地址产生回复风暴。

| result | 名称 | 含义 |
| --- | --- | --- |
| 0 / 1 | OK / ACCEPTED | 完成 / 已受理未完成 |
| 2 / 3 / 4 | UNSUPPORTED / BAD_LENGTH / BAD_FIELD | 不支持 / 长度错 / 字段错 |
| 5 / 6 / 7 | BAD_STATE / BUSY / DENIED | 状态不允许 / 资源忙 / 控制权不符 |
| 8 / 9 / 10 | SESSION_EXPIRED / STALE_REQUEST / CONFLICT | 会话失效 / 旧请求 / 重复内容冲突 |
| 11 / 12 / 13 | RANGE / IMAGE_MISMATCH / VERIFY_FAILED | 越界 / 镜像不适配 / 验证失败 |
| 14 / 15 / 16 | STORAGE_ERROR / OFFSET_ERROR / TIMEOUT | 存储错 / 偏移错 / 操作超时 |
| 17 | FAULT_ACTIVE | 故障尚未解除 |

detail=0 表示无补充。BAD_FIELD/RANGE 时 detail 是出错 payload 字节偏移；OFFSET_ERROR 的正确偏移通过 UPDATE_QUERY 获取；STORAGE_ERROR 的厂商细节由诊断页读取，不在 result 中混入 MCU HAL 枚举。

| type 十进制及十六进制 | 名称 | 请求 | OK 响应附加内容 |
| --- | --- | --- | --- |
| 0 / 0x0000 | TEST | Q + cookie:u32 | cookie:u32 |
| 1 / 0x0001 | GET_INFO | Q + page:u16 + reserved:u16 | page:u16 + schema:u16 + 页面内容 |
| 2 / 0x0002 | GET_CAPS | Q + page:u16 + reserved:u16 | page:u16 + schema:u16 + 页面内容 |
| 3 / 0x0003 | HEARTBEAT | 无请求；固定上报 | boot_id:u32, uptime_s:u32, state:u8, mode:u8, reserved:u16, faults:u32，共 16 B |
| 4 / 0x0004 | ACQUIRE_CONTROL | Q + expected_boot_id:u32 + watchdog_ms:u16 + reserved:u16 | lease_tag:u16, watchdog_ms:u16, boot_id:u32 |
| 5 / 0x0005 | RELEASE_CONTROL | Q | 无 |

TEST 保留原文编号和需 ACK 语义，cookie 是本规范补充的 payload。新增编号均为本规范拟分配。ACQUIRE_CONTROL 成功时 R.session_id 返回新控制会话，其余申请响应回显请求标识。设备只允许一个控制者；只有 DISABLED 且无故障可申请，重复申请需先释放旧会话。RELEASE_CONTROL 先禁止功率输出，再销毁会话。

控制者必须先读取当前 boot_id 再申请控制，expected_boot_id 不匹配返回 SESSION_EXPIRED。复位及控制权交接时双方清除驱动和软件队列，设备至少保持 200 ms 未使能隔离期后才接受新申请。未使能控制会话若 30 s 没有合法管理请求或新目标则释放；已使能时使用运动看门狗。任何故障进入 FAULT 时撤销会话和预装目标。

GET_INFO 页 schema=1。page0 内容为 uid:byte[12], product_id:u32, hw_rev:u16, boot_api:u16, app_version:u32, boot_version:u32, boot_id:u32，共 32 B；完整响应 48 B。UID 是产品注册的 96 位唯一标识。page1 内容为 flash_bytes:u32, ram_bytes:u32, max_image_bytes:u32, image_offset_alignment:u16, erase_unit_bytes:u16，共 16 B。未知页返回 UNSUPPORTED。

GET_CAPS 页 schema=1。page0 内容为 features:u32, supported_modes:u16, max_full_frame:u16, max_block_data:u16, write_alignment:u16, default_watchdog_ms:u16, max_watchdog_ms:u16，共 16 B。features 位 0～3 为退出能力保留位，本版固定为 0；位 4=签名升级、5=掉电断点、6=A/B 回滚、7=电流模式、8=速度模式、9=位置模式、10=CAN FD、11=CAN FD 分片、12=本版固定单位的 i32/i16/i16 合并反馈；其余 0。supported_modes 位 n 表示模式 n。max_full_frame 包含 18 B 开销，page0 表示本设备当前固件的业务能力。位 10/11/12 为本版必需能力；未实现或未公布的节点不得按本版进入运动控制，不自动降级到旧协议。

page1 内容为 max_motor_rad_milli:i32, max_speed_rad_s_milli:i32, max_iq_mA:i32, stop_policy:u8, max_axes:u8, reserved:u16，共 16 B，表示已配置的应用限制；位置下限另由参数读取。仅提供设备实际实现的能力，禁止通过设置能力位提前宣称支持。

page2 返回本次请求所在路径的有效能力，内容为 profile_id:u8, link_flags:u8, max_frame_payload:u16, max_message_payload:u16, effective_block_data:u16, nominal_bps:u32, data_bps:u32, min_target_period_ms:u16, reserved:u16，共 20 B。profile_id 固定为 2（CAN FD，保留历史编号不重排），link_flags 位 0=主动上报、1=公司分片、3=BRS 均置位；位 2 和其余位为 0。max_frame_payload 固定为 46，max_message_payload 是实际实现的有界重组上限。

max_message_payload 为本 CAN FD 端点实际实现的完整逻辑 payload 上限；effective_block_data≤max_message_payload−16，且不超过存储服务限制。UPDATE_OPEN 返回此有效块长。nominal_bps=500000、data_bps=5000000；min_target_period_ms 是已验收的最小目标周期，0 表示运动时序未验收，只开放管理和升级。

GET_CAPS page2 并不发现线路波特率。CAN FD 仲裁段、数据段速率和 BRS 由安装配置预先一致；设备识别后才能查询业务上限，不能先通过未知速率链路协商该链路的速率。

1.2 新增 page3，schema=1，内容共 32 B：advanced_features:u32, calibration_mask:u32, identification_mask:u32, max_trajectory_segments:u16, max_result_items:u16, timer_resolution_us:u32, max_start_jitter_us:u32, oscillator_bound_ppm:u16, reserved:u16, min_sync_lead_us:u32。advanced_features 位 0=阻抗基础目标、1=阻抗完整目标、2=缓存梯形轨迹、3=回零、4=设置坐标原点、5=编码器等标定、6=参数辨识、7=定时组启动、8=四时间戳校时；其余为 0。page3 仅公布当前固件实际实现的高级能力，支持完整阻抗时公布位 1。未知页面返回 UNSUPPORTED，旧固件不得因未知能力页改变运行状态。

calibration_mask 和 identification_mask 对应第 8.10、8.11 节类型位。timer_resolution_us 是任务执行定时器分辨率；max_start_jitter_us 是经该固件与硬件配置验证的最大本地启动抖动；oscillator_bound_ppm 是适用温度范围内本地时钟的最大漂移界。没有可靠界限时 advanced_features 位 7 必须为 0。同步最短提前量还必须结合链路最坏延迟，不仅取定时器分辨率。

## 8 实时电机控制

### 8.1 模式状态与单位

| 枚举 | 线上定义 |
| --- | --- |
| mode | 0=无控制模式，1=Iq 电流，2=速度，3=位置，4=位置阻抗，5=缓存轨迹，6=回零，7=标定，8=参数辨识；9～15 保留 |
| state | 0=BOOT，1=DISABLED，2=READY，3=ENABLED，4=STOPPING，5=FAULT，6=UPDATING；7～15 保留 |

线上枚举独立于固件内部枚举。位置和速度均以电机机械轴为基准，正方向和机械零点由设备参数约定；不得混用电角度、减速器输出轴角度和 r/s。目标和详细状态中的位置、速度、Iq 均为 i32，单位依次为 0.001 rad、0.001 rad/s、0.001 A。新增 type=124 仅将周期反馈的速度和 Iq 压缩为 i16，保持上述单位，范围及无效值见第 9.1 节。0x80000000 在 i32 测量中表示无效，禁止作为控制目标。所有目标还必须通过设备配置的物理范围校验。

“电流控制”指 q 轴电流目标，不直接声称为 N·m 转矩控制。需要转矩接口时另行定义 Kt、齿比、效率和标定有效性；本版不分配未经标定的转矩命令。位置多圈累计超出编码或配置范围前进入故障，不能静默整数回绕。

### 8.2 目标单帧

下表 8 B 仅为目标业务 payload；必须按第 10 节增加公司帧头和 CRC，再放入一个 CAN FD 帧。物理长度和 ID 优先级属于通信模块，不进入电机控制算法。

| 字节偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | u8 | type，103 / 104 / 105 |
| 1 | u8 | command_seq |
| 2 | u16 | lease_tag |
| 4 | i32 | target |

| type | 名称 | target 单位 | 所需 mode |
| --- | --- | --- | --- |
| 103 / 0x67 | SET_IQ | mA | 1 |
| 104 / 0x68 | SET_SPEED | 0.001 rad/s | 2 |
| 105 / 0x69 | SET_POSITION | 0.001 rad | 3 |

仅单播，SA 必须等于控制者，lease_tag 必须匹配。lease_tag 为控制会话的 16 位短标识，同一 boot_id 下不能复用；耗尽时拒绝新会话直到维护复位。设备和网关复位、释放控制权或切换控制者时必须清除所有待发目标；网关目标队列 TTL 最多 20 ms。短标识和序号用于防误投递与旧队列，不是密码学认证，不能防恶意 CAN 注入。

每个控制会话共享一个 8 位 command_seq，跨三种目标命令递增。首条合法目标接受任意起始序号；此后 delta=(new-last) mod 256，1～127 为新，0 为重复，128～255 为旧。重复和旧目标都不刷新看门狗。目标中断超过看门狗后销毁控制会话，不能以序号回绕恢复运行。

接收过程必须原子校验来源、lease、模式、序号、范围和当前状态。通过后在下一控制周期边界应用，并更新 last_applied_seq；范围错误整条拒绝，不悄悄截断。无需业务 ACK，通过状态回显序号和读取详细状态确认。发送目标绝不隐式使能。

READY 状态允许预装目标但不输出功率。模式 1～4 使能前必须有 50 ms 内的新目标，并且速度模式初始目标为 0、电流模式初始目标为 0、位置及阻抗模式初始目标与当前位置误差不大于 configured_enable_position_window；完整阻抗目标的速度和前馈电流还必须为 0。模式 5～8 使用第 8.6 节的任务准备和保活条件。运行中的变化受已配置电流、速度和加速度限幅约束。

### 8.3 软件停止单帧

type=100 / 0x64，Priority=0。布局为 type:u8, reason:u8, stop_counter:u16, signature:u32；signature 固定 0x53544F50，线上为 50 4F 54 53。reason：0=上位机请求、1=操作员急停、2=系统故障、3=通信故障，其他拒绝。stop_counter 仅用于记录，重复停止仍执行。

接受配置的停止源 0x01、0x02；目的可为本机或 FF，不要求有效控制 lease。收到后立即禁止 PWM 功率输出，撤销控制会话，置软件停止故障位；清故障后仍保持 DISABLED。广播不回复 ACK，以状态上报确认。此消息不保证机械制动、不保证设备断电，也不替代硬件急停或 STO。

### 8.4 管理命令

下表全部走公司完整报文，ACK_REQ=1；使用 CAN FD 公司单帧或分片承载。需要控制权的命令在 Q.session_id 中携带控制会话。

| type | 名称 | Q 后字段 | OK 响应附加内容与条件 |
| --- | --- | --- | --- |
| 101 / 0x65 | MOTOR_ENABLE | 无 | 无；READY 且预装目标有效才进入 ENABLED |
| 102 / 0x66 | MOTOR_DISABLE | 无 | 无；先禁止功率输出，再响应，进入 READY 并清目标 |
| 106 / 0x6A | SET_MODE | mode:u8, reserved:byte[3] | 无；仅 DISABLED 或 READY，清除旧目标 |
| 107 / 0x6B | CLEAR_FAULT | mask:u32 | remaining_faults:u32；物理原因消失且未使能，session 可为 0 |
| 108 / 0x6C | GET_MOTOR_STATE | 无 | 第 9 节详细状态，共 32 B |
| 109 / 0x6D | SET_REPORT | pos_ms:u16, speed_ms:u16, iq_ms:u16, reserved:u16 | 实际采用的同 8 B 配置；flags=0x28 |
| 110 / 0x6E | READ_PARAM | param_id:u16, reserved:u16 | param_id:u16, value_type:u8, reserved:u8, value:i32 |
| 111 / 0x6F | WRITE_PARAM | param_id:u16, value_type:u8, reserved:u8, value:i32 | 无；仅未使能，写 RAM 暂存配置 |
| 112 / 0x70 | SAVE_PARAM | expected_revision:u32 | new_revision:u32；掉电安全提交成功才 OK |

READ_PARAM、GET_MOTOR_STATE 允许 session=0。WRITE_PARAM、SAVE_PARAM、SET_MODE、SET_REPORT、ENABLE、DISABLE 需要控制会话；MOTOR_DISABLE 可在 READY/ENABLED 幂等执行。SET_MODE 后进入 READY；未支持模式返回 UNSUPPORTED。CLEAR_FAULT 不清除仍存在的物理故障，remaining_faults 非零时 result=FAULT_ACTIVE，此时响应仍附 remaining_faults，这是错误响应只带 R 的明确例外。

参数最小注册表：0=只读活动参数 revision，1=最大速度，2=最大 Iq，3=最大加速度，4=位置下限，5=位置上限，6=使能位置窗口，7=目标超时策略，8=受控停止最长时间。1～6 使用 value_type=1 即 i32，单位依次为 0.001 rad/s、mA、0.001 rad/s²、0.001 rad、0.001 rad、0.001 rad；0、7、8 使用 value_type=2 即 u32，单位分别为版本计数、枚举、ms。线上 value 字段均为 4 字节，按 value_type 解释。活动配置被 CONFIG、设置原点、结果应用或保存更新时 revision 递增；回绕前禁止新写事务并进入维护流程。高级任务自己的 config_revision 是对象版本，由对应 CONFIG/COMMIT 返回，不要求等于活动参数 revision。

参数 7：0=立即功率禁止；1=配置减速度受控停止后功率禁止。默认 0；策略 1 需实机证明电源与制动条件满足，且停止最长时间到达必须功率禁止。参数约束采用事务暂存：WRITE_PARAM 只改候选配置，SAVE_PARAM 整体验证相互约束并原子提交，成功才更新活动配置；失败不部分生效。READ_PARAM 返回活动配置，不返回未提交暂存值。expected_revision 防止旧客户端覆盖新配置；不匹配返回 CONFLICT。参数保存后清除目标，保持未使能。

### 8.5 运行状态机与超时

```text
上电自检 -> DISABLED -> 申请控制 -> SET_MODE -> READY
READY -> 预装安全目标 -> ENABLE -> ENABLED
ENABLED -> DISABLE -> READY
任何运行状态 -> 故障或软件停止 -> FAULT
FAULT -> 原因消失并 CLEAR_FAULT -> DISABLED
DISABLED -> 升级准备 -> UPDATING -> 复位 -> DISABLED
```

申请控制不会自动改变控制模式或使能输出。默认目标周期 5 ms（200 Hz），每轴只发送当前模式对应的一种目标；目标数值不变也应递增序号发送新周期报文。看门狗默认 50 ms（10 个目标周期），申请可配置 30～200 ms；这是失联保护时限，不是允许的正常控制延迟。看门狗只由通过全部校验的新目标刷新，心跳、参数读取和重复包均不刷新。超时执行参数 7 的动作，置目标超时故障并撤销控制权。受控停止期间 state=STOPPING，结束后进入 FAULT。

心跳默认 1 Hz，连续 3 次未收到由上层标记离线并发起系统停止；这是在线监测，不能替代 50 ms 运动看门狗。CAN bus-off、控制队列长期溢出、关键传感器无效必须触发本地保护；总线恢复不自动使能。

### 8.6 高级任务公共规则

新增命令沿用 Q 和 R。除明确标记为实时的 113、131、132、183、184 外，全部为单播管理命令，ACK_REQ=1，使用 CAN FD 公司完整单帧或分片传输。配置、准备、启动、终止、结果应用、校时写入均需当前控制会话；状态及结果读取可使用 session=0，只读不获得控制权。

一个设备同时只有一个高级任务，任务种类 kind：1=轨迹、2=回零、3=标定、4=辨识，分别要求 mode=5、6、7、8。job_id 是控制者分配的非零 u32，新 ARM 的 job_id 必须大于本 boot 已接受的最大 job_id，重传的同一 ARM 仍按 request_id 幂等确认；回绕前进入维护复位。状态查询 job_id=0 表示读取当前或最近任务，返回真实 job_id，供新客户端确定下一编号；无任务时返回 EMPTY 和 job_id=0。任务对象绑定 boot_id、控制者、kind、job_id 和配置版本，不以 CAN 片序作为身份。

job_state：0=EMPTY、1=PREPARED、2=ARMED、3=RUNNING、4=STOPPING、5=DONE、6=ABORTED、7=FAILED。MOTION_ARM 在 READY 创建 PREPARED 任务；MOTOR_ENABLE 校验准备条件后进入 ENABLED，任务成为 ARMED，但此时不推进轨迹、不搜索原点、不施加标定激励。只有对应 START 或有效 SYNC_COMMIT 到期后才能执行。

| type 十进制及十六进制 | 命令 | 请求布局 | 成功结果 |
| --- | --- | --- | --- |
| 113 / 0x0071 | MOTION_KEEPALIVE | type:u8, seq:u8, lease_tag:u16, job_id:u32，共 8 B | 无 ACK；仅刷新匹配任务的运动看门狗 |
| 114 / 0x0072 | MOTION_ARM | Q + kind:u8, reserved:u8, flags:u16, job_id:u32, object_id:u32, config_revision:u32，共 24 B | R + job_id:u32, initial_position:i32 |

flags 固定 0。轨迹 object_id=已提交 traj_id，config_revision=轨迹提交返回的版本；回零、标定和辨识 object_id=0，config_revision 为对应 CONFIG 返回的版本。准备时冻结该配置，不允许执行期间替换。设备至少保留最近任务终态及结果到下一次 MOTION_ARM 或复位；新任务准备会使旧结果不可再读，主机应先读取或应用旧结果。

KEEPALIVE 在 PREPARED、ARMED、RUNNING 有效，使用独立的 8 位保活序号，按 delta=1～127 判断新包，重复不刷新。默认周期 5 ms（200 Hz），超时沿用控制会话 watchdog_ms；使能前必须已有 watchdog_ms 内的合法保活。它只适用于 mode 5～8，不能替代 mode 1～4 的新目标，也不修改 last_applied_seq。原文“仅目标刷新运动看门狗”在 mode 5～8 的明确扩展就是此命令。

ARMED 待启动阶段，轨迹保持准备位置；回零和标定/辨识不施加运动激励，具体功率级是否开启取决于安全静止保持配置。超时、故障、MOTOR_STOP 或通信中断均终止任务并撤销控制权。MOTOR_DISABLE 禁止输出并使任务 ABORTED；不能随后使能自动继续旧任务。普通 START 在 ENABLED 且任务 ARMED 时接受，返回 ACCEPTED 表示已开始，最终结果通过查询获取。

主动 ABORT 采用已配置停止策略，回 ACK=ACCEPTED 后可处于 STOPPING；查到 ABORTED 才表示停止结束。正常任务完成也停止运动并禁止输出，电机回 READY，任务保留 DONE；仍需独立保持制动的产品必须使用已有机械保持措施和相应产品策略，不能假定断 PWM 等于抱闸。FAILED 撤销控制权并进入 FAULT。查询和结果读取不刷新运动看门狗。

高级状态查询与 UPDATE_QUERY 一样是只读旁路，不推进有副作用请求的去重水位，不覆盖 START/ABORT/保存结果缓存。每个对端一次最多一个只读查询在途。所有长操作的业务 ACK 都区分 ACCEPTED 和 OK，不能在受理时报告完成。

### 8.7 阻抗控制

mode=4 定义电流域位置 PD 阻抗。基础关系为 iq_cmd = kp×(p_ref−p_meas) + kd×(v_ref−v_meas) + iq_ff，并受设备电流与运动限制约束。kp 单位 A/rad，kd 单位 A·s/rad；它不是直接以 N·m 为单位的转矩阻抗。

| type | 命令 | 请求布局 | 成功响应 |
| --- | --- | --- | --- |
| 130 / 0x0082 | IMPEDANCE_CONFIG | Q + kp:i32, kd:i32, iq_limit:i32, speed_limit:i32, accel_limit:i32，共 28 B | R + config_revision:u32 |
| 131 / 0x0083 | SET_IMPEDANCE_TARGET | type:u8, seq:u8, lease_tag:u16, position:i32，共 8 B | 无 ACK；基础位置目标 |
| 132 / 0x0084 | SET_IMPEDANCE_TARGET_FULL | type:u8, seq:u8, lease_tag:u16, position:i32, speed:i32, iq_ff:i32，共 16 B | 无 ACK；完整位置速度前馈目标 |
| 133 / 0x0085 | GET_IMPEDANCE_STATE | Q | R + p_ref:i32, v_ref:i32, iq_cmd:i32, position_error:i32, limit_flags:u16, reserved:u16，共 32 B |

kp/kd 的编码分辨率为 0.000001 A/rad 和 0.000001 A·s/rad；Iq 为 mA，速度为 0.001 rad/s，加速度为 0.001 rad/s²。增益必须非负，限值必须为正且不超过活动参数限制。CONFIG 仅未使能时接受，校验完成原子更新 RAM 配置并产生新版本，不隐式持久化。新增参数 ID 20～24 对应 kp、kd、iq_limit、speed_limit、accel_limit，value_type=1；通过 READ_PARAM/WRITE_PARAM/SAVE_PARAM 持久化时遵守原有暂存事务规则，存在其他未提交暂存参数时 CONFIG 返回 CONFLICT。

131 使用本地受限轨迹发生器从实际准备位置生成 p_ref/v_ref，iq_ff=0，其完整报文放入一个 32 B CAN FD 数据区。132 原子更新 p_ref/v_ref/iq_ff，目标变化必须通过速率和边界检查；132 的完整报文放入一个 48 B CAN FD 数据区，禁止分片，须通过路径时序验收。两种目标可选其一，运行中不得混用：本次 READY 的首条有效目标选择目标格式，DISABLE 后才可切换。

两种目标与原有目标共用 command_seq 新鲜度规则，执行确认通过 GET_MOTOR_STATE 的 last_applied_seq、valid_bits 和模式完成；type=124 本身不携带执行序号。limit_flags 位 0=电流限幅、1=速度限幅、2=加速度限幅，其余 0。现有 position_impedance.c 含积分和摩擦补偿扩展；映射本规范基础 PD profile 时须关闭未通过能力定义的积分和附加补偿，不能静默改变线上增益含义。

### 8.8 轨迹下载和运行

mode=5 定义缓存的逐段绝对位置梯形轨迹。首版每段以零终速结束，可包含停留时间；不宣称支持连续样条、段间速度贯通或机器人笛卡尔插补。缓存建议 4 段，实际容量读取 GET_CAPS page3。

| type | 命令 | Q 后布局 | 成功响应 R 后字段 |
| --- | --- | --- | --- |
| 140 / 0x008C | TRAJ_BEGIN | traj_id:u32, segment_count:u16, reserved:u16 | traj_id:u32 |
| 141 / 0x008D | TRAJ_APPEND | traj_id:u32, first_index:u16, count:u16, segments:byte[20×count] | next_index:u16, reserved:u16 |
| 142 / 0x008E | TRAJ_COMMIT | traj_id:u32, content_crc16:u16, reserved:u16 | config_revision:u32 |
| 143 / 0x008F | TRAJ_START | job_id:u32 | ACCEPTED，无附加字段 |
| 144 / 0x0090 | TRAJ_ABORT | job_id:u32 | ACCEPTED 或已终止时 OK，无附加字段 |
| 145 / 0x0091 | GET_TRAJ_STATE | job_id:u32 | 公共任务状态 24 B，见下文 |

每段 20 B：position:i32, max_speed:i32, acceleration:i32, deceleration:i32, dwell_ms:u16, reserved:u16。位置单位 0.001 rad；速度单位 0.001 rad/s；加减速度单位 0.001 rad/s²。速度、加减速度必须为正且受活动参数限制。段序从 0 开始，first_index 必须等于 next_index；完整重复内容可以幂等确认，重复索引但内容不同返回 CONFLICT。count≥1，并满足 16+20×count≤路径 max_message_payload。

BEGIN/APPEND/COMMIT 只在未使能且没有活动任务时接受。COMMIT 必须收到全部段；content_crc16 按公司 CRC16 参数对顺序拼接的原始 20 B 段数据计算，不含 Q 和 traj_id。提交后轨迹只读，参数限制、坐标原点或控制者变化即失效。准备任务时检查首段可达、静止条件和当前位置；ARMED 期间漂移超出使能窗口时拒绝 START。

轨迹常规流程：BEGIN → APPEND → COMMIT → SET_MODE(5) → MOTION_ARM(kind=1) → KEEPALIVE → MOTOR_ENABLE → TRAJ_START → GET_TRAJ_STATE。轨迹可以下载多帧，但实时执行由本地定时器推进，不能依赖上位机按时送到每个点。下载缓存和执行缓存的峰值必须在编译配置中限额。

公共任务状态 24 B：job_id:u32, kind:u8, state:u8, reason:u16, step:u16, progress_permille:u16, elapsed_ms:u32, position:i32, result_items:u16, flags:u16。progress 为 0～1000，FFFF 表示无法估计；轨迹 step 为当前段索引。flags 位 0=结果有效、1=结果已应用、2=结果已持久化、3=回零有效，其余 0。

reason：0=正常、1=用户中止、2=运动看门狗超时、3=安全故障、4=任务超时、5=行程限制、6=传感器异常、7=结果不合格、8=时钟失效、9=配置变化、10=轨迹数据错误、11=原点信号未找到、12=时钟超差；厂商诊断通过独立日志解释，不复用内部枚举数值。

### 8.9 回零与坐标原点

mode=6 仅执行有界机械回零。method：1=限位开关，2=编码器 index，3=限位开关后搜索 index；不把“设置当前位置为零”当成机械回零。input_id 是产品注册的逻辑输入编号，不允许远端指定任意 MCU GPIO。

| type | 命令 | Q 后布局 | 成功响应 R 后字段 |
| --- | --- | --- | --- |
| 150 / 0x0096 | HOME_CONFIG | method:u8, direction:i8, input_id:u16, search_speed:i32, latch_speed:i32, max_travel:i32, backoff:i32, zero_offset:i32, timeout_ms:u32, debounce_ms:u16, reserved:u16 | config_revision:u32 |
| 151 / 0x0097 | HOME_START | job_id:u32 | ACCEPTED，无附加字段 |
| 152 / 0x0098 | HOME_ABORT | job_id:u32 | ACCEPTED 或已终止时 OK，无附加字段 |
| 153 / 0x0099 | GET_HOME_STATE | job_id:u32 | 公共任务状态 24 B |
| 154 / 0x009A | SET_POSITION_ORIGIN | position_at_current:i32 | applied_offset:i32 |

HOME_CONFIG 总 payload 40 B。direction 仅 −1 或 +1，搜索和锁存速度均以正数表示大小；latch_speed≤search_speed。位移单位 0.001 rad，速度单位 0.001 rad/s。max_travel、backoff、timeout 必须为正，均受产品最大值限制；应用零偏 zero_offset 是参考边沿处希望显示的位置。原点开关有效极性由本地 input_id 配置固定，去抖参数不能超出硬件允许范围。

准备时若原点信号已有效，先反向退出并验证释放，再按 direction 搜索；首次触发后退回 backoff，二次以 latch_speed 锁存。同一任务的总运动距离以各阶段绝对位移累计，不能在换向时抵消；方法 2 跳过开关步骤，只锁存 index。阶段编号：0=未开始、1=退出有效输入、2=搜索、3=退回、4=精定位、5=完成。每阶段都检查行程、超时和传感器状态。

方法 3 在限位退出和定位后继续以 latch_speed 沿配置方向搜索 encoder index，最终 index 才是零偏参考，额外搜索也计入总行程和超时。成功后产生待应用的零偏结果，不立即变更运行坐标；设备停止并回 READY。主机读取结果后通过 APPLY_TASK_RESULT 应用零偏，persist 决定是否持久化；失败不得产生有效回零结果。机型没有所需限位或 index 输入时对应 method 必须拒绝。

SET_POSITION_ORIGIN 只在未使能、静止且无活动任务时接受，使当前位置显示为指定 position_at_current，原子更新 RAM 坐标偏移，清除旧目标和已提交轨迹。该命令不移动电机、不设置“机械回零有效”标志，也不自动写 Flash。参数 ID 25 定义活动机械坐标偏移，单位 0.001 rad；原点改变后所有依赖旧坐标的任务失效。存在未提交参数事务时返回 CONFLICT，需保存时单独 SAVE_PARAM 并校验 revision。

### 8.10 编码器与电流标定

mode=7。CALIB_CONFIG 的 task_mask 只允许选择一个置位类型：位 0=编码器方向和电角度零偏、1=编码器非线性校正、2=编码器观测器参数、3=电流采样零偏。能力位与硬件和算法实现一一对应；编码器方向标定可能引起运动，不能以“校准”名义绕过控制权和行程限制。

| type | 命令 | Q 后布局 | 成功响应 R 后字段 |
| --- | --- | --- | --- |
| 160 / 0x00A0 | CALIB_CONFIG | task_mask:u32, current_limit:i32, speed_limit:i32, max_travel:i32, timeout_ms:u32 | config_revision:u32 |
| 161 / 0x00A1 | CALIB_START | job_id:u32 | ACCEPTED，无附加字段 |
| 162 / 0x00A2 | CALIB_ABORT | job_id:u32 | ACCEPTED 或已终止时 OK，无附加字段 |
| 163 / 0x00A3 | GET_CALIB_STATE | job_id:u32 | 公共任务状态 24 B |
| 164 / 0x00A4 | READ_CALIB_RESULT | job_id:u32, first_item:u16, count:u16 | 结果分页，见下文 |
| 165 / 0x00A5 | APPLY_TASK_RESULT | job_id:u32, expected_revision:u32, persist:u8, reserved:byte[3] | new_revision:u32 |

current_limit 为 mA，speed_limit 为 0.001 rad/s，max_travel 为绝对累计机械位移 0.001 rad，timeout 为 ms。电流零偏任务必须在 PWM 关闭并确认无负载电流的状态采样，此任务允许三个运动限值为 0；其他标定的限值必须为正。开始激励前验证编码器、电源、温度和测试所需约束；具体步骤由算法适配器执行，并提供公共状态进度。

标定生成候选结果，不能在未成功完成时覆盖原有参数和 LUT。READ_CALIB_RESULT 的成功 payload 为 R + job_id:u32, first_item:u16, count:u16 + entries:byte[12×count]。每项 12 B：item_id:u16, value_type:u8, valid:u8, value:i32, quality_permille:u16, reserved:u16。value_type=1 是 i32、2 是 u32；valid 为 0/1，quality 0～1000 或 FFFF 表示无评分。请求 count 为 1～min(8,max_result_items)，还应满足 20+12×count≤路径 max_message_payload；到达结果末尾可返回较少条目，first_item 大于总数返回 RANGE。

标定 item_id：1=编码器方向，i32 值 −1/+1；2=电角度零偏，i32 单位 0.000001 rad；3=LUT 点数，u32；4=LUT 内容 CRC16，u32 低 16 位有效；5=观测器带宽，u32 单位 0.001 Hz；6/7/8=Ia/Ib/Ic 采样偏置，i32 单位 ADC 原始码。LUT CRC 对候选表的规范小端 int16 数组计算，算法对应的 LUT 数值缩放由该产品参数 schema 定义，不能仅凭 CRC 跨型号套用表。此版结果查询提供 LUT 摘要，不定义跨产品的整表导入。

APPLY_TASK_RESULT 同时适用于 DONE 的回零、标定、辨识任务，要求结果全部有效且电机未使能、无未提交参数事务。回零候选可通过 READ_CALIB_RESULT 的公共结果分页读取，item_id=9 表示机械坐标偏移，i32 单位 0.001 rad。persist=0 只原子更新 RAM 活动配置；persist=1 使用原参数存储的掉电安全事务，成功落盘才 OK。先校验 expected_revision，再生成 new_revision，不得绕过版本冲突检查。LUT、传感器比例及其 schema 必须一起提交，失败保持原配置。结果应用会清除旧目标和已准备轨迹；坐标改变后必须重新准备运动。

### 8.11 电机参数辨识

mode=8。IDENT_CONFIG 的 identify_mask 只允许选择一个置位类型：位 0=绕组电阻、1=绕组电感、2=磁链、3=正反向摩擦、4=转动惯量。后四项特别依赖算法和试验条件；现有项目未验证的类型不能只因分配了编号就置能力位。摩擦辨识输出保持电流域单位。

| type | 命令 | Q 后布局 | 成功响应 R 后字段 |
| --- | --- | --- | --- |
| 170 / 0x00AA | IDENT_CONFIG | identify_mask:u32, current_limit:i32, speed_limit:i32, max_travel:i32, timeout_ms:u32, test_point_count:u16, reserved:u16, test_speeds:i32[4] | config_revision:u32 |
| 171 / 0x00AB | IDENT_START | job_id:u32 | ACCEPTED，无附加字段 |
| 172 / 0x00AC | IDENT_ABORT | job_id:u32 | ACCEPTED 或已终止时 OK，无附加字段 |
| 173 / 0x00AD | GET_IDENT_STATE | job_id:u32 | 公共任务状态 24 B |
| 174 / 0x00AE | READ_IDENT_RESULT | job_id:u32, first_item:u16, count:u16 | 与第 8.10 节相同的结果分页 |

CONFIG 总 payload 48 B，允许管理分片。四个 test_speeds 单位 0.001 rad/s。摩擦任务 test_point_count 为 1～4，使用正的速度大小并由设备生成正反向试验，未用项填 0；其他任务 count=0、四项均 0，由对应算法按限值生成激励。CONFIG 的安全限值单位与标定相同；设备根据具体任务拒绝无效的零限值或超范围配置。

辨识 result item_id：100=相电阻，u32 单位 µΩ；101=Ld、102=Lq，u32 单位 nH；103=磁链，u32 单位 nWb；104=正向库仑摩擦电流、105=反向库仑摩擦电流，i32 单位 mA；106/107=正/反向黏性摩擦系数，i32 单位 0.000001 A·s/rad；108/109=正/反向拟合 RMSE，u32 单位 mA；110=惯量，u32 单位 0.000000001 kg·m²。反向摩擦电流为负值，黏性系数为非负大小，误差指标不参与控制前馈。编码范围不够或质量不合格必须 invalid，不能饱和后报告有效。

READ_IDENT_RESULT 只读取结果；通过 APPLY_TASK_RESULT 显式应用。跨模式共用的电阻、磁链、编码器和电流比例必须进行整体参数一致性检查。饱和持续、样本不足、拟合拒绝、行程越限、速度追踪失败分别进入可查询的失败终态，不回报零值假装辨识成功。

### 8.12 多轴时钟与同步启动

本版多轴同步指已下载轨迹在共同时间开始，不保证各轴轨迹同时结束，也不提供跨轴实时插补。先逐轴完成下载、准备和使能，再预置同一执行时刻；不得把逐台收到 START 的时间当成同步时刻。协调者负责组成员清单，每轴仍有自己的控制会话，组标识不替代控制权。

| type | 命令 | 请求布局 | 成功响应 |
| --- | --- | --- | --- |
| 180 / 0x00B4 | CLOCK_EXCHANGE | Q + probe_id:u32, t1_us:u64，共 20 B | R + probe_id:u32, t1_us:u64, t2_us:u64, t3_us:u64，共 40 B |
| 181 / 0x00B5 | CLOCK_ADJUST | Q + probe_id:u32, t4_us:u64, domain_id:u16, valid_ms:u16，共 24 B | R + offset_us:i64, error_bound_us:u32 |
| 182 / 0x00B6 | SYNC_ARM | Q + group_id:u16, group_tag:u16, job_id:u32, execute_at_us:u64, max_skew_us:u32，共 28 B | R + accepted_error_us:u32 |
| 183 / 0x00B7 | SYNC_COMMIT | type:u8, seq:u8, group_tag:u16, execute_at_low32:u32，共 8 B | 无 ACK，之后查询组状态 |
| 184 / 0x00B8 | SYNC_ABORT | type:u8, seq:u8, group_tag:u16, reason:u32，共 8 B | 无 ACK，禁止输出并取消匹配组 |
| 185 / 0x00B9 | GET_SYNC_STATE | Q + group_id:u16, group_tag:u16 | R + group_id:u16, group_tag:u16, job_id:u32, group_state:u8, clock_state:u8, reserved:u16, execute_at_us:u64, error_bound_us:u32，共 36 B |
| 186 / 0x00BA | GET_CLOCK_STATE | Q | R + domain_id:u16, clock_state:u8, source:u8, master_now_us:u64, offset_us:i64, error_bound_us:u32, valid_left_ms:u32，共 40 B |

时间使用相应 boot_id 下的 u64 单调微秒计数，主时钟 domain_id 非零，由安装配置的协调者拥有；不是 UTC 日期。clock_state 为 0=未同步、1=有效、2=过期；source 为 0=无、1=本四时间戳服务、2=产品已有硬件同步服务。硬件同步服务也必须提供相同误差界接口，不能把时间分辨率当作同步精度。

CLOCK_EXCHANGE 只在未使能且无活动组时接受。t1 是主机提交请求给传输队列前的主时钟时间，t2 是设备收到完整合法请求的本地时间，t3 是设备提交响应到传输队列前的本地时间，t4 是主机收到完整合法响应的主时钟时间。每个 probe_id 在当前控制会话内唯一；设备保留最近探测的四时间戳所需数据，只有 CLOCK_ADJUST 使用且对应探测成功后才销毁。

设备由探测缓存和收到的 t4 计算 roundtrip=(t4−t1)−(t3−t2)，offset=((t1−t2)+(t4−t3))/2，master_time=local_time+offset。四时间戳偏移估计参考 [RFC 5905](https://www.rfc-editor.org/rfc/rfc5905.html)，此处使用公司 payload，不是 NTP 报文或完整 NTP 实现。整数计算必须防溢出，四个时刻及差值都校验为合法、单调和有界；roundtrip<0 或探测往返超过 2 s 直接拒绝。

不假设上下行严格对称。基础误差界至少取向上取整的 roundtrip/2，再加时间戳采集误差和探测期间漂移；随后按 oscillator_bound_ppm 随保持时间增加。valid_ms 范围 100～10000，探测结果超过 2 s 不接受校时。READY 校时允许更新偏移；组 ARMED/RUNNING 或电机已使能时禁止阶跃校时。需要更高精度或长时间保持时，应使用经产品验证的硬件同步时钟和相应 source=2 能力。

SYNC_ARM 仅接受 kind=1 的轨迹任务，轨迹已 ARMED、电机已 ENABLED 且正在保持准备位置、保活新鲜。group_id 和 group_tag 非零，tag 在本次 boot 与该控制者下不复用。execute_at_us 必须属于当前同步域，距离当前时间在路径最小提前量至 10 s 之间，并早于时钟有效期。为了约束任意两轴偏差，各轴在预定时刻的时钟误差界加本地启动抖动必须≤max_skew_us/2；无法证明满足时返回 RANGE 或 BAD_STATE，不静默按当前时间执行。

成功 ARM 冻结完整 u64 执行时刻，group_state=1 ARMED；accepted_error_us 为预定时刻预计的本轴时钟误差界加本地启动抖动。COMMIT 来自绑定控制者，tag 和 execute_at_low32 必须与已冻结值完全一致，且到达时间早于 execute_at_us−min_sync_lead_us；完整时刻已经存储，不能仅凭低 32 位另猜一次时间。COMMIT 到达后 group_state=2 COMMITTED，在预定时刻进入 3 RUNNING；完成为 4 DONE，取消为 5 ABORTED，失败为 6 FAILED，未分配为 0 EMPTY。普通 TRAJ_START 在已有活动组时拒绝。

COMMIT 的 seq 是该组独立 8 位命令序号，相同内容的重复 COMMIT 幂等，不更改执行时刻、不刷新保活；不同内容的重复序号返回冲突计数并拒绝。ABORT 的 reason 取 0=协调者取消、1=时钟失效、2=成员故障；只要来源和 tag 匹配即执行，不因序号旧而忽略安全取消。ABORT 禁止输出、撤销控制会话，任务与组转 ABORTED；单播/广播均不回复 ACK。

COMMIT 使用 Priority=2，ABORT 使用 Priority=0，均将 8 B payload 放入公司完整报文，可向 FF 广播；其他管理写入仍禁止广播。每个节点必须持续收到自己的 MOTION_KEEPALIVE，组广播不替代单轴保活。

协调者在全部成员 ARM 成功后提前发送 COMMIT 并查询 GET_SYNC_STATE。只要成员缺失确认、时钟过期或不能在截止前完成提交，应向所有成员发 SYNC_ABORT。普通通信无法在任意丢包或网络分区时保证全组“全动或全不动”；单轴可能已收到 COMMIT 而其他轴未收到。对必须原子联动的机械动作，需要共同硬件触发/使能互锁及系统级故障处理，不能由本软件广播承诺该保证。

准备期间未收到 COMMIT 到截止时间、触发前时钟过期或误差界超限，本轴取消并禁止输出，绝不迟到补启动；已 RUNNING 后按本地轨迹时基执行，时钟模型不再阶跃调整，但通信与运动故障仍按本地策略停止。

## 9 状态反馈

### 9.1 合并运动反馈：MOTION_FEEDBACK

新增 type=124 / 0x7C，共 8 B，小端，三项从同一发布快照原子读取，每 5 ms 一帧（200 Hz）。使用 header.type=124 的完整公司报文，Priority=3，payload 为以下 8 B，flags=0。完整报文含 CRC8/CRC16；26 B 公司报文填充至 32 B CAN FD 数据区。

| 字节偏移 | 类型 | 字段 | 单位 | 有效编码范围 |
| --- | --- | --- | --- | --- |
| 0～3 | i32 | position | 0.001 rad | −2147483647～2147483647 |
| 4～5 | i16 | speed | 0.001 rad/s | −32767～32767，即 ±32.767 rad/s |
| 6～7 | i16 | iq | 0.001 A | −32767～32767，即 ±32.767 A |

负数使用二进制补码。位置无效值 0x80000000，速度和 Iq 各自无效值 0x8000。传感器无效或值超出该字段可编码范围时，只将该字段置为无效值；不得强转截断、整数回绕或饱和后冒充真实测量。接收者不能把无效值当成零或有效负数，应查询详细状态。底层非整数值先按指定单位四舍五入（恰好半单位时远离零），再判断编码范围。当前单位与旧协议保持一致；若产品需要超过 ±32.767 rad/s 或 ±32.767 A 的有效周期反馈，必须另行评审明确的缩放 schema，不能私自改变倍率。

本帧没有 type 字节、mode/state、fault_summary、last_applied_seq 或时间戳：类型由公司帧头 type 确定；模式和故障通过心跳/状态事件及详细查询获得，执行序号通过 GET_MOTOR_STATE 获得。公司头 seq_id 是该反馈发送流的 16 位帧序号，可用于缺帧/重复监测，但不是目标执行序号或采样时间戳；不能证明具体目标已执行，也不是防重放认证。网关对每轴只保留最新快照，旧周期不积压补发；控制器的目标序号与运动看门狗规则保持不变。

SET_REPORT 保留 pos_ms/speed_ms/iq_ms 三个 u16 字段以避免重排管理报文，但本版要求三者相等：默认全部为 5；全部为 0 关闭周期反馈；否则均为 5～1000 ms 的 5 ms 整数倍。三者不一致返回 RANGE，不静默选择最小值。预算不足返回 RANGE；成功后不能静默降频。

状态/故障变化在下一可用发送机会触发 type=3 HEARTBEAT 格式的状态事件，Priority=3；周期心跳为 Priority=5、1 Hz。事件按 5 ms 合并限频，本地保护不等待发送；事件不替代 200 Hz 合并运动反馈。

120/121/122/123 为历史退出编号，不实现兼容入口、不发送、不复用；查询模式、故障和执行序号使用 type=108，周期心跳和状态事件统一 type=3。

控制目标与高级任务保活也为 200 Hz；使能、模式、参数、任务管理及升级命令按需发送。温度和母线电压仍在详细状态中，电机内部控制环使用各自的本地周期。

### 9.2 详细状态和心跳映射

GET_MOTOR_STATE 的 R 后 32 B：boot_id:u32, sample_counter:u32, position:i32, speed:i32, iq:i32, faults:u32, bus_mV:u16, temperature_centiC:i16, state:u8, mode:u8, last_applied_seq:u8, valid_bits:u8。所有测量从同一发布快照读取；温度单位 0.01 °C。bus_mV=FFFF、温度=0x8000 表示无效，超范围不能回绕。

valid_bits 位 0=位置有效、1=速度有效、2=Iq 有效、3=本会话存在已应用目标、4=母线电压有效、5=温度有效，6～7=0。faults 位 0=过流、1=过压、2=欠压、3=电机过温、4=驱动器过温、5=编码器错误、6=目标超时或高级任务保活超时、7=CAN bus-off、8=软件停止、9=参数无效、10=位置越限、11=硬件驱动故障、12=内部软件故障、13=回零失败、14=标定失败、15=辨识失败、16=轨迹执行失败、17=同步时钟失效或超差；18～31 保留。新增高级故障映射到 fault_summary 的“其他”位；具体 job.reason 通过对应状态查询读取。

HEARTBEAT 使用 type=3：boot_id:u32,uptime_s:u32,state:u8,mode:u8,reserved:u16,faults:u32，共 16 B payload。周期为 1 Hz，状态变化按第 9.1 节发送额外事件，发往当前控制者或产品默认目标。保留字段为 0，故障位和详细状态一致，序号随该消息流实际发送递增。完整报文 34 B，填充至 48 B CAN FD 数据区。

## 10 实时消息封装与调度

目标、停止、保活、同步提交/取消的 payload 保留第 8 节定义；公司帧头 type 必须与这些 payload[0] 一致。flags=0，外层 seq_id 用于帧关联/监测，command_seq、lease_tag、group_tag 保持原业务含义。type=124 的 payload[0] 是位置低字节，不能应用该冗余 type 检查；HEARTBEAT 同样只由公司帧头识别。

所有实时消息完整封装且禁止分片。113、131、132、183 使用 Priority=2，184 使用 0；目标队列与升级队列隔离，连续目标按目的和 type 保留最新一条并执行 TTL。SYNC_COMMIT/ABORT 是事件，不能按连续目标覆盖规则丢弃取消事件。

速度目标 payload 示例 `68 01 34 12 10 27 00 00`：type=104、command_seq=1、lease_tag=0x1234、目标 10 rad/s。封装后 26 B，再补 6 B 00，单个 CAN FD 数据区为 32 B；电机服务收到验证后的 8 B 业务对象。

## 11 固件升级业务

### 11.1 能力和状态

升级命令拟分配到通用段 50～59，所有数字均为十进制。升级与电机命令共享公司完整报文，不新增第二套外层帧。一次只允许一个升级者、一个写块请求在途；所有有副作用的请求只接受单播。升级前必须确认机械系统已进入允许断动力的状态，设备层面检查未使能并锁止输出。

升级状态：0=IDLE，1=MANIFEST_READY，2=PREPARING，3=RECEIVING，4=VERIFYING，5=VERIFIED，6=ACTIVATING，7=FAILED。存储模式：0=单应用恢复升级，1=双槽试启动回滚。设备只公布实际实现的模式。

boot_id 为每次启动变化的 u32 标识。update session_id 是当前连接的临时标识，不等于镜像身份；镜像身份为完整 SHA256。断电后基于镜像摘要、签名清单和检查点恢复，不能仅凭 session_id 接着写。

### 11.2 命令注册表

| type | 命令 | 请求 payload | OK 响应 R 后字段 |
| --- | --- | --- | --- |
| 50 / 0x32 | UPDATE_OPEN | Q + expected_boot_id:u32 | boot_id:u32, max_block:u16, alignment:u16 |
| 51 / 0x33 | UPDATE_MANIFEST | Q + manifest:byte[144] | image_size:u32 |
| 52 / 0x34 | UPDATE_BEGIN | Q + resume:u8 + reserved:byte[3] | next_offset:u32, resume_offset:u32 |
| 53 / 0x35 | UPDATE_WRITE | Q + offset:u32 + data_len:u16 + reserved:u16 + data:byte[N] | next_offset:u32, resume_offset:u32 |
| 54 / 0x36 | UPDATE_QUERY | Q | 第 11.4 节 20 B 状态 |
| 55 / 0x37 | UPDATE_FINALIZE | Q | image_sha256:byte[32] |
| 56 / 0x38 | UPDATE_ACTIVATE | Q + delay_ms:u16 + reserved:u16 | delay_ms:u16, reserved:u16 |
| 57 / 0x39 | UPDATE_ABORT | Q | 无 |
| 58 / 0x3A | GET_BOOT_RESULT | Q，允许 session=0 | boot_id:u32, app_version:u32, outcome:u8, storage_mode:u8, reason:u16 |
| 59 / 0x3B | ENTER_BOOT | Q + expected_boot_id:u32 | 无；当前 App 禁止输出后发送响应并复位 |

UPDATE_OPEN 使用 Q.session_id=0，设备必须未使能且无控制会话；成功 R.session_id 返回新的升级会话。若存在已销毁连接留下的可恢复镜像记录，允许新会话重新提交同一签名清单；不允许第二连接抢占活动升级。ENTER_BOOT 的 Q.session_id=0，仅配置的维护源可调用；其成功只表示已安排进入 Boot，主机必须等待新 boot_id 后重新 OPEN。

UPDATE_MANIFEST 成功表示已验证清单签名、产品匹配和大小限制，此时才进入 MANIFEST_READY；不意味着 Flash 已擦除。UPDATE_BEGIN 进入 PREPARING 并可返回 ACCEPTED；查询到 RECEIVING 才可写入。UPDATE_WRITE 不使用 ACCEPTED，只有本块编程及读回比较成功后才返回 OK。本版写块业务响应上限 5000 ms，设备无法满足时应拒绝本配置或在 BEGIN 阶段提前擦除；超时先 QUERY，不能盲目从下一块继续。retry_after_ms 仅表示查询间隔，不是写入成功依据。

升级会话中的 UPDATE_QUERY 是只读旁路，允许在长操作和写块等待期间执行，使用独立递增 request_id；它不推进有副作用请求的去重水位，也不覆盖写块结果缓存。其他升级命令继续停等。主机一次最多挂起一条状态查询，忽略已超时旧查询的迟到响应。

状态约束：OPEN 后为 IDLE；MANIFEST 只在 IDLE 接受；BEGIN 只在 MANIFEST_READY 接受；WRITE 只在 RECEIVING 接受；FINALIZE 仅在 RECEIVING 且 next_offset=image_size 接受；ACTIVATE 只在 VERIFIED 接受。QUERY 全状态允许，ABORT 除 ACTIVATING 外允许，已开始的 Flash 原子操作须到安全点再退出。FAILED 后只能 QUERY、ABORT 或断开后重新建立会话，不用写块隐式重启升级。

UPDATE_FINALIZE 可返回 ACCEPTED，查询到 VERIFIED 后才允许激活。UPDATE_ACTIVATE 仅在 VERIFIED 接受，先持久化启动选择和有效标志，回复 OK，再在协商 delay_ms 后复位；delay_ms 范围 100～5000，默认主机请求 500。若回复丢失，主机查询启动结果，不反复写启动元数据。UPDATE_ABORT 撤销当前会话并停止后续写入；已经擦除的旧应用不能因此恢复。

### 11.3 签名镜像清单

清单固定 144 B，所有整数小端。镜像内容是应用槽起始地址对应的原始二进制，image_size 是实际签名和散列范围，不含传输填充；空洞由打包工具按最终镜像内容固定填充，不由下载端猜测。

| 清单偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | byte[4] | magic，ASCII FWUP |
| 4 | u16 | manifest_version=1 |
| 6 | u16 | manifest_size=144 |
| 8 | u32 | product_id |
| 12 / 14 | u16 / u16 | hw_rev_min / hw_rev_max，含端点 |
| 16 | u32 | app_version，四个字节按 major/minor/patch/build 编码为数值 0xMMmmppbb |
| 20 | u32 | security_version，单调安全版本 |
| 24 | u32 | image_size |
| 28 | u16 | min_boot_api |
| 30 | u8 | image_type=1 即应用固件 |
| 31 | u8 | hash_alg=1 即 SHA256 |
| 32 | byte[32] | image_sha256 |
| 64 | u16 | signature_alg=1 即 ECDSA P256 SHA256 |
| 66 | u16 | key_id |
| 68 | byte[12] | reserved，全 0 |
| 80 | byte[64] | signature，r[32] 大端后接 s[32] 大端，不使用 DER |

签名输入为 SHA256(ASCII 字节串 "MOTORFW1" || 清单字节 0～79)，ECDSA 使用该 32 字节摘要；image_sha256 已包含于签名输入。签名 r、s 的大端编码是显式例外，其余整数仍小端。验签公钥通过受保护 Boot 或可信制造流程配置，不能使用固件包自带的任意新公钥作为信任根。key_id 未注册即拒绝。使用经过验证的密码库，不自研椭圆曲线算法。

BEGIN 前必须校验产品、硬件范围、Boot API、镜像类型、签名与 security_version≥设备持久化最低安全版本。app_version 是版本标识，不代替安全版本比较。FINALIZE 从实际 Flash 重新计算 SHA256，并检查应用向量表和入口地址属于合法应用区；通过后才持久化 VERIFIED。检查初始 MSP 位于有效 SRAM 范围且符合栈对齐、复位入口 Thumb 位和实际入口范围。CRC 检测传输错误，签名和摘要验证来源与完整内容，三者用途不同。

应用链接地址由 product_id 和 Boot API 对应的存储配置固定。双槽实现采用交换或复制到规定执行槽，或使用经验证的位置无关镜像；不能把为槽 A 链接的普通二进制直接跳转到槽 B 执行。是否支持双槽仍以设备实际能力为准。

普通应用升级禁止改写 Bootloader、信任根、永久标定区及设备身份。首版不提供任意地址写入、Boot 自升级和参数格式迁移；需要这些功能时单独评审其恢复与兼容策略。

### 11.4 写块和进度

UPDATE_WRITE 的 payload 长度=16+N，公司整帧长度=34+N。N 必须等于 data_len，1≤N≤协商 max_block；offset 从应用镜像起始的 0 开始，不是 MCU 绝对地址。采用顺序写入，offset 必须等于 next_offset，或为可验证的完整重复块。首版建议 max_block=128 B，也可按实际 RAM、擦写粒度协商 64/256/512 B。

非末块 offset 和 N 必须满足设备 write_alignment；末块可不足对齐单元，存储层仅在应用槽内以 FF 补足编程单元，填充不计入镜像 size 和 SHA256。主机用减法形式检查 N≤image_size-offset，设备必须再次检查，避免加法整数溢出。

QUERY 的 R 后 20 B：state:u8, storage_mode:u8, last_result:u16, next_offset:u32, resume_offset:u32, image_size:u32, retry_after_ms:u16, reserved:u16。retry_after_ms 为建议下次查询间隔，范围 50～5000 ms。last_result 表示最近长操作完成结果；运行中为 ACCEPTED。

next_offset 是当前运行中已成功写入且读回验证的连续末尾；resume_offset 是元数据已安全提交、断电后保证可以继续的位置，满足 0≤resume_offset≤next_offset≤image_size。不能把 RAM 中进度当作掉电持久进度。

检查点以安全擦除单元为边界提交。块跨越擦除单元前，要么先完成并提交上一单元，要么限制块不跨界。掉电恢复后，先擦除 resume_offset 所在的未提交单元，再从 resume_offset 重写；已经提交的单元不能重擦。最后不足一擦除单元只有在全部编程验证并提交结束标志后才能将 resume_offset 推至 image_size。元数据采用双记录或日志和提交标志，写入中断可识别完整旧记录。

同一运行期丢 ACK：重发相同 request_id 和数据，设备返回缓存成功结果，不重复擦写。缓存之外的重复块只在数据与已编程区域完全一致、范围处于连续已确认区间时返回当前进度；不一致返回 CONFLICT。新连接和掉电恢复必须重新 OPEN、提交同一清单并 BEGIN(resume=1)，以 QUERY 返回的位置为准。不同镜像不能复用旧进度。

BEGIN(resume=0) 清除旧升级记录并开始新下载；只在 MANIFEST_READY 且新清单验证成功后可执行。BEGIN(resume=1) 要求持久化清单身份匹配，不匹配返回 IMAGE_MISMATCH，禁止自动退化成擦除重来。

### 11.5 完整时序

```text
主机                           设备或 Bootloader
GET_INFO / GET_CAPS       ->    报告身份、Boot API、块长、存储模式
停止电机并释放控制权     ->    DISABLED
ENTER_BOOT 如有需要      ->    应答后复位，产生新 boot_id
UPDATE_OPEN              ->    创建当前连接的升级 session
UPDATE_MANIFEST          ->    验签和适配检查，通过后 MANIFEST_READY
UPDATE_BEGIN            ->    PREPARING，允许返回 ACCEPTED
UPDATE_QUERY            ->    RECEIVING，给出 resume_offset
UPDATE_WRITE(offset,N)  ->    编程和读回成功，回 OK 与 next_offset
重复写块直到 image_size
UPDATE_FINALIZE         ->    VERIFYING -> VERIFIED 或 FAILED
UPDATE_QUERY            ->    确认 VERIFIED
UPDATE_ACTIVATE         ->    提交启动元数据，回 OK，延时复位
GET_BOOT_RESULT         ->    上报实际启动版本和结果，输出保持禁用
```

升级链路空闲超过 30 s 时释放临时连接会话，但保留合法持久化进度；正在执行不可中断的 Flash 操作完成到安全点后再释放。新接口接续仍走 OPEN 和清单匹配。应用端未具备可靠后台擦写能力时，必须先进入独立 Boot，再下载，不承诺运动中本电机升级。

升级会话建立后电机状态为 UPDATING，控制申请返回 BUSY。ABORT 或会话超时后，仅在现有应用仍有效且没有未完成破坏性操作时允许回到 DISABLED；否则留在 Boot 恢复模式并保持输出禁用。Boot 必须能在应用完全损坏时独立初始化至少一个产品声明支持的维护接口。

### 11.6 启动与回滚

GET_BOOT_RESULT 的 outcome：0=正常启动，1=新镜像成功，2=试启动待确认，3=已回滚，4=留在 Boot 恢复模式；reason=0 正常，1=摘要或签名失败，2=向量表错误，3=试启动超时，4=试启动复位，5=下载未完成，6=存储错误。

outcome=1 只能在应用完成规定自检并提交内部启动成功记录后报告；跳转到复位入口不等于启动成功。单应用模式也需要应用上报启动成功；它没有旧镜像可回滚，自检不通过时保留可诊断的恢复入口。不可逆安全版本下限在应用确认成功后更新。

单应用模式下，下载开始后旧应用可能已被擦除。任何时刻掉电必须进入可通信 Boot 或有效应用；不保证回到旧版本。只有镜像完整验证成功才允许启动。ABORT 不会恢复旧应用，失败时通过维护接口重新下载。

双槽模式必须保留旧镜像。新镜像进入试启动后，由应用完成自检并通过内部 boot_confirm API 写入确认；不能由下载主机提前“确认运行正常”。试启动 10 s 内未确认或期间复位，Boot 按产品策略回退旧镜像。安全版本不可逆下限应在成功确认后提升，避免在试启动前提升而使旧镜像不能回滚。流程参考 [MCUboot 启动设计](https://docs.mcuboot.com/design.html)，不要求本项目直接采用其镜像格式。

本工程 STM32G431CB 的 128 KiB Flash 和现有约 88 KiB 应用使内置双槽不可直接成立；现有参数区起点 0x0801C000，预留 16 KiB。首版建议受保护 Boot 加单应用恢复；Boot 大小、应用链接起点、元数据区域必须按实际新构建体积确定。不得将示意分区当作已经通过链接和掉电测试的分区。

## 12 CAN FD 承载

### 12.1 单帧、分片与重组

CAN FD 使用第 4 节的 29 位 ID 结构和 EF00 业务 PGN，IDE=1、RTR=0、FDF=1。采用公司自定义承载，不宣称符合 SAE J1939 22 的 CAN FD 传输格式，仅实现本节公司分片，不使用其他传输协议。

当前主配置：仲裁速率 500 kbit/s、数据速率 5 Mbit/s、BRS=1。“5 Mbit/s”明确指数据段，不将仲裁段也设为 5 Mbit/s；仲裁值沿用此前协议默认值。GET_CAPS page2 在该原生链路返回 nominal_bps=500000、data_bps=5000000，且 link_flags.BRS=1。该配置必须通过板卡收发器和全网验证；本版仅启用该配置，不能失败后静默切速。部署网段全部节点及收发器必须支持 CAN FD 及当前速率；不将经典 CAN 兼容模式作为恢复入口。

该配置是协议目标，不代表现有 PCB 已完成 5 Mbit/s 验收。收发器型号和额定速率、线束长度、支线、终端和环路延迟当前为 UNKNOWN/TODO，必须通过物料/原理图和实测确认。发送延迟补偿 TDC 由 HAL Port 结合时钟和实际收发路径配置，原理参考 [ST AN5348](https://www.st.com/resource/en/application_note/an5348-fdcan-peripheral-on-stm32-devices-stmicroelectronics.pdf)；协议层不写寄存器或固定 MCU 专用 TDC 值。

按当前工程可读取的 170 MHz FDCAN 内核时钟，数学候选位时序为：仲裁 Prescaler=20、Seg1=12、Seg2=4、SJW=1，数据 Prescaler=2、Seg1=12、Seg2=4、SJW=1。速率分别为 170 MHz/[20×(1+12+4)]=500 kbit/s 和 170 MHz/[2×(1+12+4)]=5 Mbit/s，两段采样点均为 13/17≈76.47%。这些是现有 STM32 目标的计算候选，不是已验证的板级推荐参数；BSP/Port 必须确认实际时钟、容差、SJW、传播延迟及 TDC。更换 MCU 重新计算，不将该组合写入通用协议定义。

合法 FD 数据长度为 0～8、12、16、20、24、32、48、64 B。本协议有效帧至少 18 B，发送时选择不小于 18+len 的最小合法长度，CRC16 后填充 00；填充不计入 len 和 CRC。接收先读取足够的帧头，再验证 len≤46、物理长度恰为规定的最小合法长度、填充值及 CRC，不能把 DLC 编码值直接当成字节数。数据长度约束可对照 [SocketCAN CAN FD 文档](https://www.kernel.org/doc/html/latest/networking/can.html)。

基础实时控制、高频反馈以及新增 113/131/183/184 的 payload 为 8 B，完整长度 26 B，使用 32 B FD 数据区；完整阻抗目标 132 和 CAN FD type=3 心跳的 payload 为 16 B，完整长度 34 B，使用 48 B 数据区。停止、目标、保活、组提交/取消、实时反馈与心跳禁止分片。普通管理消息 Priority=6，实时消息使用第 4、10 节优先级。

当非实时消息的 payload 大于 46 B 时，只切分 payload；每片仍具有公司完整的 16 B 头和 2 B CRC，各片的 len 为本片长度。非尾片 payload 恰为 46 B，尾片为 1～46 B。消息 payload 恰好是 46 的倍数时，最后一片直接标记尾片，不额外发空尾片。每片使用相同 src、dst、type 和 Priority。

| 项目 | CAN FD 分片规则 |
| --- | --- |
| flags[7:6] | 首片 01、中片 10、尾片 11；仅一片使用 00 |
| seq_id | 首片 0，之后逐片加 1；不使用单帧事务 seq 作为首片值 |
| ACK_REQ | 需要业务应答的请求仅尾片置 1；首/中片为 0 |
| RESPONSE | 响应的所有片置 1，且 ACK_REQ 始终 0 |
| REPORT_REQ / RETRY | 同一逻辑消息各片保持一致；保留位仍为 0 |
| 重组键 | link_handle、链路代次、src、dst、type、RESPONSE |
| 并发限制 | 每个键仅一条消息在途；首版发送方每对端一个分片事务 |
| 缓冲限制 | 累计 payload≤路径协商上限，同时匹配该 type 的最大长度 |
| 完成条件 | 收到连续正确的尾片且重组长度满足该 type，才提交业务层 |

活动重组遇到新首片时，不覆盖旧缓冲，视为冲突并丢弃两者；发送端必须先等待完成或重组超时再重发。相同片序、相同有效头字段和相同 payload 的重复片可忽略；相同片序但内容不同、跳号、无首片尾片均终止重组。接收端不逐片发业务 ACK，不用公司的片序代替升级 request_id。

默认片间超时 200 ms、整条重组绝对超时 2000 ms，二者任一到达就丢弃；新片不会无限延长绝对时限。发送方必须根据已分配带宽先判断消息能否在 2000 ms 内传完，不能时应缩小升级块或返回 BUSY。重传只支持整条逻辑消息，不支持选片补发；请求完整发送结束后等待业务响应最长 2000 ms，再从首片重发，并保留 payload 中的 session_id/request_id、设置 RETRY。需要 5000 ms 写块响应预算时遵守第 11 节，不能提前重发写块。

分片响应也携带完整 R 前缀，它可能位于首片；分片请求的单帧响应 seq_id 回显请求尾片片序，分片响应自身 seq_id 必须从 0 开始。因此 session_id/request_id 是管理事务的主关联键，外层 seq_id 只在双方均未分片时提供额外匹配。无会话请求同样必须使用非零 request_id。

分片默认依赖同一路径、同优先级队列的有序发送，不允许跨接口条带传输。各片 CRC 只覆盖各自帧，不能宣称为重组整体的独立端到端 CRC；会话去重、类型长度检查和顺序约束必须执行，升级镜像仍必须经过整镜像摘要与签名验证。中枢透传应保持公司的有效帧字节，不改变 type 或 payload；终端只有完成重组后才能交付业务层，不向业务层提交半条消息。

### 12.2 接口与缓冲边界

一个 CAN FD 物理数据区对应一个公司单帧或分片，CRC16 后仅允许规定的 00 填充。不能把 UART 字节流搜索、USB 包边界或 TCP 收包长度作为本协议解析规则。不同节点的管理分片可以交错，但按第 12.1 节的键独立重组；禁止无限分配缓冲。协议核心仅使用通用消息和 comm_hw 契约，硬件中断和 Vendor API 留在 HAL Port/BSP。

## 13 负载和资源预算

### 13.1 CAN FD 200 Hz 容量与时序验收

CAN FD 当前短目标及合并反馈的完整公司帧均为 26 B，填充到 32 B 数据区，每轴共 400 条实时帧/s，另加心跳和状态事件。五轴为 2000 条实时帧/s，加 5 条周期心跳/s 及事件。5 Mbit/s 不等于整帧均以该速率发送：必须按 T_frame=B_nominal/500000+B_data/5000000 分别计算两段时间，纳入仲裁、CRC、位填充、ACK 和帧间隔；B_nominal/B_data 需由实际帧格式或总线分析仪获得。32 B 数据字段本身仅占 51.2 µs，五轴纯数据字段时间占 10.24%，这是不含任何协议开销的下界，不能当作总线占用验收值。完整阻抗目标另按 48 B 数据区计算。

验收以每轴 5 ms 调度周期为基准：额定全部节点和允许的后台负载下，目标从主站发布到设备应用、反馈从采样到主站完整接收，各自的最坏延迟均应不超过 5 ms，并记录周期抖动、漏周期和错误恢复行为。不同轴分配发送相位，避免所有反馈同时排队。待发目标和反馈按轴及消息类型保留最新值，过期周期不得积压后集中补发；50 ms 看门狗不作为正常周期延迟的验收标准。网关链路必须端到端验收；GET_CAPS 路径最小目标周期需≤5 ms，并单独确认合并反馈与状态事件的预算。未通过的路径配置不得宣称支持本版 200 Hz 运动档位。

### 13.2 升级和资源预算

128 B 写块对应 UPDATE_WRITE payload=144 B（Q、偏移、长度、保留位共 16 B，加数据 128 B），分为 46+46+46+6 B 四片，FD 数据区长度分别为 64/64/64/24 B。WRITE OK 响应 payload=20 B，公司报文 38 B，使用一个 48 B FD 帧。一次成功块交换共 5 个 CAN FD 帧，不包含重试和其他查询。

仅这些帧的数据字段就有 (64+64+64+24+48)×8=2112 bit，在 5 Mbit/s 下占 422.4 µs；还需计入仲裁、CRC、填充、ACK、帧间隔、调度和 Flash 操作，不能将此下界当作有效升级吞吐。

运动期间向其他节点升级的总线时间预算上限为 max(0,min(10%,60%−已分配稳态负载))，并须满足每轴 5 ms 截止时间；没有余量则暂停后台升级。目标升级设备始终未使能。全系统停机维护可单独验证更高带宽，不预先承诺升级耗时。

控制队列优先于升级，公司分片按小批量进入硬件 FIFO，不能先塞满 FIFO 再依赖 CAN ID 消除排队。已开始发送的低优先级帧不能被抢占。重组、镜像块、签名工作区、栈和队列必须静态预算；禁止按远端 len 任意分配内存。

## 14 与当前固件的实现边界

本次交付是协议设计，不修改电机运行代码。当前工程 CAN 接口使用标准 ID、FD/BRS 和 4 字节大端 float，读取到的位时序为仲裁/数据均约 1 Mbit/s；与本版扩展 ID、完整公司报文及 500 kbit/s / 5 Mbit/s 主配置不兼容，不能只改速率就称完成协议迁移。

按项目架构规则，通信应按 comm_hw → transport → protocol → command_router → mc_command_t 分层。当前 Communication/interface_can.c 直接调用 Vendor HAL 并引用电机全局变量，是待迁移的耦合点。速率和位时序属于 BSP / HAL Port，协议核心不依赖具体 MCU；Loader 与 PRODUCT/FACTORY/DEBUG APP 共用协议核心和已选线路配置，切换固件域不隐式切换总线速率。这里仅更新协议设计，未修改或烧录现有固件。

| 模块 | 后续实现工作 | 验收重点 |
| --- | --- | --- |
| wire_codec | 公司完整帧、小端、CRC、限长 | 本文黄金向量逐字节一致 |
| canfd_adapter / comm_hw Port | 扩展 ID、500 kbit/s / 5 Mbit/s、BRS、DLC 填充、公司分片重组 | 速率/时序/TDC 实测，payload≤46、逐片 CRC、超时和整消息关联 |
| command_router | 验证后的 type 分发、对端身份、服务请求对象 | 不直接写 FOC/PWM，不绕过 Safety 与控制权 |
| motor_service | 外部模式映射、目标原子更新、控制权 | 无隐式使能，丢包和重复不掩盖超时 |
| parameter_service | 类型检查、暂存、原子持久化 | 参数/标定兼容和断电恢复 |
| update_service | 同一升级状态机和命令处理 | 不引用 CAN、UART、socket 驱动类型 |
| boot_storage | 分区边界、擦写、读回、元数据 | 任意掉电不启动残缺应用 |
| host_protocol_tool | 查看状态、控制联调、升级和故障注入 | 统一测试向量和传输抓包 |

建议顺序：先完成纯 C 协议核心和 PC 对照；再实现只读 CAN 链路；然后开放控制状态机；最后安装 Boot 并实现升级与 CAN FD 恢复验证。旧协议只能通过独立固件版本或明确配置入口保留，默认禁止新旧控制入口同时写同一电机目标。

### 14.1 按硬件裁剪与 Flash 预算

新增高级业务按独立功能开关裁剪：阻抗、轨迹缓存、回零、标定、辨识和同步；命令注册表保留统一编号，未编译项返回 UNSUPPORTED。现有 FOC 代码可以作为算法基础，不表示新的协议服务已经接入：position_impedance.c 有位置阻抗，foc_traptraj.c 有单条梯形规划，foc_calibration.c 有若干编码器/电流标定，phase_resistance 与 friction_identification 有辨识基础。缓存轨迹对象、MOTION_ARM、公共任务状态、机械回零输入管理、分布式时钟和同步组均需新增或整合实现。内部 enum 中有名称不等于该能力已通过验收。

算法适配器需要增量 step/abort/status/result 接口，不得让标定或辨识长时间阻塞通信服务。现有会直接改活动参数的标定过程，应先改成候选结果存储，再接入 APPLY_TASK_RESULT；输出安全门、持续保活和可中止能力必须在开放对应 GET_CAPS 位前完成。

当前仅编译 CAN FD transport、公司 codec/reassembly 和业务 router，不编译经典 CAN、J1939 TP、RS485/TCP 适配器或兼容帧解码器。Loader 与三种 APP Variant 共享协议库，按实际功能裁剪命令服务；硬件实现置于 comm_hw HAL Port/BSP。

Loader 和 APP 均独立具备 CAN FD 及公司帧收发能力，采用同一网段速率，但可独立配置有界块长和缓冲。Loader 在 APP 无效或升级中断时仍能提供 CAN FD 恢复升级，不依赖正常 APP 转发。

现有 88.01 KiB 是旧固件占用，不包含本规范新增实现。驱动裁剪、删除旧文本协议或复用硬件驱动能节约多少 Flash 必须由对应构建的 map 差值证明；新协议缓冲和签名库的体积不能提前按零成本预算。升级块长和重组缓冲只按启用路径的实际需求分配，ISR 与主循环共享队列必须单独计入 RAM。

## 15 验收用例

| 类别 | 必须通过的用例 |
| --- | --- |
| 编解码 | CRC 黄金向量、大小端、错误 len、截断、额外字节、非法保留位、随机输入不越界 |
| CAN | 扩展 ID 地址一致性、PGN 分流、经典/FD 区分、广播限制、bus-off 恢复不使能 |
| CAN FD | 各合法 DLC、填充和 CRC 边界、首中尾片、缺片/重片/冲突、绝对超时、分片 ACK 关联 |
| 控制 | 非持有者拒绝、模式不匹配拒绝、目标越限拒绝、重复/旧序号不刷新看门狗 |
| 阻抗 | kp/kd 单位、正负误差与阻尼符号、限流、16 B 目标使用 48 B FD 数据区且禁止分片、两种目标格式不能运行中混用 |
| 轨迹 | 段容量、乱序/重复/冲突、提交 CRC、未提交不能 ARM、无 START 不运动、保活丢失本地停止 |
| 回零 | 上电信号已有效、退回无法释放、index 丢失、反复抖动、总绝对行程、任务成功不隐式改坐标 |
| 标定辨识 | 电流零偏时 PWM 关闭、限流限速限行程、中止保留旧值、无效结果拒绝应用、保存中断电 |
| 多轴同步 | 校时不对称误差界、时钟过期、u64 到 low32 校验、缺 COMMIT 不启动、重复 COMMIT 不二次启动、迟到拒绝 |
| 同步异常 | 有成员未收到提交/取消时的协调者处置、保活超时、本地时钟超差、不能宣称网络分区下组原子启动 |
| 状态机 | 带旧目标使能拒绝、停止清除 lease、清故障后仍未使能、超时策略和最长停止时间 |
| 反馈 | i32/i16 边界、无效值、公司 type 分流、同快照、200 Hz；状态事件及详细查询的模式/故障/执行序号 |
| 参数 | 错类型、上下限矛盾、旧 revision、保存中断电、标定区保护 |
| 升级 | 错产品/硬件/签名、越界块、重复块、丢 ACK、不同镜像断点拒绝、最终 Flash 摘要错误 |
| 掉电 | 擦除前后、每个编程单元、检查点提交前后、VERIFIED 提交、激活和第一次启动逐点断电 |
| 硬件裁剪 | 逐配置核验功能与 GET_CAPS 一致；未编译接口无残留依赖；App 与 Boot 独立测量体积 |
| 资源 | RAM 高水位、栈、签名工作区、队列满、升级分片限流下最坏控制延迟 |

发布前必须冻结：公司帧语义勘误与扩展登记、消息编号、节点唯一性、CRC 黄金向量、机械限值和失联停止动作、实测总线预算、Boot 分区与恢复入口、签名密钥制造流程。这些是协议发布与产品验证事项，不表示本设计已经完成固件实现或量产认证。

## 16 参考资料

公司规范和消息注册表决定应用层基础格式。J1939 资料仅用于确定 29 位 CAN 地址布局，电机 payload、命令编号、会话与升级清单是本规范新增定义。

- [公司通信协议正文](https://wcnxj7iyqdkp.feishu.cn/wiki/MM14wIJF7i8irjkx448cHu31nLb)
- [公司消息类型定义](https://wcnxj7iyqdkp.feishu.cn/wiki/DXV4wohO0ioGbskmFAFct8wFngd)
- [SAE J1939 21 数据链路层标准索引](https://saemobilus.sae.org/standards/j193921_201810-data-link-layer)
- [Linux J1939 地址与传输接口](https://www.kernel.org/doc/html/latest/networking/j1939.html)
- [MCUboot 启动与恢复设计](https://docs.mcuboot.com/design.html)

## 附录 A 字节级联调向量

以下向量由随文参考脚本生成，校验覆盖范围与正文一致。脚本用于协议联调，不是 MCU 固件实现。完整 CAN FD 单帧及公司分片向量见 motor_protocol_vectors.json；生成及自检脚本见 tools/protocol_vectors_reference.py。

<!-- VECTORS -->

CRC 校验串 ASCII `123456789`：CRC8=0xEA，CRC16=0x932D，CRC16 低字节在前。以下全部线上帧使用 CAN FD、29 位 ID、BRS=1，500 kbit/s 仲裁、5 Mbit/s 数据。

fd_test_request：CAN ID `0x18EF0302`，公司有效报文 30 B，FD 数据区 32 B，CRC 后为 00 填充。

```text
5A A5 01 20 02 03 00 00 01 00 0C 00 00 00 00 00 00 00 00 00 01 00 00 00 44 33 22 11 63 4D 00 00
```

fd_test_response：CAN ID `0x18EF0203`，公司有效报文 34 B，FD 数据区 48 B，CRC 后为 00 填充。

```text
5A A5 01 10 03 02 00 00 01 00 10 00 00 00 00 3B 00 00 00 00 01 00 00 00 00 00 00 00 44 33 22 11 BB 5F 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

fd_speed：CAN ID `0x08EF0302`，公司有效报文 26 B，FD 数据区 32 B，CRC 后为 00 填充。

```text
5A A5 01 00 02 03 68 00 01 00 08 00 00 00 00 4A 68 01 34 12 10 27 00 00 06 D0 00 00 00 00 00 00
```

fd_feedback：CAN ID `0x0CEF0203`，公司有效报文 26 B，FD 数据区 32 B，CRC 后为 00 填充。

```text
5A A5 01 00 03 02 7C 00 01 00 08 00 00 00 00 94 22 06 00 00 1C 25 B0 04 A6 DB 00 00 00 00 00 00
```

fd_stop：CAN ID `0x00EFFF02`，公司有效报文 26 B，FD 数据区 32 B，CRC 后为 00 填充。

```text
5A A5 01 00 02 FF 64 00 01 00 08 00 00 00 00 F7 64 01 01 00 50 4F 54 53 F6 6C 00 00 00 00 00 00
```

速度目标为 10 rad/s，lease=0x1234，command_seq=1。合并反馈同一快照：位置 1.570 rad、速度 9.500 rad/s、Iq=1.200 A；8 B payload 不含模式或执行序号，由状态消息提供。

CAN FD 的 128 B 写块示例：业务 payload 共 144 B，分成 46+46+46+6 B。以下四帧每帧都含独立公司帧头与 CRC；物理填充位于各帧 CRC 之后。

| 片 | CAN ID | FD 数据区长度 | 公司帧有效长度 |
| --- | --- | --- | --- |
| 0 | 0x18EF0302 | 64 | 64 |
| 1 | 0x18EF0302 | 64 | 64 |
| 2 | 0x18EF0302 | 64 | 64 |
| 3 | 0x18EF0302 | 24 | 24 |

CAN FD 写块片 0：

```text
5A A5 01 40 02 03 35 00 00 00 2E 00 00 00 00 94 40 30 20 10 06 00 00 00 00 00 00 00 80 00 00 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D AA 8F
```

CAN FD 写块片 1：

```text
5A A5 01 80 02 03 35 00 01 00 2E 00 00 00 00 8B 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40 41 42 43 44 45 46 47 48 49 4A 4B 3F 71
```

CAN FD 写块片 2：

```text
5A A5 01 80 02 03 35 00 02 00 2E 00 00 00 00 6A 4C 4D 4E 4F 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F 70 71 72 73 74 75 76 77 78 79 63 59
```

CAN FD 写块片 3：

```text
5A A5 01 E0 02 03 35 00 03 00 06 00 00 00 00 80 7A 7B 7C 7D 7E 7F BB AB
```

1.2 高级命令 payload 向量如下，仅验证字节编码，不表示设备已满足执行前置条件；公司完整帧仍需封装后放入 CAN FD 数据区。

motion_keepalive：

```text
71 01 34 12 07 00 00 00
```

motion_arm：

```text
40 30 20 10 0C 00 00 00 01 00 00 00 07 00 00 00 2A 00 00 00 03 00 00 00
```

impedance_full：

```text
84 02 34 12 E8 03 00 00 D0 07 00 00 64 00 00 00
```

sync_arm：

```text
40 30 20 10 1E 00 00 00 01 00 22 22 07 00 00 00 A0 86 01 00 01 00 00 00 20 4E 00 00
```

sync_commit：

```text
B7 01 22 22 A0 86 01 00
```

trajectory_segments：

```text
E8 03 00 00 88 13 00 00 10 27 00 00 10 27 00 00 14 00 00 00 D0 07 00 00 88 13 00 00 10 27 00 00 10 27 00 00 00 00 00 00
```

trajectory_append：

```text
40 30 20 10 14 00 00 00 2A 00 00 00 00 00 02 00 E8 03 00 00 88 13 00 00 10 27 00 00 10 27 00 00 14 00 00 00 D0 07 00 00 88 13 00 00 10 27 00 00 10 27 00 00 00 00 00 00
```

上述两段轨迹的 content_crc16 数值为 `0x3BCC`，TRAJ_COMMIT 中按小端 u16 编码。
