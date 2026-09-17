# 电机通信与固件升级协议设计规范

> 当前设计入口：[1.2.3 CAN FD 统一协议评审稿](motor_protocol_v1.md)，仅采用 CAN FD 与公司统一帧框架，尚未完成固件实现。本文为历史设计参考，链路范围以当前设计稿为准。

版本：1.0 评审稿　　日期：2026 年 9 月 6 日

适用对象：上位机、RV1126B 网关、电机控制器和 Bootloader 开发人员。

本规范定义经典 CAN 29 位扩展帧上的电机控制、状态反馈，以及可通过 CAN、RS485 和以太网承载的统一固件升级服务。公司统一报文是管理和升级的应用层格式；经典 CAN 实时电机消息采用本规范定义的 8 字节紧凑格式。升级核心只处理完整业务消息和存储操作，不依赖通信驱动。

本文提供可实现的公司标准扩展方案，新增编号和原协议勘误需在公司消息注册表评审登记后冻结。它不是 SAE 发布的电机控制标准，也不宣称完整符合 J1939 全系列或功能安全通信标准。

## 1 设计依据与兼容边界

公司依据为已读取正文的[通信协议设计 副本](https://wcnxj7iyqdkp.feishu.cn/wiki/MM14wIJF7i8irjkx448cHu31nLb)及其关联[消息类型定义](https://wcnxj7iyqdkp.feishu.cn/wiki/DXV4wohO0ioGbskmFAFct8wFngd)。公司原文规定 16 字节帧头、2 字节帧尾校验、小端、统一节点编号、通用消息 0～99 和电机消息 100～199。原文控制消息要求完整承载于单个 CAN FD 帧。

本项目明确使用经典 CAN，单帧只有 8 字节。因此原文的 18 字节固定开销、完整统一帧、控制单 CAN 帧三项不能同时成立。不能通过改用扩展 ID 解决，29 位 ID 不会增加数据区容量。

| 条目 | 本规范决策 | 与公司原文的关系 |
| --- | --- | --- |
| 物理 CAN | 经典 CAN，29 位扩展 ID，数据帧，DLC 固定 8 | 从 CAN FD 主方案扩展 |
| 管理与升级 | 完整公司报文通过 J1939 TP 承载 | 保留应用层逐字节格式 |
| 实时电机消息 | 8 字节紧凑帧，固定字段、定点单位 | 新增 CAN Classic 实时 profile |
| 网关 | 管理与升级原文转发；实时帧做明确定义的封装映射 | 修订原文一概不做字段映射的要求 |
| CRC | 完整报文保留公司 CRC8 和 CRC16；紧凑帧依赖 CAN 硬件 CRC | 必须登记紧凑帧例外 |
| 分片 | CAN 管理服务使用 TP；公司 flags 分片位为 00 | 不叠加两套分片 |
| 控制确定性 | 目标、停止和每条高频反馈均单 CAN 帧 | 保留实时消息不分片的目的 |
| J1939 范围 | 采用 ID 结构、Proprietary A 和 TP 点对点传输 | 自定义电机语义；静态地址；非全系列符合性声明 |

如果公司不接受紧凑帧和网关映射这两项扩展，则只有两个选择：使用 CAN FD，或允许完整控制报文经经典 CAN 多帧传输并撤销原单帧要求。本规范选择前表所述经典 CAN 扩展作为建议实施基线。

## 2 分层与数据路径

```text
电机服务                      固件升级服务
目标 / 使能 / 状态 / 参数      会话 / 镜像 / 写块 / 校验 / 激活
           \                    /
          公司消息类型和固定 payload
                       |
       +---------------+----------------+
       |                                |
实时电机 CAN 紧凑编码             公司完整报文编解码
       |                         /        |         \
经典 CAN 单帧              J1939 TP    RS485 字节流   TCP 字节流
```

同一升级请求在三种接口上的 type、payload、字节序、请求标识和返回码相同。CAN TP 分包、串口拆帧、TCP 粘包由适配层处理。切换接口需要重新建立会话并查询持久化进度，不能沿用旧连接的 RAM 状态。

RV1126B 向 CAN 转发完整公司报文时，逻辑源地址由对应的 CAN 虚拟节点使用；例如来自 0x01 的升级请求在 CAN 上仍用 SA=0x01。系统必须确保该虚拟地址没有另一个实体同时占用。若改用网关代理地址 0x02，必须同时更新公司 src_id 并重算 CRC，属于另一种明确登记的代理模式，不能混用。

## 3 基础编码与节点

所有多字节整数均小端。i16、i32 为二进制补码，u8/u16/u32/u64 为无符号固定宽度整数。字段按表紧密排列，无隐式填充。除另有说明，保留字段发送 0，接收非 0 返回 BAD_FIELD。固定长度消息必须严格匹配长度。

运行期时间间隔使用本地单调时钟和可正确处理计数回绕的差值计算。只有明确允许协商的字段可由远端改变；不通过目标命令直接修改安全限值。

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

## 4 CAN 标识符与承载分类

采用 J1939 的 29 位标识符结构。PDU1 的 PS 是目的地址，PGN 的低 8 位必须为 0，不能将目的地址计入 PGN。布局依据 [Linux J1939 文档](https://www.kernel.org/doc/html/latest/networking/j1939.html)；SAE 数据链路标准索引见 [J1939 21](https://saemobilus.sae.org/standards/j193921_201810-data-link-layer)。

| ID 位 | 字段 | 本规范值 |
| --- | --- | --- |
| 28～26 | Priority | 0 最高，7 最低 |
| 25 | R | 0 |
| 24 | DP | 0 |
| 23～16 | PF | 0xEF 业务；0xEC TP.CM；0xEB TP.DT |
| 15～8 | PS | 目的 node_id，广播为 0xFF |
| 7～0 | SA | 源 node_id |

```text
CAN_ID = (priority << 26) | (PF << 16) | (dst_id << 8) | src_id
业务 PGN = 0x00EF00  即 Proprietary A
TP.CM PGN = 0x00EC00
TP.DT PGN = 0x00EB00
```

公司 16 位 type 不塞入 PGN。紧凑帧以第 0 字节编码可容纳于 u8 的电机 type；完整报文在公司帧头中编码 type。接收端按 PGN 和传输后的长度区分：EF00 的原生 8 字节消息是紧凑帧，TP 重组得到的 18 字节以上消息是完整公司帧。长度 0～7 或 9～17 均非法。

| 类别 | Priority | 说明 |
| --- | --- | --- |
| 紧急功率禁止 MOTOR_STOP | 0 | 广播或单播；软件停止通道 |
| 电机目标 | 2 | 目标覆盖队列，不排队积累过时目标 |
| 快速状态和故障摘要 | 3 | 位置信息、速度、电流 |
| 在线心跳 | 5 | 1 Hz |
| 全部 TP 管理及升级 | 6 | TP.CM、TP.DT 均使用 6 |

示例：0x02 向 0x03 发送速度目标 ID=0x08EF0302；0x03 向 0x02 反馈 ID=0x0CEF0203；0x02 广播停止 ID=0x00EFFF02；0x02 向 0x03 发送 TP.CM ID=0x18EC0302。

首版建议 500 kbit/s；全网可另行静态配置 250 kbit/s，但不得在线自动切速。IDE=1、RTR=0、FDF=0，不使用 BRS。终端电阻、线长、采样点和收发器需按实物验收。收到 FD、标准 ID、远程帧、非法 DLC、未授权源地址或错误目的地址应丢弃并计数，不改变控制状态。

## 5 公司完整报文

| 字节偏移 | 类型 | 字段 | 规定 |
| --- | --- | --- | --- |
| 0 | u16 | magic | 0xA55A，线上 5A A5 |
| 2 | u8 | version | 0x01 |
| 3 | u8 | flags | 见下表 |
| 4 / 5 | u8 / u8 | src_id / dst_id | 与 CAN TP 会话地址一致 |
| 6 | u16 | type | 同一请求与响应使用相同 type |
| 8 | u16 | seq_id | 事务序号；重传不变；响应回显 |
| 10 | u32 | len | 当前 payload 字节数 |
| 14 | u8 | reserved | 0 |
| 15 | u8 | header_crc8 | 覆盖字节 0～14，不含自身 |
| 16 | byte[len] | payload | 按 type 定义 |
| 16+len | u16 | frame_crc16 | 覆盖完整帧头含 CRC8 和 payload，小端存储 |

整帧长度严格等于 18+len。CRC8：width=8，poly=0x9B，init=0，xorout=0，refin=false，refout=false。CRC16：width=16，poly=0xBAAD，init=0xFFFF，xorout=0，refin=false，refout=false。算法采用逐字节先异或到寄存器高位、逐位左移、移出最高位为 1 时异或多项式，无额外末尾补零。本 CRC16 是公司自定义参数，不称 CRC16 CCITT。

| flags 位 | 定义 | 本版本使用 |
| --- | --- | --- |
| 7～6 | 公司应用层分片 | 始终 00；CAN 使用 TP，不复用公司片序 |
| 5 | ACK_REQ | 请求需业务应答为 1；响应必须为 0 |
| 4 | RESPONSE | 请求 0，业务响应 1 |
| 3 | REPORT_REQ | 仅设置上报周期请求使用 1 |
| 2 | RETRY | 原请求重发为 1；seq 和 payload 不变 |
| 1～0 | 保留 | 0 |

flags 0x20 表示需应答请求，0x24 表示重传请求，0x10 表示响应，0x28 表示需应答的上报配置请求。响应不再触发应答。该定义消除原文 bit5 同时描述“请求确认”和“ACK”的歧义，必须在公司主协议同步勘误。

CAN-C 端点不实现应用层分片，收到 flags[7:6] 非 00 返回 UNSUPPORTED（若头部可信且可回复）。最大完整报文 1785 字节，故最大 payload 1767 字节；本板默认能力可以进一步缩小。RS485/TCP 首版也按同一设备能力限长。公司其他端点的大图片传输不受本电机设备配置影响。

## 6 经典 CAN 管理传输

每个完整公司报文作为一条 EF00 消息交给 J1939 TP，使用 RTS / CTS 点对点方式；不采用 BAM 广播升级，不采用 ETP，不另造 OTA 专用分片头。标准 TP 最大 1785 字节，每个 TP.DT 携带 1 字节包序和 7 字节内容，相关接口与数据模型见 [Linux J1939 文档](https://www.kernel.org/doc/html/latest/networking/j1939.html)。

以下 Byte 从 1 开始，仅本节沿用 TP 文献习惯；其他章节偏移从 0 开始。TP 控制字和 PGN 可与 [Linux 内核 TP 实现](https://raw.githubusercontent.com/torvalds/linux/master/net/can/j1939/transport.c)交叉核对。未在此扩展的 TP 行为应使用经过互操作测试的 J1939 TP 实现，并按项目采用的 SAE J1939 21 版本完成符合性检查。

| TP 帧 | Byte1 | Byte2～5 | Byte6～8 |
| --- | --- | --- | --- |
| RTS | 0x10 | 总字节数 u16、总包数 u8、每 CTS 最大包数 u8 | 被传 PGN 小端，00 EF 00 |
| CTS | 0x11 | 本次许可包数、下一包序、FF、FF | 00 EF 00 |
| EndOfMsgACK | 0x13 | 总字节数 u16、总包数 u8、FF | 00 EF 00 |
| Abort | 0xFF | 原因、FF、FF、FF | 00 EF 00 |
| DT | 包序 1～255 | 余下 7 字节为完整公司报文的连续内容 | 本行不含 PGN 字段 |

总包数=ceil(总字节数/7)。末包未使用的数据字节填 FF，不计入公司长度和 CRC。接收端先根据 RTS 限长分配固定缓冲；收齐后裁去填充，验证公司 magic、CRC、长度和地址，再交业务处理。TP 传输成功只表示完整消息到达，不能当作 Flash 编程成功。

首版每个方向的 (CAN 接口, SA, DA) 只允许一条 TP 会话，不以 type 区分并发，因为 TP.DT 不含 type 或被传 PGN。发送完整请求结束后等待业务应答，反方向应答可独立使用 TP。重复 RTS 按 TP 状态机处理，不能直接覆盖活动缓冲。CTS 窗口默认 4，允许接收端降为 1；配置更大窗口需验证 CPU 和 RAM。

TP 超时与重传由选定标准栈负责，不将业务 100 ms 超时强加到 TP。应用事务计时在请求 TP 成功结束后开始；普通命令应答默认 1000 ms；升级长操作采用 ACCEPTED 和状态查询。TP 失败时丢弃整条业务报文，发起方重发完整请求，保持业务 request_id 和 seq_id；不得将 TP 包序当作业务请求序号。

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

GET_CAPS 页 schema=1。page0 内容为 features:u32, supported_modes:u16, max_full_frame:u16, max_block_data:u16, write_alignment:u16, default_watchdog_ms:u16, max_watchdog_ms:u16，共 16 B。features 位 0=CAN 紧凑帧、1=TP、2=RS485、3=TCP、4=签名升级、5=掉电断点、6=A/B 回滚、7=电流模式、8=速度模式、9=位置模式；其余 0。supported_modes 位 n 表示模式 n。max_full_frame 包含 18 B 开销。

page1 内容为 max_motor_rad_milli:i32, max_speed_rad_s_milli:i32, max_iq_mA:i32, stop_policy:u8, max_axes:u8, reserved:u16，共 16 B，表示已配置的应用限制；位置下限另由参数读取。仅提供设备实际实现的能力，禁止通过设置能力位提前宣称支持。

## 8 实时电机控制

### 8.1 模式状态与单位

| 枚举 | 线上定义 |
| --- | --- |
| mode | 0=无控制模式，1=Iq 电流，2=速度，3=位置；4～15 保留 |
| state | 0=BOOT，1=DISABLED，2=READY，3=ENABLED，4=STOPPING，5=FAULT，6=UPDATING；7～15 保留 |

线上枚举独立于固件内部枚举。位置和速度均以电机机械轴为基准，正方向和机械零点由设备参数约定；不得混用电角度、减速器输出轴角度和 r/s。位置为 i32，单位 0.001 rad；速度为 i32，单位 0.001 rad/s；Iq 为 i32，单位 0.001 A。0x80000000 在测量反馈中表示无效，禁止作为控制目标。所有目标还必须通过设备配置的物理范围校验。

“电流控制”指 q 轴电流目标，不直接声称为 N·m 转矩控制。需要转矩接口时另行定义 Kt、齿比、效率和标定有效性；本版不分配未经标定的转矩命令。位置多圈累计超出编码或配置范围前进入故障，不能静默整数回绕。

### 8.2 目标单帧

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

READY 状态允许预装目标但不输出功率。使能前必须有 50 ms 内的新目标，并且速度模式初始目标为 0、电流模式初始目标为 0、位置模式初始目标与当前位置误差不大于 configured_enable_position_window。运行中的变化受已配置电流、速度和加速度限幅约束。

### 8.3 软件停止单帧

type=100 / 0x64，Priority=0。布局为 type:u8, reason:u8, stop_counter:u16, signature:u32；signature 固定 0x53544F50，线上为 50 4F 54 53。reason：0=上位机请求、1=操作员急停、2=系统故障、3=通信故障，其他拒绝。stop_counter 仅用于记录，重复停止仍执行。

接受配置的停止源 0x01、0x02；目的可为本机或 FF，不要求有效控制 lease。收到后立即禁止 PWM 功率输出，撤销控制会话，置软件停止故障位；清故障后仍保持 DISABLED。广播不回复 ACK，以状态上报确认。此消息不保证机械制动、不保证设备断电，也不替代硬件急停或 STO。

### 8.4 管理命令

下表全部走公司完整报文和 TP，ACK_REQ=1。需要控制权的命令在 Q.session_id 中携带控制会话。

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

参数最小注册表：1=最大速度，2=最大 Iq，3=最大加速度，4=位置下限，5=位置上限，6=使能位置窗口，7=目标超时策略，8=受控停止最长时间。1～6 使用 value_type=1 即 i32，单位依次为 0.001 rad/s、mA、0.001 rad/s²、0.001 rad、0.001 rad、0.001 rad；7～8 使用 value_type=2 即 u32，单位分别为枚举、ms。线上 value 字段均为 4 字节，按 value_type 解释。

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

申请控制不会自动改变控制模式或使能输出。默认目标周期 10 ms，看门狗 50 ms，申请可配置 30～200 ms。看门狗只由通过全部校验的新目标刷新，心跳、参数读取和重复包均不刷新。超时执行参数 7 的动作，置目标超时故障并撤销控制权。受控停止期间 state=STOPPING，结束后进入 FAULT。

心跳默认 1 Hz，连续 3 次未收到由上层标记离线并发起系统停止；这是在线监测，不能替代 50 ms 运动看门狗。CAN bus-off、控制队列长期溢出、关键传感器无效必须触发本地保护；总线恢复不自动使能。

## 9 状态反馈

### 9.1 紧凑实时状态

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | u8 | type：120 位置、121 速度、122 Iq |
| 1 | u8 | last_applied_seq |
| 2 | u8 | 高 4 位 mode，低 4 位 state |
| 3 | u8 | fault_summary |
| 4 | i32 | 位置 / 速度 / Iq，单位同控制目标 |

反馈单播到当前控制者，无控制者时发到 0x02。Priority=3，DLC=8。每帧独立采样，三种反馈不声称来自同一时刻；需要一致快照时读取详细状态。初始无有效目标时 last_applied_seq=0，是否存在已应用目标通过详细状态 valid_bits 的位 3 判断。精确控制执行确认还应结合会话和期望模式，不能只看一个回绕序号。

fault_summary 位：0=过流，1=过压，2=欠压，3=过温，4=编码器/传感器，5=目标超时/通信，6=软件停止，7=其他。0 表示无故障摘要；精确原因读取 faults。

默认周期：速度 20 ms、位置 50 ms、Iq 50 ms；SET_REPORT 每项 0 表示关闭，否则允许 10～1000 ms 的 10 ms 整数倍。设备可因预算拒绝配置为 RANGE，但不能成功后静默降频。状态/故障发生变化立即触发三条状态更新，并做 10 ms 合并限频，防故障抖动淹没总线。

### 9.2 详细状态和心跳映射

GET_MOTOR_STATE 的 R 后 32 B：boot_id:u32, sample_counter:u32, position:i32, speed:i32, iq:i32, faults:u32, bus_mV:u16, temperature_centiC:i16, state:u8, mode:u8, last_applied_seq:u8, valid_bits:u8。所有测量从同一发布快照读取；温度单位 0.01 °C。bus_mV=FFFF、温度=0x8000 表示无效，超范围不能回绕。

valid_bits 位 0=位置有效、1=速度有效、2=Iq 有效、3=本会话存在已应用目标、4=母线电压有效、5=温度有效，6～7=0。faults 位 0=过流、1=过压、2=欠压、3=电机过温、4=驱动器过温、5=编码器错误、6=目标超时、7=CAN bus-off、8=软件停止、9=参数无效、10=位置越限、11=硬件驱动故障、12=内部软件故障；其余保留。

CAN 心跳采用 type=123 / 0x7B 紧凑帧：type:u8, state_mode:u8, fault_summary:u8, heartbeat_seq:u8, boot_id:u32。state_mode 与实时反馈相同；心跳序号每次递增。每节点 1 Hz，单播 0x02，Priority=5。公司其他链路使用 type=3 的完整心跳；网关使用本地计时得到 uptime_s 不能假装为设备 uptime，若设备 uptime 未读到则先 GET_INFO 或通过完整状态扩展查询。本版紧凑心跳转发时完整 heartbeat.uptime_s 固定 FFFFFFFF，明确表示未知。

## 10 跨接口电机映射

统一报文携带实时控制时，其 payload 原样等于第 8 节的 8 B 紧凑数据，外层 type 必须等于 payload[0]；flags=0，外层 seq_id 只用于链路计数，command_seq 仍是控制新鲜度依据。网关验证完整帧 CRC 后提取这 8 B，按 src/dst/type 生成 CAN ID；不得重新生成 lease_tag 或 command_seq。

紧凑反馈在其他链路封装为同 type 的公司完整报文，payload 保持原 8 B，flags=0。外层 src/dst 使用 CAN SA/DA，seq_id 由网关对应发送流递增。跨接口保持业务单位，不将定点数重新转换成不确定精度的 float。上述映射仅允许 type=100、103～105、120～122；心跳使用第 9 节单独映射。

CAN 上拒绝通过 TP 接收上述实时 type，确保不出现第二条缓慢控制通道。管理控制命令 101、102、106～112 始终使用完整报文。网关不能缓存目标等待 TP 结束；实时队列与 TP 队列隔离，目标按目的和 type 保留最新一条并执行 TTL。

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

## 12 RS485 和以太网适配

### 12.1 RS485

使用半双工主从轮询，默认 115200 bit/s、8N1，可在全网未使能时静态设为其他共同支持速率。数据流直接承载公司完整报文，不加另一层 Modbus 帧；用 magic、CRC8、len 和 CRC16 找边界，不能用串口空闲间隔代替长度。

只有主站可以主动请求；从站在收到发给自己的合法请求后应答，其他从站保持静默。无自发周期上报，状态由主站轮询，SET_REPORT 在该 profile 返回 UNSUPPORTED。广播仅允许不应答的 MOTOR_STOP；广播升级、使能、参数写入一律拒绝。

主站切换收发方向后留至少 3 字符时间的总线空闲；从站待完整报文验证且总线空闲后开始响应。请求与响应均有最大长度和总时限，不以每个字节无限续期。事务等待预算至少为响应最大线上字节数×10/baud + 处理上限 + 20 ms 裕量；升级长操作采用 ACCEPTED/QUERY。超时重发前恢复接收和总线空闲，保持 request_id。

485 支持短目标报文封装，但完整串行报文开销和轮询周期必须满足同一运动看门狗，不能直接套用 5 电机 CAN 周期。升级期间目标电机保持未使能。

### 12.2 以太网

首版使用 TCP，不定义 UDP 升级。端口通过设备配置/服务发现提供，无默认占用现有公共协议端口的假设。TCP 字节流直接传完整公司报文；一次 recv 可能是半帧或多帧，按固定头与 len 拆分。连接断开立即销毁控制会话并执行失联策略，升级持久进度保留。

限制每连接缓存和设备并发连接数，首版一个维护连接加一个控制连接；同一设备仍只允许一个控制者或一个升级者，两者不能同时占有设备。部署到非受信网络时由网关提供 TLS 和身份鉴别，不能将 CAN 源地址、CRC 或 session_id 当作网络身份认证。

## 13 负载和资源预算

经典 CAN 500 kbit/s 下，预算先按一条 29 位 ID、8 B 帧占用 160 bit 计算，含保守填充和帧间隔预算；这是容量设计值，不是所有帧的精确位长，也不含错误重传。默认每电机 100 Hz 目标、50 Hz 速度、20 Hz 位置、20 Hz Iq、1 Hz 心跳，共 191 帧/s，约 6.1% 总线占用。五个电机约 30.6%，再叠加其他设备、TP、故障突发和实际重传。

单电机 1 kHz 目标加 1 kHz 一条反馈，按同一预算已占 64%；五电机不可能在该配置下实现同样速率。FOC 电流环必须本地运行，CAN 只传外层目标；需要多轴同步和更高频控制时须重新做带宽与同步方案评审。

128 B 升级块对应公司帧 162 B，需要 24 个 DT。CTS 窗口 4 时，一次成功请求约为 1 RTS + 6 CTS + 24 DT + 1 EOMA = 32 CAN 帧；写块应答 38 B，需要 6 DT、2 CTS，加 RTS/EOMA 共 10 帧。单次块交换约 42 帧，按 160 bit/帧约 13.44 ms 总线时间，尚未计 Flash 处理和调度等待。

运动期间给其他节点升级的 TP 总带宽上限建议 10%；按上述预算约 0.95 KiB/s 有效镜像数据，88 KiB 约 95 s 的线上时间，再加擦写和验证。全系统停机维护可增加预算，但仍需硬件实测。禁止沿用原文未经复核的“1 kHz 双向小于 30%”。

控制队列优先于 TP，TP 每次只向外设补充小窗口，不能先塞满硬件 FIFO 再期望 CAN ID 自动解决排队。高优先级帧不能抢占已经开始的低优先级帧；验收需测量最坏排队和控制应用延迟。本板现有 RAM 余量很小，TP 缓冲、镜像块、队列、栈和签名工作区必须静态预算，严禁按远端 len 动态申请任意大小内存。

## 14 与当前固件的实现边界

本次交付是协议设计，不修改电机运行代码。当前工程 CAN 接口使用标准 ID、FD/BRS 和 4 字节大端 float，与本版经典 CAN 扩展帧不兼容，不能仅改变一个 ID 宏就称完成迁移。

| 模块 | 后续实现工作 | 验收重点 |
| --- | --- | --- |
| wire_codec | 公司完整帧、小端、CRC、限长 | 本文黄金向量逐字节一致 |
| can_classic_adapter | 扩展 ID、DLC=8、分流、优先级队列 | 标准帧/FD/RTR 拒绝，长帧缓冲不越界 |
| j1939_tp | 单播 TP 收发、限流、超时、资源上限 | 与 Linux CAN_J1939 互通 |
| motor_service | 外部模式映射、目标原子更新、控制权 | 无隐式使能，丢包和重复不掩盖超时 |
| parameter_service | 类型检查、暂存、原子持久化 | 参数/标定兼容和断电恢复 |
| update_service | 同一升级状态机和命令处理 | 不引用 CAN、UART、socket 驱动类型 |
| boot_storage | 分区边界、擦写、读回、元数据 | 任意掉电不启动残缺应用 |
| host_protocol_tool | 查看状态、控制联调、升级和故障注入 | 统一测试向量和传输抓包 |

建议顺序：先完成纯 C 协议核心和 PC 对照；再实现只读 CAN 链路；然后开放控制状态机；最后安装 Boot 并实现升级与第二接口验证。旧协议只能通过独立固件版本或明确配置入口保留，默认禁止新旧控制入口同时写同一电机目标。

## 15 验收用例

| 类别 | 必须通过的用例 |
| --- | --- |
| 编解码 | CRC 黄金向量、大小端、错误 len、截断、额外字节、非法保留位、随机输入不越界 |
| CAN | 扩展 ID 地址一致性、PGN 分流、经典/FD 区分、广播限制、bus-off 恢复不使能 |
| TP | RTS 长度上限、CTS 窗口、乱序/丢 DT、重复 RTS、尾部填充、超时、同地址对并发拒绝 |
| 控制 | 非持有者拒绝、模式不匹配拒绝、目标越限拒绝、重复/旧序号不刷新看门狗 |
| 状态机 | 带旧目标使能拒绝、停止清除 lease、清故障后仍未使能、超时策略和最长停止时间 |
| 反馈 | 单位换算、无效值、快照一致性、故障位、已应用序号和未应用目标区分 |
| 参数 | 错类型、上下限矛盾、旧 revision、保存中断电、标定区保护 |
| 升级 | 错产品/硬件/签名、越界块、重复块、丢 ACK、不同镜像断点拒绝、最终 Flash 摘要错误 |
| 掉电 | 擦除前后、每个编程单元、检查点提交前后、VERIFIED 提交、激活和第一次启动逐点断电 |
| 跨接口 | CAN 写部分数据后断开，485/TCP 重新 OPEN 后从持久化进度继续 |
| 资源 | RAM 高水位、栈、签名工作区、队列满、TP 限流下最坏控制延迟 |

发布前必须冻结：公司第 1 节兼容修订、消息编号、节点唯一性、CRC 黄金向量、机械限值和失联停止动作、实测总线预算、Boot 分区与恢复入口、签名密钥制造流程。这些是协议发布与产品验证事项，不表示本设计已经完成固件实现或量产认证。

## 16 参考资料

公司规范和消息注册表决定应用层基础格式。J1939 资料用于确定 CAN 地址和 TP 承载边界，电机 payload、命令编号、会话与升级清单是本规范新增定义。

- [公司通信协议正文](https://wcnxj7iyqdkp.feishu.cn/wiki/MM14wIJF7i8irjkx448cHu31nLb)
- [公司消息类型定义](https://wcnxj7iyqdkp.feishu.cn/wiki/DXV4wohO0ioGbskmFAFct8wFngd)
- [SAE J1939 21 数据链路层标准索引](https://saemobilus.sae.org/standards/j193921_201810-data-link-layer)
- [Linux J1939 地址与传输接口](https://www.kernel.org/doc/html/latest/networking/j1939.html)
- [Linux TP 实现参考](https://raw.githubusercontent.com/torvalds/linux/master/net/can/j1939/transport.c)
- [MCUboot 启动与恢复设计](https://docs.mcuboot.com/design.html)

## 附录 A 字节级联调向量

以下向量由随文参考脚本生成，校验覆盖范围与正文一致。脚本用于协议联调，不是 MCU 固件实现。完整向量含 TP 分包见 motor_protocol_vectors.json；生成及自检脚本见 tools/protocol_vectors_reference.py。

<!-- VECTORS -->

CRC 校验串为 ASCII `123456789`：CRC8=`0xEA`；CRC16=`0x932D`。CRC16 数值在报文尾按低字节在前存储。

TEST request，30 B：

```text
5A A5 01 20 02 03 00 00 01 00 0C 00 00 00 00 00 00 00 00 00 01 00 00 00 44 33 22 11 63 4D
```

TEST response，34 B：

```text
5A A5 01 10 03 02 00 00 01 00 10 00 00 00 00 3B 00 00 00 00 01 00 00 00 00 00 00 00 44 33 22 11 BB 5F
```

UPDATE_WRITE 16 bytes，50 B：

```text
5A A5 01 20 02 03 35 00 05 00 20 00 00 00 00 A9 40 30 20 10 05 00 00 00 00 00 00 00 10 00 00 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F FF 51
```

SET_SPEED 10 rad/s，CAN ID `0x08EF0302`，8 B：

```text
68 01 34 12 10 27 00 00
```

SPEED_STATE 9.5 rad/s，CAN ID `0x0CEF0203`，8 B：

```text
79 01 23 00 1C 25 00 00
```

MOTOR_STOP broadcast，CAN ID `0x00EFFF02`，8 B：

```text
64 01 01 00 50 4F 54 53
```

速度样例：控制源 0x02、目标电机 0x03，当前 lease_tag=0x1234、command_seq=1、目标 10000×0.001=10 rad/s。反馈 0x23 表示 mode=2 速度、state=3 已使能，9500×0.001=9.5 rad/s。

TEST 请求完整 TP 时序，窗口 4。表中数据恰为每个 CAN 帧的 8 字节数据区：

| 类型 | CAN ID | 数据 |
| --- | --- | --- |
| RTS | 0x18EC0302 | 10 1E 00 05 04 00 EF 00 |
| CTS | 0x18EC0203 | 11 04 01 FF FF 00 EF 00 |
| DT | 0x18EB0302 | 01 5A A5 01 20 02 03 00 |
| DT | 0x18EB0302 | 02 00 01 00 0C 00 00 00 |
| DT | 0x18EB0302 | 03 00 00 00 00 00 00 01 |
| DT | 0x18EB0302 | 04 00 00 00 44 33 22 11 |
| CTS | 0x18EC0203 | 11 01 05 FF FF 00 EF 00 |
| DT | 0x18EB0302 | 05 63 4D FF FF FF FF FF |
| EOMA | 0x18EC0203 | 13 1E 00 05 FF 00 EF 00 |
