# 通信协议

线路约定基线为 2026-11-04 的 `the-system-refactor`。2026-11-09 按 APP 重构分支复核了 108、116/124、STOP 及设备绑定。本文件是唯一现行协议说明。
依据 [yg_protocol.h](../../firmware/communication/yg_protocol.h)、[yg_protocol.c](../../firmware/communication/yg_protocol.c)、[APP 适配](../../firmware/app/app_protocol.c)及[Loader](../../loader/runtime/main.c)。
除镜像清单外，所有多字节整数均为小端；表中偏移从各自 payload 的第 0 字节算起。

## 代码入口

通信实现分为两个同级文件：

- [yg_protocol.c](../../firmware/communication/yg_protocol.c)：CAN 收帧、解码、设备查询、电机与标定业务；
  主线为 `Protocol_RxInterrupt → Protocol_Process → 命令处理 → Protocol_QueueMessage`。
- [yg_protocol_boot.c](../../firmware/communication/yg_protocol_boot.c)：Loader 命令、握手票据、超时和发送完成确认。
- [yg_protocol.h](../../firmware/communication/yg_protocol.h)：值类型、线路结果码和调用接口。

每个 `ProtocolEndpoint` 持有本节点的队列和业务绑定。APP 在启动时绑定所属轴的电机与标定回调、设置节点号及设备信息，并在 BSP 启动后开启 CAN。LoaderApp 另外绑定升级交接服务。

一次命令由 `ProtocolExchange` 保存，`request` 是解码后的消息，`reply` 是本次回复。
分发器及各命令处理函数接收同一个对象。业务参数使用 `ProtocolCommand`、
`ProtocolJobRequest` 等值结构体，执行结果通过对应的结果结构体返回。
`ProtocolMotor` 提供 `start/target/stop/clear_fault` 和 `read_feedback`。108 查询及 116 对应的 124 反馈各自在回包时读取一次所属轴；目标请求先处理，再取反馈。若连续两次读取都被控制中断打断，108 回 BUSY，124 保留本条目标的结果码并清除测量有效位。
六个业务编码函数统一接收 `ProtocolResponse`，填写其中的 `data` 和 `length`。
线路字段仍按约定的小端字节顺序逐项编码。

Keil Watch 展开 `g_protocol_command` 可看 APP 最近一次解析的有效请求。上电时
`type=0`。公共字段为 `type`、`sequence`、`source_node`、`destination_node`、`flags`。
参数按命令号展开对应的 `args` 成员：

| 命令号 | Watch 中的参数 |
| --- | --- |
| 117 启动、116 目标 | `args.motor.mode` 及对应的电流、速度、位置等目标值 |
| 1/2 查询页、101 使能 | 1/2 看 `args.page`，101 看 `args.enable` |
| 3/108 状态查询、107 清故障、110 停机 | 无载荷，看公共字段 |
| 120/121/122 参数、160/161/162 标定 | 参数看 `args.parameter`（value 是原始 32 位值），标定看 `args.job` |
| 56 进入 Loader、57 确认 APP | 56 看 `args.boot_epoch`，57 看 `args.confirm.version` 和 `args.confirm.image_crc32` |

权限与参数检查通过后更新，后续合法查询也会覆盖启动命令。坏帧、错误参数、越权或未知命令保留上一条。
业务返回忙、拒绝或不支持时仍可观察有效请求。快照只含值，接收帧出队后仍有效。
它记录 APP 收到的命令；独立 Loader 固件中的升级数据由 Loader 自己处理。

以 117 启动命令为例：`Protocol_Process → handle_can_request → handle_command →
handle_start_command → request_motor_start → MotorStateMachine_RequestStart`。
`handle_start_command` 检查主机来源、4 字节长度和三个保留字节，直接从 `payload[0]` 读取模式。
它将模式、来源和序号填入 `ProtocolCommand`，调用静态绑定到 `start_motor` 的 `Protocol_Motor.start`。
业务返回后，它把结果码与细节写入 `exchange.reply`。`handle_can_request` 组好回复，
交给 `Protocol_QueueMessage` 编码入队，由 `ProtocolBoot_SendNext` 提交给 CAN 硬件。
接收中断只入队，CAN 格式、长度和 CRC 在前台解包时检查；通用请求条件在 `handle_command` 检查一次。

## 当前支持

APP 支持设备查询、启停、目标更新、反馈和三类标定。线路模式为 **1 电流、2 速度、3 位置、6 Vq、9 开环电压、10 开环电流**。
参数120/121/122及123/125、任务161/162和输出反馈126已接入；101的enable=1不支持。
只有配套 G431 LoaderApp 提供 56/57 交接和确认。AT32 APP 不因此获得 Loader 能力。
版本和支持能力应以实际设备查询为准；代码支持不等于已完成实机验收。

<a id="can-frame"></a>
## CAN 帧

CAN FD：仲裁段 **1 Mbit/s**、数据段 **5 Mbit/s**、29 位扩展 ID、FD=1、BRS=1、RTR=0。
正常节点为 1～254，主站固定为 **2**；255 为广播地址，APP 业务不执行广播请求。

```text
CAN ID = (priority << 26) | (0xEF << 16) | (destination << 8) | source
```

priority 为 0～7，APP 回复和反馈用 3。ID 的 bit25/24 为 0，源/目的节点与应用头一致。

| 应用帧偏移 | 字段 | 字节数 |
| ---: | --- | ---: |
| 0 | magic=0xA55A，字节为 5A A5 | 2 |
| 2 / 3 | version=1 / flags | 各 1 |
| 4 / 5 | source / destination | 各 1 |
| 6 / 8 | CMD / sequence | 各 2 |
| 10 | payload_length | 4 |
| 14 / 15 | reserved=0 / 前 15 字节的 CRC8 | 各 1 |
| 16 | payload，单帧最多 46 B | N |
| 16+N | 前 16+N 字节的 CRC16 | 2 |

CRC8：poly=0x9B、init=0；CRC16：poly=0xBAAD、init=0xFFFF。
均高位优先、不反射、无末异或；CRC16 按小端写入。
逻辑长度为 `18+N`，物理数据长度取能容纳它的最小值：20、24、32、48、64。
其余字节补 0，不计入 CRC；CRC、长度、填充或地址错误时丢帧，无业务回复。

| flags | 含义 |
| --- | --- |
| 0x20 / 0x24 | 要求应答的请求 / 同一请求重试 |
| 0x10 | 命令回复：CMD/sequence 与请求相同，源/目的互换 |
| 0x00 | 124 反馈：使用自己的递增 sequence |
| 0x40 / 0x80 / 0xC0 | 首/中/尾分片，仅 Loader 使用 |

APP 合法请求均为单帧，拒绝分片。控制、标定、参数和升级交接仅允许主站 2；其他来源返回 7，
但坏帧或不符合请求格式时可能直接丢弃。查询可来自其他有效节点。
例如主站向节点 3 发送 STOP、sequence=1、priority=3：ID=0x0CEF0302，物理长度 20 B：

```text
5A A5 01 20 02 03 6E 00 01 00 00 00 00 00 00 75 48 20 00 00
```

<a id="commands"></a>
## APP 命令

以下长度均指 payload。常规回复以 `result:u16, detail:u16` 开头；1/2 查询成功直接返回页面。

| CMD（十进制） | 名称 | 请求长度 | 当前回复/行为 |
| ---: | --- | ---: | --- |
| 1 | GET_INFO | 12 | 第 0～2 页，各 32 B |
| 2 | GET_CAPS | 12 | 仅第 0 页，16 B |
| 3 | GET_DEVICE_STATUS | 0 | 16 B，启动代际目前不可用 |
| 101 | ENABLE | 4 | 4 B；enable=0 停机，enable=1 不支持 |
| 107 | CLEAR_FAULT | 0 | 4 B；请求清错，不启动 |
| 108 | GET_MOTOR_STATE | 0 | 45 B 状态反馈 |
| 110 | STOP | 0 | 4 B；受理与实际关相完成区分 |
| 116 | SET_CONTROL | 20 / 32 | 4 B 结果及一帧 124 反馈 |
| 117 | START_CONTROL | 4 | 4 B；按指定模式请求启动 |
| 118 | RENEW_MOTION | 4 | 4 B result/detail，通信租约 v1 |
| 120 / 121 / 122 | READ / WRITE / SAVE_PARAM | 4 / 12 / 4 | 成功20 B，错误4 B |
| 123 / 125 | READ_SAVED / RESTORE_CONTROL_DEFAULTS | 4 / 4 | 成功20 B，错误4 B |
| 126 | OUTPUT_FEEDBACK v1 | 0 | 成功32 B，错误4 B |
| 124 | MOTION_FEEDBACK | 不接收 | 设备发送 45 B |
| 160 | START_JOB | 4 | 12 B；仅 kind=1、2、6 |
| 161 / 162 | GET_JOB_STATUS / CANCEL_JOB | 4 | 12 B；当前不支持 |
| 56 / 57 | ENTER_LOADER / CONFIRM_APP | 8 | 配套 LoaderApp 的 4 B 回复 |

未列出的 CMD 不执行，也不回复。公共 result：**0 完成、1 已受理、2 不支持、3 长度错、
4 字段错、5 拒绝、6 忙、7 未授权、8 内部失败、11 超范围、17 故障**。detail 按命令解释。

<a id="control"></a>
## 启停和目标

101 请求为 `enable:u8, zero[3]`；117 为 `mode:u8, zero[3]`。
先 117 启动，确认运行状态后再用 116 更新目标；切换模式前先 STOP。
启动不携带目标，detail=0x0105 表示当前不允许启动。收到 1 只表示受理。
STOP/失能只有在所有者确认输出关闭后才返回 0；返回 1 时继续查 108。

116 基本载荷为 20 B，开环模式 9/10 为 32 B：

| 偏移 | 字段 | 类型、单位 |
| ---: | --- | --- |
| 0 / 1～3 | mode / 保留 0 | u8 / 3 B |
| 4 / 8 | position / speed | i32，0.001 rad / 0.001 rad/s |
| 12 / 16 | iq / vq | i32，0.001 A / 0.001 V |
| 20 | electrical_frequency，仅 9/10 | i32，0.001 Hz |
| 24 / 28 | id / vd，仅 9/10 | i32，0.001 A / 0.001 V |

模式 1 用 iq；2 用 speed；3 用 position；6 用 vq；9 用 frequency、vd、vq；10 用 frequency、id、iq。
未使用字段必须为 0。目标只能更新正在运行的同一模式，不隐式启动或切模式。
线路解析保留模式 4/5 的部分字段约束，但当前业务不接受，不能作为可运行模式。

主站递增 u16 sequence。相同序号、相同目标仅在同一工作代际内视为重复；同序号不同目标、旧序号均拒绝。
回绕按有符号 16 位差比较。没有 32 槽历史或运行租约；超时不能证明上一条命令没有执行。

<a id="feedback"></a>
## 电机反馈（108 / 124）

124 是每次 116 处理关联的反馈，不是周期遥测或到位通知。
116 成功更新目标通常立即返回 0；实际行为看测量和状态。108 非空请求的反馈 result=3。

| 偏移 | 字段 | 类型、单位 |
| ---: | --- | --- |
| 0 / 2 | result / correlated_seq | 各 u16；108 的关联序号为 0 |
| 4 | fault_bits | u32 |
| 8 / 12 | reference_position / actual_position | 各 i32，0.001 rad |
| 16 / 20 | reference_speed / actual_speed | 各 i32，0.001 rad/s |
| 24 | valid_bits | u16 |
| 26 / 28 | bus_voltage / bus_current | i16，0.01 V / 0.001 A |
| 30 / 32 | reference_iq / actual_iq | 各 i16，0.001 A |
| 34 / 36 | MCU_temperature / motor_temperature | 各 i16，0.01 °C |
| 38 / 40 | vq / vd | 各 i16，0.001 V |
| 42 / 43 / 44 | node / state / mode | 各 u8 |

位置模式的 reference_position 表示本拍 S 曲线参考位置。
valid_bits 的 bit0～11 按上述 12 个测量字段顺序排列。无效值为该类型最小值
（i16=-32768，i32=-2147483648），对应位清零；NaN、无穷、超范围也按无效处理。
有效数值半值向远离零方向取整。当前没有电机温度来源，MCU 温度不是绕组温度。

state：3 自检、4 标定、5 空闲、6 运行、8 故障；mode 使用线路模式编号。
当前故障位：0 过流、1 过压、2 欠压、4 MCU 过温、5 编码器、6 CAN 断连、
9 配置/极对数/编码器未标定、12 温度传感器或未分类错误、14 电流偏置、15 电气辨识错误。
保留的错误编码不表示退役辨识功能可启动。

<a id="device"></a>
## 设备查询

1/2 请求：`session_id:u32, request_id:u32, page:u16, zero:u16`，共 12 B；前两个字段不建立运行会话。
GET_INFO 每页 32 B：

| 页 | 偏移和字段 |
| ---: | --- |
| 0 | 0 UID[12]；12 product:u32；16 hardware:u16；18 boot_api:u16；20 APP_version:u32；24 Loader_version:u32；28 boot_id:u32 |
| 1 | 0 SN[16]；16 motor_type:u32；20 identity_valid:u8；21 node:u8；22 role:u8；23 boot_state:u8；24 image_crc32:u32；28 image_profile:u32 |
| 2 | 8 个 u32：MCU、layout、parameter_ABI、schema、APP 基址、APP 容量、Loader 容量、身份页基址 |

版本高 16 位为 major、中间 8 位 minor、低 8 位 patch，实际值来自镜像/目标清单。
role：1 APP、2 Loader。boot_state：0 不可用、1 已确认、2 试运行、3 Loader。
独立 APP 的 boot_api、Loader_version、boot_state 为 0；boot_id 当前也为 0，不是启动次数。
SN 是独立生产身份，不等于 MCU UID。缺少设备描述时，1/2 返回 4 B 的 6/0。

GET_CAPS 为 16 B：`features:u32, supported_modes:u16, max_full_frame:u16, max_block_data:u16,
write_alignment:u16, default_watchdog_ms:u16, max_watchdog_ms:u16`。
当前 APP 为 `0x1FD00, 0x064E, 64, 46, 0, 500, 500`；supported_modes 的 bit n 对应线路模式 n。

GET_DEVICE_STATUS 为 16 B：`result:u16, detail:u16, role:u8, boot_state:u8, epoch_valid:u16, boot_epoch:u64`。
当前 APP 为 result=6、role=1、其余 0。升级身份使用 UID、镜像信息和主机 nonce，不使用无效 boot_epoch。

<a id="parameters"></a>
## 参数和标定

参数请求保留校验，但 APP 不读写或保存参数：

- 120：`id:u8, zero[3]`。
- 121：`id:u8, value_type:u8, value_len=4:u8, zero:u8, expected_revision:u32, value[4]`；value_type 为 1/i32 或 2/u32。
- 122：`expected_revision:u32`。

合法主站请求均回复 **2/0**；长度错为 3/0，字段错为 4/6，非主站为 7/0。
参数成功回复、revision和保存结果见下文参数服务v1。Flash 启动记录见[参数模块](../../firmware/parameters/README.md)。

160 请求：`kind:u16, zero:u16`；仅支持 1 编码器 LUT、2 电角零位、6 机械零位。
161/162 请求为非零 `ticket:u32`，均不支持；取消活动标定用 110。其他 kind 不启动任务。
三条命令统一回复 12 B：`result:u16, detail:u16, ticket:u32, kind:u16, phase:u8, effects:u8`。
受理时 ticket 非零、phase=1（进行中）、effects=1（未保存），受理时尚未保存。标定候选就绪后，前台复核原会话和实际关相状态并自动写 Flash。保存成功后回空闲，写失败锁存存储故障。
ticket 只是受理编号，不提供历史查询或后续授权。
detail：1 不支持类别、3 状态不允许、6 字段错误、7 执行失败；失败时任务元数据可全为 0。

<a id="upgrade"></a>
## Loader 升级

先向配套 APP 发 56，载荷为主机生成的非零随机 `nonce:u64`。
1/1 表示等待停机，1/0 表示 READY；APP 只有在 READY 实际发送完成后才交接。
该回复排队/发送期限为 100 ms，失败不跳转。主机随后重新发现同 UID 的 Loader。
普通 APP 不开放 56/57；未开放命令无响应。

Loader 接受 1 和 50～55；升级要求主站 2 单播。nonce 是升级事务标识，session 由 Loader 分配。

| CMD | 请求载荷 | 字节数 |
| ---: | --- | ---: |
| 50 BEGIN | nonce:u64 + manifest[96] | 104 |
| 51 WRITE | nonce:u64 + session:u32 + offset:u32 + length:u16 + data | 18+length |
| 52 FINISH | nonce:u64 + session:u32 | 12 |
| 53 STATUS | 空发现，或 nonce:u64 + session:u32 查询事务 | 0 / 12 |
| 54 ACTIVATE | nonce:u64 + session:u32 | 12 |
| 55 ABORT | nonce:u64 + session:u32 | 12 |

WRITE 的 length 为 8～128 B，offset/length 均 8 B 对齐，连续写入并读回。
主站一次只保持一个完整逻辑请求在途，同一事务重试保持 nonce、session、offset 和数据不变。

50～55 回复 40 B：0/2 result/detail:u16；4 nonce:u64；12 session:u32；16 phase:u8；17 flags:u8；
18 max_block_bytes:u16；20 next_offset:u32；24 image_bytes:u32；28 image_token:u32；32 image_version:u32；36 image_role:u32。
phase：0 IDLE、1 READY、2 ERASING、3 RECEIVING、4 VERIFYING、5 VERIFIED、6 COMMITTED、7 ERROR。
flags：bit0 待激活、bit1 仅完整性校验、bit2 主机 nonce，其余 0。
detail：1 字段、2 nonce、3 session、4 阶段、5 身份、6 范围/顺序、7 Flash、8 数据冲突、
9 镜像校验、10 认证策略不支持、11 输出/链路、12 超时、13 内部错误。

只有 Loader 使用分片：每片都有应用头和 CRC；首/中片 46 B，尾片 1～46 B；首片 sequence=0，逐片加 1，
仅尾片设置 ACK_REQUEST。地址、CMD、版本及非分片标志保持一致。
完全重复的上一片被忽略，不延长超时；片间限 500 ms，首片起总限 2000 ms。
分片序号不能替代 nonce/session/offset。

manifest 为 **96 B 大端容器**：0 magic=0x494D3331；4 profile=0x53593549；8 layout；12 MCU；16 product；
20 hardware；24 version；28 token；32 image_bytes；36 role=1；40 parameter_ABI；
44/46 schema_min/schema_max（各 u16）；48 image_crc32；52～95 为 0。其余数值均为 u32。
镜像按 8 B 补齐；CRC32 为反射 0xEDB88320，初值/末异或 0xFFFFFFFF。
计算时描述符 CRC 字段归 0，64～87 提交标记归 0xFF。
容器/提交标记由[镜像实现](../../shared/image/image_transfer.c)校验，使用 `loader/tools/package_image.py` 打包。

流程：**BEGIN → STATUS 等可写 → WRITE → FINISH → STATUS 等 VERIFIED → ACTIVATE**。
ACTIVATE 回复实际发送后才提交激活；新 APP 启动后，发 57：`version:u32, image_crc32:u32`。
57 要求镜像匹配、连续健康至少 1 s、输出关闭和保存空闲，成功回复 0/0。
再读同 UID 的 GET_INFO，确认版本、CRC、role=1、boot_state=1，才算升级完成。

当前 G431 Loader 是单槽升级，无旧 APP 自动回滚、跨复位续传或签名认证。
试运行 5 s 未确认会停止喂看门狗，重启留在 Loader。
GET_INFO 不延长默认启动窗口；空 STATUS 用于驻留发现，事务 STATUS 可续 5 s 事务租约。
分区/兼容条件看 GET_INFO 第 2 页与 `shared/targets`，不同 layout/ABI 不能混刷。

<a id="implementation"></a>
## 实现和验证

通信位于一个目录：`yg_protocol.c` 负责 CAN 收发流程、设备查询、启停、目标、反馈、标定和参数拒绝；
`yg_protocol_boot.c` 负责 Loader 命令与发送握手，公共头为 `yg_protocol.h`。APP 通过 `Protocol_Init/Start/Process`、`Protocol_RxInterrupt` 接入，
业务交给 AppProtocol/MotorStateMachine，硬件收发使用 BspCan。
RX/TX 各 8 帧；一次 ISR 最多取 3 帧，一次前台最多处理/发送各 8 帧。
执行前为回复和反馈预留空间；发送忙时保留帧，不重复执行业务。
升级票据、具体发送槽位、超时和完成回调都由 Boot 单元持有，普通业务不维护 Loader 状态。
CRC、业务和 Flash 不在接收中断执行。

字节回归数据在 [tests/fixtures/protocol](../../tests/fixtures/protocol/)，旧格式仅用于历史/拒绝测试。
运行 `python tools/run.py verify --profile pr` 检查；离线验证不连接硬件，也不代替实机验收。

## 通信运动租约 v1

2026-11-10新增Type118，CAPS.features位13声明支持。default_watchdog_ms和
max_watchdog_ms均为500，当前版本固定500 ms，不接受禁用。该值是离线开发默认，
正式运行需台架验证停机和调度。系统/试运行硬件看门狗保持独立职责。

Type117成功受理普通运动后建立租约，覆盖模式1/2/3/6/9/10和待启相阶段。
首次受理的新Type116目标续租；重复目标仅返回原结果，不延长期限。
只读查询、畸形或无权报文、过期目标和迟到续租均不续租。
监督上下文按毫秒时钟检查，不依赖CAN前台继续接收。
到期锁存CAN失联故障，使用现有STOP/故障路径关相。反馈fault_code位6可查询。
恢复通信后先107显式清错，再117新启动；GET和118均不启动运动。
标定和升级事务保持各自生命周期，不由此租约续活。

Type118只接受主站2的ACK单播，payload为4字节u32小端renewal_counter。
首次非零，之后按u32半范围严格前进，重复、零和回退拒绝，不更新时钟。
该计数在普通STOP/清错/再次启动间保留，本次上电内有效，复位后失效。
主控进程重启须保留每节点的计数高水位；当前版本不提供高水位读回或接管操作，
丢失高水位时应停止并报错，不能从1盲发或假设设备已复位。
它用于续租去重，不能充当真实启动代际或高频控制会话。
主控须持续递增同一节点的计数，超时后停止发送，显式重新启动后才恢复续租。
回复4字节result:u16/detail:u16，成功00000000，拒绝05000000。
未支持的旧设备不得依靠Type118维持运动许可。
黄金payload向量见tests/fixtures/protocol/motion_lease_v1.json。

参数、任务和输出反馈扩展按以下v1契约解释。

## 参数服务 v1

CAPS.features位14声明120/121/122/123/125整组服务。既有120/121/122请求长度不变。
123读取Flash保存值，请求与120相同；读取需关相停机并复用标定工作区。125恢复控制调参默认值，请求为expected_revision:u32。
所有命令只接受主站2的ACK单播，不能广播。错误回复保留result:u16/detail:u16共4字节。

成功回复共20字节，依次为result:u16=0、detail:u16=0、id:u8、value_type:u8、
access:u8、flags:u8、value:u32、revision:u32、saved_revision:u32。
access沿用目录位0可读、位1关相IDLE可写、位2标定结果。保存/默认回复的id/type/access/value为零。
flags位0保存版本已知，位1活动版本与保存版本不同，位2最近一次Flash写入失败。
业务detail固定为0成功、1未登记、2只读、3类型/字段错误、4范围错误、5状态不允许、
6版本冲突、7持久化失败。具体result分别为0、2、5、4、11、5、5、8。
帧长、来源及保留字段继续沿用原校验结果；例如121结构字段错误仍为4/6。

revision在每次APP启动设为1，只有实际RAM值变化才递增，UINT32_MAX后拒绝新修改和标定。
启动整轴schema14/OEN3记录有效时saved_revision=1，无记录或旧扩展时0；版本仅在本次APP运行内有意义，
不能用作boot_epoch或跨重启去重。123始终读取并验证Flash，不用RAM活动值冒充保存值。
当写失败后Flash仍留有可验证旧记录时可读取旧值，saved_revision=0表示其版本未知。

121仅在IDLE且实际关相、没有待启动/停止、故障或维护锁时改RAM。
expected_revision须等于活动版本；紧邻一次提交的同值同类型重试返回当前值且不增加版本。
同版本写入相同值也不增加版本，版本冲突不写配置。写入重新装配PI并重置积分，
维护标志禁止启相并使监督跳过中间配置扫描，ADC保持运行。
122同步返回实际写入及BSP回读结果；版本未变且已保存时重试不再写Flash。
保存失败使saved_revision未知并保留错误，不自动启相或回放目标。
标定自动保存成功同样推进revision并更新saved_revision。

125仅恢复控制整定/应用包络默认值并推进RAM版本，随后须122保存。
保留节点、生产身份、轴角色/行程、硬件参数、电机模型、热模型及编码器/Hall/输出零位与LUT。
这一范围不改变坐标和传感器模型，所以不清除既有标定资格。模型、安装方向、减速比等只读项
不能通过此服务修改，未来开放时必须单独定义标定失效。
kind7/8仍返回不支持；显式保存使用122，控制默认使用125。

目录单位、类型、范围、权限的实现来源为parameter_values.c/h和parameter_record.c。
参数目录v1登记在firmware/parameters/README.md，黄金payload在
tests/fixtures/protocol/controller_services_v1.json。

## 标定票据 v1

CAPS.features位15声明161/162。160仍只接受kind1/2/6，请求和12字节回复布局保持不变。
每次成功受理产生递增非零ticket，保留最近一次受理任务直到下一次受理或APP重启。
查询/取消须匹配此ticket，旧ticket和重启后未知ticket返回5/2。
160同序号同kind重试返回当前任务而不重启，同序号不同kind返回4/4，
半范围旧序号返回5/5。计数耗尽后拒绝新任务。ticket仅在本次APP运行内有效，重启可能重新使用数值；
客户端须在重新发现/连接恢复时丢弃旧票据。当前协议没有跨重启任务查询或持久任务代际。

161有效查询result=0，phase分别为1运行中、2完成、3取消待关相、
4取消完成、5失败；终态来自状态机真实结束事件，不由IDLE推断。
标定算法阶段内切换继续保留原ticket。完成须实际保存成功并经过COMPLETE；
Flash失败或其他故障返回失败。终态detail=7表示执行失败，8表示Flash保存失败。
运行中detail表达阶段：0x1000准备、0x1001 LUT、0x1002电零位、0x1003机械零位、
0x1004联合电零位、0x1010待发布/保存、0x1011已保存待状态机完成。
失败原因保存在任务终态中，之后107清错仍可查询。
effects=1表示尚未确认Flash持久提交，RAM候选可能已更新。2代表本任务Flash保存已完成，
包括保存失败标记/上次系数的记录更新，即使任务以失败结束也保留2；有效性仍由phase和标定标志判断。
162对进行中任务登记STOP，首次回复通常phase3，持续161查询至4或5；
对终态重复取消返回原终态。普通110同样产生可查询的取消结果。
取消受理后如硬件关相失败必须报告失败，不能伪报取消完成。

## 电机NTC与输出反馈 v1

原108/124的45字节字段和电机轴角度/速度语义保持不变。
电机NTC由独立ADC完成标志确认新样本，监督每10ms读取，超过30ms无新样本、
开短路换算无定义或未安装时bit9清零且温度为INT16_MIN。MCU温度继续使用独立槽位。
该接口补齐测量，不新增绕组NTC跳闸策略，现有热模型及MCU保护继续独立执行。

CAPS.features位16声明Type126。主站2的ACK单播空请求返回32字节v1。
偏移0为result:u16，2为detail:u16，4为version:u16=1，6为flags:u16，
8为age_ms:u32，12为hall_position_mrad:i32，16为tracked_position_mrad:i32，
20为output_zero_mrad:i32，24为gear_ratio_milli:i32，28为sample_counter:u32。
错误只返回4字节。未知版本客户端须拒绝按v1解读。

flags位0独立Hall实测角有效，位1连续跟踪角有效，位2输出机械零位已设置，
位3输出关系标定成功，位4实际存在独立Hall。无传感器不能设置位0或位4。
Hall实测角为校正后的输出单圈角减输出机械零位，回绕到[0,2pi)，
持续跟踪角由既有OutputEncoder结合Hall初始辨圈和转子增量得到，可能跨多圈。
两者使用独立字段，不能用跟踪角冒充独立传感器。
年龄自最后一次伴随新ADC进展的外环采样计算，超过10ms清位0/1，
无首样本时age_ms=UINT32_MAX；无效角使用INT32_MIN，其他标定/安装状态仍可查询。
sample_counter用于观测采样进展，不能当作真实启动代际或同步时间戳。

### 坐标指纹 v1

参数ID0x70提供u32只读坐标CRC32。120读取当前活动坐标，123读取经CRC校验的实际
schema14/OEN3保存记录。两种读取均须停机，标定或待启停时返回状态拒绝。
旧OEN1/OEN2未转换到OEN3保存前，保存指纹读取返回持久化失败。
启动将这类记录的saved_revision置0，显式122可在RAM值未变化时完成OEN3保存。
指纹依次覆盖小端存储的极对数、编码器电/机械零位/标志/方向/保留字/LUT、
Hall幅值偏置及标定标记、OEN3识别字、输出安装参数、Hall LUT状态和完整LUT。
采用与shared/checksum/crc32一致的CRC32/ISO-HDLC及串接顺序。
排除PI整定、启动电流零偏、设备温度及本次上电revision，跨重启相同坐标得到同值。
该指纹用于发现配置变化，不能提供身份认证或无碰撞保证。

主控软件零位持久记录应绑定UID/节点/减速比/方向及此坐标指纹，并核对活动和已保存
指纹一致再恢复。机械标零、LUT或输出关系校准会改变指纹，普通PI调参不改变它。
保存失败或读取不支持/失败时保持待重新确认；RAM revision与APP版本不能替代坐标指纹。

## 连续Type116目标的现状与边界

核对源码b329923，具体执行见motor_state_machine.c的目标写入、motor_control.c的
position_plan_sample/BuildPositionPlan，以及speed_control.c的速度参考斜坡。

模式3的位置目标是单轴OTG终点，终点速度和加速度为零。首次进入位置控制，
规划器以实测电机位置、速度及零加速度建立起点。运行中的改目标从既有规划轨迹的
参考位置/速度/加速度接续，保留约4ms前台建表交接时间，在对应未来参考时刻连接新段表。
实际位置参与反馈闭环和行程检查。连续发送更新的是一个可覆盖目标标量，
同值目标沿用原计划；交接期间多个目标可能合并，既有待建/待发布计划完成交接后
再对最新目标建表。116受理不表示该目标已生成段表或电机已经到达。

模式2更新速度目标，参考速度从当前speed_ramped_rad_s按配置加减速继续推进，
之后由速度PI和可选前馈生成电流。普通速度目标更新保留运行控制器状态，
实测速度作为闭环反馈；频繁目标不是每次从实测速度重新初始化斜坡。
现有模式3及能力位没有开放绕过OTG的周期位置/速度直接跟踪接口。

帧内没有定时执行时间、多轴同步标记或跨节点原子提交。多轴周期下发分别由各节点
接收、调度和规划；发送批次、序号相近和相同目标更新时间不构成同步运动承诺。
主控连续轨迹需以实测跟踪误差、总线调度偏差和设备规划语义验收，
严格共同起动/周期设定点跟踪需后续独立控制会话与执行时序扩展。
本轮仅记录这些现状，位置规划和速度控制实现沿用原执行路径。
