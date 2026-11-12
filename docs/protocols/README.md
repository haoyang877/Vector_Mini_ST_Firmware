# APP 与 Loader 通信协议设计

按 2026-11-10 的 `the-system-refactor` 工作树核对（分支提交 `a87bc5ce`）。G431 与 AT32F456 共用本页的 CAN FD 帧格式；命令由运行中的镜像注册。

| 接收镜像（两板相同） | 受理的业务 | 完成的判据 |
| --- | --- | --- |
| 独立 APP | 1/2/3 查询，101 失能，107/108/110/116/117 电机业务，160 标定；116 关联设备发出的 124 | 启动看 108 的 RUNNING 与模式，停机看数字关相及后续状态，标定由关相发布和写入结果收尾 |
| LoaderApp | 同一套电机业务，另绑定 56 停机交接和 57 镜像确认 | 56 的 READY 回复物理发送完成后才可复位；57 成功后读取身份页确认启动标记 |
| Loader | 1 查询身份，50–55 下载、校验、激活及中止 | 54 成功回包后还需激活、复位试运行及新 APP 的 57 确认 |

依据 [yg_protocol.h](../../firmware/communication/yg_protocol.h)、[yg_protocol.c](../../firmware/communication/yg_protocol.c)、[APP 适配](../../firmware/app/app_protocol.c)及[Loader](../../loader/runtime/main.c)。
命令和应答字段中的多字节整数采用小端序。升级包内的 manifest 和镜像描述符采用大端序。表中偏移从各自 payload 的第 0 字节算起。

## 如何判读一次请求

| 判定点 | 固件检查 | 诊断时看什么 |
| --- | --- | --- |
| 帧能否进入分发 | CAN ID、源/目的、长度、填充、CRC、请求标志 | 坏帧可能被直接丢弃。单帧有效载荷与总线物理长度分别核对。[CAN 帧格式](#can-frame) |
| 当前镜像能否处理 | 镜像角色、已注册服务、来源权限、字段及状态准入 | 合法请求也可能返回不支持、拒绝或忙。先通过 GET_INFO 确定角色，再查对应的[APP 命令](#commands)或[Loader 命令](#upgrade)。 |
| 能否重发 | 116 检查目标序号、值和轴代际；51 检查 nonce、session、offset 及已写数据 | 重发必须沿用原事务的数据。54 回复已在途时重发有已知票据问题，见[Loader 设计](../architecture/loader/loader_product_v3.md)。 |
| 动作是否完成 | 117 受理后由 ADC 确认启相；56 和 54 依赖对应回复的物理发送完成；57 写入确认标记 | 对启动查 108，对升级查同 UID 的 GET_INFO。运动状态、传输 ACK 和 Flash 标记各有独立的产生时机。[状态机设计](../architecture/motor_state_machine_v1.md) |

## 代码入口

APP 通信端点与业务绑定分布如下。

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

Keil Watch 展开 `g_app.endpoint.command` 可看 APP 最近一次解析的有效请求。上电时
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
它将模式、来源和序号填入 `ProtocolCommand`，调用本端点的 `endpoint->motor.start`，由 `AppProtocol_Init` 绑定到 `request_motor_start`。
业务返回后，它把结果码与细节写入 `exchange.reply`。`handle_can_request` 组好回复，
交给 `Protocol_QueueMessage` 编码入队，由 `ProtocolBoot_SendNext` 提交给 CAN 硬件。
接收中断只入队，CAN 格式、长度和 CRC 在前台解包时检查；通用请求条件在 `handle_command` 检查一次。

## 当前支持

APP 支持设备查询、启停、目标更新、反馈和三类标定。线路模式为 **1 电流、2 速度、3 位置、6 Vq、9 开环电压、10 开环电流**。
参数120/121/122及123/125、任务161/162和输出反馈126已接入；101的enable=1不支持。
双板配套LoaderApp提供56/57交接和确认，独立APP按实际CAPS报告升级能力。
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
| 190 / 191 / 192 / 193 | READ / PREPARE / COMMIT / ACTIVATE_MOTOR_CONFIGURATION | 4 / 40 / 16 / 16 | bit17能力门控；完整UID提交和显式重启 |
| 124 | MOTION_FEEDBACK | 不接收 | 设备发送 45 B |
| 160 | START_JOB | 4 | 12 B；仅 kind=1、2、6 |
| 161 / 162 | GET_JOB_STATUS / CANCEL_JOB | 4 | 12 B；票据查询及取消 |
| 56 / 57 | ENTER_LOADER / CONFIRM_APP | 8 | 配套 LoaderApp 的 4 B 回复 |

未列出的 CMD 不执行，也不回复。公共 result：**0 完成、1 已受理、2 不支持、3 长度错、
4 字段错、5 拒绝、6 忙、7 未授权、8 内部失败、11 超范围、17 故障**。detail 按命令解释。

<a id="control"></a>
## 启停和目标

101 请求为 `enable:u8, zero[3]`；117 为 `mode:u8, zero[3]`。
先 117 启动，确认运行状态后再用 116 更新目标；切换模式前先 STOP。
启动不携带目标，detail=0x0105 表示当前不允许启动。收到 1 只表示受理。
STOP/失能只有在所有者确认输出关闭后才返回 0；返回 1 时继续查 108。

### 上位机不限位位置会话 v1（APP1.6.2）

GET_CAPS bit18（0x40000）声明仅本次上位机位置会话解除行程检查。
Type117仍为4字节：`mode, start_flags, 0, 0`；`03 01 00 00`请求不限位位置会话，
`03 00 00 00`保留普通位置控制。flags=1仅接受mode=3，未知标志/非零保留位/能力未声明返回4；
来源仍须为主机节点2，其他来源返回7。旧固件拒绝新标志，主机不能静默退回有限位模式。
只有启动门禁通过才发布易失权限；启动保持当前位置，Type116格式和目标序号规则不变。
权限覆盖位置启动、绝对目标、外环输入与OTG参考，不改变速度、电流、温度、电压和有效反馈要求。
主机来源的合法空载荷Type108仅刷新位置会话反馈时间；运动租约另需新目标或Type118在500ms内续期；其他来源、非法请求不续期。超时锁存CAN断连故障并关相，
STOP、故障、复位清除权限；普通后续会话采用原轴策略。扫描与身份查询本身不解除任何限制。
轴记录、参数ABI、CRC和既有零标志报文保持兼容。
黄金输入见[位置会话向量](../../tests/fixtures/protocol/yg_protocol_host_position_session_v1_vectors.json)。

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

### 抓包示例：启动电流模式并提交 250 mA

下例为离线编码样例。主站 2 对节点 3 单播，优先级 3。请求 ID 为 `0x0CEF0302`，回复 ID 为 `0x0CEF0203`。假设轴在 IDLE 且已满足启动资格。请求 117 的序号取 122，116 的序号取 123。每行含一帧完整 CAN 数据，含尾部 DLC 填充。

```text
117 请求  5A A5 01 20 02 03 75 00 7A 00 04 00 00 00 00 3C 01 00 00 00 A4 09 00 00
117 回复  5A A5 01 10 03 02 75 00 7A 00 04 00 00 00 00 5F 01 00 00 00 A6 64 00 00
116 请求  5A A5 01 20 02 03 74 00 7B 00 14 00 00 00 00 B5 01 00 00 00 00 00 00 00 00 00 00 00 FA 00 00 00 00 00 00 00 EF 71 00 00 00 00 00 00 00 00 00 00
116 回复  5A A5 01 10 03 02 74 00 7B 00 04 00 00 00 00 F3 00 00 00 00 FB 8C 00 00
```

1. 117 的载荷 `01 00 00 00` 选择电流模式。回复载荷恰好也是这四个字节，但回复按两个 u16 解码，含义是 result=1、detail=0，即已受理。此时先用 108 确认 state=6、mode=1，再发示例中的 116。
2. 116 的载荷长度是 20 B。载荷偏移 12 的 `FA 00 00 00` 解为有符号整数 250，单位 0.001 A，即 Iq 目标 +0.250 A。帧长为 16+20+2=38 B，CAN FD 实际发 48 B。CRC16 在 10 个填充零之前。
3. 116 回复为 0/0，表示目标已更新。随后设备发出 124，用载荷中的 `correlated_seq=123` 关联该目标。要判断实际电流是否达到目标，还要检查反馈的有效位和 `actual_iq`。载荷黄金样例与生产编码器见[电流目标与反馈向量](../../tests/fixtures/protocol/yg_protocol_motor_live_v1_9_vectors.json)、[帧编解码](../../shared/protocol/yg_protocol_frame_codec.c)。

### 重发同一条 116 时如何判定

端点只保存最近一次**已接受**目标的值、序号和轴代际。设该序号为 S，当前请求序号为 R，先计算 `delta = (uint16_t)(R - S)`。通讯重试标志不能替代这项检查。

| 条件 | 应答与副作用 |
| --- | --- |
| delta=0，目标字段相同，仍在同一代际 | 返回 OK，保留原目标，无第二次赋值。 |
| delta=0，任一目标值或模式改变 | 返回非法参数；主站若要改变目标，须用新序号。 |
| delta=0，字段相同，但轴已停止并结束原代际 | 返回 DENIED，旧指令不能重新启动输出。 |
| delta 为 1–32767 | 作为新目标，继续检查当前运行态、模式和限值；只有通过后才替换最近已接受记录。 |
| delta 为 32768–65535 | 判为旧序号，返回 DENIED。例：S=65535、R=1 的差值是 2，属于向前推进。 |

START、STOP、标定票据和 Loader 分片各有各自的处理规则。116 的判定来自[目标适配函数](../../firmware/app/app_protocol.c)。

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

举例，仓库的 `live_current_feedback` 向量是节点 1 的独立反馈样本。其 `correlated_seq=123`、`state=6`、`mode=1`；`valid_bits=0x0DFA`。按上述偏移解码得到实际位置 −1.250 rad、实际速度 +2.500 rad/s、母线电压 30.50 V、参考 Iq −0.250 A、实际 Iq −0.249 A。参考位置的原始值为 INT32_MIN 且 bit0 清零，表示此帧没有有效参考位置。此向量与上文节点 3 的正电流请求分属两个示例。[反馈向量](../../tests/fixtures/protocol/yg_protocol_motor_live_v1_9_vectors.json)

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
当前 APP 为 `0x7FD00, 0x064E, 64, 46, 0, 500, 500`；supported_modes 的 bit n 对应线路模式 n。

GET_DEVICE_STATUS 为 16 B：`result:u16, detail:u16, role:u8, boot_state:u8, epoch_valid:u16, boot_epoch:u64`。
当前 APP 为 result=6、role=1、其余 0。升级身份使用 UID、镜像信息和主机 nonce，不使用无效 boot_epoch。

<a id="parameters"></a>
## 参数和标定

### 指定电机型号配置 v1

合并版 APP v1.6.2 通过 GET_CAPS features bit17 声明 190–193。旧本地配参客户端使用的 123、125、126 与远程参数和反馈服务重号，配套上位机须同步更新。
新命令不替代 120/121/122，不修改 schema14/AXS2 或生产身份 motor_type；旧固件未声明时禁止写入。

| 命令 | 请求 | 成功回复 | 副作用 |
|---|---|---|---|
| 190 READ_MOTOR_CONFIGURATION | 4 B：page:u8（0 活动、1 本次已保存），其余零 | 44 B 配置 | 只读；缺失保存页返回不支持 |
| 191 PREPARE_MOTOR_CONFIGURATION | 40 B 配置候选 | 12 B，偏移4为准备票据 | 只准备 RAM 候选，不擦写 Flash，不更改活动配置 |
| 192 COMMIT_MOTOR_CONFIGURATION | 16 B：完整 UID[12]、准备票据:u32 | 12 B，偏移4为保存修订 | 复核完整 UID、当前修订和停机后保存、回读校验，锁定本次启动 |
| 193 ACTIVATE_MOTOR_CONFIGURATION | 16 B：完整 UID[12]、保存修订:u32 | 4 B：1/0 表示受理 | 等待队列及硬件发送完成、实际关相后软件复位；新配置在启动装载 |

40 B 候选：version=1/model/node/flags=1 各u8，expected_revision:u32，speed_limit:i32(mrad/s)，
随后7个i32：持续/峰值mA、时间常数ms、进入/退出/跳闸/清除比（百万分比）。
model 1=HT8115-4、2=YQ-5708、3=YQ-5740，分别只允许节点3/4/5、7、6/8。节点 8 为 Lift，AXS2 产品角色编号为 6。
HT 热参数可全零使用生产预设，YQ 必须明确提供有效值；行程采用所选轴的产品范围。

44 B 配置回复：result/detail:u16，version=1/model/node/flags各u8，revision:u32，
speed_limit:i32(mrad/s)，上述7个i32热参数。
flags bit0 配置确认、bit1 LUT有效、bit2电零位有效、bit3本次已保存待重启、bit4保存失败且锁定、
bit5内存候选已准备（191回复）。活动页始终报告当前活动配置，保存页仅在 Flash 成功回读后提供。
12 B 成功回复：result/detail:u16，ticket_or_revision:u32，version/model/node/flags各u8；
191返回准备票据和bit5，192返回保存修订和bit3。错误回复4 B。读取热参数超出i32缩放范围返回失败。

191/192/193只接受主站2。准备票据在同次运行内递增，不回绕；相同候选重发不分配新票据，
新候选准备失败也先撤销旧票据，避免旧提交请求保存半更新候选。
192/193必须匹配完整 UID；票据不代替设备身份。修订是最近参数记录CRC32，用于冲突检测，不是认证。
提交时再次检查预期修订，拒绝准备后发生的其他参数/标定更新。同一提交重发不重复擦写。
保存只在无故障、IDLE、模式NONE、实际关相、新鲜测量和近零速时执行，先锁定启动与标定。
旧编码器、Hall和输出轴标定整体失效；写入/回读失败保持启动锁定，不自动回滚或恢复输出。
Flash仍为单份记录，掉电可能需恢复参数。保存和活动配置分开，节点变更只在重启生效。

主机保存前以只读查询检查新节点未被占用；激活后先核对新节点的完整UID，再读活动配置，
验证型号、节点、保存修订及标定无效状态。受理不代表生效，超时不能证明操作未执行。
黄金载荷见[型号配置v1向量](../../tests/fixtures/protocol/yg_protocol_motor_configuration_v1_vectors.json)。

参数请求保持原有长度和字段布局：

- 120：`id:u8, zero[3]`。
- 121：`id:u8, value_type:u8, value_len=4:u8, zero:u8, expected_revision:u32, value[4]`；value_type 为 1/i32 或 2/u32。
- 122：`expected_revision:u32`。

合法主站请求交给参数服务；长度错为3/0，结构字段错为4/6，非主站为7/0。
参数成功回复、revision和保存结果见下文参数服务v1。Flash 启动记录见[参数模块](../../firmware/parameters/README.md)。

160 请求：`kind:u16, zero:u16`；仅支持 1 编码器 LUT、2 电角零位、6 机械零位。
161/162请求为非零`ticket:u32`，分别查询和取消任务；110也可中止标定。其他kind不启动任务。
三条命令统一回复 12 B：`result:u16, detail:u16, ticket:u32, kind:u16, phase:u8, effects:u8`。
受理时 ticket 非零、phase=1（进行中）、effects=1（未保存），受理时尚未保存。标定候选就绪后，前台复核原会话和实际关相状态并自动写 Flash。保存成功后回空闲，写失败锁存存储故障。
ticket 只是受理编号，不提供历史查询或后续授权。
detail：1不支持类别、2过期票据、3状态不允许、4冲突重试、5过期序号、6字段错误、
7执行失败、8保存失败；运行中的阶段detail见标定票据v1。拒绝受理时任务元数据可全为0。

<a id="upgrade"></a>
## Loader 升级

先向配套 APP 发 56，载荷为主机生成的非零随机 `nonce:u64`。
1/1 表示等待停机，1/0 表示 READY；APP 只有在 READY 实际发送完成后才交接。
56 的停机交接窗口从登记起为 1000 ms，READY 回复的排队/发送期限为 100 ms。发送失败或复核超时结束本次交接。主机随后重新发现同 UID 的 Loader。
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
线路 phase 定义为 0 IDLE、1 READY、2 ERASING、3 RECEIVING、4 VERIFYING、5 VERIFIED、6 COMMITTED、7 ERROR。当前 Loader 激活后复位，`phase_code` 没有产生 6 的分支，完成判据是同 UID 新 APP 的确认状态。
flags：bit0 待激活、bit1 仅完整性校验、bit2 主机 nonce，其余 0。
detail：1 字段、2 nonce、3 session、4 阶段、5 身份、6 范围/顺序、7 Flash、8 数据冲突、
9 镜像校验、10 认证策略不支持、11 输出/链路、12 超时、13 内部错误。

只有 Loader 使用分片：每片都有应用头和 CRC；首/中片 46 B，尾片 1～46 B；首片 sequence=0，逐片加 1，
仅尾片设置 ACK_REQUEST。地址、CMD、版本及非分片标志保持一致。
完全重复的上一片被忽略，不延长超时；片间限 500 ms，首片起总限 2000 ms。
分片序号不能替代 nonce/session/offset。

### 一次 BEGIN 如何跨越三个物理帧

BEGIN 的 104 B 逻辑载荷由 8 B nonce 与 96 B manifest 组成。按 46、46、12 B 切成三片。第一片携带 nonce 和 manifest 的前 38 B，第二片携带随后 46 B，尾片携带最后 12 B。三片的物理长度分别为 64、64、32 B，每片独立计算头 CRC8 和 CRC16，分片序号依次为 0、1、2。成功重组后 Loader 才能校验完整清单、建立写会话并安排擦除。有效首片可先使 Loader 驻留，以免接收中途跳入 APP。

例如 nonce `0x0123456789ABCDEF` 在线路上传为 `EF CD AB 89 67 45 23 01`，其后 manifest 的大端 magic 为 `49 4D 33 31`。这两个相邻字段使用不同字节序。升级黄金向量给出了这组字节，其历史 layout 用于验证编码；实际升级须匹配设备的 MCU、layout、ABI 等目标值。[升级载荷向量](../../tests/fixtures/protocol/yg_protocol_upgrade_wire_v3_vectors.json) · [分片接收器](../../shared/protocol/yg_protocol_fragment.c)

### 下载过程中发生超时或重发

| 时刻 | 设备如何处理 | 可用的核对方式 |
| --- | --- | --- |
| 活动事务中重复 BEGIN | 来源、nonce 和 manifest 全部相同则返回该会话状态；其他 BEGIN 返回忙 | 比较回复中的 session、token、image_bytes 和阶段。 |
| WRITE 回复丢失 | 保持同一绑定、偏移和字节重发。旧偏移处逐 8 B 比较实际 Flash，内容一致才可复用；下一写入不能跳过 `next_offset` | 用绑定的 53 读取 `next_offset`，结合 WRITE 结果码判断是否前进。 |
| 分片中断 | 片间与总时限独立于 5 s 下载租约。片序或元数据不匹配不能组装出有效请求 | 观察物理片序、CRC 和 ACK_REQUEST，重组完成后再查事务状态。 |
| 54 回复在途 | 票据须匹配具体回复帧的物理完成事件。AT32 首次回复仍占槽时重发 54，第二次提交失败可撤销原票据 | 激活后的完成证据是同 UID 新 APP 的版本、CRC 和确认状态；已有边界见[Loader 设计](../architecture/loader/loader_product_v3.md)。 |

APP 的 56 查同一交接请求时要保持来源、nonce 和 sequence 一致。首次 56 锁定新启动并请求 STOP，PENDING 期间不颁发 ACK 票据。READY 的发送完成才触发 APP 的复位路径。该事务与复位后由 Loader 分配的下载 session 分别管理。[APP 交接](../../firmware/app/app_loader.c) · [Loader 事务](../../loader/core/upgrade.c)

manifest 为 **96 B 大端容器**：0 magic=0x494D3331；4 profile=0x53593549；8 layout；12 MCU；16 product；
20 hardware；24 version；28 token；32 image_bytes；36 role=1；40 parameter_ABI；
44/46 schema_min/schema_max（各 u16）；48 image_crc32；52～95 为 0。其余数值均为 u32。
镜像按 8 B 补齐；CRC32 为反射 0xEDB88320，初值/末异或 0xFFFFFFFF。
计算时描述符 CRC 字段归 0，64～87 提交标记归 0xFF。
容器/提交标记由[镜像实现](../../shared/image/image_transfer.c)校验，使用 `loader/tools/package_image.py` 打包。

流程：**BEGIN → STATUS 等可写 → WRITE → FINISH → STATUS 等 VERIFIED → ACTIVATE**。
ACTIVATE 回复实际发送后才提交激活；新 APP 启动后，发 57：`version:u32, image_crc32:u32`。
57 在前台处理。LoaderApp 要求连续 11 个至少 100 ms 的健康检查间隔、当前健康且通信就绪、实际关相、无已登记的 56 事务，并由 BootHw 核对版本、CRC 和试运行标记。成功回复 0/0。
再读同 UID 的 GET_INFO，确认版本、CRC、role=1、boot_state=1，才算升级完成。

两板 Loader 均采用单 APP 槽和 CRC32 完整性校验。下载中断后须重新提交完整镜像。
试运行 5 s 未确认会停止喂看门狗，重启留在 Loader。
GET_INFO 不延长默认启动窗口；空 STATUS 用于驻留发现，事务 STATUS 可续 5 s 事务租约。
配套镜像的 layout、ABI 及分区值来自 `shared/targets`。当前 G431 独立 APP 的第 2 页也使用配套布局，尽管该工程实际从 `0x08000000` 链接。识别独立镜像时还要核对第 0 页 boot_api 和实际工程。启动决策与激活票据边界见[Loader 状态机](../architecture/loader/loader_product_v3.md)。

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
端点测试应分别观察物理收帧、业务动作次数、回复和反馈，而不只比较返回码。改变线路字段时核对四类场景：CRC 或填充错误时无业务副作用；合法帧但字段错误时收到该命令规定的错误；成功 116 同时有回复与可关联的 124；TX 拥塞或旧目标重发时不会执行第二次业务写入。分片重复、超时与回绕由独立的 Loader 分片用例覆盖。[端点回归](../../tests/unit/native/test_yg_protocol_link.py) · [分片回归](../../tests/unit/native/test_yg_protocol_fragment.py)
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
同版本写入相同值也不增加版本，版本冲突不写配置。
定点解码到float32后可能合并邻近整数；只有实际物理值变化才推进版本，成功回复返回实际可表示值。写入重新装配PI并重置积分，
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
