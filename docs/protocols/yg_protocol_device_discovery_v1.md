# 当前设备发现与支持范围 v1

更新：2026-10-16。适用于 `codex/system-refactor` 本批源码；身份字段已接入版本化目标清单，本轮仅离线验证。
这是当前接入入口；历史提案和 H1 快照不作为现行支持清单。

## 设备身份与能力

1/2 请求保持 12 B 页查询：session_id:u32、request_id:u32、page:u16、reserved=0:u16。
GET_INFO支持page0..2，GET_CAPS仅page0；小端。请求身份按既有编码处理，不代表控制授权或生命周期代际。

| GET_INFO 偏移 | 字段 | 当前来源 |
| ---: | --- | --- |
| 0 | uid[12] | board_hw_read_uid，真实 MCU UID；读取失败不启动通信 |
| 12 | product_id:u32 | shared/targets 共同清单 TARGET_PRODUCT=0x59473236，与升级包一致 |
| 16 | hardware_revision:u16 | TARGET_HARDWARE=1，声明编译目标修订，不表示自动识别实际板卡 |
| 18 | boot_api:u16 | 配套APP/Loader为3；独立APP为0 |
| 20 | application_version:u32 | TARGET_APP_VERSION；Loader无完整激活APP时为0 |
| 24 | boot_version:u32 | 配套构建TARGET_LOADER_VERSION；Loader自己报告，独立APP为0 |
| 28 | boot_id:u32 | 0，不代表持久启动身份、试启动或升级交接票据 |

GET_INFO page0为32B，保持原字段偏移；page1的SN/电机类型与状态、page2目标几何见[升级v3](yg_protocol_upgrade_wire_v3.md)。零版本仍表示未知。
原主机如把零产品编号作为匹配条件，需要改用共同目标身份。

GET_CAPS 为 16 B：features:u32、supported_modes:u16、max_full_frame:u16、
max_block_data:u16、write_alignment:u16、default_watchdog_ms:u16、max_watchdog_ms:u16。
当前features=0x00001D00、supported_modes=0x0004，仅公布已有核心SPEED，最大完整帧64B、载荷46B。
其余三字段仍为 0，未声明的看门狗/写入约束不能解读为可关闭保护或任意 Flash 写入。
117已有零目标SPEED启动链路；实现能力声明与实机验收结论分开，见[能力v2](yg_protocol_core_speed_caps_v2.md)。

## 当前业务路径

| Type | 已接线行为 | 限制 |
| ---: | --- | --- |
| 1/2/3 | 身份、能力、角色查询 | 角色与身份有效位分别编码；不伪造启动代际 |
| 101/110 | 失能、STOP 请求与输出禁止确认 | 101(enable=1) 是低侧制动候选且当前不支持；确认不等于转子静止 |
| 107 | CLEAR 请求，经所有者检查恢复资格 | 不启动，不恢复旧目标；清错后重新自检 |
| 108 | 真实状态快照 | 不提供无来源传感器值 |
| 117 | 普通控制零目标启动，SPEED能力已公布 | 真实主状态机决定准入；其他候选模式未公布能力 |
| 116/124 | 同模式 RUNNING 目标更新、受理与实际应用终态 | 不隐式使能/跨模式；APPLIED 不等于到位，详见 motor wire v1 |
| 120/121/122 | 标量读写、活动 revision、异步保存 | 实际存储提交后才成功；不开放校准完成位任意写入 |
| 160/161/162 | 明确维护种类的受理、查询、取消 | 依据配置/标定资格和实际资源归还，不自动启动控制 |
| 57 | 配套LoaderApp持久确认 | 主站版本/CRC匹配、连续健康、关相与保存空闲；不使能 |
| 56 | 配套 LoaderApp 的升级交接 | 普通 APP 不开放；硬件布局支持及数字关相等由既有所有者复核 |

MIT、低侧制动、运行中隐式跨模式、动态参数表和周期运动状态流未开放。
不能根据存在枚举或编码函数推断业务已实现，也不能将源码完成表述为实机通过。
当前117线路见[启动v0.2](yg_protocol_ordinary_start_wire_v0_2.md)，
能力声明与旧APP兼容以[核心SPEED能力v2](yg_protocol_core_speed_caps_v2.md)为准；旧设计提案只作历史。

## 参数与版本兼容

当前参数为 schema12，4304 B 及 16 KiB 分区保持，ABI=0x000C10D0。
只装载完整 CRC 正确的当前记录；schema11 和更旧记录回退未确认默认值，
不转换、不自动擦写，旧 LUT/标定不会自动获得新版本有效性。
现场需在匹配 APP/Loader 下重新配置、标定并显式保存；回退旧镜像需匹配参数备份。
完整细节以[schema12 装载政策](parameter_schema12_loading_policy_v1.md)为准。

线路契约分别见[电机命令](yg_protocol_motor_wire_v1.md)、
[参数与 CLEAR](yg_protocol_parameter_wire_v1.md)、[升级v3](yg_protocol_upgrade_wire_v3.md)。

## 身份黄金字节

测试 UID 置零，历史通用编码fixture的 GET_INFO payload 的 32 B 小端字节如下。
生产 UID 来自设备；不将此测试值写入固件身份。

```text
00 00 00 00 00 00 00 00
00 00 00 00 36 32 47 59
01 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
```

此历史向量只验证通用编码，当前配套版本还由真实Loader+客户端离线测试核对。
实施边界见[收口记录](../plans/completed/2026-09-29-discovery-contract-closeout-v1.md)。
