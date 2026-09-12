# Loader 软件架构

> 范围：常驻 Loader（`loader/`）及其与 APP、上位机、Flash 布局、构建系统的关系。
> 配套阅读：`loader/docs/UPDATE_FLOW.md`（升级流程）、`docs/loader/IMPLEMENTATION_PLAN.md`（设计冻结记录）。

## 1. 目标与非目标

**目标**

- 复位后校验 APP（记录 + 全镜像 CRC + 向量表），有效则跳转，无效则驻留 Recovery。
- 通过 CAN-FD 接收固件升级：擦除、编程、校验、写记录、激活跳转，全程可重试、可掉电恢复。
- **永不擦写参数/标定区**（`0x0801C000–0x0801FFFF` 全程保持不变）。
- Loader 与 APP 解耦：不依赖 FOC/PID/状态机/通信协议栈；仅复用 vendor HAL 子集与 CMSIS。

**非目标（当前版本）**

- 不做 A/B 双镜像、固件签名/防回滚、Loader 自升级、持久化 boot request。
- 不做 UDS/CANopen 等标准协议；无经典 CAN 分片（仅 CAN-FD）。

## 2. 系统组成与边界

```
┌───────────────────────────── PC（上位机） ─────────────────────────────┐
│ loader/tools/loader_updater.py   单次升级客户端（协议 + 重试 + 计时）    │
│ loader/tools/loader_cycle_test.py 循环耐久测试（CSV/失败快照）          │
│ loader/tools/loader_proto.py     帧编解码 / CRC32 / 常量（纯函数）      │
└───────────────▲───────────────────────────┬───────────────────────────┘
                │ USBCANFD 适配器            │ CAN-FD 1M/1M, BRS
                │ (tools/usbcan_adapter.py)  │
┌───────────────┴───────────────────────────▼───────────────────────────┐
│                            STM32G431CB 板                              │
│                                                                        │
│  ┌───────────────── APP（0x08004000）─────────────────┐                │
│  │ 应用固件（FOC/通信等）                                │               │
│  │ 新增：CAN 0x66 ENTER_BOOT → 安全停机 → 写 Mailbox → 复位 │              │
│  └──────────────────┬──────────────────────────────────┘               │
│                     │ RESET（Mailbox 保留）                             │
│  ┌──────────────────▼──────────────────────────────────┐               │
│  │              Loader（0x08000000，12 KiB）             │              │
│  │  loader_main  启动决策 / 会话状态机 / 命令处理          │              │
│  │  loader_proto 帧解析/构造（无 I/O）                    │              │
│  │  loader_can   FDCAN 传输（RX 环 + ISR）                │              │
│  │  loader_flash 带校验的擦/写/回读 + 记录页               │              │
│  │  loader_crc   CRC-32/ISO-HDLC（= zlib.crc32）         │              │
│  │  loader_jump  镜像校验 + 跳转（VTOR/MSP/PRIMASK）       │              │
│  └──────────────────────────────────────────────────────┘              │
│                                                                        │
│  Flash: Loader(12K) | 记录页(2K) | 备用(2K) | APP(96K) | 参数(16K)      │
│  RAM:   Mailbox(32B) | 各固件 RW/ZI/栈                                   │
└────────────────────────────────────────────────────────────────────────┘
```

**边界规则**

- Loader **不包含**、不链接：FOC、PID、位置环、轨迹、标定、USB、APP 通信协议栈、APP 状态机。
- Loader 允许使用：CMSIS、STM32G4 HAL 子集（rcc/gpio/fdcan/flash/pwr/cortex/hal core）、
  `Core/Src/system_stm32g4xx.c`、`MDK-ARM/startup_stm32g431xx.s`、`shared/boot_mailbox.h`。
- APP 与 Loader 仅通过 4 个契约交互：**Flash 分区、Boot Mailbox、记录页格式、CAN 协议 v1**。

## 3. 模块清单

| 文件 | 职责 | 关键接口 |
|---|---|---|
| `firmware/loader_main.c` | 启动决策、50ms 窗口、会话状态机、命令处理、去重缓存、会话/通道常量 | `main()`、`SysTick_Handler`、`Error_Handler` |
| `firmware/loader_proto.c/.h` | 帧校验/构造、大端读写（纯函数，无 I/O） | `ldr_parse_request`、`ldr_build_reply`、`ldr_get_u16/u32`、`ldr_put_u16/u32` |
| `firmware/loader_can.c/.h` | FDCAN 初始化（PB8/PB9 AF9、1M/1M FD+BRS、精确过滤器 `0x7D0+node`）、RX 环形缓冲(8)、ISR、发送 | `loader_can_init`、`loader_can_rx_pop`、`loader_can_send` |
| `firmware/loader_flash.c/.h` | 页擦除、双字编程+回读校验、记录页读/写（magic+CRC 校验） | `loader_flash_erase_app/_record`、`loader_flash_program`、`loader_record_read/_write` |
| `firmware/loader_crc.c/.h` | 位算法 CRC-32/ISO-HDLC（无查表，省 Flash） | `loader_crc32` |
| `firmware/loader_jump.c/.h` | 镜像全量校验（记录/CRC/SP/Reset 向量）、跳转序列 | `loader_image_valid`、`loader_jump_to_app` |
| `firmware/loader_cfg.h` | Loader 侧布局与协议常量（分区、ID、opcode、结果码、块大小、窗口） | — |
| `shared/boot_mailbox.h` | APP/Loader 共享的复位信箱（`0x20000000`，32B，跨复位保留） | `boot_mailbox_request/consume/clear` |

### 关键设计决策

1. **复位交接而非函数跳转**：APP 收到 `0x66` 写 Mailbox 后 `NVIC_SystemReset()`，
   Loader 上电即接管（电源级天然安全）。
2. **记录页是唯一"可引导凭证"**：`ERASE` 时**先清记录页**再擦 APP——任何时刻掉电都不会
   引导到不完整镜像；升级中断 = Loader Recovery。
3. **直接跳转需恢复 PRIMASK**：`loader_jump_to_app()` 在设置 VTOR/MSP 后必须
   `__enable_irq()`，否则 APP 中断全部失效（曾实测：APP 主循环在跑但 CAN 无应答）。
4. **PROGRAM 块不跨 2 KiB 页**：便于失败重传定位与页级回读验证；上位机按页边界对齐切块。
5. **停止等待 + 单条去重缓存**：同 `seq+session+op+完整请求字节` 的重复请求直接重放缓存应答，
   不重复执行（擦除/编程幂等保护）。
6. **CRC 统一 ISO-HDLC**：固件与 `zlib.crc32` 等价，黄金向量 `"123456789"→0xCBF43926`。

## 4. Flash / RAM 布局

| 区域 | 地址 | 大小 | 访问方 |
|---|---|---|---|
| Loader | `0x08000000–0x08002FFF` | 12 KiB（实占 10.71） | Loader |
| App 记录页 | `0x08003000–0x080037FF` | 2 KiB | Loader 读/写 |
| 备用 | `0x08003800–0x08003FFF` | 2 KiB | 预留 |
| APP | `0x08004000–0x0801BFFF` | 96 KiB | Loader 擦/写；APP 执行 |
| 参数/标定 | `0x0801C000–0x0801FFFF` | 16 KiB | APP（Loader 禁用） |
| RAM Mailbox | `0x20000000–0x2000001F` | 32 B | APP 写 / Loader 读 |
| RAM | `0x20000020–0x20007FFF` | 31.97 KiB | 各固件 RW/ZI/栈 |

记录页结构（24 B，`ldr_app_record_t`，小端存储）：

```
magic(0x41505031 "APP1") | size(u32) | crc32(u32) | version(u32) | image_type(u32) | record_crc(u32)
record_crc = CRC32(前 20 字节)
```

## 5. 与 APP 的契约

| 契约 | Loader 侧 | APP 侧 |
|---|---|---|
| 分区 | `shared/flash_layout/flash_layout.h`（`loader_cfg.h` 别名引用） | scatter `0x08004000/0x18000`、IROM 设置 |
| VTOR | 跳转前设 `SCB->VTOR=0x08004000` | `SystemInit()` 用 `APP_VTOR_OFFSET=0x4000` 设置 |
| Mailbox | `boot_mailbox_consume()`（启动时） | `boot_mailbox_request(ENTER_LOADER)` + 复位 |
| 进入升级 | 等待 `0x7D0` 帧 | 收到 `CAN_ENTER_BOOT`（`shared/protocol/boot_handoff.h`）→ 停机 → 复位 |
| 记录页 | 读写/校验 | 不访问 |
| 参数区 | 不擦写 | `Bsp/flash.c` 独占 |

## 6. 构建系统

- 工程：`loader/mdk/Vector_Mini_ST_Loader.uvprojx`（独立 Target，**不复用 APP 工程**）。
- Scatter：`loader/mdk/Vector_Mini_ST_Loader/Vector_Mini_ST_Loader.sct`
  （`LR_IROM1 0x08000000 0x3000`、`RW_IRAM1 0x20000020 0x7FE0`）。
- 源文件集：6 个 Loader C 文件 + startup + `system_stm32g4xx.c` + 10 个 HAL 模块
  （hal/cortex/rcc/rcc_ex/gpio/fdcan/flash/flash_ex/pwr/pwr_ex）。
- 尺寸门限：**`LR_IROM ≤ 0x3000`（12 KiB）**，超限链接直接报错。
- 工程可再生成：`loader/tools/loader_build/gen_loader_project.py`（从 APP 工程克隆内存/ini
  设置并按新布局重基路径）；APP 工程迁移脚本：`patch_app_projects.py`。

## 7. 资源占用（实测，Release）

| 项 | 值 |
|---|---|
| Loader ROM | 10,972 B（10.71 KiB / 12 KiB，余量 1.29 KiB）|
| Loader RW+ZI | 4,552 B（含 0xC00 栈与堆）|
| 初次启动延时 | ≤ 51 ms（50 ms 窗口 + 校验）|
| 冷启动校验耗时 | 全镜像 CRC（82.8 KiB）约 40 ms |

## 8. 运行环境注意事项

- **单 bank Flash**：擦/写期间取指停顿，Loader 全程无其他实时任务，但不要把时钟/中断依赖
  放在擦写窗口内；看门狗在 Loader 中未启用（生产化需补喂狗策略）。
- **中断**：Loader 自带向量表（`0x08000000`），仅实现 `SysTick_Handler` 与
  `FDCAN1_IT0_IRQHandler`，其余为 startup 的弱默认（死循环）——新增中断需显式实现。
- **FDCAN 过滤器**：仅精确接受 `0x7D0+node`；全局过滤器拒绝不匹配帧。
- **TX 无完成回调**：发送为"入 FIFO 即返回"，忙时有限重试 10 次。

## 9. 安全与可靠性现状

已实现：记录 CRC、全镜像 CRC、向量合法性、擦前失效记录、编程回读、页边界约束、请求去重、
掉电即 Recovery。

生产化 TODO（未实现）：固件签名与防回滚、Loader 区 WRP 写保护、记录页 A/B 冗余、
flash 持久化 boot request、显式 Gate-Disable GPIO 安全态、看门狗、升级确认计数。

## 10. 变更影响矩阵

单一来源（先改这里，再跑同步测试）：

- `shared/flash_layout/flash_layout.h` — 分区/记录页/RAM 窗口
- `shared/protocol/loader_can_v1.h` — 升级协议 v1 全部常量
- `shared/protocol/boot_handoff.h` — `ENTER_BOOT`/`GET_MODE` 命令号
- `shared/boot_mailbox.h` — Mailbox 格式

| 要改的东西 | 必须同步修改 |
|---|---|
| Flash 分区/记录字段 | `shared/flash_layout/flash_layout.h` + sct + `loader_proto.py` 常量 + APP scatter/uvprojx + 本文档 |
| CAN 帧格式/opcode | `shared/protocol/loader_can_v1.h`（wire 不兼容需 bump `LOADER_CAN_PROTOCOL_VERSION`）+ `loader_proto.c/.h` + `loader/tools/loader_proto.py` + 本文档/升级流程文档 |
| 块大小/页约束 | `shared/protocol/loader_can_v1.h`（`LOADER_CAN_MAX_CHUNK`）+ `loader_updater.py`/`loader_fault_test.py`（已消费常量）+ `loader_flash.c` 页校验 |
| Mailbox 格式 | `shared/boot_mailbox.h`（两端同时生效） |
| ENTER_BOOT/GET_MODE 命令号 | `shared/protocol/boot_handoff.h` + `Communication/interface_can.h`（编译期引用）+ `loader_proto.py` |
| 以上任意变更 | 跑 `python tests/test_protocol_single_source.py`（离线；红点=未同步清单），wire 变更再跑 `tests/test_loader_proto.py` + HIL 冒烟 |

> 机制：`loader_cfg.h`、`interface_can.h`、上位机常量只允许**别名/引用** shared 头，
> 禁止字面量副本；`tests/test_protocol_single_source.py` 解析 shared 头并校验
> Python 常量、Keil sct/uvprojx、APP 枚举、参数页地址，任何一处漂移即测试失败。
> 运行时防线：wire 版本字节 + APP `0x67` revision 握手，旧端会显式拒绝而非误解析。
