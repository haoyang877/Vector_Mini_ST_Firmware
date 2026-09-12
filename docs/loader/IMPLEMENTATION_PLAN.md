# Loader 实现计划（v1，2026-09-12 冻结）

目标：实现 CAN-FD 升级 Loader；APP 可被反复升级；实测 ≥1000 次升级成功。
决策原则：以"今晚跑通并压测"为最高优先级，协议与分区按本文件冻结，实施中不再变更。

## 1. 冻结决策

### 1.1 Flash 分区（128 KiB，页=2 KiB）
| 区域 | 页 | 地址 | 大小 |
|---|---|---|---|
| Loader | 0–5 | `0x08000000–0x08002FFF` | 12 KiB |
| App 记录页 | 6 | `0x08003000–0x080037FF` | 2 KiB |
| 备用 | 7 | `0x08003800–0x08003FFF` | 2 KiB |
| **APP（重定位）** | 8–55 | `0x08004000–0x0801BFFF` | 96 KiB |
| 参数/标定区（不动） | 56–63 | `0x0801C000–0x0801FFFF` | 16 KiB |

- Loader 硬上限 `0x3000`；APP 硬上限 `0x18000`（超出即失败，必须瘦身）。
- 参数区在迁移与升级全程保持字节不变（升级只擦 APP 区）。

### 1.2 RAM 布局
- `0x20000000–0x2000001F`：Boot Mailbox（Loader/APP 共享，不初始化，跨复位保留）。
- `0x20000020–0x20007FFF`：正常 RW/ZI/栈（两个固件的 scatter 同步改）。
- Mailbox：`magic=0xB00710AD`，`cmd=1` 表示请求进入 Loader。

### 1.3 APP 入口命令
- 现有协议新增参数 `CAN_ENTER_BOOT = 0x66`（4 字节零载荷）。
- APP 收到后：电机 Disable → 写 Mailbox → `NVIC_SystemReset()`。不回包。

### 1.4 升级协议 v1（CAN-FD，标准 ID，BRS 开，1M/1M）
- 请求 ID：`0x7D0 + node`；应答 ID：`0x7E0 + node`。测试节点 `node=0`。
- 帧（总长 ≤64B）：
  `[0]=0xB1 [1]=0x01 [2]=opcode [3]=flags=0 [4:6]=seq BE16 [6:8]=plen BE16 [8:12]=session BE32 [12:12+plen]=payload [..]=CRC32 BE [..]=0xFF 补齐`
  CRC32 = ISO-HDLC（`zlib.crc32` 等价；`"123456789"→0xCBF43926`），覆盖 12B 头+payload。
- 应答 payload 前 2 字节 = result（BE16）：0 OK，1 BAD_FRAME，2 BAD_CRC，3 BAD_STATE，4 BAD_OFFSET，5 BAD_SIZE，6 FLASH_ERR，7 VERIFY_FAIL，8 BAD_OP。
- Opcode：
  - `0x01 GET_INFO` → `[result2][loader_ver u32][app_base u32][app_capacity u32][node u8][pad3]`（BE）
  - `0x02 GET_STATUS` → `[result2][state u8][last_err u8][pad2][next_offset u32][image_size u32][image_crc u32][session u32]`
  - `0x20 BEGIN_UPDATE` payload=`[size u32][crc u32][version u32][type u16][pad2]` → 校验尺寸，生成 session，状态=MANIFEST；**不擦除**。应答 `[result2][session u32][app_base u32]`
  - `0x21 ERASE` payload 空 → 擦 APP 全区（同步执行，应答前完成，主机超时≥5s）
  - `0x22 PROGRAM` payload=`[offset u32][data ≤40B]`；约束：`offset==next_offset`、8 字节对齐、非末块长度为 8 的倍数、不跨 2KiB 页。应答 `[result2][next_offset u32]`
  - `0x23 VERIFY` payload 空 → 全镜像 CRC32 比较。应答 `[result2][computed_crc u32]`
  - `0x24 ACTIVATE` payload 空 → 写 App 记录页 → 先应答 → 跳 APP
  - `0x25 ABORT` payload 空 → 会话复位（记录仍有效则保持可跳转）
- 重传/去重：Loader 缓存"上一请求(seq,session,op,payload) 与应答"，重复请求原样重回应答，不重复执行。
- 停止等待：单请求在途；主机超时 500ms、重试 5 次（ERASE 20s）；VERIFY 10s。

### 1.5 App 记录（页 6，`0x08003000`）
```
u32 magic=0x41505031 ("APP1"); u32 size; u32 crc32; u32 version; u32 type;
u32 record_crc32;   // 前 20 字节的 CRC32
```
- 启动/跳转验证：记录 CRC 有效 → 计算 `[APP_BASE, +size)` 全镜像 CRC 比对 → SP 落在 `0x20000020..0x20008000` 且 8 字节对齐（允许 0x20008000 栈顶）→ Reset 向量 bit0=1 且落在 APP 区内。
- 验证通过且 Mailbox 无请求 → 跳 APP；否则留在 Loader 等待升级。
- 掉电保护：擦除前不破坏旧记录？v1 简化：ERASE 时先清记录 magic（使旧镜像不可引导），升级中断则留在 Recovery——被升级对象可以重下。

### 1.6 其他
- 移除 APP 的 USB（约 -19.5 KiB）以满足 96 KiB；保留 RTT。
- 跳转由 Loader 完成：关中断、停 SysTick、停 FDCAN、设 VTOR=`0x08004000`、设 MSP、跳 Thumb。
- 50ms 启动窗口：复位后 50ms 内收到有效 Loader 帧则留在 Loader，否则验证 APP 并跳转。

## 2. 实施阶段

- P0 环境与链路：JLink/USBCAN 已验证；设备 node0 可通信；整片备份 `outputs/loader_migration_20260912/original_flash_full.bin`。✅
- P1 APP 改造：USB 移除、scatter/VTOR 重定位、Mailbox、`0x66` 命令。
- P2 Loader 固件：`loader/firmware/`（main/can/flash/crc/jump）+ 新 Keil Target。
- P3 宿主工具：`loader/tools/loader_proto.py`、`loader/tools/loader_updater.py`、`loader/tools/loader_cycle_test.py`。
- P4 构建 + JLink 烧录（Loader + APP，不碰参数区）+ 单次升级打通。
- P5 1000 次循环升级压测（CSV 日志 + 失败现场快照）。
- P6 收尾：汇总证据、回到干净可编译状态、更新文档。

## 3. 风险与对策
- APP 超 96 KiB：先移除 USB；仍超则继续裁（RTT/未用模块）。
- CAN 稳定性：沿用已验证的 1M/1M；出错重试 + 失败快照。
- 升级中断：Loader 常驻 Recovery；App 记录失效即不可跳转，防砖。
- 压测时长：单次约 8–12s，1000 次 ≈ 2.5–3.5h；先跑通再压测。
