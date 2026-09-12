# Loader 升级流程

> 范围：从"运行时 APP"到"新固件激活并跳转"的完整流程，含协议时序、状态机、
> 异常/掉电恢复与实测数据。配套阅读：`loader/docs/ARCHITECTURE.md`。

## 1. 三种进入 Loader 的路径

| 路径 | 触发 | 行为 |
|---|---|---|
| A. 主动升级 | APP 运行时收到 CAN `0x66` | 安全停机 → 写 Mailbox → 复位 → Loader 驻留升级 |
| B. 冷启动 | 上电/复位，无 Mailbox 请求 | 50 ms 窗口监听升级帧；无帧且记录有效 → 校验并跳 APP |
| C. Recovery | 记录无效 / 升级中断 / 无有效 APP | 驻留 Loader，持续等待升级请求（永不跳非法 APP）|

## 2. 完整升级时序

```
Host                        APP                  Loader
  │                          │                     │
  │  ① TX 0x066 (ENTER_BOOT) │                     │
  │─────────────────────────▶│ ModeSwitch_Handle(Motor_Disable)
  │                          │ boot_mailbox_request(ENTER_LOADER)
  │                          │ NVIC_SystemReset() ─┐
  │                          │                     │ RESET（Mailbox 保留）
  │  ② poll GET_INFO (0x7D0) │                     │ 读 Mailbox=ENTER → 驻留
  │────────────────────────────────────────────────▶│
  │◀───────── GET_INFO 应答（loader_ver/app_base/capacity/node）
  │                                                  │
  │  ③ BEGIN(payload: size,crc,version,type)         │ state=MANIFEST
  │────────────────────────────────────────────────▶│（不擦除）
  │◀───────── [result][session][app_base]            │
  │                                                  │
  │  ④ ERASE                                         │ 先清记录页 → 擦 APP(48页)
  │────────────────────────────────────────────────▶│ state=RECEIVING
  │◀───────── [result]        （~1.1 s，同步返回）     │
  │                                                  │
  │  ⑤ PROGRAM(offset,≤40B) × N  （停止等待）          │ 编程+回读校验
  │────────────────────────────────────────────────▶│ next_offset += len
  │◀───────── [result][next_offset]                  │（页边界处块长自动缩短）
  │        ... 共约 2 千块，~3.5 s ...                │
  │                                                  │
  │  ⑥ VERIFY                                        │ 全镜像 CRC32 比对
  │────────────────────────────────────────────────▶│ state=VERIFIED
  │◀───────── [result][computed_crc]                 │
  │                                                  │
  │  ⑦ ACTIVATE                                      │ 写记录页（magic/size/crc/version）
  │────────────────────────────────────────────────▶│ 应答 → 30ms → 清 Mailbox
  │◀───────── [result]                               │ → VTOR/MSP/PRIMASK → 跳 APP
  │                                                  │
  │  ⑧ poll GET_MODE (0x001)                         ▼ APP 运行（新固件）
  │◀───────── APP 应答 = 升级成功
```

## 3. APP 侧处理（路径 A）

`Communication/interface_can.c`，命令 `CAN_ENTER_BOOT = 0x66`（沿用现有 `(node<<8)|param` ID 体系）：

```c
if (param_id == CAN_ENTER_BOOT) {
    ModeSwitch_Handle(Motor_Disable);                    /* 安全停机 */
    boot_mailbox_request(BOOT_MAILBOX_CMD_ENTER_LOADER); /* 0x20000000 写 magic+cmd */
    __DSB();
    NVIC_SystemReset();
}
```

- 不回包（复位即可）；Host 通过轮询 Loader 的 `GET_INFO` 确认已进入。
- Mailbox 位于 RAM 前 32 B（两个固件的 RW/ZI 都不覆盖），**跨复位保留、掉电丢失**。

## 4. Loader 状态机

```
            ┌──────────────────────────── ABORT ────────────────────────────┐
            │                                                               │
            ▼                                                               │
   ┌─── IDLE(0) ──BEGIN──▶ MANIFEST(1) ──ERASE──▶ RECEIVING(2) ──VERIFY ok──▶ VERIFIED(3)
   │                           ▲                    │      ▲                    │
   │                           │                 VERIFY 失败 └──(可再 ERASE/重传)│
   │                      (可再 BEGIN)               │                          │ ACTIVATE
   │                                                ▼                          ▼
   │                                         ERROR(4) ◀── FLASH 错误       写记录页 → 跳 APP
   └───────────────────── 任意状态可 ABORT ────────────┘
```

门控规则：

| 命令 | 允许状态 | 说明 |
|---|---|---|
| GET_INFO / GET_STATUS | 任意 | 只读，无 session 要求 |
| BEGIN | 任意 | 重新分配 session；不擦除 |
| ERASE | 1/2/3/4 | **先清记录页**再擦 APP；失败 → ERROR |
| PROGRAM | 仅 2 | `offset==next_offset`、8B 对齐、≤40B、不跨 2 KiB 页 |
| VERIFY | 仅 2 且 `next_offset==size` | CRC 失败留在 RECEIVING（可重传/重擦） |
| ACTIVATE | 仅 3 | 写记录页成功才应答并跳转 |
| ABORT | 任意 | 会话归零（记录仍有效则下次复位仍可引导） |

## 5. 协议速查（v1）

> 常量单一来源：`shared/protocol/loader_can_v1.h`。改协议先改它，然后跑
> `python tests/test_protocol_single_source.py`——失败点即"还有哪些文件没同步"的清单。

- 传输：CAN-FD、标准 11 位 ID、BRS 开、1 M/1 M；请求 `0x7D0+node`，应答 `0x7E0+node`。
- 帧：`B1 | 01 | op | flags=0 | seq BE16 | plen BE16 | session BE32 | payload | CRC32 BE`，
  0xFF 补齐到合法 DLC（16/20/24/32/48/64）。CRC 覆盖 12B 头 + payload。
  请求/应答 payload 首 2 字节 = result（BE）。

| op | 名称 | 请求 payload | 应答 payload（含 result2） |
|---|---|---|---|
| 01 | GET_INFO | 空 | `ver u32, app_base u32, capacity u32, node u8, pad3` |
| 02 | GET_STATUS | 空 | `state u8, last_err u8, pad2, next_offset u32, size u32, crc u32, session u32` |
| 20 | BEGIN | `size u32, crc u32, version u32, type u16, pad2` | `session u32, app_base u32` |
| 21 | ERASE | 空 | 无（同步完成，Host 超时 ≥20 s）|
| 22 | PROGRAM | `offset u32, data(≤40B)` | `next_offset u32` |
| 23 | VERIFY | 空 | `computed_crc u32` |
| 24 | ACTIVATE | 空 | 无（应答后跳转）|
| 25 | ABORT | 空 | 无 |

result 码：`0 OK / 1 BAD_FRAME / 2 BAD_CRC / 3 BAD_STATE / 4 BAD_OFFSET / 5 BAD_SIZE /
6 FLASH_ERR / 7 VERIFY_FAIL / 8 BAD_OP`。

**去重**：`seq + session + op + 完整请求字节` 完全一致的重复请求，Loader 直接重放缓存应答，
不重复执行；Host 超时重试用相同 seq 即命中缓存。

## 6. 上位机升级算法

`loader/tools/loader_updater.py: update_firmware(client, image, version)`

```
begin(size, crc32(image), version)          # 超时 1s，重试≤5（同 seq）
erase()                                     # 超时 20s
offset = 0
while offset < len(image):                  # 按 2 KiB 页边界切块
    step = min(40, 2048 - offset % 2048)
    chunk = image[offset:offset+step]
    padded = pad8(chunk, 0xFF)              # 末块补齐 8B 倍数
    assert program(offset, padded) == offset+len(padded)   # 超时 1s，重试≤5
    offset += len(padded)
verify()                                    # 超时 10s，CRC 必须一致
activate()                                  # 应答后 Loader 跳转
wait_app_alive()                            # 轮询 GET_MODE，5s
```

要点：
- **镜像必须 ≥ 实际代码且尺寸 8 字节对齐**；`fromelf --bin` 产物去掉尾部 0xFF 后使用。
- 分块必须避开 2 KiB 页边界（否则 Loader 回 `BAD_OFFSET`）。
- 每次请求 seq 自增；超时重试复用同一 seq（命中缓存安全）。

## 7. 异常与掉电恢复矩阵

| 场景 | 结果 | 恢复动作 |
|---|---|---|
| 应答丢失 | Host 重试同 seq | 缓存重放，无副作用 |
| ERASE/PROGRAM 中途掉电 | 记录页已清 → 记录无效 | 复位后 Loader Recovery；重跑完整升级 |
| PROGRAM 某块回读失败 | 返回 FLASH_ERR，状态 ERROR | 重新 BEGIN→ERASE 下载（v1 无断点续传）|
| VERIFY 失败 | 返回 VERIFY_FAIL，留在 RECEIVING | 重传错误块或整体重擦 |
| ACTIVATE 后 APP 跑飞 | CAN 无应答（Host 超时）| SWD 介入；见 TODO（升级确认/回滚）|
| 升级中断后再次上电 | 停留在 Loader（50 ms 窗口不跳）| Host 直接从头升级 |
| 参数区 | 全程不擦写 | — |

> 已知限制：v1 无断点续传、无签名防回滚、无升级确认计数；ACTIVATE 跳转若立即返回
> 会 `NVIC_SystemReset()` 重新进 Loader（避免死跳）。

## 8. 实测数据（本仓库验证批次）

- **1000/1000 次升级成功**（CSV：`outputs/loader_cycles_full/cycles.csv`，`summary_1000.json`）。
- 单周期：均值 **4751.7 ms**（min 4652.5 / max 4793.9），阶段均值：
  Loader 交接 2.3 ms + 升级 4700.3 ms + APP 应答 49.1 ms。
- 升级阶段细分（单次典型）：begin 1 ms / erase 1079 ms / program 3465 ms / verify 42 ms /
  activate 23 ms / app_alive 101 ms。
- 冷启动（无请求，记录有效）：复位后 ~1.2 s 内 APP 已在 CAN 应答。

## 9. 复现与验收清单

```powershell
# 1) 构建
UV4 -b loader\mdk\Vector_Mini_ST_Loader.uvprojx -t Vector_Mini_ST_Loader   # 0 err / 0 warn, ≤0x3000
# 2) 首次烧录（JLink，保留参数区）
#    erase 0x08000000,0x08003FFF → loadfile loader\mdk\...\Vector_Mini_ST_Loader.hex
# 3) 单次升级
python loader\tools\loader_updater.py --image <app_image.bin> --node 0 --version N
# 4) 冷启动验证：JLink r 复位（Mailbox 为空）→ 直接 GET_MODE 应应答
# 5) 耐久测试
python loader\tools\loader_cycle_test.py --image <app_image.bin> --node 0 --count 1000
```

验收项：① 单次升级 ok/activated；② 复位后自动跳 APP；③ 升级中断电停留在 Loader；
④ 参数区字节不变（对比升级前后整片 dump）；⑤ 1000 次 0 失败。
