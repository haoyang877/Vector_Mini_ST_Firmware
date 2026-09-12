# Resident Loader（CAN-FD 固件升级）

独立自包含的常驻 Loader 模块：复位后校验 APP 并跳转；或通过 CAN-FD 接收固件升级、
校验、写入记录并激活跳转。Loader 永不擦写参数/标定区。

## 目录

```
loader/
├── docs/       软件架构（ARCHITECTURE.md）+ 升级流程（UPDATE_FLOW.md）
├── firmware/   Loader 固件源码（独立 Keil Target 编译）
├── mdk/        Keil 工程 Vector_Mini_ST_Loader.uvprojx + scatter + 构建产物
└── tools/      上位机工具
    ├── loader_proto.py       帧编解码/CRC/常量（纯函数）
    ├── loader_updater.py     单次升级客户端 + CLI
    ├── loader_cycle_test.py  循环升级耐久测试
    ├── loader_fault_test.py  故障注入 HIL 套件（坏镜像/协议误用/救援）
    ├── fault_images.py       合成故障镜像生成器（6 种受控缺陷）
    └── loader_build/         工程生成与 APP 工程迁移脚本
```

## Flash 分区（128 KiB，页 = 2 KiB）

| 区域 | 地址 | 大小 |
|---|---|---|
| Loader | `0x08000000–0x08002FFF` | 12 KiB（实占 10.71 KiB）|
| App 记录页 | `0x08003000` | 2 KiB |
| 备用 | `0x08003800` | 2 KiB |
| APP | `0x08004000–0x0801BFFF` | 96 KiB |
| 参数 / 标定 | `0x0801C000–0x0801FFFF` | 16 KiB（Loader 永不擦写）|

RAM：`0x20000000–0x2000001F` 为 Loader/APP 共享 Boot Mailbox（跨复位保留，不在任何固件
的 RW/ZI 区内）。

## 构建

```
UV4 -b loader\mdk\Vector_Mini_ST_Loader.uvprojx -t Vector_Mini_ST_Loader
```
或 Keil 打开工程按 F7。**尺寸门限 0x3000（12 KiB）**，超限链接即报错。

## 首次烧录（JLink / SWD）

```
erase 0x08000000, 0x08003FFF
loadfile loader/mdk/Vector_Mini_ST_Loader/Vector_Mini_ST_Loader.hex
verifybin ... , 0x08000000
```
注意只擦 `0x08000000–0x08003FFF`；**不要 Erase Full Chip**（会丢参数区）。
烧录后记录页为空 → Loader 停在 recovery，等待一次 CAN 升级写入记录后才会跳转。

## CAN 升级

```
# 单次升级（APP 收到 0x66 后自行复位进入 Loader）
python loader\tools\loader_updater.py --image <app_image.bin> --node 0 --version N

# 循环耐久测试
python loader\tools\loader_cycle_test.py --image <app_image.bin> --node 0 --count 1000

# 故障注入 HIL 套件（CAN + JLink；含坏镜像/协议误用/50ms 窗口救援；末尾自动恢复好 APP）
python loader\tools\loader_fault_test.py --image <app_image.bin> --node 0

# 仅生成故障镜像 + manifest
python loader\tools\fault_images.py --outdir outputs\loader_fault_images

# 故障镜像离线单测（无硬件）
python tests\test_loader_fault_images.py
```
`<app_image.bin>` 为 APP 的 raw 镜像（`fromelf --bin` 产物，去掉尾部 0xFF 亦可）。

## 协议摘要（v1）

> 常量单一来源：`shared/protocol/loader_can_v1.h`（Loader/上位机共享）；
> 一致性由 `python tests/test_protocol_single_source.py` 强制。

- 请求 ID `0x7D0 + node`，应答 ID `0x7E0 + node`；CAN-FD、标准 ID、BRS、1M/1M。
- 帧：`0xB1 | 0x01 | op | flags | seq BE16 | plen BE16 | session BE32 | payload | CRC32 BE`，
  0xFF 补齐到合法 DLC；CRC32 = ISO-HDLC（`zlib.crc32`）。
- 命令：`01 GET_INFO / 02 GET_STATUS / 20 BEGIN / 21 ERASE / 22 PROGRAM / 23 VERIFY /
  24 ACTIVATE / 25 ABORT`。
- PROGRAM 块 ≤40B、8 字节对齐、**不跨 2 KiB 页边界**；应答带 `next_offset`。
- 重传：相同 seq/session/内容的重复请求由 Loader 缓存直接重应答，不重复执行。

## APP 交互

- APP 收到 CAN 命令 `0x66 (ENTER_BOOT)` → 电机 Disable → 写 Mailbox → 复位；
  Loader 读 Mailbox 后驻留升级模式。
- 记录页：`magic=0x41505031("APP1"), size, crc32, version, image_type, record_crc`；
  Loader 启动时校验记录 CRC + 全镜像 CRC + MSP/Reset 向量后才跳转。
- 跳转前 Loader 会恢复中断使能（PRIMASK 清零）——直接跳转不清 PRIMASK 会导致 APP 中断失效。

## 参考

- **软件架构**：`loader/docs/ARCHITECTURE.md`（模块职责/边界/分区/构建/变更影响矩阵）
- **升级流程**：`loader/docs/UPDATE_FLOW.md`（时序/状态机/协议速查/异常恢复/实测数据）
- 设计/协议冻结文档：`docs/loader/IMPLEMENTATION_PLAN.md`
- 迁移/验证证据：`outputs/loader_migration_20260912/`、`outputs/loader_cycles_full/`
