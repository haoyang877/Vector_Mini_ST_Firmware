# Loader 设计评审与优化计划（2026-09-13）

> 范围：`loader/`（固件 + 工具 + Keil 工程）、`shared/` 契约、升级验证证据、量产就绪度与 485 扩展路线。
> 方法：源码走查 + 原理图核对（`docs/SCH_Vector_Mini_ST_2026-07-22.pdf`）+ FD6288Q 数据手册 + 既有压测证据审计。
> 结论：协议/工具链已具备量产骨架；本批补齐 **运输层解耦（link seam）、bus-off 恢复、功率级安全态** 三项 P0，其余按 §6 清单推进。

## 0. 四个问题的直接回答

| 问题 | 结论 | 摘要 |
|---|---|---|
| 是否和硬件解耦 | **协议层已解耦，运输/存储/时钟仍直连 STM32G4 HAL** | 本批实施 `loader_link` 运输层接口；换 MCU 时重写面缩小为 3 个 port 文件 + startup/linker |
| 升级压测是否足够 | **协议级足够，系统级不足** | 已有 1000/1000 循环、14/14 故障注入；缺真实断电、bus-off、多节点、Flash 耐久预算、电压/温度边界 |
| 能否上产线量产 | **还差关键项** | 本批补：功率级安全态、bus-off 软恢复；待补：Product/HW ID 与 image_type 校验（记录 v2）、看门狗策略、WRP、真实断电 HIL |
| 485 扩展能力 | **软件具备（宿主端已抽象 + 本批固件 seam）；本板无 485 收发器硬件** | 需新增 `loader_uart.c` + 流式帧解析 + 宿主 `SerialBus`；多 drop 寻址需协议 v2，点对点先行 |

## 1. 架构解耦评估

### 1.1 分层现状（耦合点清单）

| 文件 | 职责 | 当前耦合 | 换 MCU / 换链路影响 |
|---|---|---|---|
| `loader_proto.c/.h` | 帧校验/构造（纯函数，无 I/O） | **无** | 0 修改 |
| `loader_crc.c/.h` | CRC-32/ISO-HDLC（位算法） | **无** | 0 修改 |
| `loader_jump.c/.h` | 镜像校验 + 跳转（VTOR/MSP/PRIMASK） | Cortex-M 通用（CMSIS） | 0 修改（Cortex-M 内） |
| `loader_flash.c/.h` | 页擦/双字编程/回读 + 记录页 | `HAL_FLASH`、STM32 页语义 | 重写（port 文件） |
| `loader_can.c/.h` | FDCAN 传输（PB8/PB9、1M/1M） | `HAL_FDCAN`、引脚、位时序 | 重写（port 文件，本批已加 seam） |
| `loader_main.c` | 启动决策/状态机/命令处理 | `HAL_Init`、`SystemClock_Config`（PLL 170M） | 时钟配置段重写 |
| `shared/flash_layout.h`、`shared/protocol/*` | 分区与协议单一来源 | **无**（被 loader/工具/APP 共同引用） | 0 修改 |

评价：**协议/CRC/布局/记录格式已彻底解耦**；运输层此前在主逻辑里直写 CAN API（`loader_can_send/rx_pop` 共 6 处调用），是换 MCU/加 485 的最大阻碍——本批评审后立即修复（§5）。

### 1.2 本批实施：`loader_link` 运输层接缝

新增 `loader/firmware/loader_link.h`（不引入 vendor 类型）：

```c
bool loader_link_init(uint8_t node);                 /* 失败→调用方停机 */
bool loader_link_rx_pop(ldr_rx_frame_t *out);        /* 仅返回本节点帧 */
void loader_link_send(const uint8_t *data, uint8_t len); /* 尽力发送应答 */
void loader_link_shutdown(void);                     /* 跳转 APP 前静默 */
```

- `loader_can.c` 降级为「CAN 传输实现」，`hldr_can` 改 file-static；
- `loader_main.c` / `loader_jump.c` 只依赖 `loader_link.h`；
- 核心逻辑不再引用 `LDR_REQ_BASE/LDR_RSP_BASE`（应答寻址由传输层负责）；
- 新增传输（485/其他 MCU）时：只新增一个实现文件，核心 0 修改。

### 1.3 遗留（记录在案，不在本批）

- `loader_flash.c` 的存储栈仍直连 `HAL_FLASH`：作为「HAL Port」候选文件可接受；后续若按 `AGENTS.md` 建立 `hal/ports/stm32g4/storage/`，将其平移即可。
- `SystemClock_Config` 属 BSP 职责；换 MCU 时随 startup/linker 一起重写，属预期成本。

## 2. 升级压力测试评估

### 2.1 已有证据（可复核）

| 证据 | 结果 | 路径 |
|---|---|---|
| 1000 次循环升级 | **1000/1000 成功**；单周期均值 4751.7 ms（min 4652.5 / max 4793.9） | `outputs/loader_cycles_full/summary_1000.json` + `cycles.csv` |
| 故障注入 HIL（14 用例） | **14/14 通过**（46.1 s）：协议误用 7、坏镜像 4、窗口救援 1、复位环救援 1、中断升级恢复 1 | `outputs/loader_fault_full_20260912_223900/summary.json` |
| 离线单测 | `test_loader_proto.py`（帧/黄金向量）、`test_loader_fault_images.py`、`test_protocol_single_source.py`（常量防漂移） | `tests/` |
| 单次升级全链路 | ok/activated，分阶段计时留档 | `outputs/loader_upgrade_sot_20260913_005317/` |

### 2.2 覆盖缺口（按优先级）

| # | 缺口 | 现状 | 建议 |
|---|---|---|---|
| G1 | **真实断电矩阵** | 「中断升级」用 JLink 复位模拟，未真断 B+/3V3 | 继电器在 ERASE/PROGRAM/VERIFY/ACTIVATE 各阶段随机断电 ≥100 次，验收：全部进 Recovery 且可重升级 |
| G2 | **bus-off / 总线故障** | 固件无恢复逻辑（本批已实现软恢复）；无 HIL 用例 | 升级中途短接 CAN_H/L 或拔线，验收：恢复后主机重试成功 |
| G3 | **多节点** | 协议支持 `0x7D0+node`，仅验证过单节点；`LDR_CAN_NODE` 硬编码 0 | 双节点同总线互不误触发；节点号配置化（记录页或编排） |
| G4 | **Flash 耐久预算** | 每次升级擦 48 页 APP + 1 页记录；**耐次数未核对**（UNKNOWN，见 §7） | 按数据手册核算「每器件最大升级次数」并写入规格 |
| G5 | 接近上限镜像 | 当前镜像 83.4 KiB / 上限 96 KiB | 补 ~96 KiB 边界镜像升级用例 |
| G6 | 电压/温度边界 | 未测 | 量产 EOL 抽样（低压 13 V / 高压 30 V、-20/+60℃） |
| G7 | 参数区不变性 | 设计上不擦写（代码可证）；1000 次压测未见自动整片 dump 对比留档 | 在循环测试中加「参数区 CRC 前后比对」自动化 |
| G8 | 看门狗 | Loader 未启用 IWDG（见 §3.2 P1） | 超时必须覆盖最坏 1.1 s 擦除阻塞（喂狗点设计） |

### 2.3 结论

- **协议正确性**：1000 次循环 + 14 类故障注入已达到可量产的置信度；
- **系统鲁棒性**：G1/G2/G3/G4 是上产线前必须补齐的证据项，其中 G2 的**代码侧修复已在本批完成**，仅缺 HIL 用例。

## 3. 量产就绪度评估

### 3.1 已具备

- 掉电安全次序：`ERASE` 先清记录页再擦 APP —— 任意时刻掉电不会引导不完整镜像（Recovery 是一等状态）；
- 逐块编程回读校验 + VERIFY 全镜像 CRC32 + SP/Reset 向量合法性校验；
- 50 ms 启动窗口救援（故障注入已验证）；
- 参数/标定区全程不擦写；记录页 CRC 保护；
- 常量单一来源 + `test_protocol_single_source.py` 防漂移；
- 工具链：单次升级 / 循环压测 / 故障注入 / 救援，均有 CLI；
- 开发期防护：Keil 下载布局门禁（`host_app/tools/check_app_layout.py` 已接入 APP/HIL 两工程）+ 下载后 Loader 完好性校验（`host_app/tools/verify_loader_intact.py`）。

### 3.2 差距清单

**P0（本批）**

| 项 | 动作 | 本批状态 |
|---|---|---|
| 功率级安全态 | Loader 启动即显式驱动 6 路 PWM 输入为低（见 §3.3 硬件确认） | 本批实施 |
| CAN bus-off 恢复 | `FDCAN_IT_BUS_OFF` + 清 `CCCR.INIT` 软恢复 | 本批实施 |
| 运输层解耦 | `loader_link` seam（§1.2） | 本批实施 |

**P1（量产前）**

| 项 | 内容 | 备注 |
|---|---|---|
| 记录 v2 + ID 校验 | Product ID / Hardware ID / image_type 校验；BEGIN payload 扩展；协议版本 bump（设计见 §6.2） | 需产品注册值确认（UNKNOWN） |
| 看门狗策略 | IWDG 启用 + 喂狗点（超时 > 最坏阻塞 ~1.1 s） | 与 APP 策略协调 |
| 真实断电 HIL | G1 矩阵 | 需继电器工装 |
| 耐久预算 | G4 核算 + G7 自动化 | — |

**P2（长期）**

| 项 | 内容 |
|---|---|
| 固件签名 / 防回滚 | 安全威胁模型成立时引入 |
| 记录页 A/B | **当前价值低**：单镜像直写下，掉电窗口只影响记录自身，A/B 不能恢复旧 APP（旧 APP 已被擦）；待引入「升级确认/回滚」机制时一并做 |
| Loader 自升级 | 当前仅 SWD；明确维护策略 |
| WRP 保护 | 量产件可选；代价：更新 Loader 需先解保护。当前以「构建门禁 + 下载后校验」替代 |
| 产线烧录 SOP | Loader+APP+记录一次夹具完成 |

### 3.3 功率级安全态硬件确认（原理图 + 数据手册）

- 门极驱动 U7 = **FD6288Q**（三相半桥驱动，QFN24）；
- **无 ENABLE/nSLEEP/nFAULT 引脚**；VCC 常供 12 V；无外部下拉；功率级唯一控制通道 = 6 路 PWM 输入（HIN1-3/LIN1-3）；
- FD6288Q 特性：**输出与输入同相**、输入内置下拉电阻 RIN=140–260 kΩ（typ 200 kΩ）、内置直通防止与 200 ns 死区；
- 映射：PA8→HIN3、PA7→LIN3、PA9→HIN2、PB0→LIN2、PA10→HIN1、PB1→LIN1；
- **结论（已确认，非猜测）**：六路 PWM 输入全低 = 功率级安全态。复位默认（模拟输入 + 内部下拉）本已安全，但 Loader 现在**显式**把六脚配置为推挽输出低，消除悬空/扰动风险。APP 启动后由 TIM1 MspPostInit 重新配置为 AF6，无冲突。

## 4. 485 扩展能力评估

### 4.1 宿主端（已具备）

`loader/tools/loader_updater.py` 已有 `Bus(Protocol){send, receive}` 抽象 + `LoaderClient` + `update_firmware()`；新增 `SerialBus` 实现即可复用全部升级/重试/去重逻辑。

### 4.2 固件端（本批 seam 后）

1. 新增 `loader_uart.c` 实现同一 `loader_link` 接口（init/rx_pop/send/shutdown）；
2. **流式帧解析**：现有帧含 magic(0xB1)+版本+plen+CRC32，可重同步；CAN 的 DLC 补齐（0xFF 填到 16/20/24/32/48/64）对串口非必需——发送侧按 12+plen+4 精确长度，LOADER 解析已兼容「无补齐」帧（仅要求 `len ≥ need`）；
3. **半双工方向控制**：DE/RE 引脚 + 发送完成中断，收发切换时序；
4. **寻址**：点对点（一路一设备）先行；多 drop 需在帧头加地址字节 → 协议 v2 + 版本字节拒绝旧端，防误解析；
5. 时间预算：115200 bps、40 B/块、停止等待 → 单次 83 KiB 约 15–25 s；921600 约 3–5 s，可接受；
6. **硬件前提**：本板无 485 收发器（原理图无相关器件），需新板或飞线；扩展能力为软件层就绪。

### 4.3 工作量

`loader_uart.c`（~250 行）+ 串口驱动 + 宿主 `SerialBus`（~150 行）+ HIL ≈ 2–3 天（含硬件）。

## 5. 本批实施记录（code）

| 变更 | 文件 | 说明 |
|---|---|---|
| 运输层接口 | `loader/firmware/loader_link.h`（新） | init/rx_pop/send/shutdown + `ldr_rx_frame_t`；核心只依赖此头 |
| CAN 传输实现 | `loader/firmware/loader_can.c`（改） | 实现 `loader_link_*`；`hldr_can` file-static；新增 shutdown；新增 bus-off 恢复回调；节点过滤双保险 |
| 旧头移除 | `loader/firmware/loader_can.h`（删） | 由 `loader_link.h` 取代 |
| 主逻辑 | `loader/firmware/loader_main.c`（改） | 改用 `loader_link_*`；新增 `power_stage_safe_init()`；ID 校验移出核心 |
| 跳转 | `loader/firmware/loader_jump.c`（改） | 以 `loader_link_shutdown()` 替代直接 FDCAN 操作 |
| bus-off | `loader_can.c` | `FDCAN_IT_BUS_OFF` 通知 + `HAL_FDCAN_ErrorStatusCallback` 清 `CCCR.INIT`（硬件随后等 129×11 隐性位） |

验证证据见 §5.1（构建/HIL 结果于实施后回填）。

## 6. 记录 v2 与协议演进设计（P1 冻结候选）

### 6.1 记录页 v2（24 B → 32 B）

```
magic("APP1") | size u32 | crc32 u32 | version u32 | image_type u32 |
product_id u32 | hardware_id u32 | record_crc u32
```

- 校验扩展：`image_type ∈ {1=product, 2=factory, 3=debug}`（编号待注册表确认）；`product_id` 必须匹配 Loader 编译期值；`hardware_id` 0 = 不校验（未分配时），非 0 = 必须匹配；
- 迁移：新 Loader 读到旧记录（20+4 B 布局）→ record_crc 校验失败 → Recovery → 一次重升级即恢复；写档告知。

### 6.2 协议 v2 变更点

- `BEGIN` payload：`size, crc, version, type u16, product_id u32, hardware_id u32`（16→24 B）；
- `LOADER_CAN_PROTOCOL_VERSION` 0x01→0x02（两端拒绝旧版，防误解析）；
- 新增结果码 `BAD_IDENTITY`（product/hw 不匹配）；
- 同步面：`shared/protocol/loader_can_v1.h`、`loader_cfg.h` 别名、`loader_proto.py`、`loader_updater.py`、`host_app mdrive_core` 协议镜像、`test_protocol_single_source.py`、两份文档。

## 7. UNKNOWN / 待确认清单（禁止猜测项）

| # | 项 | 状态 |
|---|---|---|
| U1 | Flash 耐久次数（ST 数据手册值） | **UNKNOWN**，需查 STM32G431 数据手册"endurance" |
| U2 | Product ID / Hardware ID 注册值 | **UNKNOWN**，需产品注册表；记录 v2 实现前必须确认 |
| U3 | image_type 枚举编号（product/factory/debug） | 候选 1/2/3，需与公司协议文档对齐 |
| U4 | 看门狗启用策略（Loader+APP 协同） | 待决策 |
| U5 | 485 硬件（收发器/DE 引脚/波特率） | 硬件不存在，待新板定义 |

## 8. 证据路径索引

- 架构：`loader/docs/ARCHITECTURE.md`、`loader/docs/UPDATE_FLOW.md`、`docs/loader/IMPLEMENTATION_PLAN.md`
- 压测：`outputs/loader_cycles_full/`、`outputs/loader_fault_full_20260912_223900/`、`outputs/loader_upgrade_sot_20260913_005317/`
- 测试：`tests/test_loader_proto.py`、`tests/test_loader_fault_images.py`、`tests/test_protocol_single_source.py`
- 硬件：`docs/SCH_Vector_Mini_ST_2026-07-22.pdf`（FD6288Q/U7）、FD6288Q 数据手册（Fortiortech）
- Keil 下载安全：`host_app/tools/check_app_layout.py`、`host_app/tools/verify_loader_intact.py`
