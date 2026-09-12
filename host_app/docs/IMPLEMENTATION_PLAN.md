# PC 上位机实现计划（CAN 升级 / 参数设置 / 曲线查看）

> 目标平台：Windows 10/11 + USBCANFD-200U（创芯 ControlCANFD.dll）。
> 状态：**W0 / M1 / M2 完成，打包（M3 首项）完成**（2026-09-12）。
> 核心库、CLI（11 命令）、GUI 五大页签实装；升级支持阶段进度与中止；
> 已产出 `dist/mdrive-host.exe` + `dist/mdrive-cli.exe` 并通过 5/5 exe 验收；
> 36 项单元测试全绿。剩余 M3：多产品 profile、录包回放固化、多节点曲线叠加。

## 1. 目标与范围

**三大功能**

| ID | 功能 | 说明 |
|---|---|---|
| F1 | **固件升级** | 选 `bin` → 校验 → 经 Loader 协议升级 → 跳转验证；进度/取消/失败日志 |
| F2 | **参数设置** | 全部 CAN 参数（0x00–0x67）：读/写/批量读/导入导出/范围校验/只读标记；**写入并保存到设备（新增 0x68）** |
| F3 | **曲线查看** | CAN 状态流（10–200 Hz，48B 帧）实时滚动曲线；多通道/多节点；CSV 录制 |
| F4 | **Flash 参数读取与解析** | 经 CAN 分块读取 Flash 完整参数记录（≈2.2 KB，含 1024 点编码器 LUT）；解析具名字段；导出；**一键绘制角度误差曲线** |

**支撑功能**：连接与节点扫描、协议版本检查、帧日志与诊断、配置持久化。

**v1 不做**：云端/远程、脚本引擎、CANopen/UDS、跨平台、A/B 升级、参数标定向导。

## 2. 技术选型（推荐）

**Python 3.13 + PySide6(Qt6) + pyqtgraph + numpy**

理由：
1. **零重写复用**：现有协议层全是 Python（`loader_proto`/`loader_updater`/`can_parameter_protocol`/
   `can_motor_status`/`rtt_control_frame`），GUI 可直接 import。
2. **实时曲线**：pyqtgraph 基于 Qt GraphicsScene，200 Hz×多通道滚动刷新无压力；
   matplotlib 仅适合离线报告（现有 `analyze_*` 已用）。
3. **许可与部署**：PySide6 为 LGPL；`pip install` 即用；本仓库已是 Windows-only（WinDLL），无跨平台负担。
4. **打包**：PyInstaller 可出单文件 exe 交付产线。

备选（不推荐）：PyQt5/6（API 类似，许可更严）；C#/WPF（需重写全部协议，放弃）；
Web/Tauri（多一层 IPC 与运行时，收益低）；Tkinter+matplotlib（实时性能不足）。

## 3. 目录结构

```
host_app/
├── README.md
├── requirements.txt            PySide6, pyqtgraph, numpy（可选 pylink）
├── app/
│   ├── main.py                 入口（QApplication + MainWindow）
│   ├── config.py               DLL 路径/节点/默认速率/最近文件（QSettings/JSON）
│   ├── transport/
│   │   └── can_transport.py    封装 usbcan_adapter：打开/关闭/设备信息 + send/receive（抽取
│   │                           loader_updater.LoaderBus 的实现），独占 DLL，trace 钩子
│   ├── protocol/
│   │   ├── param_schema.py     参数表：ID/名称/分组/编码/单位/范围/只读/SET-GET 对
│   │   ├── param_client.py     包装 can_parameter_protocol：单写/单读/校验/批量读
│   │   ├── loader_client.py    适配 loader/tools：LoaderClient / update_firmware（进度回调）
│   │   └── status_decode.py    复用 can_motor_status.decode → 统一样本结构
│   ├── services/
│   │   ├── can_worker.py       单工作线程：独占全部总线 I/O；任务队列 + Qt 信号
│   │   ├── upgrade_service.py  升级状态机（镜像校验/进度/取消/结果）
│   │   ├── param_service.py    参数读写、批量、导入导出
│   │   ├── stream_service.py   状态流：速率配置、解码、环形缓冲
│   │   ├── recorder.py         CSV/JSON 记录（与现有 feedback.csv 列兼容）
│   │   └── rtt_service.py      可选：pylink RTT 2 kHz 采集（v2 解码）
│   └── ui/
│       ├── main_window.py      Tab 容器 + 状态栏（连接/节点/速率/错误）
│       ├── pages/connect_page.py
│       ├── pages/upgrade_page.py
│       ├── pages/params_page.py
│       ├── pages/plots_page.py
│       └── pages/log_page.py
├── tests/                      离线单测（帧编解码/schema/解码）
└── packaging/                  PyInstaller spec
```

## 4. 线程与并发模型（关键设计）

协议**无事务序号**（GET 重试的迟到应答无法关联），因此总线访问必须串行：

```
UI 主线程 ──(Qt Signal：样本批次/进度/日志/错误)── CanWorker(QThread)
    │                                                    │
    └──(任务队列：scan / read / write / update / stream*)──┘
CanWorker 独占：ControlCANFD DLL、设备句柄、LoaderBus、参数 Client、RX 轮询循环
```

- 所有 CAN 收发只在 CanWorker 内发生；UI 不直接碰 DLL。
- Worker 单循环：`receive()` 取帧 → 按 ID 分发：
  - `0x7F0+node` → 状态流样本 → Signal 到曲线页；
  - 其余 → 匹配当前挂起的参数请求（同 ID 即应答）；
  - 错误帧（`0x20000000`）→ CanRecovery + 日志。
- 高吞吐保护：状态流 48B 帧按批量（如 20 ms 批量）发 Signal，避免逐帧跨线程。
- 空闲/高负载时轮询间隔自适应（有挂起请求时 1 ms，纯监听时 2 ms）。

## 5. 功能规格

### 5.1 连接页
- 打开/关闭适配器；显示设备信息（序列号、硬件/固件版本、DLL SHA256）。
- 节点扫描：对 0–7 发 `GET_PROTOCOL_REVISION(0x67)`，列出在线节点与协议版本（期望 2）。
- 显示各节点 `mode/fault`（GET 0x01/0x4D）。DLL 路径可配置（默认沿用
  `D:/Work/Code/motor_ctrl_app/.local/canfd-sdk/ControlCANFD.dll`）。

### 5.2 固件升级页（F1）
- 选择 `.bin`：自动去尾部 0xFF、8 字节对齐、计算 CRC32、显示 size/CRC/版本（可手填或时间戳）。
- 一键升级流程（全部在 Worker 内执行，复现 `update_firmware`）：
  `校验镜像 → 发 0x66 进 Boot（或提示手动）→ wait_loader_alive → BEGIN → ERASE → PROGRAM(进度) → VERIFY → ACTIVATE → wait_app_alive`。
- 进度条按字节；阶段耗时表（begin/erase/program/verify/activate/app）；
  取消按钮 = 编辑态 `ABORT`（擦除后不可回滚，UI 明确提示）。
- 失败：显示 result 码与错误详情，导出 `failure_*.json`（复用现有格式）。

### 5.3 参数页（F2）
- 左侧分组树：**模式/电流/速度/位置/位置阻抗/级联/摩擦/电机模型/CAN/遥测**；右侧参数表：
  `名称 | ID | 当前值 | 单位 | 写入值 | 范围 | 只读`。
- 数据源 `param_schema.py`（从 `interface_can.h` + `can_parameter_protocol.layout()` +
  `interface_can.c` 校验范围整理为机器可读表）。
- 操作：单项读 / 写 / 写后校验（verify）、全部读取（串行+进度）、导出/导入 JSON、只读灰显。
- 编码：`mA16 / mrad32 / crad32 / F32`，单位显示（A、rad、rad/s、rad/s²、Ω、H、Wb、°C、kbps、ms、1/s…）。
- 安全与兼容：
  - 写前范围校验；影响电机的命令（mode/current/speed/position）二次确认；
  - `0x4E` 编码器反向仅 `Motor_Disable` 时可写；
  - `0x29` 固件应答 ID bug（回 0x2B）：读取走兼容路径并提示；
  - GET-only 遥测（0x2D/0x2F/0x31/0x33/0x35/0x37/0x39/0x3F/0x41/0x43/0x45/0x47/0x49/0x4B/0x4D/0x59–0x62）
    用直接 GET 发送（绕过 `Client.read` 的 SET+1 限制）；
  - 协议版本 <2 时告警。

### 5.4 曲线页（F3）
- 数据源：**CAN 状态流**（默认）/ RTT（可选，2 kHz，仅 mode 3 有效，需 JLink）。
- 通道多选（状态流 12 字段；RTT 12 通道），颜色/单位/缩放。
- 速率：10–200 Hz 整数（发 `0x64` 配置，`0x65` 回读），启/停。
- pyqtgraph 滚动窗口（5/10/30/60 s 或自动），暂停/继续、游标测量（ΔX/ΔY）、导出 PNG/CSV。
- 多节点叠加（最多 8 节点，不同颜色）。
- 无效值（int32 `0x80000000` / int16 `0x8000`）→ 曲线断开（None），并统计 invalid/丢帧。
- 录制：勾选写入 CSV，列与现有 `feedback.csv` 兼容（`time_s,node,fault,mode,position_target_rad,…`）；
  时间轴使用**主机接收时间**（CAN 帧无设备时间戳，UI 注明 jitter）。

### 5.5 日志/诊断页
- 原始帧日志（TX/RX/时间/ID/DLC/bytes，可过滤、可暂停）。
- CanRecovery 事件、升级/参数操作审计。
- 一键导出诊断包（zip：日志 + CSV + 当前参数快照 + 配置）。

### 5.6 Flash 参数页（F4，新增）

- **完整记录读取**：经新增 CAN 命令分块读取 Flash 参数记录（`0x0801C000`，≈2.2 KB），
  INFO 校验 magic/schema/size/record_crc32，CHUNK 按偏移分块（每块 ≤56 B）。
- **参数解析**：按 `flash_schema.json`（从 `Foc/foc_param.h` 生成）解析为具名字段，分组：
  基础/电流标定/电机参数/电流环/速度环/位置环/摩擦/编码器/轴配置；
  解析器校验 magic（0x454E4332）、schema 版本（4–10，未知则仅导出原始数据）。
- **展示**：字段表（名/偏移/类型/原始值/工程值/单位）；选中字段原始字节高亮；
  编码器 LUT 摘要（1024 点、最大误差、RMS）。
- **导出**：JSON（全字段）、CSV（字段级）、原始 blob（bin）；可"与在线参数对比"。
- **角度误差曲线**：读 LUT 后按既有公式转换并绘制
  （`raw_deg = i·64 × 360/65536`、`err_deg = LUT[i] × 360/65536`，反向取负；
  对照 `Communication/interface_usb.c:1259-1272` 旧 USB 导出实现）；
  支持两次读取叠加（校准前/后对比）、PNG/CSV 导出。
- **固件配合**：新增 `CAN_GET_FLASH_PARAM_INFO = 0x69`、`CAN_GET_FLASH_PARAM_CHUNK = 0x6A`；
  需扩展 CAN-FD 回复缓冲（现协议回复仅 4 字节 → ≤64 B）。

## 6. 复用矩阵

| 现有组件 | 处理 | 说明 |
|---|---|---|
| `loader/tools/loader_proto.py` | **直接 import** | 纯函数编解码/CRC/常量 |
| `loader/tools/loader_updater.py`（LoaderClient/update_firmware） | **包装** | 阻塞+进度回调，在 Worker 执行；加取消 |
| `LoaderBus`（TX/RX 实现） | **抽取** | 通道/波特率做成可配置 |
| `tools/usbcan_adapter.py`（CanFD） | **包装** | 仅用于打开/关闭/设备信息 |
| `tools/can_parameter_protocol.py`（Client/encode/decode） | **包装** | 补直接 GET 与批量 |
| `tools/can_motor_status.py`（decode/FIELDS） | **直接 import** | 状态流解码 |
| `tools/can_recovery.py` | **包装** | 阈值做成界面可配 |
| `tools/rtt_control_frame.py` | 可选直接 import | RTT v2 解码 |
| `tools/dual_axis_*` 脚本 | 仅参考 | 不调用其阻塞 main() |
| `loader/tools/loader_cycle_test.py` | 不进 GUI | 保留为回归脚本 |

**新增**：`param_schema.py`（机器可读参数表）、`can_worker.py`、`recorder.py`、全部 UI。

## 7. 里程碑

| 阶段 | 内容 | 预计 |
|---|---|---|
| M0 | 环境（requirements）+ 主窗口骨架 + transport 冒烟（扫描节点/读协议版本） | 0.5 天 |
| M1 | 参数页：schema + 读写/校验/批量/导入导出/只读 + **Flash 参数/角度误差页（含固件 0x69/0x6A）** | 2.5 天 |
| M2 | 曲线页：状态流配置+解码+滚动绘图+CSV 录制 | 1.5 天 |
| M3 | 升级页：镜像校验+进度+取消+失败日志 | 1.0 天 |
| M4 | 打磨：日志/诊断、多节点、配置持久化、PyInstaller 打包 | 1.0 天 |
| M5 | 可选：RTT 2 kHz 高速曲线（pylink） | 1.0 天 |

合计约 5.5 天（M0–M4）；RTT 另计。

## 8. 风险与待确认（决策点）

1. **参数持久化（已决策 A）**：固件新增 `CAN_SAVE_PARAM = 0x68` → 触发既有 `Save_Param` 保存流程；
   上位机提供"写入并保存到设备"，以 `GET_MODE` 从 `Save_Param` 回到 `Motor_Disable` 判定保存完成。
   UI 细节见 `host_app/docs/UI_DESIGN.md` §5/§9。
2. **参数 schema 漂移**：~70 项参数/范围靠人工从 C 代码整理。建议附带一个从
   `interface_can.h`/`can_parameter_format.h` 解析生成 JSON 的脚本 + 校验用例。
3. `0x29` 固件应答 ID bug（回 0x2B）：上位机兼容 + 建议固件修复。
4. `Client.read()` 只支持"配对 SET→GET+1"，无法直接读 GET-only 遥测：新增直接 GET 通道。
5. 适配器固定通道 0 / 1M/1M：如需多通道或多速率，扩展 `usbcan_adapter`（GUI 暴露配置）。
6. DLL 线程安全未文档化：**单 Worker 独占**（强约束）。
7. 依赖安装：仓库无 requirements；计划新增 `requirements.txt` + 启动脚本（`python -m app`）。
8. 状态流无设备时间戳：曲线时间轴用主机接收时间（抖动需在 UI 说明）。
9. 设备区分仅靠节点 ID（0–7），无硬件序列号。
10. **Flash 参数读取依赖固件扩展**：新增 0x69/0x6A 命令并把回复缓冲扩到 ≤64 B CAN-FD；
    与 0x68 一并作为"固件配合包 B"实施。

## 9. 验收标准

- 连接：扫描到节点并显示协议版本 2；设备信息正确。
- 参数：读全部（含只读遥测）、写单项并回读校验一致、JSON 导入导出一致。
- 曲线：200 Hz 下 12 通道滚动流畅（UI 不卡顿）、无效值断开、CSV 与现有 schema 兼容。
- 升级：GUI 选择 bin → 升级成功 → 新固件运行（底层已 1000/1000 验证）。
- Flash 参数：完整读取（magic/schema/size/CRC32 校验通过）、解析与固件一致、JSON 导出可用；
  角度误差曲线 1024 点与旧 USB 导出公式一致（校准前后可叠加对比）。
- 全程 UI 无阻塞；任何失败有结果码、原因与可导出的日志。

## 10. 参考

- 参数协议：`Communication/interface_can.h`、`software/communication/protocol/can_parameter_format.h`、
  `tools/can_parameter_protocol.py`
- 状态流：`docs/can_motor_status_stream.md`、`tools/can_motor_status.py`
- 升级协议：`loader/docs/UPDATE_FLOW.md`、`loader/tools/loader_proto.py`
- 宿主规范参考：`docs/communication_development_plan.md`（含 transport/protocol/service 分层建议）
- 曲线数据（可选高速）：`tools/rtt_control_frame.py`、`Foc/foc_task.c`（12ch@2kHz）
