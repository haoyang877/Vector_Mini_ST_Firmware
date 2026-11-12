# 上位机长期可维护设计（架构 / 技术路线 / 功能蓝图 / 维护机制）

> 配套：`host_app/docs/IMPLEMENTATION_PLAN.md`（功能与里程碑）、`host_app/docs/UI_DESIGN.md`（页面方案）。
> 目标：**5 年生命周期**内可演进——固件换版本、新增产品/板卡、人员更替、产线批量化时，
> 上位机只需"加数据、少改代码、零猜协议"。

## 0. TL;DR（五条设计纲领）

1. **三件套拆分**：无 GUI 的核心库（`mdrive_core`）+ 薄 GUI（`mdrive_gui`）+ 无人值守 CLI（`mdrive_cli`）。
2. **一切设备差异数据化**：Device Profile（产品/板卡）、Param Schema（参数表）、曲线通道、能力集全部是外部文件；代码只解释数据。
3. **契约先行 + 机器校验**：schema 从固件头文件生成，CI 校验漂移；协议有版本与兼容策略。
4. **录包回放**：总线流量可录制/回放，任何现场问题都能离线复现并固化为回归用例。
5. **从第一天工程化**：pyproject + uv.lock + ruff + pyright + pytest + CI + 安装包；语义化版本 + CHANGELOG + ADR。

---

## 1. 设计原则（决定五年后的成色）

| # | 原则 | 具体含义 | 反面示例（必须避免） |
|---|---|---|---|
| 1 | 分层单向依赖 | GUI → Services → Core → Transport 逐层向下 | UI 事件回调里直接写协议解析 |
| 2 | 核心库无 Qt | `mdrive_core` 不 import PySide6，可在 CI 无硬件运行 | 协议逻辑只在 GUI 里，无法自动化测试 |
| 3 | 数据驱动 | 参数/产品/通道差异全部外部化 | `if product == "lz_v3":` 硬编码分支 |
| 4 | 契约化 | 协议 revision、schema、帧格式都有版本与测试 | 固件改字段，上位机靠人肉发现 |
| 5 | 可回放可诊断 | 一键导出诊断包；录包回放测试 | "现场复现不了"成为长期借口 |
| 6 | 单一状态源 | AppState 集中；操作幂等、有结果码 | 状态散落在各页面控件里 |

## 2. 技术路线

### 2.1 选型
| 层 | 选型 | 理由 |
|---|---|---|
| 语言 | **Python 3.13** | 协议资产（loader/tools、can_parameter_protocol、解码器）已全部是 Python；团队现有技能栈 |
| GUI | **PySide6 (Qt6, LGPL)** | 成熟、长期维护、控件齐全；LGPL 适合产品分发 |
| 绘图 | **pyqtgraph** | 实时滚动曲线标准方案；与 Qt 深度集成 |
| 数据校验 | **pydantic v2** | schema/profile 加载即校验，错误信息可读 |
| 数值 | numpy | 缓冲/降采样/统计 |
| 传输 | 现有 ControlCANFD ctypes 封装 + `CanTransport` 抽象 | 未来可插 SocketCAN/串口/RTT/回放 |
| 工具链 | **uv**（依赖与锁）、ruff（lint+format）、pyright（类型）、pytest(+pytest-qt) | 现代统一工具链，新人一条命令跑起 |
| 打包 | PyInstaller + Inno Setup | 产线可安装、可离线、可固定版本 |
| CI | GitHub Actions 或本地 runner | lint→type→test→schema-check→build→artifact |

### 2.2 为什么不选其他
- **C#/WPF**：界面更好，但现有协议/测试/脚本资产全部作废，重写成本 >> 收益；如未来确需，可复用本设计的 Core 语义重写 GUI。
- **Web/Tauri/Electron**：多一层运行时与 IPC，产线部署复杂；本场景局域网/单机 USB 设备，无优势。
- **继续散装脚本**：无版本、无测试、无 UI、无法交接——正是本设计要解决的问题。

## 3. 架构（三件套 + 数据目录）

```
host_app/
├── packages/
│   ├── mdrive_core/                  ★ 无 Qt、无 UI 的核心库（可独立发版/被脚本引用）
│   │   ├── transport/
│   │   │   ├── base.py              CanTransport / Frame 抽象（Protocol）
│   │   │   ├── controlcanfd.py      现有 usbcan_adapter + LoaderBus 合并迁入
│   │   │   ├── replay.py            录包回放（测试）
│   │   │   └── simulator.py         纯软件模拟设备（CI/离线开发）
│   │   ├── protocol/
│   │   │   ├── param.py             参数编解码（自 tools/can_parameter_protocol 迁入）
│   │   │   ├── loader.py            Loader 协议（自 loader/tools/loader_proto 迁入）
│   │   │   ├── status.py            48B 状态流（自 tools/can_motor_status 迁入）
│   │   │   └── rtt.py               RTT 12ch（自 tools/rtt_control_frame 迁入）
│   │   ├── schema/
│   │   │   ├── models.py            ParamSchema/DeviceProfile/Capability（pydantic）
│   │   │   └── loader_io.py         schema 加载 + 版本兼容检查
│   │   ├── device/
│   │   │   ├── session.py           单设备会话（请求串行化、超时/重试策略）
│   │   │   ├── discovery.py         节点扫描/协议探测/能力发现
│   │   │   └── capabilities.py      能力位图 → 功能开关
│   │   └── services/
│   │       ├── upgrade.py           升级状态机（含进度/取消）
│   │       ├── parameters.py        参数事务：写入→校验→保存(0x68)
│   │       ├── telemetry.py         遥测会话：速率配置/批处理/环形缓冲
│   │       └── recorder.py          录包/CSV 记录（含 canlog 回放格式）
│   ├── mdrive_cli/                  ★ 无人值守：产线脚本 / CI 冒烟 / 批量化
│   └── mdrive_gui/                  ★ 薄 GUI：页面绑定 + 交互，不含业务逻辑
├── profiles/                        ★ 数据（随固件版本生成，代码零改动）
│   ├── registry.yaml                产品/板卡 → profile 映射
│   └── lz_v3/params.json            参数 schema（生成+人工注释范围/单位）
├── tests/
│   ├── unit/                        编解码/CRC/解码黄金向量
│   ├── schema/                      生成器 vs 固件头文件一致性（CI 门槛）
│   ├── replay/                      真实录包回放回归
│   └── gui/                         pytest-qt 关键交互
├── docs/
│   ├── adr/                         架构决策记录（每条决策一页）
│   └── USER_MANUAL.md
├── packaging/                       PyInstaller spec + Inno Setup 脚本
└── pyproject.toml / uv.lock
```

**依赖方向（单向）**：`mdrive_gui → mdrive_core`、`mdrive_cli → mdrive_core`；core 永不反向依赖。

## 4. 功能蓝图（分层发布）

### P0 — MVP（1 周内，直接可用）
- 连接/断开、设备信息、节点扫描、协议 revision 检查
- 参数：读全部/写单项/回读校验/**写入并保存（0x68）**/JSON 导入导出
- 升级：选 bin→进度→跳转结果（复用已验证的 Loader 协议）
- 曲线：CAN 状态流滚动显示（10–200 Hz）、CSV 录制
- 日志页 + **诊断包一键导出**（帧日志 + 参数快照 + 版本）

### P1 — 完整版（+1 周）
- 参数：分组/搜索/批量/未保存高亮/快照对比（diff 两次导出）
- 曲线：多节点叠加、双游标 ΔX/ΔY、PNG/CSV 导出、暂停
- Device Profile：多产品/多板卡切换（不同参数表与通道集）
- 能力发现：功能按固件能力自动启停（旧固件不显示不支持项）
- CLI：`mdrive scan / readall / write / save / upgrade / record`（产线与 CI 用）
- 工程化：pyproject/uv.lock/CI/安装包

### P2 — 进阶（按需）
- 生产模式：批量序列号、产测步骤编排、良品报告
- RTT 2 kHz 高速曲线（需 JLink）
- 脚本接口：`import mdrive_core` 稳定 API（自动化测试/客户定制）
- 报告生成（PDF/Excel）、数据回放分析器
- 应用内更新检查、i18n、主题

## 5. 契约与版本策略（长期兼容的根基）

**三层版本**

| 版本 | 现况 | 客户端策略 |
|---|---|---|
| App 版本 | SemVer `v1.4.2` | CHANGELOG；安装包可回退 |
| 协议 revision | 固件 `GET_PROTOCOL_REVISION(0x67)=2` | **支持 N 与 N-1**；连接时校验，过旧只读模式 |
| 参数 schema 版本 | `profiles/*/params.json` 带 `schema_version` | 与固件 revision 关联；加载时校验 |

**握手与能力**
- 连接即读 `0x67` 与能力集（建议固件后续加 `GET_CAPABILITIES`，返回位图：Save/RTT/Stream/Friction…）。
- GUI 按能力渲染：老固件自动隐藏"保存"等新功能按钮，而不是报错。

**Schema 生成与防漂移（关键机制）**
1. `mdrive_core.schema.loader_io` 附带生成器：解析 `interface_can.h`、`can_parameter_format.h`、
   `interface_can.c` 校验分支 → 输出 `profiles/<product>/params.json`（ID/名称/编码/单位/范围/只读）。
2. CI 任务：重新生成并与提交文件 diff；固件 PR 改了参数而 schema 未更新 → **CI 红灯**。
3. Loader 协议（v1）冻结；如需变更 → 版本号递增 + 客户端双版本支持。

**弃用政策**：标记 deprecated → 文档记录 → 至少两个 minor 版本后移除；CHANGELOG 显著说明。

## 6. 测试体系（可演进的安全网）

| 层级 | 内容 | 触发 |
|---|---|---|
| 单元 | 帧编解码、CRC 黄金向量、解码器、schema 校验 | 每次提交 |
| Schema 一致性 | 生成器 vs 固件头文件 diff | CI/Actions |
| **回放回归** | 真实会话录成 `*.canlog`，回放 core 全流程（无硬件） | 每次提交 |
| GUI | pytest-qt：表格编辑/状态机渲染/升级进度 | 每次提交 |
| HIL 冒烟 | 真机：扫描→读参→写参→保存→升级→曲线 20s | 发布前/夜跑 |
| 安装包冒烟 | 全新 Win 沙箱安装→启动→连接 | 发布前 |

**录包回放工作流（长期最值钱的一环）**：
现场问题 → 导出诊断包（含 canlog）→ 回放复现 → 固化为回归用例 → 修复 → 永不复现。

## 7. 长期维护机制

| 维度 | 机制 |
|---|---|
| 仓库 | 先 monorepo 子目录（与固件联动）；包边界干净，条件成熟即可拆独立 repo（独立发版/权限） |
| 文档 | ADR（每条架构决策）、用户手册、发布说明、半天 onboarding 指南 |
| 代码规范 | ruff+pyright 门禁；review 红线：协议逻辑禁止写进 GUI |
| 依赖 | uv.lock 锁定；**季度依赖巡检**（安全/兼容）；升级走 PR+CI |
| 迁移收敛 | 逐步把 `tools/*.py`、`loader/tools/*` 的协议实现**收敛进 `mdrive_core`**，旧脚本改为薄封装，消除三份拷贝 |
| 日志诊断 | 结构化日志（JSON 可选）+ 诊断包；现场问题零扯皮 |
| 安全 | 二次确认（危险写/擦除/保存）；未来固件签名后校验镜像；产线模式可加权限 |
| 发布 | 语义化版本 + CHANGELOG + tag 出安装包；产线固定版本可追溯 |
| 人员更替 | 协议知识全部数据化（schema/文档），改参数不需要读 C 代码 |

## 8. 路线图（含工程化前置）

| 阶段 | 交付 | 天数 |
|---|---|---|
| W0 | 工程化打底：pyproject/uv、目录骨架、CI 雏形、core 迁移 loader/param/status 协议 | 1 |
| W1 | P0：连接页 + 参数页（含固件 0x68） | 2 |
| W2 | P0：升级页 + 曲线页 + 诊断包 | 2 |
| W3 | P1：profile/能力发现/CLI/打包/回放测试落地 | 2–3 |
| W4+ | P2 按需 | — |

**关键**：W0 的"核心库边界 + CI + 回放格式"是可维护性的分水岭，不可省略。

## 9. 反模式清单（评审时逐条对照）

- ❌ GUI 事件回调里写协议/重试逻辑
- ❌ 参数表硬编码在界面代码里
- ❌ 魔法数字/字符串散落（ID、结果码、字段偏移）
- ❌ GUI 线程直接调用 DLL（必须单 worker 独占）
- ❌ 无回放测试；固件每次升级靠人手点
- ❌ 依赖散装（无锁文件、无 CI）
- ❌ 发布只有 zip，无版本号/无 CHANGELOG
- ❌ 为"快"跳过 schema 生成，之后两边永远漂移

## 10. 与现有固件平台哲学的对应

固件侧的 AGENTS.md 要求"HAL API 表达产品能力、MCU/PCB 可替换、配置数据化"。
上位机按同样哲学落地：**Transport 可替换（相当于 HAL Port）、Device Profile 数据化（相当于 BSP）、
Core 与 UI 解耦（相当于 Core 与 HAL 解耦）、协议契约版本化（相当于 flash layout 单一来源）**。
两边同一套思维，长期维护成本最低。
