# Project Harness 使用说明

Project harness 把环境、项目知识、验证命令、架构边界和结果证据放回仓库，供开发者、
CI 和编码智能体使用。它不会把硬件动作加入默认自动化。

## 环境

```powershell
python -m pip install uv==0.12.16
uv sync --locked --extra dev
uv run python tools/run.py doctor --profile pr
```

依赖版本由 `pyproject.toml` 与 `uv.lock` 固定。Zig 来自锁定的 `ziglang` 包。当前
`harness.toml` 固定 Keil 5.36.0.0 与 SEGGER J-Link 9.64；这些工具和 CAN 厂商驱动
仍由外部安装，通过环境变量或命令行配置，不写入仓库。

| 变量 | 用途 |
| --- | --- |
| `HARNESS_CC` | 可选的 Zig 编译器覆盖路径 |
| `KEIL_UV4` | release profile 使用的 `UV4.exe` |
| `JLINK_DLL` | HIL 使用的 SEGGER 动态库 |
| `JLINK_PROBE_SERIAL` | 明确选择的探针序列号 |
| `VECTOR_BENCH_ID` | 物理工作台标识 |
| `MOTOR_CTRL_APP_ROOT` | 双轴 CAN 工具依赖的适配器工程 |
| `HARNESS_CJK_FONT` | 报告绘图可选的中文字体 |

## 验证级别

- `quick`：工程布局、Python 单元和离线集成测试。
- `pr`：quick 的能力，加全部原生 C、架构、文档和仓库 hygiene 检查。
- `release`：PR 检查，加普通/HIL Keil 全量构建和发布 manifest；要求干净工作树。
- `hil`：`doctor` 只检查依赖和本机配置，不打开探针。

每次 `verify` 在 `outputs/runs/<run-id>/` 写入日志与 `summary.json`。release 额外写入
`release-manifest.json`，包含当前 Git SHA、工作树状态、两个固件目标和 AXF/HEX/MAP
哈希。旧目录中的产物不能作为本次验证通过的依据。

## 代码风格门禁

所有模块共享根目录的 `STYLE.md`、`.editorconfig`、`.clang-format` 和 Ruff 配置。使用：

```powershell
uv run python tools/run.py format
uv run python tools/run.py format --check
uv run python tools/run.py lint
uv run python tools/run.py lint --module firmware
```

默认 `format` 只迁移新文件或已发生内容变化的历史文件；`format --all` 才会显式重排
全部历史债务。`tools/harness/style_debt.json` 以文件哈希固定临时豁免，文件一旦发生变化，
豁免立即失效。新模块应从 `templates/` 中受 CI 检查的 C/Python 示例开始。
仓库自有代码必须包含必要的中文注释；注释用于解释接口契约、单位、不变量和硬件风险，
而不是复述语句。缺少中文说明的新文件会由 `lint` 拒绝。

## CI 与硬件边界

GitHub 托管 Windows runner 执行 PR 离线门禁；带 Keil 许可证的 self-hosted runner
执行构建和 release。HIL 不自动触发。HIL 命令必须给出工作台、电机 profile、场景、
预期固件 SHA-256，并使用 `POWER_LIMITS_VERIFIED` 明确确认物理安全条件。失败记录必须
保留 STOP/紧急关闭结果；无法确认关闭时应断开电机电源，不得自动恢复运行。
