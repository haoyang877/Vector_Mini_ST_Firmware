# HIL 退役：删除 HIL 工程与工装，仅保留 normal 工程 v1.0

日期：2026-09-19。状态：**已完成**（代码/配置/文档/PR 验证；release 档证据待干净工作树补）。授权：用户裁决「完整拆除 HIL」（2026-09-19）。

## 1. 意图

仓库只保留一个 Keil 工程（`Vector_Mini_ST`）与一条构建/发布链路；删除 HIL 固件变体
（`SERVO_HIL_ENABLE`）、HIL 台架工装与测试、`profiles.hil` / `layers.hil` 配置及对应
文档口径。退役后 `verify --profile release` 只构建单目标；quick / PR 档流程不变。

## 2. 删除范围（已冻结）

### 2.1 Keil 与固件

- `firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST_HIL.{uvprojx,uvoptx}`；
  未跟踪残留：`.uvguix`、`DebugConfig/*HIL*.dbgconf`、`*HIL*structure_build.log`。
- `tests/hil/firmware/servo_hil.{c,h}`。
- `foc_task.c` 与 `stm32g4xx_it.c` 的 `SERVO_HIL_ENABLE` 条件块；normal 工程
  include 路径中的 `tests/hil/firmware`。

### 2.2 台架工装（HIL 链，全部消费 HIL 运行产物或驱动 HIL 固件）

- `tools/bench/servo_hil_run.py`、`servo_hil_emergency.py`、`run_mode3_validation.py`。
- `tools/bench/scopes/pro_lks_servo_hil.lksscope`。
- `tools/flash/servo_hil_flash.py`。
- `tools/analysis/`：`analyze_servo_hil.py`、`plot_servo_hil.py`、`analyze_servo_landing.py`、
  `plot_servo_landing.py`、`analyze_positioning_comparison.py`、`analyze_motion_noise.py`、
  `plot_noise_optimization_report.py`（`trial.json` 消费者）。

### 2.3 测试与配置

- `tests/unit/servo_hil_test.c`、`test_positioning_comparison.py`、`test_motion_noise_analysis.py`。
- `tests/integration/test_servo_hil_recovery.py`、`test_mode3_validation.py`。
- `tests/hil/**`（firmware + profiles）。

### 2.4 门禁与配置

- `harness.toml`：`[profiles.hil]`、`[architecture.layers.hil]`、style 路径、`[environment]`
  中 HIL 专用键。
- `doctor --profile hil`；`build_firmware` / `check_project_layout` / `verify` /
  `firmware_artifacts` 的双目标逻辑；`tools/project_paths.py` include 目录。
- `check_hygiene.py` safety_files 段；`check_docs.py` 旧路径映射；`style_debt.json` /
  `interface_debt.json` 的 HIL 条目。
- 夹具同步：`run_position_servo_tests.py`（去 `servo_hil_test` 构建）、
  `test_rtt_control_telemetry.py`（去 HIL scope 引用）。

### 2.5 明确保留（非 HIL）

- CAN 台架工具（`can_*`、`dual_axis_*`、`motor_axis_record`、`canfd_diagnostics`）。
- `analyze_hold_noise.py`（通用 TSV 分析）与其测试、`rtt_control_frame.py`、
  `test_rtt_control_telemetry.py`（去 HIL scope 后）。
- `pyproject.toml` 的 bench extra：若可离线重锁则移除 `pylink-square` / `pyelftools`；
  否则登记为余量，不动 `uv.lock`。

## 3. 不变量

- normal 固件行为与镜像不变：基线 Code=81820 / RO=4888 / RW=248 / ZI=31352
  （AXF SHA-256 `f1e9…c4e0`、HEX `c8b3…1b70`，2026-09-19 实测）。
- CubeMX 生成区只删 USER CODE 内的 HIL 分支；不改 `.ioc` 输入。
- 非 HIL 工作流、参数 schema 11、CAN ABI、发布命名规范均不动。

## 4. 验收

1. `check_project_layout` 单工程 0 错误；`build_firmware --target all` 仅构建 normal，
   0 Error / 0 Warning，尺寸与哈希对齐基线。
2. `verify --profile pr` 全绿（doctor / format / lint / project-layout / tests /
   architecture / interfaces / docs / hygiene）。
3. 全仓 `\bHIL\b|servo_hil|SERVO_HIL` 仅剩历史报告与完成计划的记录性叙述（清单见
   §6 执行记录）。
4. release 档待工作树干净后补（当前存在在途未提交改动）。

## 5. 回滚

全部删除均可经 git 恢复；基线提交 `dda0c780`；工作树含在途改动，回滚时逐文件核对。

## 6. 执行记录

### 2026-09-19 代码/配置侧（主树含在途改动，基线提交 `dda0c780`）

- **删除面**：HIL Keil 工程 2 个（+ 未跟踪 `.uvguix`/dbgconf/build-log 残留 3 个）、
  `tests/hil/**`（firmware 2 + profiles 11）、HIL 台架与工装 7 个、HIL 测试 5 个、
  `pro_lks_servo_hil.lksscope`；`tools/flash/` 目录随之清空移除。
- **固件**：`foc_task.c`、`stm32g4xx_it.c` 的 `SERVO_HIL_ENABLE` 分支清零（均在 USER CODE 区内）；
  normal 工程 IncludePath 去掉 `tests/hil/firmware`。
- **门禁/配置**：`build_firmware`/`check_project_layout`/`verify`/`doctor` 单目标化；
  `harness.toml` 去掉 `[profiles.hil]`、`[architecture.layers.hil]`、jlink/bench 环境键与
  style 路径；`project_paths`/`run.py`/`check_hygiene`/`check_docs` 同步；
  `style_debt` −18 条、`interface_debt` −1 条。
- **依赖**：移除无消费者的 `bench` extra 及 `pylink-square`/`pyelftools`（连带 `psutil`）；
  `uv lock --check` PASS，`uv sync --locked --extra dev` PASS。
- **构建**：`build_firmware --target all` 仅构建 normal，**0 Error / 0 Warning**，
  Code=81820 / RO=4888 / RW=248 / ZI=31352（与基线一致）；HEX SHA-256
  `C8B3204B…1B70A` **逐字节一致**；AXF 尺寸不变、哈希随调试行表变化。
  日志：`outputs/build/logs/Vector_Mini_ST.log`。
- **测试**：`tests/run.py --cc <zig>` 原生 15 项 + integration 全 PASS；
  unit 唯一失败为 5 条文档断链（由文档清扫处理）。
- **文档清扫（2026-09-19）**：`check_docs` 0 problems（修复断链 5 条：mode3_parameter_guide ×2、
  motor_axis_profiles ×2、noise_optimization_summary 报告 ×1）；在途计划「双目标/双变体 →
  单目标」口径更新（含未跟踪计划 hardware-boundary-closure、e-framework-interface-prep）；
  `tech-debt-tracker` HIL-001 关闭标注。
- **PR 档终验（2026-09-19）**：`verify --profile pr` **PASS**（doctor/format/lint/project-layout/
  tests/architecture/interfaces/docs/hygiene 全绿）；证据：
  `outputs/runs/20260919T072345136533Z-dda0c780/summary.json`。
- **发布证据（2026-09-19，提交 `c39d0d25`，dirty=false）**：`verify --profile release`
  **PASS（11/11）**，含 Keil 构建与发布 manifest；run
  `outputs/runs/20260919T091846109938Z-c39d0d25/summary.json`。

## 7. 关联文档

- [硬件边界收口](2026-09-19-hardware-boundary-closure.md)：T1–T5 的「双目标」验收口径
  随本计划改为单目标。
- [技术债跟踪](../tech-debt-tracker.md)：HIL-001 随退役关闭。
