# 绕组热保护设计与测试文档整理

版本：v1.0。日期：2026-10-17。基准分支：`codex/ovp-36v`。

## 目标与范围

整理现有绕组热保护的计算、调度、限流、故障、通信和存储设计；归档
2026-10-16 零速速度闭环叠加 Id 的实测结果，明确逻辑验证与真实温升标定的区别。
本次只修改文档及配套插图，不改固件、Flash 参数或台架脚本，不连接硬件。

## 交付与证据

- 设计：`docs/architecture/motor_winding_thermal_design_v1.md`。
- 报告：`docs/reports/2026-10-16-zero-speed-id-thermal-test.md`。
- 曲线：`docs/assets/motor_thermal_20261016/zero_speed_id_thermal.png`。
- 入口：既有使用指南与 README 中增加交叉引用。
- 实测原始证据：`outputs/thermal_zero_speed_20261016_bench01/`，保留忽略提交。

采用真实实现及本轮 `summary.json`、`analysis.json`、前后 Flash 备份进行核对。
测试镜像基于 `7e8808a5fc54358febf8363624213f16dad763c2` 的冻结副本，
额外注入仅属于临时台架镜像；不把该基准节点称为完整测试镜像的已提交源码。

## 验收与回退

核对公式、默认参数、单位、真实模式行为、数据摘要与 SHA-256；检查新改文档的
本地链接、图像可读性以及 `git diff --check`，运行目标分支既有离线目录检查。
不把本轮未触发的热故障锁存、真实绕组温升或其他模式标为通过。

用户提供的新版仓库规约要求 quick 配置，但本目标分支尚无 `verify`、`format`、
`lint` 命令、`pyproject.toml` 和 `uv.lock`；不改动工具框架来扩张文档任务。
以现有 `check_project_layout` 和本次文档检查记录实际结果，不声称 quick 验证通过。
文档回退仅撤销本次新增文档、插图及精确入口修改，保留原始试验证据和无关用户修改。

## 整理结果

2026-10-17 已完成设计、报告、插图和入口引用，修正旧指南对外环抗饱和的过强概括。
设计与报告经独立源码和数据复审；删除了本轮无法支持的 `max_heat` 暂为零归因，
保留非原子 SWD 采样的真实限制。23 个文档链接、3 个入口、证据 SHA-256、
完整 Flash 恢复、时间常数及降额公式复算均通过。普通与 HIL 工程目录检查均为 0 错误，
`git diff --check` 通过。文档检查摘要位于 `outputs/thermal_documentation_20261017/summary.json`。
未重新通电、下载或修改任何固件和电机参数。
