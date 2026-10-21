# 模块核心解耦集成记录

日期：2026-10-08。分支 `codex/system-refactor`；提交基点
`b0077b60b3eb84d8d72b1ec42c62141573b3245d`，包含保留的未提交现场输入。
本报告描述选择性集成工作树，不能使用基点 SHA 单独复现当前源码，也不是发布声明。

## 本批实际内容

| 模块 | 独立交付 head | 集成结果 |
| --- | --- | --- |
| 位置串级 | e0545fed98de4b1107eba67df1fa084f66f5aae1 | PI/Iq 与摩擦状态分开，核心只使用窄参数；继承上一批必要简化 |
| 编码器标定 | b6f08c5b115199ad1e73a77db62c65afe3d761f8 | 提取 encoder_calibration_core，任务保留采样/应用/退出 |
| 齿槽标定 | 79cf380dfb58abe2e23ca85801c7e0c7a5cf5874 | task 与 foc_cogging_runtime 分开，复用既有计算核心 |
| MotorAxis | 197054ac045837c55cd3f7aa349d4a18a963e9d7 | 参数事务独占原状态，Axis 调度和转发；保留现场零偏/PI/Watch 修改 |

只选上述任务自己的源码、对应夹具和文档，不合入各工作树的现场冻结提交。
普通 APP 和 LoaderApp 各登记三个新增 C 文件；从当前工程的对应源节点复制选项，
不覆盖优化设置，不登记头文件，不修改 uvoptx。备份、逐文件哈希及候选归属保存于
`outputs/coordination/capacity-review-20261008/integration_manifest.json`。

职责图与实际删除依据见[模块核心职责](../../architecture/module_core_responsibilities.md)；
详细行为与回退见[本批计划](../../archive/plans/2026-10-08-module-core-decoupling-v2.md)。

## 证据边界

ROOT 已审实际差异和匹配源码/工件哈希，复用各独立模块的直接证据：位置 792382 拍全部
旧字段及输出一致；编码器核心/任务比较；齿槽控制轨迹与运行独立链接；参数事务/保存
调用序列与状态相同。独立结果不能替代当前组合工程。
MotorAxis 独立 PR 因继承文档断链总体 FAIL，本记录不改写为独立 PR PASS。

组合 PR：PASS。doctor、格式、lint、工程布局、测试、架构、接口、文档、hygiene 均通过，
记录 `outputs/runs/20261008T072954285958Z-b0077b60/summary.json`；输入为基点上的当前
dirty 工作树，不是假称 clean 提交。只对最终组合运行一次 PR，没有重复各独立验证套件。

YG_ESC_APP normal 完整编译：0 Error / 0 Warning，新增三个源文件均实际编译。
日志 `outputs/build/logs/YG_ESC_APP.log`；MAP/AXF/HEX 位于
`outputs/build/keil/YG_ESC_APP/`，实际哈希与源码输入见
`outputs/coordination/capacity-review-20261008/combined_evidence.json`。

| 当前 normal 工程资源 | 字节 |
| --- | ---: |
| Code | 90080 |
| RO-data | 3260 |
| RW-data | 256 |
| ZI-data | 26648 |
| ROM / load span | 93596 |
| 静态 RAM | 26904 |

ROM 约 91.40 KiB，RAM 约 26.27 KiB。本次普通 APP 从 Flash 起始链接，
不是配套 LoaderApp 容量证明；LoaderApp 仅更新源文件接线，未重新构建或验收升级。
不同冻结输入的节省/增量不能相加推算当前资源。
当前没有硬件接触、下载或实机 PASS；动态栈/中断叠加和时序仍未测。

## 保留与未完成

控制数学、保护、功率/取消/故障顺序、schema12、固定维护区和 Watch 符号保留。
主状态机仍为九状态；状态流程专项、输出轴编码器、位置扭矩域迁移不在本批实施。
组合实机时序、动态栈、电机运动及 LoaderApp 容量/升级尚未验证。
回退使用本批备份或逐项逆补丁，只回退本批内容，不覆盖现场修改或原始工程。
