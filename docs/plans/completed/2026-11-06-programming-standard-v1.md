# 统一编程规范与 PI 标准样例 v1

日期：2026-11-06。范围仅为重构分支的规则收敛、PI 样例及相关文档；不操作硬件。

## 目标与边界

- STYLE.md 唯一定义编程决策、命名、格式和注释；ARCHITECTURE.md 定义职责、依赖与所有权。
- AGENTS.md 只保留入口、工作流程和验证要求，通过引用使用上述规则。
- 选择真实使用的 pi_controller.c/.h，整理单位、接口契约、中文注释与格式。
- PI 的公开符号、结构体成员/布局、运算次序、抗饱和、输出跟踪及重置行为保持不变。
- 不新增 PI 配置层、初始化状态、错误码、硬件依赖或重复运行状态；不为缩短名字改调用方。
- 模板只修正与实现不符的说明，不改变实现、接口或失败行为。

## 基线与调用者

- HEAD：d110f17d9e05cfd41b7d5ccca9e715942586d2c5；工作树包含其他会话的未提交变更。
- 修改前文件及状态清单：outputs/style-unification-20261106/baseline/。
- 已检查 FOC、速度环、位置环、无感启动、标定与 APP 生命周期的 PI 调用及字段使用。
- 修改前 FOC/PI 原生对照通过 61,440 次状态及 PWM 次序比较。
- 修改前 ARM Cortex-M4 hard-float、O2 模块编译通过；对象保存在 baseline/pi_controller.o。
- 修改前 PR：outputs/runs/style-unification-20261106-baseline/summary.json。
  format、lint、工程路径、架构、接口、文档、卫生检查通过；tests 失败。
  已有失败包括 AT32 工程分组一致性和 RTT 厂商文件固定哈希；最终记录完整对照结果。

## 执行与验收

1. 收敛三份规则，明确函数/文件拆分、结构体、全局观察、边界检查和抽象的判定表。
2. 整理 PI 样例并解释适用边界；修正模板与规则的冲突。
3. 比较 PI 的 C 语法记号与公开布局；仅允许形参 sample_period 改为 sample_period_s、
   私有函数 pi_clamp 改为 clamp_output，不改变公开符号。
4. 使用同一编译器比较修改前后 ARM 模块对象；复用现有 FOC/PI 回归，不增加同构测试。
5. 执行 PR 验证及 git diff --check，区分既有/并行变更失败与本次引入问题。
6. 记录实际结果和未测范围，完成后移入 completed 并维护索引。

验收不以整仓既有失败为通过；本次范围必须无新增失败。编译对象相同仅支持本模块
生成代码等价，不代表整机时序、完整固件镜像或硬件已经验收。

## 回退

按本次 diff 或 baseline 文件副本逐文件回退，仅撤销本任务修改；不得重置整个工作树。
不覆盖其他会话的霍尔、输出轴、参数、工程和测试改动。没有协议或参数 ABI 迁移。

## 结果

本任务范围完成：规则、标准样例及离线验证已落实。没有进行全仓代码迁移或硬件操作。

- STYLE.md 从 211 行收敛到 158 行，统一决策优先级、拆分判定、词汇、检查边界和样例说明。
- AGENTS.md 从 100 行收敛到 65 行，只维护入口、流程与验证要求。
- ARCHITECTURE.md 从 56 行调整为 92 行，以职责表、依赖、所有权和实时边界替代易失效的能力清单。
- PI 保留四个公开操作和原结构体布局；整理私有限幅命名、采样周期单位、生命周期、
  中文注释和格式。未增加运行状态、接口、计算分支或测试框架。
- 模板初始化的失败说明由“保持未使能”修正为实际的“不修改对象”；实现原样保留。
  模板入口明确按模块边界选择，不机械复制 initialized、配置对象或错误枚举。

### 验证结果

| 验证 | 结果 |
| --- | --- |
| PI 源码和头文件记号对照 | 排除注释、空白及两个局部命名调整后完全一致，公开布局和运算保持不变 |
| ARM 模块编译 | Cortex-M4、hard-float、C99、O2、Wall/Wextra/Werror 通过 |
| PI ARM 对象 | 修改前后逐字节一致，均 2028 字节；没有生成代码或对象大小变化 |
| 模板 ARM 对象 | 仅注释调整，修改前后逐字节一致，均 1440 字节 |
| FOC/PI 对照 | 修改前后均通过 61,440 次完整状态、PI 和 PWM 顺序比较 |
| 完整 PR | 修改前后结果相同：45 个测试组通过，unit 组各有下面两个既有失败 |
| 其他 PR 检查 | doctor、format、lint、project-layout、architecture、interfaces、docs、hygiene 均通过 |
| 补充入口检查 | STYLE、ARCHITECTURE、AGENTS、模板 README 共 22 个本地链接有效；归档后复查 |

两个既有失败均位于 tests/unit/test_at32_target.py：

- test_full_app_grouping_and_common_options_match_old_board：AT32 工程分组/共同选项不一致。
- test_keil_rtt_is_isolated_and_vendor_files_are_pinned：RTT 厂商文件与固定哈希不一致。

完整 PR 仍为 FAIL，未通过本任务弱化测试或修改无关工程；本次未新增失败。
由于工作树有并行变更，上述结论针对记录时的源码与检查，不宣称整仓发布可用。
本次未链接完整固件、未烧录、未进行电气/时序或实机验收；模块对象相同不替代这些验证。

### 证据与复现

- 修改前/后 PR：outputs/runs/style-unification-20261106-{baseline,final}/summary.json。
- 测试组与失败项对照：outputs/style-unification-20261106/pr-comparison.json。
- 模块对象与源码对照：outputs/style-unification-20261106/equivalence.json。
- 对象编译命令及输出：outputs/style-unification-20261106/after/*_compile.json。
- 本地入口检查：outputs/style-unification-20261106/entry-links.json。
- PI 对象 SHA-256：53f1fd085dd4eed8a6ffa78b24ef124091425d98368350c83ad086a3d63dbead。

在同一锁定环境使用 python tools/run.py verify --profile pr 复查；
模块对照脚本保存在 outputs/style-unification-20261106/verify_equivalence.py，
依赖同目录冻结的 baseline 与本机 ARMClang。该脚本及编译日志仅作为生成证据，不进入源码。
