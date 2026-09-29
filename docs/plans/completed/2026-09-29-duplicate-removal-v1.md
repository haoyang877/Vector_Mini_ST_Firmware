# APP/Loader 重复实现收敛 v1

## 目标与基线

基线 `05d98016`，从重构主线基线创建隔离工作树，实施分支 `codex/remove-duplicate-implementations`。
移除没有生产调用的旧位置/速度入口和完整位置输出接口；将真实跨模块使用的 CRC32、
小端字节读写与确认请求规则收敛到窄职责模块，评估标量数学合并的资源收益；FOC 只复用共同采样和输出流程。
不添加兼容别名、运行开关、通用工厂或可写全局。

修改前 release 全门禁 PASS：`outputs/runs/duplicate-baseline-20260929/summary.json`。
源码冻结与三工程构建留在 `outputs/duplicate-removal-20260929/baseline-*`。
三工程资源对照使用冻结的 05d98016 功能源码，并覆盖已提交的 ce867243 APP 工程分组与 5ce5c5a5 主工程更名。
实施分支已纳入 ce867243/5ce5c5a5，基线与候选均为相同八组布局；不将分组变化算作本轮收益。

## 不变边界与验收

- CAN 帧、参数 schema 11、邮箱、清单、描述符及 CRC 值和分块续算语义保持。
- 生命周期、故障优先级、停止关相、维护准入及辨识流程保持；保留各边界安全检查。
- 位置/速度外环仍 2 kHz，电流环仍 20 kHz；输出顺序、PI 跟踪、滤波及快速计时边界保持。
- 不改变结构布局、枚举数值、默认参数或硬件输入；无分配、阻塞或新增初始化依赖。
- CRC 核心归 shared；motor 可依赖 shared 的纯计算契约，不反向依赖 APP/Loader。
- 独立黄金参考和协议示例保留；历史丰富输出仅可存在于测试，生产只保留控制输出和遥测。
- 全部调用方和工程输入更新；共享函数有边界原生测试，真实外环缓存与控制路径有等价比较。
- PR、release 和三个 Keil 工程通过，资源/ABI/输出变化逐项说明；证据包含当前源码和产物哈希。

## 子任务

1. 迁移位置场景与配置缓存测试到生产入口，删除旧执行和完整输出接口。
2. 移动 CRC32 核心并替换邮箱/镜像副本，更新三工程、目标生成器和全部离线链接输入。
3. 统一协议字节序和确认请求条件，保留报文长度、权限、状态映射与错误检查。
4. 抽取文件内 FOC 公共计算；评估标量数学合并，无收益时保留小的局部实现。
5. 原生边界/等价比较、格式、lint、架构、接口、黄金向量和 release 资源验收。
6. 更新架构文档、提交并合回重构主线；将已合并的临时工作树用 archive_worktree 归档清理，证据先复制到主线 outputs。

## 回退与限制

本轮提交可整体 revert；实施前主线为 ce867243，功能源码基线为 `05d98016`。不覆盖其他工作树、用户修改或 ignored 实验数据。
本轮离线，不接触 J-Link/CAN，不烧录、复位、写参数或命令电机。
主机等价和 ARM 编译不能替代 MCU 周期/栈峰值和实机关相、升级验收；未执行时明确记录。
数学 NaN/负零或不同定时语义不能为消除几行代码而合并。

## 实施结果

删除旧 Task_Position_Mode、Task_Speed_Mode、PositionCascade_Update/UpdateTarget、
PositionCascadeOutput 及空 data_type.c；外环复位入口改为 MotorOuterLoop_RequestReset。
生产位置/速度外环继续由真实 2 kHz worker 执行，摩擦辨识仍调用 SpeedMode_Run。

标准 CRC32 只保留 shared/checksum 一份。持久数据、邮箱和镜像事务共用；镜像继续使用
原始续算状态，适配已取反的共享接口。协议小端转换与确认单播请求规则归窄职责头，
报文容量、业务权限和各边界安全检查保持。FOC 文件内共享采样、滤波和调制输出步骤，
两个控制策略保持独立。APP/Loader 工程、目标数据及全部离线链接输入已更新。

标量数学统一候选使 ARMCC 多次实例化非内联函数，普通 APP ROM 增加 120 B，已回退。
公共数学仍归 common/utils；不同特殊值或高速内联语义的局部小函数保持文件私有。
独立 host 示例、Python 黄金参考、不同业务状态映射和分层安全检查不强行合并。
本轮不增加兼容头、状态所有者或可写全局。

原目录出现 Keil v2 并行整理时，本轮修改转入新隔离工作树；纳入已提交 ce867243，
保留并行整理的八组布局和调试偏好。随后纳入主工程 YG_ESC_APP 更名，并重新检查组合构建。
Loader 生成器同时将继承的 RTE/层名称写为实际 Loader 名称，保持独立工程身份；两生成器重跑无差异。无关用户修改不进入本轮提交。

## 验收证据

- 完整 PR：outputs/runs/duplicate-pr-final-20260929/summary.json，所有门禁 PASS。
  测试包括 48 个 Python 单元、19 个集成、28 个原生套件与位置核心等价比较。
- 位置核心通过 790,068 拍完整状态/输出对照；真实外环缓存通过 25,200 拍。
  历史丰富输出只存在于测试参考，当前场景从公开控制结果及遥测组合。
- FOC 在诊断计时开启/关闭两种构建合计 122,880 拍完整状态、PI、PWM 和事件顺序一致。
  共享契约有 44 组独立 zlib CRC 向量、六种分块长度、未对齐字节序及 25,600 组请求条件。
- ARMCC5 Cortex-M4.fp 的 318 项保留布局、偏移和故障枚举值一致：
  outputs/duplicate-removal-20260929/abi/summary.json。删除的内部丰富输出类型不再属于生产接口。
- 新 Loader 配套 APP 实际包 89,784 B，96 B 清单、描述符/向量/CRC 以及实际包启动/救援内存模型通过；
  94,208 B APP 槽剩余 4,424 B。开发版本 0，完整性校验，release_approved=false。
- 三工程均 0 错误、0 警告，资源与产物哈希见 build-comparison.json：

| 工程 | 基线 ROM / B | 当前 ROM / B | 差值 / B | RAM / B |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP | 88224 | 88136 | -88 | 31552，未增加 |
| Loader 配套 APP | 89872 | 89744 | -128 | 31592，未增加 |
| Loader | 13520 | 13472 | -48 | 4952，未增加 |

FOC 两个共同步骤均被 ARMCC 内联，没有增加外部调用，两个策略局部栈帧均为 64 B。
寄存器分配发生变化，CurrentWithReference 680→692 B，Vq 632→636 B；整体镜像仍缩小。
静态反汇编与差异在 foc-arm-static.json 及 *-instruction-diff.txt；这些不构成 MCU 耗时或栈峰值验收。

首轮 Python import lint、试验数学头引起的纯头清单/重复 Math_Abs 桩失败已修复或随
试验回退，不新增豁免；保留失败日志。基线 LoaderApp 因 Windows 路径长度失败后，以
outputs/ref 短路径重新完整构建，三工程基线全部以成功的当次构建为准。

去重提交 df67416 的干净源码 release 已 PASS：
outputs/runs/duplicate-release-final-20260929/summary.json。
更名合并后的最终干净提交必须通过 release：
outputs/runs/duplicate-release-merged-20260929/summary.json。
release 门禁本身只构建普通 APP；Loader 配套 APP 和 Loader 另行完整构建，
不把 --target all 当作三工程证据。最终源码、产物哈希和全部结果统一归
outputs/duplicate-removal-20260929/summary.json，以其实际状态作为完成依据。
合入 codex/system-refactor 后，先复制 ignored 证据，再归档本轮 duplicate-removal 临时工作树。

本轮完全离线，未执行物理关相、掉电、实机升级、MCU 周期/栈峰值或 CubeMX GUI 再生成。
