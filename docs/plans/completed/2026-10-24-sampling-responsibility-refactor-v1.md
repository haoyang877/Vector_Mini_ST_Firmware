# 双板采样职责拆解实施 v1

日期：2026-10-24。用户已授权按BSP统一策略实施第一批采样拆解、双板编译与离线回归；禁止烧录。

## 目标与基线

按[统一策略](../active/2026-10-24-bsp-unification-strategy-v1.md)把AT32采样配置、运行采集、存储协调、
诊断发布及上层适配拆开；STM32沿完整初始化/触发/IRQ/原码/温度链路对应归位。
使用共同的采样硬件契约，不只重命名原有大文件，也不新增一份电流换算或故障策略。
HEAD为8dd29950，实际1149份工作文件快照及原有修改列表在
`outputs/sampling-refactor-20261024/source-before/`、`source-before.json`、`git-before.txt`。

本次授权覆盖采样平台内部接口、直接启动/中断接线、工程/链接和相应测试调整。
原有motor_hw感测接口继续兼容，控制算法、参数、协议、标定零偏与保护的实现不改。
保留20kHz/2kHz、ADC四次累加、触发位置、顺序、DMA边界、错误编号、故障关相、
Flash前后暂停恢复和各目标温度/NTC/sin-cos开关。
STM32既有HAL返回处理与IRQ清标志顺序保持；不借拆解新增或删除保护。

## 实施分工

| 位置 | 职责 |
| --- | --- |
| platform/api/bsp_sampling.h | 两板共同的采样初始化、启动、快帧确认、原码访问及辅助/温度样本契约 |
| 各板bsp/bsp_sampling.c | 采样生命周期、硬件采集及错误处理；对上提供同一契约 |
| AT32 bsp/adc_sampling_setup.c | GPIO、ADC/DMA、序列/过采样的配置与校准 |
| AT32 bsp/adc_sampling_storage.c | Flash事务所需的停转换、DMA重建及新鲜样本恢复 |
| AT32诊断模块 | sin/cos快照及原有发布分频，不进入电流换算 |
| 两板ports/motor_sensing及温度端口 | 将共同采样契约适配到既有motor_hw接口，芯片温度换算保持原语义 |
| board_startup与IRQ入口 | 按原顺序调用采样契约；厂商向量/HAL回调保留 |

AT32采样模块仍单独拥有采样状态，拆分帮助函数通过私有显式对象协作；
不把状态暴露给APP，不把诊断对象当作控制输入。
必要的兼容声明与调用同步迁移，工程源清单和Commissioning SRAM/Flash位置一起更新。

## 验收与回退

先构建STM32普通/Loader配套、AT32关相/限时四目标基线，保存镜像、MAP及资源。
迁移后重建上述四目标与受影响的AT32基础台架；要求0错误0警告，解释加载字节和资源差异。
原生回归覆盖实际拆分模块的启动、采样/过采样、丢帧、DMA异常、关相、暂停恢复、
温度/NTC禁用、sin/cos分频、序号回绕和载波/ADC抢占；新增STM32实际采样链路测试。
执行PR与release离线检查；脏工作树使用实际源码一致的独立冻结副本，不提交/清理用户工作树。
时序、寄存器/事件序列、控制及参数/协议冻结文件哈希参与对比，编译不能替代实机验收。
所有证据归`outputs/sampling-refactor-20261024/`；未执行实机测试明确保留。
回退按本批文件清单恢复source-before中的文件，删除仅本批新增文件，不reset/clean或覆盖其他修改。

## 结果

第一批采样职责拆解及双板离线验收完成，全程未下载、复位或操作电机。
共同接口为bsp_sampling.h；两板原码适配文件内容相同。
AT32拆为采样运行/唯一状态、ADC/DMA配置、存储协调和sin/cos诊断；
STM32将校准、CH4/注入启动、原码/温度采集归BSP，温度换算归独立端口，
原CubeMX初始化和USER CODE回调的确认/清位时序保持。
完整职责见[采样链路](../../architecture/sampling_chain_v1.md)。

### 编译与回归

- 工作树最终PR全部通过：104项Python单元、14项集成、36组原生测试。
- 实际源码一致的独立冻结副本release全部通过，含本次旧板重建和发布清单；
  冻结提交5bcd3e08ea964665fcfe11c36e93e3a33287e369，仅在outputs副本，未提交或移动原分支。
- STM32普通/Loader配套、AT32关相/限时共四份完整APP，以及Foundation、Dry、Power、
  Combined、ClosedLoop五份AT32台架工程均0错误0警告。
- 冻结副本另外显式重建Loader配套及AT32两目标；四个APP的实际HEX加载字节与工作树逐字节一致。
- 已归档原始快照的AT32采样回归再次通过；新回归直接执行拆分后的实际模块，
  同时验证STM32实际启动顺序、JEOS回调、原码、温度换算/触发及序号回绕。
- 181份APP/Motor/Parameters/Communication/Shared/Loader业务与ABI文件保持SHA-256不变。
  原motor_sensing接口声明与布局的非注释token不变；14个关键函数体在明确状态/符号映射后相同，
  外设配置及诊断发布体也已核对一致。

| 完整APP | 迁移前Code/RO/RW/ZI | 迁移后Code/RO/RW/ZI | Flash/RAM增量 |
| --- | --- | --- | --- |
| STM32普通 | 77432/3140/188/18324 | 77556/3144/196/18324 | +128B/+8B |
| STM32 Loader配套 | 79624/3276/220/18340 | 79752/3276/228/18340 | +128B/+8B |
| AT32关相 | 72992/3448/364/34284 | 73088/3448/292/34356 | +96B/+0B |
| AT32限时 | 74064/3472/352/34296 | 74152/3472/292/34372 | +88B/+16B |

镜像与迁移前不相同：多编译单元的静态链接适配、STM32采样状态/温度序号、
AT32私有状态对象及编译器数据布局产生上述小幅变化，不声称二进制等价。
没有改动参数记录、运行配置或诊断对象的公开布局；原DMA缓冲转为私有采样对象成员。
Commissioning的快采样/原码/DMA/载波函数仍在0x20000000起始的ER_FAST，
初始化与存储协调在Flash。ER_FAST使用0xAF1C/0xE000，RW_IRAM1使用0x8738/0x9000，
参数区、8KiB栈及顶端UNINIT诊断区未改变或溢出。

首次PR仅缺少新诊断模块的中文意图注释，修复后最终PR通过。
首次Commissioning链接的诊断空节规则产生1条告警，去除本目标未启用的空诊断匹配后，
全部目标重新构建0错误0警告；原失败日志保留，未屏蔽告警或降低检查门禁。

### 证据与后续边界

统一证据目录为`outputs/sampling-refactor-20261024/`：
`before/builds.json`、`after/builds.json`、`semantic-comparison.json`、
`resource-comparison.json`、`execution-regions.json`、`frozen-image-comparison.json`、
`source-frozen.json`及`summary.json`。
工作树PR见`outputs/runs/sampling-refactor-pr-20261024-final/summary.json`；
release见本证据目录`frozen/outputs/runs/sampling-refactor-release-20261024/summary.json`。
冻结排除的个人uvoptx、JLinkSettings.ini和.vscode逐项列于清单，不参与编译；
其余1153份快照文件在冻结时与工作树字节一致，收尾仅补充当前文档与结果记录。

本批不新增STM32载波丢帧或传感器CRC检查，也不削弱AT32已有保护。
统一能力状态、合并快帧消费及其他BSP功能继续由总策略承接，不扩大本批控制行为。
新镜像的实机快环时限、采样噪声、栅极输出和标定/运行尚未重新验证；
离线通过不能继承旧镜像时序结论，后续仅按明确授权测试。
