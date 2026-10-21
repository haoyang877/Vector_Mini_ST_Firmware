# APP 启动身份来源离线交付记录

日期：2026-10-08。独立分支 `codex/boot-epoch-provider`，尚待 ROOT 统一接线、链接及验收。

- 完整基线：`7f44bd083902081b00d1761648c158c34350d098`。
- 源码提交：`ef1025389f16d0fea1f4f3a3b73e6fe27a73a38a`。
- 干净源码 HEAD 的 PR：`outputs/runs/20261008T131538513623Z-ef102538/summary.json`，PASS，
  `dirty=false`、`hardware_contacted=false`。
- 逐文件SHA256及新增工程源清单：`outputs/boot-epoch-v1/source-manifest.json`。

本记录是源码和离线证据，不是新固件发布、实机Flash或整机防重放通过声明。
格式、地址和兼容定义见[启动身份记录v1](../../archive/protocols/boot_epoch_record_v1.md)，
目标、回退及未完成接线见[实现计划](../../archive/plans/2026-10-08-boot-epoch-provider-v1.md)。

## 已交付的实际实现

provider在已关相、采样中断和通信尚未开始时初始化。完整扫描两页，追加或轮换已识别页，
8B顺序编程且最后双字提交，整条记录回读验证后发布稳定u64。
未初始化或失败时为0，同次初始化不重试、不改变身份。
未知、撕裂、损坏记录拒绝且不擦；最大计数不回绕。

平台BSP对epoch擦写仅允许61/62页，对参数保存仅允许56..58页。
保留59/60页、63身份页、4304B schema12和16KiB总边界均不改。
实际参数保存唯一调用由参数事务传入sizeof(InterfaceParam)，未发现其他现用宽写调用。
现场61/62页是否空白未读设备，启动时识别未知非空即拒绝，不能把源码未使用当现场空白。

Type3状态源接入完整u64和有效位，普通APP与LoaderApp镜像阶段分别处理。
新功率线路的epoch匹配由H协议边界完成，本实现不更改底层主状态机、readiness、
控制数学或旧单项开发入口；这些源码与基线无差异。
旧117、单项维护和Watch不能据本项被宣称已经具备跨复位防重放。

## 检查和证据范围

| 检查 | 实际输入和结论 |
| --- | --- |
| provider生产核心 | 1024次程序重启、14次页轮换，计数单调且同次初始化稳定 |
| 写入中断 | 一条32B记录的0..32字节共33个中断位置；完整未发布记录下一启动计入，非空撕裂保持不可用 |
| 擦除中断 | 0、1、8、31、32、2047、2048字节位置；只保留已确认最新页，部分擦除不修复 |
| 非法记录 | 未知尾部／版本、CRC损坏、空洞、重复、缺口、跨页冲突、零值，拒绝且无新写擦 |
| 失败和耗尽 | 输出开启、读取失败、持久提交后读回失败／不一致、u32进位、u64最大值后耗尽，均无已发布身份重用 |
| 写域生产体 | flash_write_param和epoch Program/Erase真实生产体；越界／溢出／对齐／输出条件拒绝，参数保存不改保留区 |
| 状态源生产体 | normal和LoaderApp，u64高低位、零身份和有效位；镜像确认不冒充epoch资格 |
| 启动组合生产体 | 身份初始化位于MotorAxis_Init后、采样和通信前；身份返回失败不被伪装成新线路能力 |
| PR全部检查 | 格式、lint、工程路径、Python/原生测试、架构、接口、文档与hygiene全部PASS |

原生原始日志：`outputs/boot-epoch-v1/native/`。
全套结果与日志：上述PR运行目录；测试不调用真实端口、探针、CAN SDK或Flash。
双位ECC/NMI未在设备注入，模型失败与源码边界不能代替异常实测。

## 局部容量与工程接线

ARM Compiler 5.06 update7 build960，用相同 `Cortex-M4.fp`、C99、microlib、
`--enum_is_int`、`--split_sections`、`-O2 -Ospace` 独立编译当前与基线对象。
命令、源码／对象SHA256和fromelf大小在 `outputs/boot-epoch-v1/arm_objects/summary.json`。

| 对象 | ROM变化，Code＋RO＋RW初值 | RAM变化，RW＋ZI |
| --- | --- | --- |
| 新boot_epoch_stm32g4 | +696B（624＋56＋16） | +16B |
| 既有flash | +268B | 0B |
| board_config普通APP | +4B | 0B |
| status_adapter普通APP | +32B | 0B |
| 对象级合计 | +1000B，约0.98KiB | +16B |

这不是实际AXF/MAP增量：未做完整链接，不含按工程逐文件选项、链接裁剪和组合上下文的差异。
CRC核心已有生产调用，没有再实现一份CRC；新增对象不分配堆或大常驻数组。
栈、启动耗时、擦除耐久和动态时序尚未测量。

唯一新增工程C源：`firmware/platform/stm32g4/ports/board/boot_epoch_stm32g4.c`。
ROOT需将它加入普通APP和LoaderApp的Platform组；Loader不链接本provider。
API与BSP头的既有include目录可解析，新共享常量已与目标JSON一致。
原Keil工程及用户选项均未修改。PR的project-layout PASS不证明新源已经进入最终链接。
最终候选还需ROOT执行适用release验证和容量核对，本轮没有最终固件工件可发布。

## 恢复限制和待收口项

ECCD会沿现有NMI死循环停在已关相、通信未开始的初始化；可能没有在线诊断回复。
这是初始化fail-closed，不能被写成自动恢复或运行期故障处理已经验收。
未知和撕裂记录需要受控维修；维修及迁移策略未实现，不提供擦计数回1的恢复命令。
全片擦除、旧固件覆盖、外部未知写入会破坏连续性保证。

历史整片或完整16KiB备份回滚61/62页，即使记录CRC有效，也可能复用曾发布编号；
当前实现不具备抵抗这类回滚的硬件保证。参数保留验收须分别核对RAW前三页、
生产身份63页和epoch61/62页，正常启动计数变化不算标定破坏。
恢复旧固件／参数不得无区分覆盖epoch页或擦后回到1；安全维修及回滚处理尚未实现，
分域规则见[记录契约](../../archive/protocols/boot_epoch_record_v1.md)的备份恢复章节。

待ROOT：工程接线、内存所有权最终核对、完整构建／容量、H新线路的实际epoch匹配与能力位、
正式生产旧功率线路权限政策，以及后续明确授权的设备验收。
本轮未连接硬件、烧录、复位或运动，不报告实机PASS。
