# 双板 Loader 离线开发记录

日期：2026-11-02。工作分支：`codex/dual-board-loader`。
本轮实现和验证开发候选，不连接硬件、不刷写、不启动电机。

后续更新：用户另行授权后已进行[AT32实机预检](dual_board_loader_at32_bench_20261102.md)，
发现SWD回读不稳定并阻止安装，同时修正维护工具；本文的构建/源码审计仅对应当时冻结版本。

## 交付范围

1. G431保留现有升级能力；AT32F456CCU7新增独立Loader、板级/Flash/CAN端口与配套APP。
2. 共用原有升级和镜像核心。Type1页0..2、Type50..57、1M/5M CAN FD和96B描述符格式未改变。
3. 两板都采用前128 KiB内的16 KiB Loader、96 KiB APP和16 KiB参数/身份。
   APP起址0x08004000，参数起址0x0801C000，身份起址0x0801F800。
4. Keil日常工作区仅列出两板LoaderApp；APP的AXF/HEX/BIN只含APP加载域。
   AT32保留ER_FAST及其Flash装载副本，后处理验证同源镜像与启动标记。
5. 维护工具提供双次全片备份、完整保护区比较、保存参数/LUT查看、受控初装及AT32整16 KiB原样迁移。
   Rust维护页调用同一Python实现；不同目标包互拒。
6. 初装按未激活APP→显式参数迁移→Loader→全片验证→最后激活页顺序执行，最终保持CPU暂停。
   旧AT32非空参数未给迁移包时拒绝；新布局拒绝重复迁移。
7. 两板普通APP、Loader和LoaderApp已纳入release构建清单；台架激励工程不进入默认构建。

使用步骤见[双板指南](../../guides/dual_board_loader.md)和[Keil下载说明](../../guides/keil_loader_app_debug.md)。

## 复审修正

- AT32拆分32位编程执行8B逻辑写入；半写标记不补写，读回不符或Flash错误锁存到复位。
- CAN只允许一个PTB在途槽，64位票据关联真实终态；超时、旧完成位、取消、bus-off均不能误报成功。
- LoaderApp不再同时启动和喂BSP短看门狗；健康/确认期限只由启动服务控制，避免旁路喂狗。
- AT32包保留SRAM快代码布局；G431补齐BSP板型宏，目标选择头显式优先于公共头。
- SWD初装核对UID/容量/关相，暂停后再次核对；临时冻结继承WDT只写调试寄存器。
  正常从Loader复位启动撤销冻结。探针关闭错误和全片回读错误均不报成功。
- SWD最终激活提交完整首APP页，避免下载算法按页擦除时损坏向量；不预先确认APP。
- 协议成功与完整保护区验证独立报告。只有真实快照比较通过才设置`protected_regions_verified=true`。

## 构建与资源

版本：Loader 2.2.0 / APP 1.6.0。以下包含链接填充和SRAM代码的Flash装载副本。

| 板卡 | Loader / 16,384 B | APP / 98,304 B | 保留参数/身份 | 分区总量 |
| --- | --- | --- | --- | --- |
| G431 | 14,540 B | 63,464 B | 16,384 B | 131,072 B |
| AT32F456 | 14,808 B | 62,632 B | 16,384 B | 131,072 B |

配套包分别见[G431清单](../../../outputs/loader-development/g431-verified/summary.json)、
[AT32清单](../../../outputs/loader-development/at32-final-candidate/summary.json)。
每个清单记录AXF、HEX、BIN、CAN包的SHA256；均为开发候选，`release_approved=false`。

## 离线验证

- [PR检查](../../../outputs/runs/dual-board-loader-final-pr/summary.json)：
  151项Python单元、16项集成以及42组原生测试通过；格式、lint、分层、接口、文档和仓库检查通过。
- 新增AT32真实端口寄存器模型覆盖CAN精确票据/迟到完成/总线故障、Flash写域/错误锁存/超时/计时回绕、
  APP确认身份/输出联锁/5s期限/半写两方向/看门狗期限。模型测试不替代Flash电气行为实测。
- 主机Rust 17项测试通过；两板真实包均通过离线装载，错板拒绝。Windows可执行程序构建成功，
  [AT32界面预览](../../../outputs/loader-development/ui-at32-final/preview-summary.json)标记未连接硬件。
- 维护工具9项内存探针测试通过，涵盖整片只读、尾区全字节保护、错误UID、迁移摘要绑定、坏CRC、
  非空旧参数缺迁移包拒绝、重复迁移拒绝、暂停失败零Flash写入及未确认初装。
- [release检查](../../../outputs/loader-release/outputs/runs/dual-board-loader-final-release/summary.json)通过：
  干净冻结提交`250b892fa17e08e5431605c54ae86a9df3077ba8`，六个工程全部零错误、零警告，
  AXF/HEX/MAP工件及SHA256记录齐全。[源码审计](../../../outputs/loader-development/final-source-audit.json)
  确认当前594个代码/构建输入与冻结副本一致。文档状态在检查完成后更新。
  验证副本使用真实输出目录、完整本地Git历史及较短路径，分别排除了临时目录归属、历史FOC基线缺失
  和Keil头文件路径长度问题；中断或失败的运行不计入通过。

新worktree继承357项未提交变更；本轮仅叠加Loader相关修改，未重置或清理其他工作。
[任务差异清单](../../../outputs/loader-development/development-diff.json)以继承状态为基线，
共享协议与升级核心均无本任务修改。冻结副本只用于验证，未向原分支提交、合并或推送。

## 实机待验收

两板首次安装、AT32参数迁移、Keil反复下载前后逐字节比较、各提交阶段真实断电、
WDT实际期限、Flash停顿、栈余量和重定位快环时序均为NOT_RUN。
每板100轮CAN升级也为NOT_RUN，历史main/G431成功次数不计入本候选。
下一阶段按[既定计划P4](../../plans/active/2026-11-02-dual-board-loader-v1.md)核对设备、UID、
动力隔离和恢复备份后执行。本轮未修改选项字节或实际设备参数。
