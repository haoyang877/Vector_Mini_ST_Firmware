# 删除AT32启动状态 v1

用户要求删除system.c中的s_startup_state。删除变量及Bsp_Init/Start的所有读写，
BspWatchdog_Start不再检查它；不新增替代状态变量。公共函数签名不变，AT32初始化和启动
由main负责按序各调用一次。独立s_watchdog_started仍保护看门狗重复启动和喂狗。
STM32同名状态不在本轮范围，Flash、看门狗配置结果和运行期故障处理保留。

基线为at32-init-assumed-success-20261103。同步AT32生命周期测试和公共接口说明，
运行完整PR及正式目标编译；不操作硬件。回退仅恢复本轮启动状态变量、判断和文档。

## 完成结果

AT32源码及两份正式MAP均无s_startup_state；未引入替代启动阶段变量。
Bsp_Init/Start按原顺序直接执行，BspWatchdog_Start仅保留独立看门狗状态及配置结果检查。
调用方、AT32生命周期测试和公共接口说明已同步；STM32实现与同名状态保持不变。

完整PR通过：44组原生、140项单元、16项集成，格式、lint、布局、架构、接口、文档及卫生通过。
证据为outputs/runs/at32-startup-state-removal-20261103-pr/summary.json。
四个正式目标均0错误0警告，证据为outputs/at32-startup-state-removal-20261103/builds/。
两份STM32 HEX与上一轮相同；两个AT32目标均减少Code60字节、RO4字节，
RW减少4字节而ZI对齐增加4字节，RAM合计不变。对照见同轮resource_comparison.json。
本轮未烧录、未操作硬件。
