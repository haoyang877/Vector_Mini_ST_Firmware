# 双板中断入口集中到main v1

## 目标与边界

按用户确认的方案，将正式APP的ADC、监督定时器和SysTick向量入口集中到各板main.c，
各平台的interrupt_map.h只定义向量符号映射。寄存器、HAL派发、完成标志与时基仍归BSP。
两板main模板保持一致，不新增运行期注册、函数指针、状态副本或goto。
CAN与异常处理不在本轮范围；AT32独立台架只接续移出的SysTick入口。

## 基线与不变项

基于当前工作树保存修改前源码和固件产物，不回退其它会话的文档与既有去goto修改。
保持20kHz快环、2kHz监督、1kHz时基、NVIC优先级、初始化顺序、参数与协议ABI。
保持AT32先确认/清ADC完成位再执行快环；STM32先回调再清JEOS及HAL错误回退。
监督任务结束后才触发STM32下一次内部温度转换；健康计数每个实际任务仍仅增加一次。

## 实施与验收

1. 保存源码、运行现有派发/生命周期基线并构建两板正式APP及STM32 Loader APP。
2. 将向量别名移入各板入口目录，main定义三个薄入口；BSP提供对应硬件处理函数。
3. STM32仅在生成文件USER CODE区域将原处理器改名为BSP函数，保留其完整函数体。
4. 离线验证双板入口路由、硬件确认顺序、STM32快派发/HAL回退、时基与健康计数。
5. 执行格式、lint、PR和release所需检查；比较前后固件资源及处理路径反汇编。

生成证据位于outputs/main-interrupt-routing-20261102/。
本轮不连接硬件、不烧录、不驱动电机；离线结果不作为实机时序验收。

## 完成结果（2026-11-02）

两平台main模板逐字一致，通过本地interrupt_map.h分别映射ADC、监督及SysTick。
新增三个BSP硬件处理入口，没有增加运行期注册或状态。AT32三个原处理函数除名称外
函数体逐字一致；STM32生成文件的非USER CODE区逐字一致，原HAL回调/清标志顺序保留。
监督健康计数移到App_SupervisorTick末尾，与快环一样在任务完成后只推进一次。

AT32的三个新向量在两份scatter中显式保留于原SRAM快代码区。反汇编确认每个入口只有
一条尾跳转，不增加栈帧，不经Flash跳板；旧处理主体保持。STM32三个入口同样是尾跳转。
这仍是额外一条分支，不宣称执行周期或镜像逐字节一致。

四个正式目标修改前后均构建成功；最终再构建五个AT32独立台架，共9个目标，均0错误0警告。
最终资源对照（Code+RO为ROM；RW/ZI为数据RAM，SRAM代码另列）：

| 目标 | ROM变化 | RW/ZI变化 | SRAM代码变化 |
| --- | ---: | ---: | ---: |
| STM32普通APP | +16 B | 0 B | 0 B |
| STM32 Loader APP | +16 B | 0 B | 0 B |
| AT32关相APP | +16 B | 0 B | +16 B |
| AT32调试APP | +24 B | 0 B | +16 B |

当前工作树完整PR检查通过：140项Python单元、16项集成、42组原生C检查；
其中覆盖双板真实main路由、无事件不调度、每事件仅一次计数、AT32清标志先于回调、
STM32监督回调先于下一次内部温度触发及SysTick回绕。
原STM32 ADC快速派发两种回调配置各12289组通过；FOC冻结参考61440组比较通过。

冻结同一固件源码到独立Git验证副本后，release全部门禁通过，包含Keil重建和产物哈希。
副本只读复用原仓库历史对象；不提交、不重置主工作树，也不覆盖用户的独立调参文档。
最终原工作树仅文档继续同步，固件、测试和工程输入与冻结候选一致。

证据：

- [PR摘要](../../../outputs/runs/main-interrupt-routing-20261102-pr2/summary.json)
- [冻结候选release摘要](../../../outputs/runs/main-interrupt-routing-20261102-release3/summary.json)
- [四个正式目标构建](../../../outputs/main-interrupt-routing-20261102/after-final/builds.json)
- [五个台架目标构建](../../../outputs/main-interrupt-routing-20261102/after-bench-final/builds.json)
- [资源和SRAM入口对照](../../../outputs/main-interrupt-routing-20261102/resource-comparison.json)
- [处理函数及源码对照](../../../outputs/main-interrupt-routing-20261102/behavior-comparison.json)
- [冻结源码清单](../../../outputs/main-interrupt-routing-20261102/release-source-manifest.json)

首轮PR及台架暴露时基/监督符号尚未完全迁移，修正后上述检查通过。
首次release副本共用输出目录导致路径解析测试失败；改为独立输出后又发现缺少历史FOC
参照对象，补齐只读对象访问后第三轮通过。失败日志原样保留，没有放宽检查或替换参考。
没有运行CubeMX再生成GUI或实机时序测试；未烧录、未启相。

## 回退

按保存的源码副本仅撤销本轮中断迁移、测试和文档；保留其它工作区修改。

## 2026-11-03：映射头归属调整

按用户要求将两份interrupt_map.h从projects应用目录移到各自BSP MCU目录，
main.c保留相同include，通过各工程已有MCU包含路径选择平台。不保留旧路径转发头。
仅路径、头文件保护宏和测试引用变化；中断名称、函数体、时序、优先级、参数/协议ABI不变。
分层规则只额外允许应用引用两份纯映射头，不开放其它MCU实现头。

基线为本节修改前工作树，预处理结果和头文件副本存于outputs/interrupt-map-move-20261103/。
验收为两平台main预处理文本前后相同、四个正式目标构建、完整PR检查。
回退仅恢复本节路径、测试、规则和说明；不撤销11月2日的中断入口迁移及其它用户修改。
已完成。两平台main预处理文本分别与迁移前逐字一致；四个正式目标均0错误0警告，
HEX与11月2日归档候选相同，Code/RO/RW/ZI资源未变化。
完整PR检查通过（140项Python单元、16项集成、42组原生C检查），未烧录或操作硬件。
证据：[预处理对照](../../../outputs/interrupt-map-move-20261103/preprocessor-comparison.json)、
[构建对照](../../../outputs/interrupt-map-move-20261103/build-comparison.json)、
[PR摘要](../../../outputs/runs/interrupt-map-move-20261103-pr/summary.json)。
