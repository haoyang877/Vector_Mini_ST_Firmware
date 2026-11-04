# ADC快中断调用链简化 v1

## 目标与不变行为

删除main中只转发App_FastTick的BspApp_OnFastSample，将实际轴快环和健康计数直接作为
APP对BSP事件的实现。删除STM32仅供该回调使用的Adc_CompleteFast包装，条件就地判断。
保留main的真实向量入口、BSP硬件边界、AT32台架共用完成确认及STM32快速派发/HAL回退。
不引入函数指针、注册、额外状态、宏别名或板型分支。

20 kHz节拍、NVIC优先级、采样/控制计算、状态/故障/关相行为、参数和协议ABI不变。
AT32仍先确认并清除主/辅ADC完成位，再执行快环；STM32仍先执行快环，再清JEOS/JEOC。
轴任务结束后每个有效事件只增加一次健康计数。快环入口继续在AT32的SRAM执行。

## 基线、验收和回退

基线为本次修改前的the-system-refactor工作树（含用户已有修改），源码和三目标构建存于
outputs/adc-irq-simplify-20261103/before/，不使用HEAD替代该工作树基线。
修改前路由、AT32配对/清标志、STM32两种回调配置各12289例、四组APP事件轨迹均通过。
三目标为AT32正式APP、STM32普通APP及Loader APP，均0错误0警告。

验收：真实事件入口的路由/健康计数、STM32句柄和完成位过滤、原HAL快派发回退、
四组APP逐事件基线对照、完整PR及release门禁、三目标构建和资源/入口反汇编对照。
调整AT32 scatter的入口所属对象，由main.o转为app.o，保留同一SRAM区域。
不接触硬件，离线结果不等于实机时序验收。当前无栈峰值或实测周期数据。

回退时仅按before/source逐项还原本次修改的文件；保留本次之前和期间的其他用户修改。

## 结果

双板main删除纯转发事件函数；APP原App_FastTick函数体仅改名为BspApp_OnFastSample，
内部轴任务与健康计数逐字不变。STM32完成位检查合并到HAL回调内；生成IRQ未修改。
两板main仍逐字一致，未删除AT32正式APP与台架共用的Adc_CompleteFast。

已通过：双板真实main路由、AT32无事件/配对失败/清标志顺序、STM32句柄/完成位过滤及
回调后清标志、STM32原快派发两种配置各12289例，以及四组APP修改前后完整事件轨迹对照。
完整PR检查通过：124项Python单元、16项集成和43组原生C检查；格式、lint、工程、分层、
接口、文档和卫生检查均通过。三套正式固件重新构建均0错误0警告。

| 目标 | Code变化 | RO/RW/ZI变化 | SRAM代码变化 |
| --- | ---: | ---: | ---: |
| AT32正式APP | -16 B | 0 B | +8 B |
| STM32普通APP | -24 B | 0 B | 0 B |
| STM32 Loader APP | -24 B | 0 B | 0 B |

AT32原BspApp_OnFastSample在SRAM，但经跳板转到Flash的App_FastTick，再跳回轴快环。
合并后整个事件函数在0x200005cc直接调用MotorAxis_FastTick，去除Flash往返跳板。
轴任务返回后的计数指令及push/pop保持；这不表示整条链的实测时延或栈峰值已验收。
镜像哈希因代码和布局变化不同，协议、参数ABI与数据RAM未变化。

当前源码已冻结到独立Git验证副本执行release门禁；原工作树及其索引不提交、不重置。
副本只读引用原仓库对象，输出与原工作树分离。最终release门禁全部通过，包含Keil重建和
AXF/HEX/MAP哈希；普通STM32 APP的HEX与原工作树本次构建逐字一致。
首轮副本未复制旧文档引用的86项本地证据，导致文档检查及其单元测试失败；补齐原始附件、
统一验证进程UTF-8后重跑通过。没有修改或放宽检查，失败日志保留。
冻结后仅本计划继续补记结果；其他会话同时修改的协议与服务绑定文件不纳入本轮源码证明。

证据：

- [修改前构建](../../../outputs/adc-irq-simplify-20261103/before/builds.json)
- [修改后构建](../../../outputs/adc-irq-simplify-20261103/after/builds.json)
- [资源与反汇编](../../../outputs/adc-irq-simplify-20261103/comparison.json)
- [源码对照](../../../outputs/adc-irq-simplify-20261103/source-comparison.json)
- [PR摘要](../../../outputs/runs/adc-irq-simplify-20261103-pr/summary.json)
- [冻结源码清单](../../../outputs/adc-irq-simplify-20261103/release-source-manifest.json)


- [最终摘要](../../../outputs/adc-irq-simplify-20261103/summary.json)
- [冻结候选release摘要](../../../outputs/adc-irq-simplify-20261103/release-source/outputs/runs/adc-irq-simplify-20261103-release2/summary.json)

用户随后要求AT32在main直接执行轴任务和计数；当前实现见[v2](2026-11-03-adc-irq-simplify-v2.md)。
