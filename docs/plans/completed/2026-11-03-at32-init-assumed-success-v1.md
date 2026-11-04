# AT32外设初始化默认成功 v1

## 用户要求与范围

用户已删除system.c中的部分初始化状态判断，并明确要求按外设初始化默认成功处理，
删除初始化异常路径。本轮限定AT32的时钟、PWM/监督、SysTick和BSP启动编排。
McuSystem_Init/Start改为void，调用方负责各调用一次；就绪位仍等待，不再返回超时或频率异常。
外层Bsp_Init/Start保留公共签名和调用顺序检查，但不再把外设初始化结果转为失败。
Flash_Init的AT32调用仅建立启动访问资格，不影响后续Flash读写的校验与错误语义。
看门狗启动/Flash窗口恢复、运行期故障和关相策略不在本轮删除范围。

## 实施与验收

保留现有时钟、分频、PWM和监督配置顺序；清除已失去用途的状态量和失败分支。
同步两个私有声明、独立台架调用、初始化测试及接口说明，不恢复用户删除的状态守卫。
当前基线已有未使用s_timing_started的编译错误；修改前文件保存在
outputs/at32-init-assumed-success-20261103/，先前通过的固件证据为keil-calibration-save-20261103。
验证正常配置和调用顺序、外层一次性状态约束、运行保护回归、完整PR和九个相关固件目标。
不访问或操作硬件；时钟硬件异常不再作为此接口的可返回结果。
简化后编译器曾内联两个McuSystem函数，造成既有Flash段匹配警告；显式禁止这两个入口内联，
保留原链接布局，未修改scatter或Flash分区。

## 回退

仅撤回本轮初始化签名、流程、调用方和测试文档，保留用户已有删除及此前功能修改。

## 完成结果（2026-11-03）

clock_init、McuSystem_Init和McuSystem_Start改为void；保留硬件就绪等待，
移除时钟超时/频率核验返回、SysTick返回处理、未使用状态及BSP内的初始化失败分支。
AT32正式APP与五个底层台架均已同步；公共BSP的调用顺序检查仍保留。
看门狗、Flash运行期结果、关相与故障行为没有删改。

专项与完整PR通过，最终代码在独立冻结副本通过release全档检查，含44组原生、140项单元、
16项集成，以及格式、lint、布局、架构、接口、文档、卫生、构建和发布清单。
主工作区未提交；冻结副本代码/链接脚本/工程输入共401项与当前工作区哈希一致。
证据：outputs/runs/at32-init-assumed-success-20261103-pr/summary.json、
outputs/runs/at32-init-assumed-success-20261103-final-release/summary.json，
冻结清单与一致性记录位于outputs/at32-init-assumed-success-20261103/。

四个正式目标和五个AT32台架目标均0错误0警告。
与上一轮保存功能构建相比，两份STM32 HEX一致；两个AT32正式目标均减少Code384字节、
RO48字节，RW/ZI不变。McuSystem两个入口仍按原scatter放在Flash。
构建证据在同轮final-builds/、bench-builds/、release-build/和resource_comparison.json。

验证中出现的初次内联段匹配警告已通过保留独立Flash入口消除；
初次冻结副本因Git alternates路径CRLF无法读取历史参考，修正为LF后完整重跑通过。
本轮未烧录或操作电机；时钟硬件异常会停留在就绪等待，不再作为可返回的初始化错误。
