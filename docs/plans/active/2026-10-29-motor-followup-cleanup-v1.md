# motor归属与命名复核整改 v1

## 目标

清理不存在实现的公开声明；将MCU温度测量文件和接口按真实来源命名；统一速度控制前缀、
头文件保护宏和测量注释，去除拆分后的无关包含。合并任务结果值类型到motor_types，
将仅位置控制使用的常量收回实现。保留热模型、型号准入、标定核心等已确认的独立边界。
按独立阶段评估并处理FOC计算、测量状态和PWM提交边界。

## 不变项

不改变模型和阈值、浮点求值顺序、20kHz/2kHz调用次序、热历史和降额、保护故障时序、
功率许可、参数/Flash/协议ABI或目标默认值。保持AT32 SRAM快环与Flash后台LUT位置。
保留当前工作区全部既有更改，不下载固件或连接硬件。

## 基线与验收

当前工作区PR基线：`outputs/runs/20261029T101142600092Z-2dbb4003/summary.json`；
7个目标源/HEX快照：`outputs/motor_cleanup_20261029/`。按后续范围决定，以STM32和AT32
两套正式APP为当前构建验收目标；执行PR与release级检查、FOC参考向量、状态机固定版本对照、
AT32 map SRAM/Flash核验及正式APP的HEX比较；实机快环时间与闭环行为如未授权则记录未测。
检查输出只能代表实际版本和运行；脏工作区不能形成release manifest。

## 分阶段与回退

1. 类型与命名清理；保留热配置、型号选择和标定核心；独立验证。
2. 确定FOC及测量边界的所有调用者和真实副作用，采用可测的小步变更。
3. 复核目录、头文件及项目、文档，记录实际验证和未完成项。

按阶段回退对应源、引用及工程项；不得覆盖既有工作区改动或中断资源所有权。

## 已实施的PWM边界

`foc.c`负责`Foc_Prepare`计算与`Foc_Complete`后处理；`foc_output.c`保留`Foc_Step`
及兼容入口，唯一职责为调用计算、提交一次PWM并执行后处理，失败仍锁存原故障。
计算失败时不提交PWM并正确结束计时；提交失败仍执行原后处理。新增直接边界测试，
冻结向量两种计时设置各61440组、热保护及PWM失败路径离线通过。
当前测量字段仍保存在FocState，避免在缺少目标时序证据时变更状态布局或Watch语义。

## 本轮核验记录

- 类型和命名调整后PR检查通过：`outputs/runs/20261029T101645533365Z-2dbb4003/summary.json`。
- 配置统一后工作区的PR检查通过：`outputs/runs/20261029T103248089696Z-2dbb4003/summary.json`。
- 固定HEAD的状态/功率/事件轨迹及262144组规则对照通过，见
  `outputs/motor_cleanup_20261029/test_run_state_final_v2/`；基线编译只使用冻结版本自身配置。
- FOC两种计时配置各61440组对照、PWM边界和热保护通过，见
  `outputs/motor_cleanup_20261029/test_foc_final/`与`test_thermal_final/`。
- 两套正式APP构建0错误0警告；AT32快环和LUT链接位置见
  `outputs/motor_cleanup_20261029/map_placement_final.json`。
- 两套正式APP的HEX均与基线不同，见`hex_comparison_final.json`；工作区同时包含独立的
  配置统一及参数格式变更，因此不可将差异归因于单一改动，也不主张字节等价。
- 用户最终确认无感启动配置统一位于`firmware/config/application_config.h`。
  本计划的不变项描述仅约束本次motor调整，不覆盖并行工作中用户另行授权的参数格式变更。
- 尚未接触硬件，20kHz实际执行时间、闭环与功率级行为未测；测量字段仍留在FocState。
- release检查记录：`outputs/runs/20261029T103716776060Z-2dbb4003/summary.json`。
  格式、lint、布局、测试、架构、接口、文档和hygiene通过；工作区未提交，
  `clean-release-source`失败导致本轮不执行发布构建，`release-manifest`亦失败。
  该检查期间配置工作仍在进行，不作为最终冻结源码的发布证明。
- 配置入口最终版本的完整PR检查通过：`outputs/runs/config-two-app-schema14-final2-20261029/summary.json`；
  最后文档路径检查通过。
