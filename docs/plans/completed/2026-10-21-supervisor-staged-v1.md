# 2kHz监督前处理、计算与提交 v1

日期：2026-10-21。基线`a7aaa046c77927bf94815488aa5399f88922fe31`，APP1.5.7 / Loader2.1.0。
用户要求优先处理状态和故障，再执行获准控制，最后复核提交；保留启动ready且不重复推进状态机。

## 改动与不变项

- MotorAxis监督仍先处理采样节拍看门狗及编码器慢估计；温度移到状态前处理之前。
- 同一主状态机拆为PrepareSupervisor及CommitSupervisor。前处理每拍消费一次采样进展、
  STOP、任务/保存/取消结果、恢复和自检，推进状态、派发工作并应用新目标。
- 只有STARTING/RUNNING且未故障/停机/升级交接时，才允许调用既有外环。
  在栈上的MotorSupervisorStep中保存代际、监督序号、运行值及PI计算副本；不增加全局运行对象。
- 控制算法及其浮点表达式不改。最后检查真实故障、STOP、代际和守卫，
  只发布本拍有效外环字段及PI；SPEED/POSITION必须取得本拍计算及原外环ready才可启相。
- 后处理不得再次消费任务结果、自检稳定窗口或保存衔接；监督epoch只在完成提交阶段增一次，
  同一提交票据不得重复提交。两个阶段不是两次完整Tick或第二个生命周期。
- 四个App入口、八状态/线上编码、协议、参数schema14/12/13、保护阈值与控制数学保持；
  14个Watch全局实例及其声明/尺寸保持，计算载体只在监督栈上存在。
- 原有参数说明、Keil工程/调试配置及.vscode修改保留，不重置或提交这些现场内容。
  新候选APP1.5.8，Loader仍2.1.0。不开启硬件、下载、复位或运动。

## 有意改变的调度行为

新SPEED/POSITION目标在本拍外环前应用，不再固定等下一外环拍。
STOP、故障、运行守卫及新温度在正常计算前处理；计算期间的新故障在提交时再次撤销结果。
算法暂存和最终发布增加有界栈/短临界区工作；必须记录资源及尚未复测的真实中断时间。
不把代码先后顺序或离线桩结果当作真实PWM关断证据。

## 验收、回退与证据

复用真实owner、规则及外环夹具，补最小监督集成场景：新目标同拍计算、
STOP/故障不执行外环、启动无ready不启相、计算中故障/失效代际不发布、
CURRENT/VQ不依赖外环ready、自检/epoch每拍一次及重复提交无效。
比较控制/保护源码及Watch声明；运行PR、主工程编译并冻结配套debug产物，记录Flash/RAM/栈。
不扩展全量硬件矩阵；本批不自动执行台架。

回退仅撤销本批独立提交，参照上述基线；不reset/clean或改其他工作树引用。
离线、构建和剩余门禁记入本计划，生成证据放`outputs/supervisor-staged-v1/`。

## 本批结果

实施提交`3fe9653e`；消除初始化临时栈对象后的最终源码
`94b3182456564b6933658bc9d1cceaf1c20e8a23`，APP1.5.8 / Loader2.1.0。
代码、离线验证和debug构建完成；正式release、硬件与时序验收未完成。

- PR检查通过：`outputs/runs/20261021T114838684752Z-94b31824/summary.json`。
  包括格式、lint、工程布局、离线测试、架构、接口、文档与仓库规则检查。
- 定向夹具链接真实Axis监督、状态owner及规则，边界桩注入计算中STOP/快速故障、
  本拍温度故障和局部计算失败；同拍目标、SPEED/POSITION ready、CURRENT/VQ、
  INIT稳定窗口、监督epoch、重复提交及失效代际均通过。
  相对固定基线的既有owner/功率/事件轨迹与规则输出一致；
  证据：`outputs/supervisor-staged-v1/zero-copy-final/summary.json`。
  这是原生夹具结果，不是真实功率关断或电机运动证明。
- 主工程与配套APP/Loader均0错误、0警告。冻结包目录
  `outputs/delivery/supervisor-staged-94b3182-20261021/`，版本、源SHA及产物摘要见summary.json。
  保留的用户参数注释和Keil设置如实登记为dirty输入，不冒充clean正式release。
- 正常APP Flash79504B、RAM18504B；配套APP Flash81920B、RAM18552B。
  相对1.5.7构建，Flash分别增加544B及552B，链接RAM不变。
  Loader仍Flash14520B、RAM4344B，BIN与1.5.7配套包中的Loader逐字节一致。
- 14个Watch对象声明、链接尺寸及归属对象文件不变；217份motor/parameters/platform/
  shared codec源码与基线一致，现场motor_profile_defaults.h只有原有注释修改。
  见`outputs/supervisor-staged-v1/review.json`。
- ARMCC静态调用图：Axis监督自身栈120B，最大调用深度480B；
  TIM7链最大调用深度520B。它们不是实测峰值，不包含硬件异常入栈、
  抢占嵌套及未解析路径的完整证明；不得据此声称20kHz时序或MCU占用率通过。
  初始化使用直接memset，避免编译器为零初始化额外保留一份临时结构体。

剩余：新候选真实启停/标定及升级、故障关断、中断耗时/最长路径、
实际嵌套栈及MCU占用率，合并到当前候选剩余验收计划；本批不启动台架。
