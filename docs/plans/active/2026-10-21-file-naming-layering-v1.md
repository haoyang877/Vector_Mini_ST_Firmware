# APP文件命名与分层统一 v1

日期：2026-10-21。用户要求落实逐文件职责对照中的建议，让实际目录、Keil树和模块名称一致。

## 目标与不变项

基线：`3344c3ed`，固件源`05528f86437ddeec44433965baa211e06e2f139e`，APP1.5.5。
只调整文件、模块名及归属，合并已有任务入口；不改变控制公式、求值顺序、保护阈值、
主状态转换、参数schema14/12/13、通信字节、队列容量、采样时序和Flash失败策略。
Watch对象名称、类型和唯一所有者保持；调试必须使用重链接后的匹配AXF。
本批候选APP1.5.6，Loader仍2.1.0；版本标识变化不表示已发布或实机通过。
不增加生产兼容包装、第二套状态或新功能；离线夹具仅为固定历史基线保留路径解析。
本批不连接硬件、不下载、复位或运动。
现场未提交的参数说明、RAM/计时计划、编辑器设置以及其他会话的工作不纳入本批。

## 文件与目录调整

| 旧项 | 新项与职责 |
| --- | --- |
| app/runtime/motor_execution | motor_command：只验证命令；ValidateTarget改用MotorCommand前缀 |
| motor/foc/motor_control_execution | motor_control：已准入控制；ControlStep改为MotorControl_Step |
| motor/foc/foc_sensorless | flux_observer：磁链估计 |
| motor/foc/foc_sensorless_run | sensorless_startup：标定所需启动步骤 |
| motor/foc/foc_speed | speed_control：速度斜坡及速度PI |
| motor/protection/foc_errhandle | motor_fault：电机故障锁存与恢复事实 |
| app/composition/board_config | app_startup：启动、业务绑定及Loader交接组装 |
| app/composition/foc_task与bsp_task | app_tasks：合并文件，保留原IRQ入口函数及顺序 |
| services/parameters | firmware/parameters：配置和参数记录；foc_param改parameter_record，parameter_value_service改parameter_values |
| services/lifecycle/motor_stop_service | app/interfaces/motor_stop_request：请求与关断确认，不执行功率动作 |
| services/telemetry/motor_status | app/interfaces/motor_status_mailbox：ISR/前台只读快照 |
| services/lifecycle其余值契约头 | app/interfaces：APP对外命令、进展和readiness契约 |
| communication/protocol/yg_protocol_transfer | communication/can/yg_protocol_frame_queue：有界帧队列 |

APP公共接口是现有值契约及停机/快照边界的归属，独立于APP内部实现。
通信只允许引用该窄接口，禁止依赖runtime和主状态机内部；接口自身不读APP全局或HAL。
参数层依赖电机值类型和平台存储能力，不依赖APP实现。Keil改为20_Parameters，
停机与快照进入00_App；每份自有源仍只有一个登记位置，不显示.h。

主状态机动作/纯规则、标定任务/纯计算、热模型/保护历史以及平台能力/芯片实现继续分开。
yg_protocol_link保留单份链路会话所有者，本批不增加新的注册或回调框架。

## 验收与回退

逐项核对全部调用、Keil源/头搜索路径和逐文件编译选项，更新实际文件树对照及引用。
历史报告和冻结Git引用保留其历史名称；当前测试路径随迁移，固定基线比较兼容旧路径。
对归一化函数体、状态/故障/输出轨迹、Watch符号/大小和参数/通信边界作比较，
执行PR与适用release检查，并构建normal和配套APP，记录资源及产物哈希。
硬件功能/20kHz耗时本批不复测，不继承旧二进制实测结论。

回退只撤销本批独立差异，以基线为参照；不reset/clean，不覆盖其他工作或实验数据。
若发现算法、输出、阈值或运行顺序变化，停止命名迁移并定位，不能降低保护绕过。

## 实施与证据

生产迁移、工程接线和文件职责导航已落实。主工程107→106源文件，配套APP109→108；
减少来自任务入口合并，算法与状态没有删改或复制。文件树七组中的20_Services改为20_Parameters，
停机与快照源进入00_App，Keil源清单仍不显示.h。

独立静态复核见`outputs/naming-layering-v1/review.json`与`project-details-review.json`：
31对迁移、87个函数体及其余21份生产文件只含约定的include/guard/API更名；
合并的四个任务函数、静态计数和分频宏一致，14个Watch对象声明保持。
Motor/parameters没有到APP的反依赖，通信只到app/interfaces，接口自身只依赖motor值类型。

两工程的源清单、include路径、逐文件选项与Keil调试Target配置已核对。
app_tasks明确继承原foc_task的Optim=4/oTime=1，保留快入口策略；
原bsp_task没有文件级覆写，其指示器代码随合并采用同一编译单元优化，属于已记录的构建变化。
其余逐文件选项不变。UVOPT仅同步文件浏览清单，调试/下载/Watch Target配置相同。

真实基线所有者比较：`outputs/naming-layering-v1/native-owner/summary.json`，
当前与3344c3ed的owner/power/event轨迹及262144条规则一致。
历史文档保留原叙述，只修复指向当前移动文件的42处链接；路径迁移表同步当前目标。
后续PR、构建、资源及产物身份在最终冻结后补记；本候选实机与20kHz时序尚未复测。
