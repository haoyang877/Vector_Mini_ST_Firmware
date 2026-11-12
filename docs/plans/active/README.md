# 当前未结计划


核对日期：2026-10-26。此目录包含当前事项及尚未归档的阶段记录；旧阶段状态不代表当前代码。
版本与工作树仍可能变化，执行前重新冻结源码；本索引不新增硬件操作授权。

| 计划 | 当前状态 |
| --- | --- |
| [AT32 硬件功能检测与阈值](2026-11-12-at32-hardware-check.md) | 六类通道的设计及阈值评审稿，先做离线验证 |
| [Hall 单拍突跳根因排查](2026-11-06-hall-spike-rootcause-v1.md) | 无运行滤波；分层片内捕获及实机单变量验证 |
| [双板 Loader 开发与评审 v1](2026-11-02-dual-board-loader-v1.md) | 离线实现和PR/release通过；11 月 9 日 PITCH 初装和两轮 CAN 升级通过；历史预检及其余硬件场景见计划 |
| [AT32 位置和速度外环 4 kHz](2026-11-05-outer-4khz-v1.md) | ADC 每五拍派发；板上时序和参数由轨迹会话调试 |
| [单轴位置 S 曲线 OTG](2026-11-05-position-otg-v1.md) | 重构分支实现与离线连续性验证，板上时序待测 |
| [运行中 q 轴与低延迟测速验证](2026-11-05-speed-120hz-validation-v1.md) | 运行中 q 轴幅相和 120 Hz 目标按板上证据验收，低速波动独立核对 |
| [标定参考角精度修正](2026-11-05-calibration-angle-precision-v1.md) | 精度和四组回归通过；已烧录并重标定，快环最坏执行时间待测 |
| [全速度域阶梯与闭环带宽上限](2026-11-05-speed-domain-bandwidth-v1.md) | 阶梯与频响矩阵完成，中高速约70 Hz；低速精度和标定参考质量待闭环 |
| [转速环整定与带宽实测](2026-11-04-speed-tuning-bandwidth-v1.md) | 已测高增益共振风险，0.015/0.3细扫约72 Hz，低速与38 Hz纹波仍待闭环 |
| [含延迟的电流PI重整定](2026-11-02-delay-aware-current-loop-v1.md) | D轴实测1193.9Hz达标；0.75A限速停止，剩余稳态与阶跃未测 |
| [无感启动跑通v1](2026-10-31-sensorless-bringup-v1.md) | 无感标定连续三次完成，配套电零位完成；无感算法未改，新标定在RAM |
| [统一电机测试执行v1](2026-10-31-unified-motor-validation-v1.md) | 历史实测记录；2026-11-03 对应代码与工程已退役 |
| [统一电机测试代码接入v1](2026-10-31-unified-motor-code-v1.md) | 历史阶段；2026-11-03 已删除对应代码与工程 |
| [电机统一台架测试用例v1.1](2026-10-31-unified-motor-test-cases-v1.md) | 已确认新板5708/8:1及现有设备；按板载采样/空载收敛，缺设备项目标记阻塞；待评审，未执行 |
| [编码器标定最小实现v3](2026-10-30-calibration-control-boundary-v3.md) | 合并core，单一实现文件；StepLut两参数，电零位共享绑定；验证见记录 |
| [编码器标定最小化v2](2026-10-30-calibration-minimal-v2.md) | 私有会话与阶段存储复用，任务代码减少464字节、BSS减少48字节；双板等价及故障测试通过，整仓仍有既有失败 |
| [固件工程残留清理](2026-10-30-firmware-cleanup-v1.md) | 清理无用接口、工程路径和过时导航；保留当前行为，离线证据单独记录 |
| [标定准入去重v5](2026-10-30-lifecycle-admission-v5.md) | 695→690行；双板前后测试通过，整仓仍有既有失败 |
| [生命周期原地去重v4](2026-10-30-lifecycle-dedup-v4.md) | 711→695行；前后双板行为及对象编译通过，整仓仍有既有失败 |
| [APP核心精简v2](2026-10-30-app-core-v2.md) | 15→13文件，2249→1981行；核心专项及编译通过，整仓仍有既有失败 |
| [三相与母线电流统一更新](2026-10-30-current-update-merged-v3.md) | 已合并；双板相关编译与采样测试通过，整仓仍有既有失败 |
| [APP整体精简](2026-10-30-app-minimal-v1.md) | 20→15文件，2417→2249行；双板及行为对照通过，整仓仍有既有失败 |
| [APP接口最小化](2026-10-30-interfaces-minimal-v1.md) | 8→3文件，511→169行；双板及专项通过，整仓仍有既有失败 |
| [三类标定通信入口](2026-10-30-calibration-communication-v1.md) | 160/kind 1、2、6已接入；双板通信到完成联测及相关ARM编译通过，整仓存在既有失败 |
| [电流采样接口一致性](2026-10-30-current-sampling-consistency-v2.md) | 电流独立入口已由统一更新v3合并 |
| [生命周期必要核心v3](2026-10-30-lifecycle-essential-v3.md) | 保留标定，1109→718行；双板及专项通过，整仓仍有既有失败 |
| [三相电流直接赋值](2026-10-30-current-sampling-direct-v1.md) | 历史阶段，已由采样接口一致性v2取代 |
| [标定与完成事件接入](2026-10-30-calibration-events-v1.md) | 三种标定完成后回空闲，自检仍占位；两板专项与相关ARM编译通过，整仓仍有既有失败 |
| [生命周期仅保留核心](2026-10-30-lifecycle-core-v2.md) | 仅保留核心，1871→901行；两板编译及核心专项通过，整仓仍有既有失败 |
| [五态runtime最小化](2026-10-30-runtime-minimal-v1.md) | 16→10文件，净减678行；两板编译/相关专项通过，整仓仍有既有失败 |
| [五状态重写，自检占位](2026-10-30-state-five-rewrite-v1.md) | 42行五态核心；自检不实现；双板编译及相关测试通过，整仓PR仍有既有失败 |
| [INIT统一电流偏置校正](2026-10-30-current-offset-init-v2.md) | 独立校正三相和母线电流，仅INIT执行；两板专项/相关编译通过，整仓PR仍有既有失败 |
| [母线硬件测量与滤波系数](2026-10-30-motor-measurement-bus-v1.md) | 硬件电流接入、可调低通及两板相关编译通过，整仓PR仍有既有失败 |
| [编码器反馈输入精简](2026-10-30-encoder-update-inputs-v1.md) | 快慢估计只传状态，初始化绑定所需配置；两板专项及65拍对照通过，整仓PR仍有既有失败 |
| [电机状态机最小核心拆分](2026-10-30-state-core-minimal-v1.md) | 历史阶段，已由五状态重写取代 |
| [MotorAxis仅保留三个核心入口](2026-10-30-motor-axis-three-api-v3.md) | 三个核心入口；两板编译和定向测试通过，诊断调度删除，整仓发布门禁未通过 |
| [上电电流零偏校正精简](2026-10-30-current-offset-2048-v1.md) | 2048拍已实现；两板校零及离线编译通过，整仓PR仍有既有失败 |
| [MotorAxis运行职责收拢](2026-10-30-motor-axis-ownership-v2.md) | 源码和定向验证完成；两板离线编译、参数等价、校零/快照通过，整仓PR仍有既有失败 |
| [磁链观测器输入精简](2026-10-30-flux-observer-inputs-v1.md) | 仅接收电机模型和αβ物理量；两板专项及逐拍对照通过，整仓仍有既有检查失败 |
| [移除应用热模型调用](2026-10-30-remove-thermal-calls-v1.md) | 初始化、快慢环、准入及配置提交不再调用热模型；未编译或测试 |
| [删除一次性通电窗口](2026-10-30-remove-power-window-v1.md) | 删除窗口、功率契约、调试变量、工程引用及专用测试；未编译或测试 |
| [MotorAxis快环精简](2026-10-30-motor-axis-fasttick-minimal-v1.md) | 删除重复采样和goto，统一结果发布；未编译或测试 |
| [ADC应用入口放入main.c](2026-10-30-adc-main-entry-v1.md) | 两板main.c显式绑定ADC快环，硬件IRQ仍归MCU；未编译或测试 |
| [BSP公共接口最简化](2026-10-30-bsp-public-minimal-v1.md) | 公开头10→8，闲置时基/复位/诊断删除，内部及台架声明收回；未编译或测试 |
| [ADC/PWM全部归MCU](2026-10-30-adc-pwm-mcu-local-v1.md) | 零偏及换算合入两板MCU，删除独立公共实现及辅助接口；未编译或测试 |
| [CAN长度换算就地实现](2026-10-30-can-local-dlc-v1.md) | 已删除独立换算文件和函数；转换直接放入两板收发实现，未编译或测试 |
| [BSP第一轮精简](2026-10-30-bsp-simplify-round1-v1.md) | 清理重复头、反向配置依赖及闲置初始化，统一启动归属；未测试或编译 |
| [删除LED和UART](2026-10-30-remove-led-uart-v1.md) | 删除两功能、调用及资源配置；保留四外设、Flash及必要系统支撑；未编译或测试 |
| [全BSP硬件边界核对](2026-10-30-bsp-hardware-boundary-audit-v1.md) | 共用层只保留逻辑，LED硬件初始化归MCU；接口和工程引用静态核对，未测试或编译 |
| [PWM硬件与逻辑分离v2](2026-10-30-pwm-hardware-boundary-v2.md) | 已迁移供审阅；BSP只换算占空比，MCU处理全部PWM硬件；未测试或编译 |
| [ADC硬件与逻辑分离v2](2026-10-30-adc-hardware-boundary-v2.md) | 已迁移供审阅；MCU处理ADC硬件，BSP只保留逻辑；未测试或编译 |
| [双板ADC/PWM统一最简接口](2026-10-30-dual-adc-pwm-minimal-v1.md) | 两板共用值接口和运行源，工程宏选板，上层无ADC/PWM板型分支；未测试或编译 |
| [Encoder硬件归MCU v3](2026-10-30-encoder-hardware-boundary-v3.md) | MCU直接实现统一接口及全部硬件操作，删除公共转发文件；未测试或编译 |
| [两板编码器统一最简接口v2](2026-10-30-encoder-unified-minimal-v2.md) | 统一接口阶段记录；当前硬件边界见v3 |
| [CAN硬件与逻辑分离v2](2026-10-30-can-hardware-boundary-v2.md) | BSP仅长度换算；MCU直接实现统一收发、启停和IRQ；未测试或编译 |
| [两板CAN统一最简接口](2026-10-30-can-unified-interface-v1.md) | 统一接口阶段记录；当前分工见CAN硬件与逻辑分离v2 |
| [AT32 CAN最简实现](2026-10-30-at32-can-minimal-v1.md) | 先行及codec合并阶段记录，当前实现见两板统一计划 |
| [AT32编码器最简实现](2026-10-30-at32-encoder-minimal-v1.md) | AT32先行阶段记录；当前接口与硬件边界见v3 |
| [AT32 PWM最简实现](2026-10-30-at32-pwm-minimal-v1.md) | 实现待审阅；MCU仅初始化，BSP直接写寄存器，删除保护/状态/结果码；按用户要求未测试或编译 |
| [AT32 ADC最简实现](2026-10-30-at32-adc-minimal-v1.md) | 实现待审阅；MCU仅初始化/寄存器与DMA暴露，删除ADC保护；按用户要求未运行测试或编译 |
| [采样快环删除软件保护](2026-10-30-remove-sensing-protection-v1.md) | 实施中：采样只搬运物理值并保留母线低通；离线验证，未烧录/未台架 |
| [FOC 测量边界方案](2026-10-29-foc-measurement-boundary-v1.md) | 测量对象分离及两板离线验证完成；目标时序未测 |
| [motor 归属与命名复核整改](2026-10-29-motor-followup-cleanup-v1.md) | 实施中；保持控制与保护行为，目标时序待验证 |
| [motor 分层与命名整理](2026-10-29-motor-layering-v1.md) | 结构与命名整理及离线验证完成；目标时序和实机行为待验证 |
| [BSP CAN状态精简](2026-10-29-bsp-can-simplification-v1.md) | 已完成：终态处理集中、冗余状态去除；新板普通APP编译与两模式离线对照通过，Code减少24字节，未烧录 |
| [新板sin/cos 24.5周期](2026-10-29-sincos-24p5-v1.md) | PA5/6采样24.5、NTC640.5，保留4次硬件平均；普通APP单工程与29个专项场景通过，未烧录 |
| [AT32三相ADC第二步优化](2026-10-29-adc-phase-12p5-v1.md) | 三相12.5/母线电压24.5；Off接入NTC/sin/cos四次硬件平均，冻结副本单工程与专项回归通过；未烧录，输出轴角度未接入 |
| [AT32 ADC 60MHz第一步优化](2026-10-29-adc-60mhz-v1.md) | 60MHz/24.5周期已实施，保留12位/4次过采样；九工程与PR全通过，旧板HEX不变，理论9.87µs待实测，未烧录 |
| [BSP整理后AT32标定与控制链路验收](2026-10-27-bsp-motor-control-hil-v1.md) | 台架范围已确认，准备当前候选和分阶段实测 |
| [BSP逻辑与文件架构收敛](2026-10-26-bsp-architecture-consolidation-v1.md) | 离线整理、九工程、PR及干净快照release通过；统一AT32实测后置，本阶段未操作硬件 |
| [BSP外设初始化与功能接口归位](2026-10-26-bsp-peripheral-ownership-v1.md) | 后续已实施；原评审阶段记录保留，当前状态见上项 |
| [双板ports目录对齐](2026-10-24-ports-directory-alignment-v1.md) | 五类目录及接口实现表完成；九目标重建/HEX/资源一致，109单元/14集成/36原生通过，正式release受未提交门禁阻止；未烧录 |
| [双板BSP接口、能力与命名统一](2026-10-24-bsp-unification-strategy-v1.md) | 10/26精简审查完成：ports归BSP、common归共享实现，测量/调试归位；仅文档与quick通过，目标目录尚未迁移 |
| [双板真实main与平台装配分离](2026-10-24-board-application-entry-v1.md) | AT32台架遗留清理300→264行；RTT配置合并，三目标HEX不变，冻结副本release通过；测量/遥测职责迁移未实施，未烧录 |
| [统一编译后新板关相实测](2026-10-24-armcc-off-hil-v1.md) | 统一main的Off完整APP已烧录并关相验收通过；ADC/MT6835/CAN及2kHz连续采集通过，快环40.42us，参数保留；转动及控制未复测 |
| [双板Keil工程对齐](2026-10-24-keil-project-alignment-v1.md) | 工作区YG-ESC及双板APP名称已统一；四工程重建/镜像字节一致，冻结副本release通过；Off关相验收见上项，限时版控制尚未复测 |
| [共用APP新板启动与速度实测](2026-10-24-shared-app-speed-hil-v1.md) | 关相启动及2kHz零缺口采集通过；±1/±2点达标，±3仍未达；release通过，参数未保存、已关相 |
| [双板release检查修复](2026-10-24-release-check-repair-v1.md) | 工作树PR与干净冻结副本release全通过；三个目标HEX不变，未接触硬件 |
| [双板共用配置收敛](2026-10-24-dual-board-config-v1.md) | 配置收敛、三个目标编译和定向离线验证完成；后续冻结副本release已通过 |
| [新板完整APP标定与闭环](2026-10-23-at32f456-app-commissioning-v1.md) | 历史实测记录；2026-11-03 已删除 Commissioning 工程 |
| [新板MCU温度可选策略](2026-10-23-mcu-temperature-optional-v1.md) | 关相调试目标临时可选；默认限流修正为3A，实机待机/故障0及120次CAN读取通过 |
| [PA4母线电流APP接入](2026-10-23-pa4-bus-current-app-v1.md) | 按硬件能力选择反馈；关相偏置及120次CAN读取通过，带载绝对精度待验证 |
| [STM32G4 APP入口统一](2026-10-23-stm32-application-entry-v1.md) | 历史目录迁移完成；当前各板application/main.c为真实入口，平台实现归platform_app.c；本次入口分离仅离线验证 |
| [电机配置分离与1kHz默认值](2026-10-23-motor-profiles-1khz-v1.md) | HT有/无阻尼环、独立YQ型号配置；仅编译和离线验证 |
| [Rust GUI实机验收](2026-10-21-host-gui-bench-v1.md) | 当前host_app/rust候选；原计划未记录GUI实机终态，不沿用CLI结果 |
| [当前候选剩余验收](2026-10-21-current-candidate-acceptance-v1.md) | 合并旧计划中仍有效的当前候选/复位/时序证据缺口，未测保持NOT_RUN |
| [AT32F456 最小改动适配](2026-10-22-at32f456-port-v1.md) | 接口契约冻结，已分离旧板配置；先平台与MT6835，输出轴最后；驱动未实施 |

阶段完成和未测边界分别记录，详见[当前限制](../tech-debt-tracker.md)。
