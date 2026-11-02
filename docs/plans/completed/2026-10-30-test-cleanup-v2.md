# 当前接口测试收敛 v2

## 目标

处理前轮剩余16组原生失败，清理退役接口/行为用例，保留当前功能的有效验证，
使完整离线PR检查通过。不以删除断言、伪造结果或放宽生产安全条件取得通过。

## 基线与边界

基线结果为outputs/runs/firmware-cleanup-v1-final-20261030/summary.json。
本轮开始的源码和哈希位于outputs/test-cleanup-v2-20261030/before.zip及before-files.json。
优先修改测试接线、假硬件和过时预期；生产缺陷单独记录修复和针对性证据。
保留协议编号、schema14/AXS布局、控制算法、现有BSP语义及用户正在进行的标定重构。
全程离线，不烧录、不连接探针，不提交或回退用户已有修改。

## 执行与验收

1. 将失败逐项归类为接口迁移、历史功能退役或真实缺陷，记录覆盖替代关系。
2. 公共能力、PWM/ADC/编码器/CAN通过真实生产实现与可控假硬件验证；
   退役状态机/超时/热调度不伪装成仍存在的功能。
3. 更新外环、参数、Flash和协议夹具，保持当前行为与独立边界/黄金数据检查。
4. 所有原生入口、Python单元/集成、格式、lint、接口、架构、文档及hygiene全部通过。
5. 如果修改生产实现或工程输入，执行适当构建，并分别报告离线与硬件证据。

回退只恢复本轮实际变更文件；完成后保存差异、逐组结果和剩余限制。

## 逐组迁移与覆盖

| 原失败入口 | 当前覆盖与清理 |
| --- | --- |
| test_bsp_features | 两板×两电机选择、固定能力、缺失/未知配置拒绝；删除逐功能开关矩阵 |
| test_bsp_function_contract | 两板完整四次累加域、三相独立偏置、母线单次域、单位与NaN；删除状态/时间戳夹具 |
| test_at32_foundation | 时钟失败、初始化、两级临界区、PWM、开闭环RL模型与RTT；迁移闭环编码器/PWM接线 |
| test_at32_io | 完整ADC/CAN初始化、采样rank/周期、过采样域、DMA半缓冲、NTC、停止再启动 |
| test_at32_encoder | 两板真实Begin/ReadAngle，遍历65536个响应，核对CS/MOSI、DMA、位宽 |
| test_at32_storage_temperature | RAW布局、扇区边界、擦写/回读失败、忙锁定；温度未校准时只有原码 |
| test_power_stage_start | 两板65536个门控初值、六路开关、无关位/计数器保留、全占空比域；删除提交窗口重试 |
| run_position_servo_tests | 真实位置核心、双板main、BSP/APP启动顺序、Flash及监督启动失败 |
| test_outer_loop_runtime | 真实外环、反馈、状态复位、配置限流和无效配置关断 |
| test_wheel_speed_limits | 254地址、APP/Loader一致性、schema14/AXS黄金值、CRC、不可变输入及参数回读 |
| test_encoder_sample_overlap | 实际快环在五态、完整/未完整零偏、速度/位置、故障组合中的事务先后关系 |
| test_mcu_temperature | 实际校准换算、供电补偿、滤波、原始值过温、无效结果超时与故障锁存 |
| test_yg_protocol_link | 实际端点、排队/重放、接收中断预算、单帧硬件完成、队列/在途超时、迟到完成 |
| test_comm_bus_off → test_can_hardware | 两板DLC/ID/远程帧、收发载荷、忙拒绝、非消费完成查询；自动恢复已退役 |
| test_motor_thermal | 真实纯核的热历史、降额/跳闸/冷却、拒绝原子性、当前禁用调度入口及YQ缺失配置拒绝 |
| test_speed_feedforward | 保留规划/jerk/PI数值检查；显式预算接口承担动态限流，不再注入不存在的热调度 |

公共夹具只替换寄存器或模块边界，函数体从生产文件读取；期望值使用独立公式和板卡数值。
没有恢复已删除的状态机、错误码、配置开关、软件完成队列或热调度。
删除30个仅服务退役接口的C/C++/头文件，CAN入口同步改名；移除记录在
`outputs/test-cleanup-v2-20261030/removed-fixtures.json`，均可由本轮快照恢复。

## 真实缺陷修复

- AXS关节修订1的13个冻结解释常量缺失：从已发布历史值恢复到记录解码器私有域，
  保持schema14布局、CRC和既有roll/pitch黄金值。当前应用整定不改变旧记录解释。
- AT32零校准供电表达式存在不可达代码警告：改为等价条件表达式，仍返回NaN。
- 两个AT32台架源文件分别补齐`math.h`与`bsp_adc.h`直接依赖，修复真实链接/编译错误。
- 接口扫描器跳过完整预处理逻辑行，避免把多行宏吞入后续函数声明；
  回归用例同时确保真正缺失契约仍会被报告，没有增加债务豁免。

## 验收证据

- 16组历史失败全部完成迁移；当前40组原生、126项Python单元、16项集成全部通过。
- 完整PR门禁通过：格式、lint、工程布局、测试、架构、接口、文档和hygiene无新增问题。
  收尾冻结复核入口为`outputs/runs/test-cleanup-v2-current-20261030/summary.json`。
- 10项Keil工程逐项重建均为0错误/0警告，包括四个APP、五个AT32台架与独立Loader；
  日志及资源行见`outputs/test-cleanup-v2-20261030/build-summary.json`。
- 已运行release profile，其全部离线检查通过；工作区原有及本轮改动未提交，
  `clean-release-source`未通过，因此该次release没有自动构建/发布manifest。
  独立构建日志不冒充正式发布包；提交并冻结源码后仍需重跑release。
- 快照、SHA256、删除清单、本轮补丁和汇总保存在`outputs/test-cleanup-v2-20261030/`。
  原有未提交改动保留，本轮未提交代码。
硬件验证未运行；离线检查不能证明板上时序、温升或实际电机表现。

核验期间并行标定重构修改了调用边界和四项APP工程；保留其实现，并在最终源码上
重新执行完整PR及四项APP构建。并行差异单列，不计入本轮清理删除量。
