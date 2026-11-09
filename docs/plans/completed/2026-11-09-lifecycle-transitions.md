# 按主状态迁移整理电机生命周期

基线 `2d1616b0`。沿启动、停机、标定、故障恢复追踪状态与 PWM。独立的 `motor_state_machine` 按值计算五态迁移。`motor_lifecycle` 判定提交事件的时机，持有本轴状态和功率许可。控制目标、外环结果与遥测在 `motor_lifecycle_exchange.c` 交接。

- 保持五态与线上编号、请求错误优先级、转换前的功率确认、故障锁存、会话撤销和标定 Flash 事务。ADC 可抢占；同轴多个执行环境沿用已验证的交接顺序。SELF_TEST 保留占位。
- 涉及两板 Keil 工程与 LoaderApp 的共享输入、模块边界和相关原生测试。核对全状态/事件组合及真实启动、STOP、标定、抢占与清错场景，跑 PR 和三个生产镜像的 Keil 全量构建。
- 审查状态主线的文件、函数和成员命名。启动请求、启相许可、实际输出、停止和故障动作应能从名字区分。保持公共 C 函数签名及 CAN 编号。
- 运行目标直接按六个控制模式分支，逐个核对允许字段、错误优先级、热限流、开环频率，以及外环和 ADC 各自的生效时机。
- 核对 ROM、静态 RAM、未验证的实板时序。回退本提交即可恢复原实现，无协议或参数迁移。

## 实现与核对

- `MotorStateMachine_NextState` 是独立模块的唯一入口，无设备依赖和可变状态。`MotorLifecycle` 在确认硬件或保存结果后提交事件，保留旧的纯查询入口以兼容既有 C 调用方。运行调度和执行算法仍分别属于轴和 motor 控制层。[五态路径与代码入口](../../architecture/motor_state_machine_v1.md)
- 启动请求标志使用 `start_pending`，ADC 复核的启相会话使用 `phase_enable_epoch`。前台目标处理直接列出六个控制枚举，各自校验允许字段及限值。电压、电流目标沿用先填槽、最后发布序号的顺序。
- 独立编译规则和测试，验证全部 30 种状态事件组合与非法枚举。双板运行测试覆盖六种模式各七个字段、热限流、STOP 与故障优先、标定保存及过期结果。将同一组集成刺激用于本分支与基线，每板各 560 条快慢拍与 PWM 记录逐字节相同，记录位于 `outputs/lifecycle-states/trace-comparison.json`。
- G431 普通 APP、AT32F456 普通 APP、G431 LoaderApp 全量构建均为零错误零警告。相对基线，ROM 分别增加 584、912、592 字节，三者静态数据 RAM 的变化均为零。MAP 确认 AT32F456 的迁移函数在 SRAM，三个镜像的双精度运行库符号和 F64 指令均为零。镜像哈希见 `outputs/lifecycle-states/images.json`。
- 完整 PR 和 release 离线检查记录在 `outputs/runs/20261109-state-machine-pr-final` 和 `outputs/runs/20261109-state-machine-release`。基线的 RTT 固定哈希和 104 项历史文档诊断单独核对。未烧录或测量实板 ADC 时序。
