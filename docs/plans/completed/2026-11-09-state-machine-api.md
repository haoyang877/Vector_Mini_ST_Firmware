# 合并电机状态机源码与接口

基线 `d82730b1`。状态机、目标写入、外环提交与反馈统一到 `motor_state_machine.c/.h`，删除旧实现单元、头文件与兼容转发。库外调用者需要迁移 C 名称并重新编译。保持现有状态与控制模式编号、结构布局、协议和中断时序。

同步 APP 调用点、两板 Keil 工程、Watch 表达式和宿主测试。删除单用的启动、母线窗口和清错转发，省去相同执行上下文中的重复检查和无事件的状态分派。保留在 ADC 实际启相、停机确认及标定写入前的实时复核。对照旧版状态/PWM 轨迹及目标值，运行 PR 和三个生产镜像构建。回退本提交可恢复旧 C 接口。

## 核对结果

- 五态、目标校验和写入、外环输入和结果提交、反馈快照共用 [motor_state_machine.c](../../../firmware/app/motor_state_machine.c) 和 [motor_state_machine.h](../../../firmware/app/motor_state_machine.h)。两个原编译单元合计 44 个函数，现有 39 个。状态和会话号改为只读字段，实际功率、故障恢复和 Flash 仍通过各自的接口。
- 空闲无请求时不扫描整套参数。外环只复核一次运行输入，ADC 在实际启相时再读实时条件。已锁存的故障无需再次执行原子上报。标定前台无候选时只读交接标志，有候选时在控制临界区内复核，写 Flash 前再次核对会话。
- 与基线对照，两板各自的 633 条状态与 PWM 记录及 8,448 组目标输入返回码和控制值逐项一致。三目标 Keil 构建均为零错误、零警告，互斥、抢占、六种控制模式、标定保存和双轴嵌套测试通过。逐板 ROM、静态 RAM 差异及镜像哈希见 `outputs/state-machine-api/images.json`，轨迹对照见 `outputs/state-machine-api/trace-comparison.json`。
- 完整 PR 检查使用 `outputs/runs/20261109-state-api-final/summary.json`。基线有两项既有失败，涉及 RTT 文件哈希和历史文档诊断。仅进行离线验证，未测实板 ADC 时序。
