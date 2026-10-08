# 位置串级基础核心裁剪 v6

用户最新范围覆盖旧高级功能保留方案。基点
`0ec1aa900e13cbb4d1bb92891e12a34c9a3fd14f`，生产提交
`ccd90b71f44d810e962e12940d3c4e332d550975`，分支 `codex/position-minimal`。

## 保留的直接计算

目标直接作为位置参考，速度指令为 `Kp*(target-position)-Kd*filtered_speed`，
按原 `min(speed_limit, maximum_speed*1.50)` 限幅，再交原速度 PI。
速度增益仍乘 current_limit_a；保留 PI 条件积分、输出跟踪和原 10 Hz 可旁路滤波。
全复位清控制状态；换目标不清已有支撑积分。电流上限下调在分频等待拍也立即作用。
到位仅按既有误差窗口和旧 0.03 rad/s 门限单拍报告，不再启动 HOLD。
原 50/10 增益上界、0.003141593 rad 应用误差窗口上界及原速度/电流限制不提高。

## 删除与公开接口

删除轨迹/S 曲线/jerk/双路径、摩擦及加速度前馈、governor、静差增强、积分迁移、
多层 HOLD/捕获/确认、额外 HOLD 滤波、阶段 profiling 和额外诊断快照。
删除 PositionServoPhase、PositionCascadeTelemetry、GetTelemetry、ShouldDeferTelemetry。
保留 UpdateControl、UpdateTargetControl、Reset、GetConfiguration，函数签名未变；
ControlOutput 原五字段/布局保留，position_reference 现在等于目标。
Config 缩为 12 字段：周期、分频、目标、误差窗口、滤波、最大速度/速度上限、
位置 Kp/Kd、速度 Kp/Ki、电流上限。软件结构 ABI 改变，必须同步重编调用者。
本任务未修改 raw4304B/schema12/CRC/LUT 或持久参数/线路 ABI。

## 接线与检查边界

只提交位置三生产文件；ROOT/L 更新 motor_execution，ROOT 删除 RTT/轨迹工程输入。
不将新对象与旧调用者对象混合链接，不单独给整包容量或发布结论。
位置 runner 只保留基础闭环和板级初始化，保留其他脚本导入的 function_source。
退役轨迹/HOLD/摩擦回放和旧完整位置等价/缓存/全字段套件；基础 C 场景重新验证数学、
限幅、抗饱和、滤波、复位、实时限流、非法值拒绝及输出不发布。
Board_Init 夹具删除 BootEpoch 桩/事件11，保留真实初始化顺序、关相及通信失败关相。
独立树旧 Board_Init 仍含 Epoch，局部检查使用 ROOT 删除后的只读快照并记录哈希。
本批按用户范围仅做必要检查，ROOT 负责最终组合编译，不运行旧高级等价/整套矩阵。
没有硬件动作；新的直接运动不承诺原平滑/保持效果。回退为本轮两条提交的反向补丁。
