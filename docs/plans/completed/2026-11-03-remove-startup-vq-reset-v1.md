# 删除启动重复Vq清零 v1

按用户要求仅删除MotorAxis_Init中基于configuration_source的vq_target_v清零分支。
保留BspFlash_Read、Param_Download及Param_Return_Default全部原文与执行顺序。
正常上电的s_motor_state由静态存储初始化为零；不扩大为重复初始化接口。
基线：`outputs/motor-axis-remove-vq-reset-20261103/motor_axis.before.c`。
验收：精确差异、完整PR档及双板构建；回退仅恢复删除的四行。不烧录或操作硬件。

## 结果

已删除指定四行，参数读取、装载及默认回退保持原文。
首轮启动校正测试以被删除if作为源码提取终点而失败；已改用MotorCurrentOffsets_Init定位，
两板启动校正专项通过。最终与通信初始化可读性整理共同通过完整PR：
`outputs/runs/app-init-stages-20261103-final/summary.json`。
普通双板APP均0错误0警告，未烧录。
