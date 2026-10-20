# 快环执行预算恢复 v1

## 目标与基线

在 `codex/system-refactor` 修复真实配套 APP 启动交接时的快环漏拍，不修改控制算法。
基线实机源码为 `31941dbc3740b80c430c38eac747cfe6771e4e82`，APP 1.5.0 / Loader 2.1.0。
第三轮标定记录位于 `outputs/bench/core-machine-31941dbc-20261021/lut-calibration/attempt-03/`：
锁速达到 4000 拍后约 30 ms 报 SENSORLESS 13，尚未进入 LUT 采样；STOP、关相及资源释放已确认。
同一采样方法下，运行快拍约 17.5 kHz，空闲约 20.1 kHz，监督拍约 2 kHz。
这与固定 50 us 控制时基不一致；具体耗时仍由新候选实测判定。

旧 OVP 提交 `1c3daf32b285c4e7187ea992e30513d9a063c7c6` 的相同故障已有超预算证据。
本次只复用已明确遗漏的执行配置，不合并整个旧分支。

## 变更边界

- 真正快环已由 `foc_task.c` 移至 `motor_axis.c`，其 O3/时间优先配置未随迁移保留。
- 恢复 `motor_axis`、`foc_sensorless`、`foc_sensorless_run`、`foc_speed`、
  `motor_thermal`、`motor_thermal_control`、`encoder_calibration_task` 的 O3/时间优先。
  其余文件及全局编译级别不变；配套工程从主工程生成。
- 在 CubeMX `USER CODE` 区域恢复 Flash 指令预取，在外设与控制器启动前生效。
- APP 修订号递增为 1.5.1；Loader 2.1.0、布局 6、schema 14 不变。
- 保留数学、PI、锁定滤波、保护阈值、参数和 Watch 入口。暂不改调度、CCM 分配或中断清旗顺序。

## 验收与回退

执行 PR 离线检查及真实配套工程构建，记录容量、产物和哈希。T 唯一持有硬件，
保留当前参数/身份，使用新冻结包依次核对启动通信、运行快拍、LUT/零位及速度闭环。
原有 300 rpm、母线 3 A、相电流 6 A 上限及更严格的软件限制不变。
未实际通过的项目保持未通过，不以 OVP 或离线结果代替。

回退仅撤销本计划对应的执行配置、预取及修订号，重新冻结原配套程序；不覆盖标定参数和身份。
本文件建立时尚未运行修复候选，结果待追加。
