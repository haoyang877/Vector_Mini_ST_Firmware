# 编码器标定重建：Mode 13 / Mode 15 v1.0

日期：2026-09-20。状态：**已实施，离线验证通过；实机验收待台架**。
背景：2026-09-19 按用户裁决整体删除 `foc_calibration.{c,h}` 五任务（"删除 → 在新模块结构上重建"）；
用户要求优先恢复 **Mode 13（观测器 LUT）** 与 **Mode 15（电角度零位）**。

## 1. 范围（严格两条链路）

- 恢复：`Task_Calib_EncoderObserver`（Mode 13）、`Task_Calib_EleAngelOffset`（Mode 15）及其最小依赖。
- **不恢复**：`Task_Calib_R_L_Flux`（Mode 4）、`Task_Calib_EncoderOffset`（Mode 5）、
  `Task_Calib_CurrentOffset`（Mode 11）——准入仍拒绝（`foc_errhandle.c`），dispatch 无分支，
  实现函数不存在。
- `CalibStep` 枚举保留全部历史编号（供 Keil Watch 按数值对照；未重建步骤不会出现）。

## 2. 实现

| 项 | 内容 |
| --- | --- |
| 新模块 | `firmware/motor/identification/foc_encoder_calibration.{c,h}`：两个任务 + LUT 采样/构建辅助（Q15 差、候选 LUT 应用/提交、稳定性/跟踪判定、制动判定） |
| 无感配置 | `SensorlessStartup_EncoderCalibConfig` 恢复到 `foc_sensorless_run.{c,h}`（Mode 13 无感启动参数集） |
| 常量契约 | 标定常量簇由 `hw_conf.h`（bsp）上移 `platform/api/control_config.h`（含阻尼环条件分支）；全部被 13/15 消费 |
| 接线 | `foc_mode_dispatch.c` 两个 case；`foc_run_state.c` 会话（exclusion / OperationFor / 完成条件 / 入口启相）；`foc_errhandle.c` 解除 13/15 拒绝 |
| 迁移 | 任务不再直接停相/写模式：错误经 `MOTOR_WORK_FAULT`、完成经 `SWITCH_MODE(Save_Param)+power_off`；本地中止只清理控制状态。Mode 15 旧"高边短接保持"改为状态机停相（与其余标定同构，行为偏差已注记） |
| Keil | `foc_encoder_calibration.c` 登记普通工程（uvoptx 用户调试设置不在提交内） |

## 3. 验证

- `test_run_state.py`：新增 Mode 13/15 会话用例（入口启相 + 完成关相切往保存链）；
  差分 46,464 组 tick 保持全一致。
- quick 档 PASS：`outputs/runs/20260919T162813621215Z-71e0687f/summary.json`。
- Keil normal：0 Error / 0 Warning，Code=86920 / RO=4964 / RW=392 / ZI=31344。

## 4. 实机验收（待台架）

顺序：上电 → Mode 13 LUT 标定（约 5 圈采样 + 1 圈验证）→ Mode 15 电角度零位 →
转速闭环（约 10 rad/s，经 CAN）。
前提：CAN 物理链路恢复（当前设备因断链处于 bus-off；固件已含自恢复）。
台架工装可沿用 `outputs/hil_encoder_20260919/` 的 SWD/CAN 脚本（pylink 需临时安装，或改用 JLink Commander）。

## 5. 回滚

单提交 revert；新增符号为独立模块，回滚无残留。
