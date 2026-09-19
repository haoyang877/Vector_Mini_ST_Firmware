# FOC 硬件解耦：PWM 契约与 MCU 温度归位 v1.0

日期：2026-09-19。状态：阶段 1/2/3/4 完成——标定迁移随 FOC-Calibration 功能删除取消；
过渡 shim 已删除、`foc_algorithm.h` 接口契约债已还清；感测边界与功率级封装已落地。

范围：本记录覆盖两条具体交付：MCU 结温换算头归位 `platform/api`，以及
`foc_algorithm.c` 的 PWM 寄存器访问剥离与调用方迁移。更大范围的 `hw_conf`/BSP 归位
（H2.2/H3/H4b/H5）与整体进度以
[foc_run 归位与瘦身](2026-09-19-foc-run-placement.md) 的硬件解耦（H 阶段）为准，
本记录是其中 H4a/H2.2 的细化证据。

## 意图与验收

FOC 与快速环代码不再直接写定时器寄存器、不再依赖板级 `hw_conf.h` 宏；PWM 输出统一经
`platform/api/motor_hw.h` 契约由 `platform/stm32g4/ports/motor/` 实现。验收条件：

1. `motor/foc`、`motor/identification` 相关文件不出现 TIM1 寄存器写与 `PWM_TIM_PERIOD`；
2. 架构棘轮零新增违规，已还清条目（`foc_algorithm.c -> hw_conf.h`、
   `foc_phase_resistance.c -> hw_conf.h`）从基线删除；
3. 全部 PWM 调用方迁移后删除 `Set_*_Duty`/`PWM_TurnOn*` 过渡 shim，并还清
   `foc_algorithm.h` 接口契约债；
4. 每阶段以 `tests/run.py`、`verify --profile pr`、Keil 主工程 0 Error/0 Warning 验收。

## 阶段与进度

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 1 | `mcu_temperature.h` 从 motor/foc 移到 platform/api（motor 只允许 include common/platform_api）并重写为中文契约；`foc_algorithm.c` 剥离 TIM1/`hw_conf`，新增 `motor_hw_pwm_set_duty`/`motor_hw_pwm_set_phase_duty` 契约与 `ports/motor/motor_pwm_stm32g4.c`；双 Keil 工程登记；债务基线清理 | 完成（23ef80a） |
| 2 | 新增 `motor_hw_pwm_force_high_sides` 契约与端口实现；`foc_phase_resistance.c` 改用契约并把 `hw_conf.h` 换成 `control_config.h`（架构债 −1）并全文件清理；`foc_mode_dispatch.c` 改用三相契约 | 完成（本轮） |
| 3 | ~~`foc_calibration.c` 迁移~~（功能已整体删除，调用方一并消失）；删除过渡 shim（`Set_*_Duty`、`PWM_TurnOnHigh/LowSides`）及声明、补齐 `foc_algorithm.h` 中文契约并还清其接口债 | 完成（d78baf15） |
| 4 | `foc_sensing.c` 改接 `platform/api/motor_sensing.h`（原始计数出参 + 平台换算常量）：Vbus/三相电流/温度判定不再含寄存器、HAL 句柄或板级宏；`foc_errhandle.c` 去 `tim.h`/HAL，功率级启停走 `power_stage_hw_*`；三个原生夹具同步到新 seam | 完成（本轮） |

## 关键决策

- 过渡 shim（`Set_*_Duty`、`PWM_TurnOnHighSides/LowSides`）暂留 `foc_algorithm.c`：
  唯一剩余调用方 `foc_calibration.c` 带 style 豁免且正被并行 2b 修改；改名会使豁免哈希
  失效并要求 1479 行全量清理，与并行改动直接冲突。先迁移无豁免、无并行改动的
  `foc_phase_resistance.c` 与 `foc_mode_dispatch.c`。
- `motor_hw_pwm_force_high_sides()` 独立成契约：标定短接测试的意图是"三相上桥臂常开"，
  比 `set_duty(1,1,1)` 表达更准确；与逐相 `set_phase_duty` 构成最小完备集。
- `foc_phase_resistance.c` 只用到 `FOC_FREQ`（属 `control_config.h`），不需要板级
  `hw_conf.h`；替换 include 后该架构债直接还清，零新增违规。
- 逐相 `motor_hw_pwm_set_phase_duty` 只服务标定类逐相操作；控制路径一律三相更新，
  避免逐相写产生中间状态。
- 90°C 跳闸与 100 ms 失效窗口仍留在 `platform/api/mcu_temperature.h` 并注明是保守默认值；
  其版本化参数化归后续故障保护设计，拆分到 `motor/protection` 需连带清理
  `foc_sensing.c` 与夹具，另行立项。
- 2026-09-19：标定功能删除后 `motor_hw_pwm_set_phase_duty` 暂无调用方；作为逐相操作的
  平台契约保留，待标定重建时复用（不因"当下无调用者"删契约）。
- 2026-09-19：感测边界收敛到并行会话的 `platform/api/motor_sensing.h`（原始计数出参 +
  平台拥有的换算常量），不再另建 SI 出参契约；折入设计评审要点：温度换算与 JEOS/启动
  时序留在端口、偏置保持整数计数域、功率级启停六调用顺序不变（HAL 每次调用会切换 MOE）。

## 验证证据

（编写期间并行会话的在途改动使仓库级验证间歇失败；以下为逐步收敛的事实记录。）

- 定向检查（本轮改动文件）：`format --check`、`lint`、`check_architecture`、
  `check_interfaces`、`check_project_layout` 全部通过；两条已还清债务从基线删除。
- Keil 编译：本轮三个文件 `foc_phase_resistance.c`、`foc_mode_dispatch.c`、
  `motor_pwm_stm32g4.c` 均 0 警告 0 错误；整机链接在编写期间被并行 2b 的中间态
  `foc_run_state.c`（`CANMsg` 未定义）阻塞，非本轮改动；其收敛后需补双目标 0/0 记录。
- `verify --profile pr`：doctor/format/lint/project-layout/architecture/interfaces/docs/hygiene
  通过；`tests` 剩余失败项（`run_can_status_tests`、`test_bus_voltage_protection`）位于
  并行会话迁移中的心跳与 `foc_sensing` 夹具；本记录相关文件不被任何原生夹具编译，
  无因果。
- 日志：`outputs/runs/20260919T024410120567Z-23ef80ab/`、
  `outputs/runs/20260919T024513181612Z-23ef80ab/`、
  `outputs/runs/20260919T024620102523Z-23ef80ab/`、`outputs/build/logs/`。

阶段 3（2026-09-19，随标定功能删除解锁）：

- 删除 5 个过渡 shim（`Set_A/B/C_Duty`、`PWM_TurnOnHigh/LowSides`）及声明；全仓
  （含未跟踪文件）确认无调用方；`foc_algorithm.c` 独立 `zig cc -Wall -Wextra -Werror`
  编译 0 错误。
- `foc_algorithm.h` 补齐 11 条中文接口契约与结构说明；接口债条目清除
  （interfaces 检查 107 known / 0 new）；format/lint 均 0 new。
- 无感交接夹具同步删除已死的 shim 桩；夹具 PASS（双变体）。
- 过渡 shim 删除后 `foc_algorithm.c` 仅经 `motor_hw_pwm_set_duty` 触及硬件。

阶段 4（2026-09-19，感测与功率级边界）：

- `foc_sensing.c`：`Vbus_Update`/`Current_Cal`/`Temperature_Update` 改调
  `motor_hw_vbus_sample_raw()`/`motor_hw_current_sample_raw()`/`motor_hw_temperature_poll()`，
  运算顺序与历史实现逐位一致；移除 `adc.h`、`hw_conf.h`、`stm32g4xx_ll_adc.h`。
- `foc_errhandle.c`：`Stop/Start_PWM_Generate` 改为 `power_stage_hw_*` 薄包装，移除 `tim.h`
  与 HAL 调用；全文件补齐花括号与中文注释，style 豁免还清。
- 夹具：`test_current_precision` 改测新 seam（偏置标定功能已删除，直接采用标定值）；
  `test_mcu_temperature` 以 `motor_hw_temperature_poll` 假实现验证 JEOS/启动/超时语义；
  两者 PASS。`test_bus_voltage_protection` 的本轮失败经对照实验证实为并行会话在途的
  "已删除标定模式拒绝"块与其待迁移夹具冲突，非本记录改动。
- 门禁：`architecture`/`interfaces`/`lint`/`project-layout` 0 new；Keil 双目标
  0 Error / 0 Warning（本次 `foc_sensing.c` 已参与链接）。
- 日志：`outputs/tests/`、`outputs/build/logs/`。
