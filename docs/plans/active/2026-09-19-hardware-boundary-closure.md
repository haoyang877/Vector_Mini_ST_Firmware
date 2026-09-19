# 硬件边界收口：仅 platform/stm32g4 为硬件域 — AI 交接说明 v1.0

日期：2026-09-19。状态：**已完成**（T1–T5；release 档证据待干净工作树补）。面向接手的 AI/开发者。

定则（用户裁决，本文件的验收原则）：**只有 `firmware/platform/stm32g4/**` 包含与
实际硬件相关的内容；`app` / `motor` / `services` / `communication` / `common` 不得
包含硬件相关内容，只调用 `platform/api` 的接口。**

## 1. 现状审计（2026-09-19 实测）

官方检查器 `python tools/harness/check_architecture.py`：PASS（11 条已知债，0 新增）。
其中 **6 条为硬件类残留**；另 5 条为层级方向类（motor→services），与硬件无关。

| 区域 | 结论 | 证据 |
| --- | --- | --- |
| `motor/` | ✅ 达标 | 0 处硬件 include、0 处 HAL/寄存器；只依赖 `common` + `platform/api`（策略已锁死） |
| `common/` | ✅ 达标 | 仅标准库（stdint/stddef/stdlib/math） |
| `platform/api/` | ✅ 达标 | 仅标准库 + 自身契约头（12 个契约，见 §3） |
| `platform/stm32g4/` | ✅ 硬件域 | `bsp/`、`ports/`、`cubemx/` |
| `services/` | ⚠️ 剩 2 处 | ① `foc_param.h:4 → main.h`（实测无 main.h 符号，死引用）；② `foc_param.c:4 → common_inc.h`（实际只用 `CURRENT_SENSE_SHUNT_MILLIOHM/_2_/_6_` 三个宏） |
| `communication/` | ❌ 剩 1 块 | `interface_can.c:5 → fdcan.h`、`:6 → delay.h`、`:13 → hw_conf.h`；`interface_can.h:4 → main.h`；**24 处 `HAL_FDCAN_*` 真实调用**（滤波/init/start/stop/中断使能/TX 提交）+ `hfdcan1` |
| `app/` | ❌ 剩 1 块 | `common_inc.h`（聚合 `main.h` + `gpio/adc/fdcan/dma/spi/tim` + `delay/led/rgb/flash/hw_conf/bsp_task`，被 7 处包含）；`board_config.c:21-49` **14 处 HAL 启动序列**；`bsp_task.c` 直调 `LED_Task`/`Set_RGB_BreathingColor`；`foc_task.h:4 → main.h`（死引用）；`foc_run.c:11 → hw_conf.h`（死引用） |

策略现状：`harness.toml` 的 `[architecture.layers.app].allows` **显式包含 `platform_stm32`
与 `cubemx`**——所以 app 的硬件直用当前不违规。`motor`/`services`/`communication` 的
allows 均不含硬件层，其残留是"已登记待还"的债务（`tools/harness/architecture_debt.json`），
棘轮已锁死不允许新增。

## 2. 目标形态

```text
firmware/platform/stm32g4/   ← 唯一硬件域（bsp / ports / cubemx）
firmware/platform/api/       ← 12 个纯契约（能力 / 单位 / 时序 / 失败语义）
其它全部区域（app / motor / services / communication / common）
                             ← 零硬件：只 include platform/api + 标准库 + 软件自身头
```

`third_party/SEGGER RTT` 为唯一例外（调试通道，不配置外设；E 侧同样保留）。

## 3. 可用接口面（platform/api，15 个）

`board_hw.h`、`comm_hw.h`、`control_config.h`、`critical_hw.h`、`current_sense_profile.h`、
`encoder_sensor.h`、`encoder_spi.h`、`indicator_hw.h`、`mcu_temperature.h`、
`motor_hardware_profile.h`、`motor_hw.h`、`motor_sensing.h`、`param_store.h`、
`power_stage_hw.h`、`time_hw.h`。

参考实现模式（照此办理）：编码器解耦（[2026-09-19-encoder-decoupling.md](2026-09-19-encoder-decoupling.md)）
——契约 + 每型号 driver + 端口三层拆分，行为逐位保持，附实机只读证据。

## 4. 任务分解（按顺序执行，一次一项一提交）

### T1 services 清零（小）

- [x] `foc_param.h`：删 `#include "main.h"`，显式 `<stdint.h>`/`<stdbool.h>`；`MAGIC_WORD`
      迁入本头（schema 常量归位，数值未变）。
- [x] `foc_param.c`：删 `#include "common_inc.h"`，改显式 `current_sense_profile.h`/`data_type.h`/
      `utils.h`/`<math.h>`；`CANMsg` 直连改为 `param_comm_bridge.h` 窄桥
      （`CAN_NodeId_Get/Set`、`CAN_HeartbeatMs_Get/Set`，实现留通信层）。
- 不变量：参数 schema 11 与布局不动；读写流程语义不动。
- 验收：`architecture_debt.json` 删除 2 条（11→9）；单目标编译 0 错 0 警；参数相关夹具 PASS。

### T2 app 显式化 + 死引用清理（小-中）

- [x] 删除 `firmware/app/common_inc.h`（含 Keil 工程条目）；实际消费者 10 个（7 个 app +
      `main.c` + `stm32g4xx_it.c` + `foc_param.c`）已全部改最小显式 include。
- [x] `foc_task.h`：`main.h` → `<stdint.h>`/`<stdbool.h>`（编译验证）。
- [x] `foc_run.c`：删 `hw_conf.h` 死引用，补显式 `control_config.h`（控制时基常量经其传递）。
- [ ] 说明：本步之后 app 仍允许显式包含 `platform_stm32` 的 BSP 头
      （led/rgb/flash/delay/bsp_task）——那是 T4 的范围；本步只拆聚合、去死引用。
- 验收：`firmware/app` 不再出现 CubeMX 聚合头（main/gpio/adc/fdcan/dma/spi/tim）；
      单目标 0/0；全量测试。
- 建议做法：先列出每个文件当前实际使用的符号 → 一次性替换 → 立即单目标编译；
      避免多轮小步造成中间态拖长。

### T3 communication 契约化（中）

- [x] 扩展 `platform/api/comm_hw.h`：启动/滤波、波特率切换与 TX 提交契约
      （`comm_hw_can_start`/`comm_hw_can_set_baudrate`/`comm_hw_can_try_send_reply`，
      与既有 RX/状态帧保持同一"非阻塞、有界、无等待"风格）。
- [x] 端口实现放 `firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c`；
      `HAL_FDCAN_*`、`hfdcan1`、滤波结构、TX Header 组装全部下沉，致命失败经 `Error_Handler()`。
- [x] `interface_can.{c,h}`：删除 4 个硬件 include，改调 `comm_hw_*` + `time_hw_now_ms()`；
      电流限值改用 `CURRENT_SENSE_PROFILE_*` 全名（`hw_conf.h` 别名在通信层清零）。
- [x] 逐符号核对 `hw_conf.h`：审计结论为历史残留，已随本次清理移除。
- 不变量：**CAN 编号/帧格式/速率/心跳语义不变**；初始化失败、重连、队列满行为不变；
      不新增线命令。
- 验收：`architecture_debt.json` 删除 4 条（9→5）；单目标 0/0；CAN 相关夹具 PASS；
      可选：J-Link 只读冒烟（不烧录、不使能）。

### T4 app 深水区：板级启动与指示器（中大）

- [x] `board_config.c` 的 HAL 启动序列（ADC1/2 校准、TIM1 CH1-3+OCN、CH4、注入启动、
      JEOC→JEOS 切换、TIM7）下沉 `platform/stm32g4`（板级启动模块），app 只调用平台入口；
      **顺序、条件（"仅配置有效时启动三相"）、时序逐字保持**。
- [x] `delay_init` 等归位平台；`bsp_task.c` 的 LED/RGB 输出走 `platform/api` 指示器契约
      （用户已裁决 2026-09-19：采用契约方案；app 保留模式→颜色/错误闪烁策略与
      `Led_Cnt`/`RGB_Cnt` 计数，平台负责 LED/RGB 驱动）。
- [x] `rtt_telemetry.c` 去聚合依赖后保留 SEGGER RTT（第三方调试通道）。
- 验收：`firmware/app` 零硬件头（RTT 除外）；单目标 0/0；启动路径相关夹具/冒烟通过。

### T5 门禁收紧（小，收尾）

- [x] `harness.toml`：`[architecture.layers.app].allows` 去掉 `platform_stm32`、`cubemx`。
- [x] 确认 `check_architecture` 0 新增；`architecture_debt.json` 只剩 5 条 motor→services
      方向项（硬件类清零）。
- 效果：本文件的定则成为机械门禁——之后任何人（含 AI）再往这些区域引入硬件头会被直接拒绝。

## 5. AI 执行规则

**必做**

1. 一次一个任务、独立提交；沿用"先证据后迁移"的既有节奏。
2. 每改一个头先做全仓引用分析（`git grep`），得到**完整**调用方清单再动手。
3. 每步跑：`python tools/harness/check_architecture.py`、Keil 单目标 0 错 0 警、相关原生夹具；
   PR 档全量验证用 `uv run python tools/run.py verify --profile pr`（先 `uv sync --locked --extra dev`）。
4. 迁移是"搬家"：平台层的数值/顺序/语义与现状逐字一致；不改值、不改时序。

**禁止**

1. 不改 `cubemx/**` 生成区；不动 CubeMX 输入。
2. 不改 CAN 编号/帧格式/单位、参数 schema 与布局、20 kHz 快环与外设时序语义。
3. 不为通过检查删测试/放宽阈值；不引入新依赖。

**需要停下询问的决策点**

1. `delay` 在 CAN 流程中的阻塞语义归位方式（`time_hw` 轮询 vs 专用等待契约）。
2. LED/RGB 的归位形态：新增 `indicator_hw` 契约 vs 下沉为平台任务。
3. T5 收紧后若与并行工作冲突，先对齐再执行。

## 6. 验证与证据

- 机械门禁：`check_architecture`（棘轮）+ `check_interfaces` + `check_project_layout`。
- 构建：`build_firmware --target all`（Keil 单目标）0 Error / 0 Warning。
- 测试：`tests/run.py` 全量；PR 档证据落 `outputs/runs/<run-id>/summary.json`。
- 收尾证据写入 §7 执行记录（逐项追加）。

## 7. 执行记录

### T4 板级启动与指示器契约化（2026-09-19 完成）

- 新增契约：`platform/api/board_hw.h`（`board_hw_start(bool)`，顺序即契约）、
  `indicator_hw.h`（LED 闪烁语义 + 呼吸色）、`param_store.h`（参数区读/存）；
  端口实现：`ports/board/board_startup_stm32g4.c`、`indicator_stm32g4.c`、
  `param_store_stm32g4.c`（Keil 单工程已登记）。
- app 改写：`board_config.c` 只保留启动编排（顺序逐字保持：outer_init → 参数装载 →
  状态初始化 → `board_hw_start(条件)` → CAN 滤波器）；`bsp_task.c` 只保留调度与
  模式→颜色策略；`foc_run_state.c` 故障指示改走 `indicator_hw_set_led`。
- 附带归位：`RTT_SAMPLE_*` 与 `RTT_JSCOPE_DESCRIPTOR` 由 BSP `hw_conf.h` 迁入
  `platform/api/control_config.h`；`foc_run_state.c` 过流阈值改用
  `MOTOR_SENSING_OVERCURRENT_TRIP_A`；`common_inc.h` 的全部 app/cubemx 消费者已清除
  （仅余 `foc_param.c`，留给 T1/T2 收尾）。
- 验收证据：`firmware/app` 零硬件头（RTT 除外，定向 grep 0 命中）；Keil 单目标
  **0 Error / 0 Warning**（Code 81820 → 81888，+68 B 为跨 TU 调用/新增映射函数，
  RO/RW/ZI 不变）；夹具 `board_startup_test`（顺序、条件与 170 MHz 延迟基准）与
  `board_orchestration_test`（编排顺序 + 配置条件）PASS；run_state 差分 46,464 tick
  一致；`verify --profile pr` PASS
  （`outputs/runs/20260919T073422336263Z-dda0c780/summary.json`）。
- 决策记录：LED/RGB 采用**指示器契约方案（用户裁决 A）**；同步清理 6 条样式债豁免
  （board_config.c、bsp_task.c、foc_run.c、foc_task.c/h、hw_conf.h）。
- 余量：release 档待干净工作树；T1（`foc_param.c`）→ T2 余量 → T3 → T5。

### T1 services 清零 + T2 聚合头拆除（2026-09-19 完成）

- `foc_param.h` 去 `main.h`（显式 `<stdint.h>`/`<stdbool.h>`），`MAGIC_WORD` 迁入本头
  （schema 常量归位，数值未变）；`flash.h` 删除该宏并补齐两个函数的中文 Doxygen 契约。
- `foc_param.c` 去 `common_inc.h`：`CANMsg` 直连改为新增窄桥
  `services/parameters/param_comm_bridge.h`（`CAN_NodeId_Get/Set`、`CAN_HeartbeatMs_Get/Set`，
  实现留通信层、存储所有权不变；不迁移冻结的 CAN 编号枚举）。
- `app/common_inc.h` 已删除（Keil 工程条目同步移除）；全仓消费者清零。
- 验收：`check_architecture` PASS（**11→9**，0 新增）；Keil 单目标 **0 Error / 0 Warning**
  （Code 81924，+36 B 为桥调用）；`check_interfaces` PASS（79 known，3 条债务还清并移除）；
  `tests/run.py` 全量 PASS（wheel speed 夹具改走真实桥实现）；`verify --profile pr` PASS
  （`outputs/runs/20260919T074038460357Z-dda0c780/summary.json`；随后文档回填经
  `...074137...` 的 docs/interfaces/architecture 复核通过）；样式债再清 3 条。
- 决策记录：CANMsg 落位的 Oracle 咨询三次均因环境超时无产出，按执行者分析采用窄桥方案，
  取舍依据记录于本条目。
- 并发说明：收尾期间检测到并行会话正在执行 T3（新增 `comm_hw_can_try_send_reply`、
  `interface_can` 去硬件引用）；其未完成的夹具同步使 `run_can_status_tests` 暂时为红，
  且 4 条架构债条目已可移除（9→5）。该部分归并行会话收口，本执行未触碰其文件。
- 余量：release 档待干净工作树；T3（并行会话进行中）→ T5。

### T3 communication 契约化 + T5 门禁收紧（2026-09-19 完成）

- 契约与端口：`comm_hw.h` 扩展 `comm_hw_can_start`/`comm_hw_can_set_baudrate`/
  `comm_hw_can_try_send_reply`；滤波启动、波特率切换与应答 TX Header 组装下沉
  `ports/comm/comm_control_stm32g4.c`（状态帧/接收保留在 `comm_status_stm32g4.c`；
  致命失败经 `Error_Handler()`，与迁移前逐语义一致）。
- `interface_can.{c,h}`：删除 delay/hw_conf/fdcan/main 四个硬件 include，改调 `comm_hw_*`；
  限值改用 `CURRENT_SENSE_PROFILE_*` 全名；CAN 流程内不再存在 BSP `delay` 依赖。
- 收口补完（本次）：并行会话遗留的端口实现缺失、夹具同步与 4 条已解决债务条目由本次补齐；
  `run_can_status_tests` 增加节点滤波/波特率切换/应答端口断言。
- T5：`harness.toml` 的 app 层 allows 去掉 `platform_stm32`/`cubemx`——本文件的定则
  自此为机械门禁。
- 验收：Keil 单目标 **0 Error / 0 Warning**（Code 81980）；`check_architecture` PASS
  （**5 known，0 新增**，硬件类债务清零）；`verify --profile pr` PASS
  （`outputs/runs/20260919T074843783569Z-dda0c780/summary.json`）。
- 统一协议测试（2026-09-19）：原生 `run_can_status_tests` 8 项全 PASS（金帧、编解码、
  RX 路由、端口队列/滤波/波特率/应答、派发优先级、命令适配器）；单元协议套件
  `test_can_motor_status` + `test_can_parameter_protocol` + `test_canfd_diagnostics`
  10 项 PASS；双轴 CAN 工具套件 12 项 PASS。
- 发布证据（2026-09-19，提交 `c39d0d25`，dirty=false）：`verify --profile release`
  **PASS（11/11）**，含 `keil-build` 与 `release-manifest`；产物 HEX `8c1f5787…`（245,085 B）、
  AXF `90866aac…`、MAP `b380d742…`，0 错误 0 警告；证据：
  `outputs/runs/20260919T091846109938Z-c39d0d25/summary.json`。
  （预检记录：`doctor --profile release` 六项工具链全 PASS。）

## 8. 关联文档

- [foc_run 归位与瘦身](2026-09-19-foc-run-placement.md)：H5 反向依赖清单
  （interface_can 批次 4 条 + 服务/协议 7 条）。
- [Encoder 解耦](2026-09-19-encoder-decoupling.md)：分层拆分与验证的参考模式。
- [对接 deepseek-e 收尾与供料清单](2026-09-19-e-framework-interface-prep.md)：
  本工作与 E 框架硬件接口（hardware_api）一一对应。
- [技术债跟踪](../tech-debt-tracker.md)：ARCH-001/002 的退出条件。
