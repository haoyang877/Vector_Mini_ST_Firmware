# 编码器标定任务等价简化验收 2026-10-08

版本 v1。范围：`encoder_calibration_task.c`、已有维护工作区夹具及本模块文档。
独立分支 `codex/encoder-calibration-simplify`，实现完成，等待 ROOT 审查；没有自动集成。

## 结论与窄差异

Mode13 的 22 处失败收尾合并为文件私有函数，保持先故障上报、再控制清理、返回原错误码。
完成共用同一控制复位；移除置空引用前无意义的条件，阶段末尾失败直接返回。
无感运行已经锁存故障的路径仍只清理并传回该故障，不重复上报。
Mode15 仅将说明注释改为中文，代码和 public header 未改。
没有新增运行状态、接口、框架、配置或依赖；没有改 MotorAxis、工作区、FOC 保护或保存层。
本轮减少整包 ROM **32 B**，静态 RAM 不变；不能宣称更大的算法缩包收益。

阶段职责、单位、数据流、准入、候选发布、取消/停止和保存边界见
[职责文档](../../archive/architecture/encoder_calibration_task_v1.md)；不变项与回退见
[计划](../../archive/plans/2026-10-08-encoder-calibration-simplify-v1.md)。

## 完整提交与冻结输入

| 身份 | 完整提交 |
| --- | --- |
| 集成树起点 | `b0077b60b3eb84d8d72b1ec42c62141573b3245d` |
| 继承 firmware/test dirty 构建基线 | `10ef59b174d4765776d13aff79c7c1ead1258a23` |
| 补齐既有文档依赖后的 refactor base | `7c3725b7678d19fb3949b74ea54d9b130a292769` |
| 已验证实现 head | `ddd37716bb275b6cb6776884d2bdc424d2544a46` |

最终报告提交 HEAD 在交付消息列出。ROOT 只审查/集成本轮实现与报告提交，
不将两个冻结继承提交作为本轮修改再次回灌。
新工作树从核实的集成 HEAD 创建，原树所有修改和输出保留。整包比较冻结 firmware
现场内容、用户 Watch 配置及对应离线夹具。首次 PR 发现 HEAD 引用未提交的 ordinary-start
提案，单独冻结该文档，未改变固件输入。下表全部为继承修改，不计入本轮收益；
其他用户文档/实验改动没有复制或修改。

集成树主源码 SHA256：
`a5e8d511ddd41517be7bdf12882c0b2c1903a9ab8195bcf02bd2c729723e9bee`。
独立 checkout CRLF 原字节 SHA256：
`cf9dde8d8580e7290d1b2645ae50e1abfa8c4822583fde04e37b7e73a624c1e3`；LF 规范化后与集成树完全一致。
实际最终构建源码 SHA256：`d5da247cfd2f59fa38991c34eaa745897a40b7d11fbb44e4a9e9deb25993e295`。
冻结基线到实现的 firmware/shared 构建输入差异只有本模块 `.c`；头文件、数学/profile、
调用者、项目/链接边界不变。原字节冻结清单如下，机器记录为
`outputs/encoder-calibration/inherited-dirty.json`。

| 继承路径 | 冻结时 SHA256 |
| --- | --- |
| `firmware/app/composition/yg_protocol_owner_adapter.c` | `0132cddbaaaa614f52fa18527c76b87d170f46ff6b611b73db0742fafe2e292c` |
| `firmware/app/motor_state_machine/motor_state_machine.c` | `a4bfbe3949d3e3f19b7321bac6bca6584873d262ada5151bbdb90b6a0c623e1c` |
| `firmware/app/runtime/motor_axis.c` | `f12472f452b602aaf991a9eb08332b2ad877c32269dfc0a88bc64ffea6fac0bb` |
| `firmware/communication/can/yg_protocol_link.c` | `e07eadcdfed0d0245a9127248fdf2d3259e1eb0f9ed2638524e9f062a1cdc3d4` |
| `firmware/communication/protocol/yg_protocol_job.c` | `7a295c3ce523b7ab3878637c6d8469ac08dfad9b5aa5876e6262fd9e9256ab2d` |
| `firmware/communication/protocol/yg_protocol_job.h` | `8ee0f07b59cab40b52c3ffe2ccdbdacab00512915c05883fac0cd9ad0930727f` |
| `firmware/communication/protocol/yg_protocol_motor.c` | `60b0c2d316634f162c08cff80111bf401c92e1f231871e1c90f555fe2f02f6a7` |
| `firmware/motor/feedback/angle_feedback.h` | `b65c25d1e6211fde951541911fa4241d2632c7f771c3946bbda9cd17e9d6acf6` |
| `firmware/motor/foc/foc_sensing.h` | `0cbfd417d12eaca2acdfe06b5d922a1f19f6df20a154ff01adc6db9aaf295882` |
| `firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvoptx` | `7de7a02dec7405493bb22ed962d8402717f2c491c7d793471e25e0bd251d65d7` |
| `tests/run.py` | `bc4f4a08501cc2d103c92b90d45417ecaa024ca8be0482d55a396c9104996d33` |
| `tests/unit/native/execution_dispatch_test.c` | `039db31abf01a2bec3402b1f22b9424a7c5fbaaf45ac49d426ebcb8b1a04bd4f` |
| `tests/unit/native/run_state_owner_test.c` | `750ea6ef8538bdd52b0a8298f3657d599c7ae43b23777f3f1853c77f09591f32` |
| `tests/unit/native/test_current_precision.py` | `5ca92b30e397e213e535a6b5e4681e5a7f3137c559dff2c0b5b0e1b03a4580e6` |
| `tests/unit/native/test_execution_dispatch.py` | `b43b07fb0c9e50f8989e01d0268d3446d7c5d8ec02cb5110e9af678f4a989295` |
| `tests/unit/test_speed_loop_bandwidth.py` | `f8898f8f917f5efb5191ca851cdb303a6af5c71db3ae00fc9de81e3c8412c9a8` |
| `tests/unit/yg_protocol_contract_test.c` | `a5128540defe7f3bf916c65fb73a3bef03390be438725907c46dc3cf462c1a81` |
| `tests/unit/yg_protocol_link_test.c` | `49184f493671596504a3bd5178b24060e4fd857972549523c7986b72d1d2a323` |
| `tests/unit/native/test_startup_current_offsets.py` | `bb0b74230453bab22cabdc5d0b4af6035f3fb9fa37de0ecef0f55efa0bb9db6e` |
| `tests/unit/startup_current_offsets_test.c` | `ed4440904246678e12091c5bb075abfce459e61ca3ecf37ddbc6551db0b9bc63` |
| `docs/protocols/yg_protocol_ordinary_start_proposal_v0_1.md` | `b4aae662b162d62d6422434485b6107a76711aa7be724f915e05c9f5b52cc6db` |

## 当前同输入容量与栈

两侧均用同一 normal `YG_ESC_APP.uvprojx`、既有逐文件选项与 ARMCC 5.06 update7 build960
重建真实 APP；各 0 errors / 0 warnings。未放宽链接边界，只测此工程。

| 指标，B | 冻结 baseline | 实现 head | 差值 |
| --- | ---: | ---: | ---: |
| encoder_calibration_task.o Code | 3610 | 3578 | -32 |
| 对象 RO | 6 | 6 | 0 |
| 对象 Code+RO | 3616 | 3584 | -32 |
| 对象 RW / ZI | 128 / 0 | 128 / 0 | 0 |
| StepLut 符号代码 | 2422 | 2404 | -18 |
| StepElectricalZero 符号代码 | 448 | 448 | 0 |
| APP Code | 90228 | 90196 | -32 |
| APP RO-data | 3120 | 3120 | 0 |
| APP RW-data / ZI-data | 304 / 26600 | 304 / 26600 | 0 |
| APP map ROM（含压缩 RW 加载数据 12 B） | 93360 | 93328 | -32 |
| APP 静态 RAM | 26904 | 26904 | 0 |
| StepLut 栈帧 / 最大静态调用链 | 88 / 360 | 88 / 360 | 0 |
| Mode15 栈帧 / 最大静态调用链 | 56 / 184 | 56 / 184 | 0 |
| 全局静态最大栈估算 | 548 + unknown | 548 + unknown | 0 |

首次候选使用结果暂存返回，整包反增 8 B，未作为收益交付；改为分支直接返回后减 32 B。
随后去掉不可达 `break` 的 #111-D 警告，最终零警告。
构建只因代码变化重跑；最终固件输入冻结后没有为报告或同输入重复重建。

## 等价性与验证

- 已有 `maintenance_workspace` 夹具直接编译完整标定 `.c`，替代无感调度、电流输出等边界；
  原工作区互斥/地址/容量/复用检查保留，真实 PI 复位和数学实现参与链接。
  同夹具对冻结源码与最终源码都 PASS：10 类失败/准入情形的错误码、先上报后复位、
  startup/三组 PI/速度和电流指令清理、引用释放、有效参数的低速停止完成、重复取消。
  不再按字符串只抽取少量函数，可用 `--reference-source` 对冻结源码复查。
- 对象段比较：Mode15、环形差、候选插值、稳定/跟踪、制动准入和停止速度 section 字节
  完全一致；128 B `.data` 一致。6 B `.constdata` 为编译器结果构造模板，随构造位置变化
  （baseline `02 00 00 00 05 02`，head `05 00 00 00 02 00`），容量不变；
  失败/完成结果语义经同夹具检查。源码差异不涉及采样、构表、残差数学或门限。
- `format --check`、`lint`、最终 `verify --profile pr` 全 PASS。PR summary 绑定干净提交
  `ddd37716bb275b6cb6776884d2bdc424d2544a46`、Python3.13.15、Zig0.16.0；
  59 个 unit、22 个 integration 和全部原生入口通过，架构/接口/文档/卫生检查通过。
  仅保留既有格式/样式债务豁免，没有新增债务。
- 首次 PR 原生全 PASS，unit 的文档检查与 docs gate 因集成 HEAD 的两个缺失链接失败；
  冻结已有提案后通过。夹具开发初次遗漏配置对齐时间，baseline/head 均触发宿主负浮点
  转 unsigned 保护；补齐既有 profile 时间后通过，未改固件。

当前证据：`outputs/runs/encoder-calibration-pr-final-v1/summary.json`。
baseline/head 日志、对象、map、调用图、AXF/HEX 和反汇编在
`outputs/encoder-calibration/baseline/`、`outputs/encoder-calibration/head/`；
机器容量、对象段比较、全部产物哈希与继承清单为
`outputs/encoder-calibration/capacity-and-artifacts.json`。

| 产物 | SHA256 |
| --- | --- |
| baseline `YG_ESC_APP.axf` | `39707c510972907edd5b12b16c25bb7982872ff328c49c61dcddc66d718e80aa` |
| baseline `YG_ESC_APP.hex` | `20489a47fdd360b375fd75c46c61f1d1dbe842bcc9c41815f1acdf084f5f579b` |
| baseline `YG_ESC_APP.map` | `9ea8543856d2fbe84186967205202d2d813bc037c9e794ee1eb14a8749c8580d` |
| head `YG_ESC_APP.axf` | `9e6c96aa2d283d647125c799972eb12b177a6fa4d7621f9c1688fbffc6e7e53d` |
| head `YG_ESC_APP.hex` | `62b0d36142b7d327c23857f8dace82db8a8690cce601c68f38c8d5140bb878c3` |
| head `YG_ESC_APP.map` | `808393d4de639bd64aaf2eb4ccef3a8079e142f780d32aee931283f05fa76e54` |

## 复用、未测与未实施

复用既有无感交接、主状态所有者、工作区和离线夹具。最终固件输入之后的报告变更
只补文档检查，不重复构建或全量原生测试。ABI、枚举编号、Watch 状态、控制数学、
20 kHz/2 kHz 调度、每周期 8 bin、真实 owner 关相和后台 SAVE 路径保持原实现。

未测实际 ISR 时间/抖动、栈水位、物理对齐/跟踪/残差、实物输出关断与 Flash 保存。
静态调用图含 unknown，不能代替动态栈证据；离线零指令检查不能证明物理关相。
没有 J-Link/USBCAN、烧录、复位、运动、认证修改或 release/deploy 声明。
历史标定成功与 SENSORLESS13 失败没有当作本轮 PASS。

未实施算法/单位/ABI/阈值调整、LUT 算法删减、共享状态重构或新增纯计算框架。
回退仅撤销本轮实现与报告，原集成树和继承快照不动。ROOT 审查后再决定集成。
