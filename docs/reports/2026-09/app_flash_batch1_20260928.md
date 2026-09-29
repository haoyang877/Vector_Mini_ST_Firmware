# APP 容量优化第一批结果（2026-09-28）

基准 `f0c2517`，ARMCC 5.06 update 7 build 960。仅离线操作，未烧录或访问硬件。
本批净减少 Loader 配套 APP 加载跨度 **4080 B**、RAM **16 B**，仍超限
**26592 B（25.97 KiB）**。容量适配没有完成，不能发布诊断镜像。

## 分步实测

| 阶段 | 普通 APP ROM | Loader APP ROM | Loader APP 加载跨度 | 本步跨度收益 |
| --- | ---: | ---: | ---: | ---: |
| f0c2517 基准 | 102240 B | 110504 B | 110544 B | — |
| 退役位置参数运行字段 | 101992 B | 110256 B | 110296 B | 248 B |
| 整数实现线路数值编码 | 99896 B | 108160 B | 108200 B | 2096 B |
| 通信、服务及启动模块空间优化 | 98608 B | 106424 B | 106464 B | 1736 B |

最后普通 APP RAM 31896 B；Loader APP RAM 32128 B，距离可用 32736 B 剩余
608 B（不代表运行栈余量）。Loader 独立构建保持 20036 B ROM / 5152 B RAM。
Loader APP 的 ROM 与加载跨度相差 40 B，为向量表到固定镜像描述符的地址间隙。

真实 Loader APP 仍在 78 KiB 区域链接失败。为统计使用虚拟扩大容量的
`NONDEPLOYABLE-app` scatter/link，只输出诊断 AXF/MAP，不输出 HEX；正式布局未扩大。
相对本轮容量缩减开始的 120160 B 跨度，累计减少 13696 B。

## 采用的修改

### 旧位置参数清理

提交 `afe1920` 删除没有控制调用者的四个位置增益/积分限值运行字段、赋值和校验。
schema11 原槽位改名为 reserved，读取忽略、导出清零；有效参数及标定偏移不变。
不提供旧 API 别名、转换或自动 Flash 回写。私有运行结构缩小 16 B，SWD 必须使用
同版 ELF。见 [schema11 退役槽位约定](../../protocols/parameter_schema12_loading_policy_v1.md)。

### 软件双精度依赖

提交 `e13e150` 用 binary32 位分解与有界整数运算实现原有舍入，分别保持状态编码
和参数编码不同的端点规则。当前缩放的乘积精确，未改变浮点编译语义或控制算法。
软件双精度加减、乘法、转换、比较和 round 对象退出链接；替代代码开销已计入
2096 B 整包净收益。见 [数值编码等价契约](../../protocols/numeric_encoding_v1.md)。

### 逐模块编译空间对比

在独立工程导出原始 ARMCC 参数，对 34 个通信、服务、启动及共享对象分别测量
O2/空间与 O3/空间，共 68 次单对象替换整包链接。未改变全局优化、控制快环选项、
HAL 源码或 ABI；选取 17 个有明显收益的对象。相同收益优先 O2，O3 额外收益
至少 16 B 才单独采用。

| 模块 | 采用选项 | 单独替换整包收益 |
| --- | --- | ---: |
| app_lifecycle | O2/空间 | 288 B |
| yg_protocol_link | O2/空间 | 256 B |
| image_transfer、nv_sha256 | O2/空间 | 各 136 B |
| yg_protocol_fragment | O2/空间 | 120 B |
| boot_handoff、parameter_value_service | O2/空间 | 各 104 B |
| yg_protocol_frame_codec | O2/空间 | 96 B |
| yg_protocol_job | O2/空间 | 88 B |
| yg_protocol_owner_adapter | O3/空间 | 80 B |
| yg_protocol_status_adapter | O2/空间 | 72 B |
| nv_record | O2/空间 | 64 B |
| yg_protocol_motor_status | O3/空间 | 56 B |
| foc_param、motor_axis_profile、yg_protocol_motor_adapter | O2/空间 | 各 48 B |
| motor_stop_service | O2/空间 | 40 B |

单独替换的对齐与组合链接不同，采用后净收益以 **1736 B** 为准，不能累加表项。
Loader APP 生成器复制普通 APP 的明确空间选项；启动附加模块按已测 O2 生成，
生成器重复执行输出相同。独立 Loader 工程不继承这些 APP 专属选项。

## 功能占用变化

按原来的功能归属和 Code + RO 口径重新汇总：

| 功能 | 基准 | 本批后 |
| --- | ---: | ---: |
| C 与数学运行库 | 5550 B | 3394 B |
| CAN 通信、协议与业务适配 | 14467 B | 13685 B |
| 主状态机与停机生命周期 | 10232 B | 9916 B |
| Loader 接入、镜像及记录校验 | 7368 B | 6950 B |
| 参数、轴配置及存储服务 | 4700 B | 4318 B |
| 调度、控制适配与电机状态 | 5840 B | 5832 B |

位置/轨迹、标定、FOC、角度反馈及 HAL 等功能没有删除；微小跨项变化包括私有
结构偏移和链接布局引起的代码变化。全部对象逐项结果保存在本批证据目录。

## 验证及限制

- 普通 APP 和独立 Loader 重建均 0 错误、0 警告；配套 APP 真实容量链接失败，
  虚拟诊断链接成功。
- 22 项统一原生检查全部通过，包括数值编码 49358928 次对比、当前参数黄金
  布局、STOP/FAULT/取消、位置外环、配置缓存、标定、协议核心/链路和 Loader。
- 集成测试通过，架构、工程布局、hygiene 通过；修改源码的局部 format/lint 通过。
- 修复旧协议夹具对已退役 UPDATE 接口的引用、任务回复类型、显式身份绑定和
  runtime 启动边界；不恢复旧接口，也不降低现行业务断言。
- Loader 纯内存模型接入统一原生列表，修复夹具的 `base + address - origin`
  中间越界指针，改为先算有界偏移。此修复仅涉及测试模型。
- 完整 release 已运行但未通过：既有 16 项格式、603 项 lint、34 项新增计数的
  接口文档问题，以及 10 个历史产物链接。Python 单元 39 项中 1 项因这些文档
  链接失败；release 还要求干净工作区，用户 Keil GUI 修改保持未提交。
- 没有快环耗时、栈水位或实机验收证据；不宣称发布通过。

证据目录为 `outputs/capacity-fit/batch1/`：分步 map/log/AXF、编译对比 JSON、
完整功能账单、源码和产物哈希清单。统一验证为
`outputs/runs/capacity-batch1-verified-20260928/summary.json`。

后续仍按计划处理配置校验与状态协调的重复，再用剩余缺口评估 Loader 确认职责
或维护 APP 的产品取舍。本批没有改变这些契约，也没有移出标定功能。
