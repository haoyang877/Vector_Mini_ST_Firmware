# 齿槽核心任务解耦 v2 交付记录

日期：2026-10-08。独立分支 `codex/cogging-core-simplify`；本轮 base
`6348f09cc258a6a55cdb13d117f8a584bf5b7db6`，保留 v1 终态精简，不回退现场。
实现后的源码 SHA 与提交分类见 `outputs/cogging-core-v2/review-bundle.json`。
原继承冻结 `7031f5cfac8c315205792c51427f75de48076cf1` /
`fa876d996a4a15b84b7c81031c0a96b06fbf830c` 均不是本轮待集成提交；本轮未新增继承修改。
模块相关集成现场/独立基线原始 SHA 见 `baseline-inputs.json`：采样/map/补偿纯实现一致，
任务源/头差异只来自已交付 v1；集成树与默认项目目录均未写入。

## 实际结构与状态归属

一页职责/数据流见[核心边界 v2](../../archive/architecture/cogging_core_boundaries_v2.md)。
本轮复用 `CoggingCalibration_Start/Update/Abort` 和 `CoggingMap_Build/Valid`，
三份纯实现 `.c` 逐字节规范化比较不变，没有另造算法。

新 `foc_cogging_runtime.c/.h` 接管活动记录、补偿请求/应用、台架 guard/遥测。
任务保留采样/保持及前台认领，不再读写这些运行状态；运行实现不包含任务头、不读采样状态。
候选发布从直接写私有活动表改为既有 `FocCogging_ImportMap` 值复制边界，再置保存挂起。
主状态机、MotorAxis、维护、参数与共享工作区未改，功率/epoch/真实保存仍归原所有者。
原头包含运行声明，全部公开函数签名与 guard/frame/control/record 布局保持。

## 删除理由和实际调用证据

| 原实现 | 证据与处理 |
| --- | --- |
| SetCompensation 在 CRC 前的 mode/error 赋值 | 同一局部 info 字段在 critical_hw_enter 后无条件重写，之间未被消费；只删这两次赋值，保留提交前模式/故障复核 |
| 私有 signature 包装 | 两处生产调用（TableValid、BuildCandidate）及一个成表夹具改为直接调用已有 Cogging_EncoderSignature；签名输入/字节顺序不变 |
| 任务直接 include string/critical/compensation | 表复制/补偿/guard 移到运行所有者，任务无相应操作；去掉直接依赖 |
| compensation.h 依赖 calibration.h | 声明只用 CoggingMapRecord 等记录值，改为 cogging_map.h，运行链不再包含采样过程类型 |

未发现可安全删除的真实保护、补偿分支或取消状态；本轮全部保留，不用关闭功能代替解耦。
既有补偿纯实现的私有准入身份缓存保留，未为状态重排改变现有公共控制对象布局。
规范化函数体审计只允许上述资格赋值/签名展开/发布转交四处差异，其他函数体一致，
结果见 `outputs/cogging-core-v2/comparison.json`。

真实生产链仍为：MotorMaintenance→FocCogging_Task→采样核心；
MotorExecution→TorqueGuard/Apply/Observe→运行适配/补偿核心；
MotorStateMachine→认领/取消/ReleaseFinalize/SaveResult/Service；
MotorAxis 前台→BuildCandidate→APP epoch/关相复核→PublishCandidate；
foc_param→ResetMap/ImportMap/ExportMap→运行活动记录。公开入口无需改调用者。

## 工程接线与当前容量

ROOT 需应用[Keil 源登记补丁](../../plans/patches/2026-10-08-cogging-runtime-keil-v2.patch)：
普通 APP 与 LoaderApp 的 Motor 组各增加 `foc_cogging_runtime.c`，文件路径仍相对。
新条目逐文件设置复制原任务条目（Optim=3、oTime=0 等）；不覆盖任何既有设置或 uvoptx。
**原工程未登记新源，直接构建前须应用补丁。** `git apply --check` 在本独立输入通过；
本轮只构建普通 APP 的 outputs 工程副本，原工程 SHA 不变，LoaderApp 未构建。
副本仅新增源并重定位路径/输出，所有原逐文件选项比较一致；实际 map 含两份适配对象。
工程输入和副本 SHA 见 `outputs/cogging-core-v2/project-inputs.json`。

复用 v1 最终 normal map 为同一冻结输入基线（未重复构建基线），新副本重建一次，
ARMCC 5.06 update7 build960，0 错误/0 警告，链接边界未放宽。

| B | v1 基线 | v2 | 差值 |
| --- | ---: | ---: | ---: |
| 任务+运行对象 Code+RO | 3154 | 2208+910=3118 | -36 |
| 任务+运行对象 RW / ZI | 60 / 4256 | (48+12) / (2112+2144) | 0 / 0 |
| 独立 normal Code / RO | 90160 / 3124 | 90120 / 3124 | -40 / 0 |
| 独立 normal map ROM | 93296 | 93256 | -40 |
| 整包 RW+ZI | 26904 | 26904 | 0 |

本轮只归因这份独立冻结输入的 -40 B，不把 v1 收益或其他任务收益累加当 ROOT 组合包实测。
Task 自身帧仍 48 B，2 kHz 服务仍 24 B；峰值动态栈、WCET/抖动未测。
原始日志、AXF/HEX/MAP/对象/工程副本与哈希在 `outputs/cogging-core-v2/keil-input/` 和
`comparison.json`；产物用于离线接线/比较，不作发布或部署结论。

## 验证、复用与未测

- v1 完整齿槽日志原样复用作比较：v2 同一夹具仍 15,030,360 拍，控制/状态摘要
  `37c45ddf`、候选 CRC `72892f5a`，原采样/补偿/guard 日志逐字节一致。
- 新运行边界用公共头独立编译/链接：只加入 runtime/map/compensation/math/CRC，
  不提供 Task、CoggingCalibration、PI 或电流环符号，记录/控制/guard/frame 大小校验通过。
- 复用已有 cogging_finalize 与 owner 夹具，取消等待归还、旧表保持、RAM 与保存分离通过。
  迁移初次因夹具仍调用私有 signature 未编译通过，已改为已有纯核心入口；未放宽检查。
- format --check、lint 与完整 PR 全部通过：
  `outputs/runs/20261008T065629453293Z-23b4b771/summary.json`，
  tested SHA `23b4b771142e21b423226fa2f39bddb0284caa24`，Git clean，hardware_contacted=false。
  文档收尾没有改变固件/夹具/工程补丁输入，复用本次 PR 并单独检查最终文档链接。
- 没有实机、Flash 断电、真实关相、WCET/抖动或峰值栈证据；未运行 release、LoaderApp
  或多工程矩阵。没有未接入的原型；共享工程补丁须由 ROOT 集成后统一应用。

回退只撤销 v2 本轮源码/文档提交及登记补丁，保留 v1 与继承现场。
本轮完成后停在 ROOT 待审，不自动扩展到其他模块。

## 实际源码 SHA-256

源码提交：`23b4b771142e21b423226fa2f39bddb0284caa24`。以下为编译所读文件的原始字节，
换行规范化比较另在 comparison.json；最终完整 HEAD 与交付索引见 review-bundle.json。

| 路径 | SHA-256 |
| --- | --- |
| `firmware/motor/calibration/foc_cogging_calibration.c` | `b5724145e6d4c3a2789d6831e25b8562a630075fdd022613630c9c3020ddd1bc` |
| `firmware/motor/calibration/foc_cogging_calibration.h` | `4349898b75d53c431d4d48d4816314d4e5b44fb51adc4ee87f546be70949f9f9` |
| `firmware/motor/calibration/foc_cogging_runtime.c` | `3b82945c01ca4e38157691122d2fa6c1c195215489ccd04473435ba8b320ef30` |
| `firmware/motor/calibration/foc_cogging_runtime.h` | `3feb49ad7d84069ad98e6ee740abc30e0fc275c3030cde27a6e3366cd3424853` |
| `firmware/motor/foc/cogging_compensation.h` | `896be3af3f13b1799f25cc535ec849210f70589f71924f84d9e7aeaeb9184489` |
| `firmware/motor/calibration/cogging_calibration.c` | `d12a55adf8c210019a9c6d29c73db81a5608621dcf03145cbc8f77feaa8b2f9b` |
| `firmware/motor/calibration/cogging_map.c` | `887ede014b2c6d95b08742034779527be0794c84a1c44166d9ea39f6f29f259b` |
| `firmware/motor/foc/cogging_compensation.c` | `9774b5f71305e1a1a9a3a6638a7f096b3c4092da7adc0872940873ab130a2dfc` |
