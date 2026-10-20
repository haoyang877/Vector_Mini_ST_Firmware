# Loader 多 MCU 预备改造与离线验证

日期：2026-10-21。只做已有 G431 的边界重构和离线验证，未新增实际 MCU 型号，未连接硬件。
原始基线为 `f199f7d4`，首轮实现为 `179b3d7b`；随后吸收主线 `14f68e0f`，组合代码为 `8290dcf6`。

## 已完成改造

| 范围 | 结果 |
| --- | --- |
| 分层检查 | 纳入 Loader API、核心、组合、产品、MCU、板级和目标配置；禁止核心直接引用目标或硬件私有头 |
| 公共接口 | 从 G431 实现目录移到 `loader/ports/include`，接口描述中去掉特定中断寄存器名称 |
| 目标传递 | 同一目标清单贯穿工程生成、APP 后处理、打包、CLI 和 Qt 主机构建/部署 |
| 目标拒绝 | 未知目标、错误身份、越界或不支持的编程/启动能力明确失败；生成头与所选清单不符时拒绝构建 |
| 板级安全 | 关相动作与关闭状态核对归 `loader/boards/vector_mini_st`，保留原寄存器条件 |
| 参数读取 | 产品层负责版本、节点与空白判定，MCU 层只提供有界只读访问，拒绝进入生产身份页 |

`LoaderParameters_ReadNode` 替代内部的参数解析入口，调用方和测试已同步。
线上协议、参数 ABI 和存储布局保持不变；不表示内部 C 接口名称完全没有变化。
未修改共享镜像算法、协议编解码或升级核心。未增加新的硬件库、动态目标注册表或设备访问入口。

## 兼容性对照

原始基线和首轮实现使用相同工具链、G431 目标和 token 构建。
默认目标和显式 `--target vector_mini_st_g431` 的 Loader/APP 程序、CAN 镜像和清单相同。
APP BIN、CAN 镜像和清单与原始基线相同；Loader 因函数边界变化不是逐字节相同。
源码对照确认关相判断与参数解析逻辑在接口替换后保持一致。

| 项目 | 原始基线 | 首轮实现 |
| --- | ---: | ---: |
| Loader Flash | 14,492 B | 14,520 B |
| Loader RAM | 4,344 B | 4,344 B |
| 配套 APP Flash | 80,480 B | 80,480 B |
| 配套 APP RAM | 18,552 B | 18,552 B |

Loader Flash 增加 28 B，仍在 16 KiB 预算内，剩余 1,864 B。栈预留未缩减，运行时峰值未测。
以上 APP 对照仅衡量本轮边界重构；后续吸收主线的控制代码变更另由组合构建记录描述。

## 验证证据

- 首轮完整 release：`outputs/runs/20261020T161512882917Z-179b3d7b/summary.json`。
- 基线：`outputs/delivery/mcu-boundary-baseline-20261021/summary.json`。
- 显式目标与默认目标：`outputs/delivery/mcu-boundary-explicit-20261021/summary.json`、
  `outputs/delivery/mcu-boundary-default-20261021/summary.json`。
- 工件哈希、目标常量及逐字节对照：`outputs/optimization/mcu-boundaries-comparison.json`。
- 源码语义对照：`outputs/optimization/mcu-boundaries-source-comparison.json`。
- 合并后完整 release：`outputs/runs/20261020T161737031499Z-8290dcf6/summary.json`。
- Qt 组合构建和离线测试：`outputs/host_app/mcu-boundaries/summary-merged.json`。

分层反例、显式目标传递、非法目标拒绝、生成头不匹配、工程头依赖，以及真实 Loader
内核上的冷启动、交接、确认、故障驻留、参数边界和板级关闭均已纳入离线检查。
临时目标只存在于测试夹具中，用来验证身份选择，不代表支持第二种芯片。

合并后的资源和配套包证据在 `outputs/delivery/mcu-boundary-merged-20261021/summary.json`。
最终 Loader Flash 为 14520 B，RAM 为 4344 B；
最终配套 APP Flash 为 80392 B，RAM 为 18552 B。
该清单逐项关联当前源提交、目标、工具链产物及哈希；不把主线其他改动造成的 APP 大小变化
归因于本次 Loader 边界重构。

## 后续边界

仍保留现有 v3 的 8 字节编程、等长擦除单元、128 KiB APP 上限、Cortex-M 启动向量和
12 字节 UID 约定。新芯片的 Flash、ECC、时钟、CAN、跳转和 APP 端口仍需结合选型实施。
生产身份写入工装仍为 G431 专用，UART 尚未实现。

所有实机证据仍属于此前冻结配套版本。本次结果不宣称新构建已经通过升级、掉电或电机测试，
也不自动批准发布。后续操作按[MCU 适配指南](../../guides/loader_mcu_porting_v1.md)执行。
