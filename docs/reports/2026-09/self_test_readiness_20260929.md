# 基础自检与模式就绪完成报告

日期：2026-09-29；构建/功能源码：9ea2b014；合并目标 codex/system-refactor。

## 行为

- 未确认配置、未标定和部分标定基础健康后停在 READY；不自动运动，不将准备缺项锁成故障。
- 配置确认、标定资格和每种模式启动资格分离。转矩维护需要先确认配置；任务仍按自身前置条件受理。
- 自检汇总八项事实，连续 4 拍通过；500 ms 期限、时钟回绕及关断失败 FATAL 有测试。
- 120/121 新增准备资格 ID 0x4D～0x51；108/124 payload 和主状态编号不变。
- schema12 复用退役槽保存确认与整条 CRC，长度 4304 B；旧版本不迁移、不自动回写。
- 未配置设备使用有效节点 1；默认速度保持 0.5 rev/s，避免新节点套用更大的轮子默认硬上限。

[详细设计](../../architecture/motor_readiness_v1.md)、
[存储契约](../../protocols/parameter_schema12_loading_policy_v1.md)、
[执行与回退](../../plans/completed/2026-09-29-self-test-readiness-v1.md)。

## 验证与容量

最终 release profile 通过；所有离线 unit/integration/native 测试、接口、架构、文档、
风格和卫生检查通过。普通 APP 与两个额外 Keil 目标均 0 错误、0 告警。

| 目标 | Keil ROM（B） | RAM（B） |
| --- | --- | --- |
| YG_ESC_APP | 90772 | 31592 |
| Loader 布局 APP | 92400 | 31632 |
| YG2026_ESC_LOADER | 13468 | 4952 |

Keil ROM 是 Code+RO+RW 统计；实际打包镜像含链接器实际装载布局，为 91992 B，
小于 94208 B 槽，剩余 2216 B。真实包通过 Loader 内存模型的冷启动、交接、损坏镜像、
驻留和 Flash 等待检查；不是实物联调。版本 0 开发包、仅完整性 CRC，release_approved=false。
产物哈希与日志在 outputs/self-test-readiness/summary.json，统一证据在
outputs/runs/self-test-readiness-accepted-20260929/summary.json。

未配置/四种标定组合、确认前拒绝、允许的标定顺序、无母线配置、掉电重载、CRC 损坏、
非法确认标志、过期成功不得放行、健康窗口中断、计时回绕和关断失败均有原生覆盖。
相阻/摩擦算法运行轨迹与基准一致；本次没有改变其计算或快环频率。

## 实物边界

未烧录、未写设备参数、未控制电机。500 ms/4拍为软件默认值，实际启动时序与监督开销尚未测量。
数字关断不证明功率管电气正常；未增加不存在的驱动器故障输入、输出轴传感器或自动转矩自检。
schema11 设备后续部署需匹配 APP/Loader，并重新配置/标定或按独立计划处理备份参数。
原工作区电机实验和 Keil 用户界面设置保留；临时 worktree 合并后通过归档工具清理。
