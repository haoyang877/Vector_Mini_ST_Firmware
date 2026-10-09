# 齿槽最小任务与交接事实 v3

历史文档：2026-10-09标记。v1.3.0已删除齿槽任务、运行补偿及对应工作区。
当前任务范围见[核心维护模块](motor_maintenance_core_v3.md)。

日期：2026-10-08。唯一任务入口按以下顺序执行，公开接口保持。

1. **启动**：校验资格/电压/温度/电流/相位，锁定身份、角度和限流；复位控制器、预载零矢量，报告 PREPARED。
2. **定位采样**：每 20 kHz 先检查会话安全，再累计 iq；每 10 拍计算均值，推进已有 2 kHz 采样核心，未结束时执行原定点保持；正常拍调用电流计算。
3. **统一结束**：启动失败、安全失败、采样失败或完成均清零四个目标并复位控制器；清理完成才清 active，报告 FAILED 或挂起 FINALIZE。
4. **候选交接**：关相/epoch/工作区由 APP 核实；任务认领并调用已有成表核心，把完整候选交运行适配，等待真实存储结果通知。

采样算法仍为 `cogging_calibration`，成表/CRC 为 `cogging_map`，运行补偿/guard 仍为
`foc_cogging_runtime`。任务不拥有功率许可、Flash 或第二套生命周期。

算法观察对象 `s_cogging_calib` 保持；任务标量集中到 `s_task`，物理量名字带单位。

| 任务事实 | 值 | 意义 |
| --- | --- | --- |
| active | false / true | 控制清理完成才归零，不能从采样 COMPLETE 推导 |
| pending | NONE / FINALIZE / SAVE | 原互斥成表/保存挂起的单一表示；SAVE 不占候选工作区 |
| claim | NONE / ACTIVE / CANCELLED | 认领中取消继续占用到归还，取消后拒绝成表/发布 |
| 采样/保持数据 | 起始坐标、身份、限流、10 拍计数/均值、斜坡/迟滞速度 | 一个任务所有者写；数学和求值次序保持 |

```text
采样完成 → 清理控制 → pending=FINALIZE
APP 认领 → pending=NONE、claim=ACTIVE
候选发布 → pending=SAVE（不是 Flash 成功）
归还认领 → claim=NONE；保存结果由真实提交通知
认领中取消 → claim=CANCELLED；归还后清 pending 并记录取消
```

NONE 是私有非零编码，通过显式静态初值保持 RW 初始化压缩；这些值不进入记录/协议。
旧记录/参数、公开入口、采样/保持数学和全部保护阈值保持。源码行数没有作为简化目标。
证明、删除依据、源码/产物哈希与限制见[报告](../reports/2026-10/cogging_minimal_task_20261008.md)。
