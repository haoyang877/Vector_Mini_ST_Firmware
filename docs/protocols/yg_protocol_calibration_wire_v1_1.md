# 标定任务通信载荷 v1.1

日期：2026-09-30。基于 [v1](yg_protocol_calibration_wire_v1.md) 的加法扩展；头部、
Type 160～162、4B 请求、12B 回复、ticket、阶段和 effects 均不变。

| kind:u16 | 维护任务 | 完成效果 |
|---|---|---|
| 1 | 编码器 LUT | 沿用 v1 |
| 2 | 电角零位 | 沿用 v1 |
| 3 | 齿槽转矩 | 沿用 v1 |
| 4 | 相电阻辨识 | 活动 RAM 结果，不自动写入 Flash |
| 5 | 摩擦辨识 | 只产生候选，不自动应用或写入 Flash |

START 仍为 `kind:u16, reserved=0:u16`。kind 0 非法；6 及以后可通过线路解码，
由业务返回 `UNSUPPORTED/detail=1`。STATUS/CANCEL 仍只认原始 ticket；
同 source/sequence/kind 的 START 重试仍返回同一任务。只有 `phase=COMPLETED`
才表示算法完成，`effects=COMMITTED` 才表示持久提交，不能从 START 受理推断成功。
这两项标定只映射已有维护状态机，不开放旧 Mode 编号、任意功率参数或绕过准入的入口。

兼容性：新版设备保留 kind 1～3 原有语义，新增 4～5；旧版设备对 4～5 返回
`UNSUPPORTED`。主站必须按设备响应判断支持，不能仅凭合法载荷认定设备具备该能力。
新增黄金载荷见 [v1.1 向量](yg_protocol_calibration_wire_v1_1_vectors.json)。
