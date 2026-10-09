# 单项维护任务通信载荷 v1.2

日期：2026-10-08。本版收口 [v1.1](yg_protocol_calibration_wire_v1_1.md) 的任务种类。CMD ID `0x00A0`、`0x00A1`、`0x00A2`，4 B 请求、12 B 回复、ticket、序号、阶段及结果码的字节格式不变。

| kind:u16 | 当前处理 | 含义 |
| ---: | --- | --- |
| 1 | 支持 | 编码器 LUT |
| 2 | 支持 | 电气零位 |
| 3 | 撤回 | 原齿槽标定；START 返回 `UNSUPPORTED/detail=1` |
| 4 | 撤回 | 原相电阻辨识；START 返回 `UNSUPPORTED/detail=1` |
| 5 | 撤回 | 原摩擦辨识；START 返回 `UNSUPPORTED/detail=1` |
| 6 | 支持 | 机械零位 |
| 7 | 支持 | 维护 SAVE |
| 8 | 支持 | 恢复默认参数 DEFAULTS |

这些线路编号不等于 `MotorMaintenanceKind` 内部枚举。kind 0 是非法字段；未知非零 kind 由业务返回 `UNSUPPORTED/detail=1`。撤回的 3～5 保留编号但不再进入维护所有者，也不会返回成功 ticket；主站不得把旧版设备支持过的种类推断为新版仍支持。

`0x00A1 STATUS` 和 `0x00A2 CANCEL` 仍只按原始 ticket 查询或取消。kind 7 是单项维护 SAVE；`0x007A SAVE_PARAM` 仍负责活动参数的版本化保存，两条命令的 ticket 与 revision 不混用。每项任务是否完成、结果是否持久化，仍以回复的 `phase`、`effects` 和实际所有者状态为准；收到 START 的 `result=1` 不表示完成。

兼容性：v1/v1.1 的头、长度与旧 kind 数值没有重排；旧主站发送 kind 3～5 会得到明确不支持。v1/v1.1 对 kind 6～8 可能返回不支持；主站应按当前设备回复判断。新建请求和回复载荷见[黄金向量 v1.2](yg_protocol_calibration_wire_v1_2_vectors.json)，v1.1 向量只代表当时产品范围。
