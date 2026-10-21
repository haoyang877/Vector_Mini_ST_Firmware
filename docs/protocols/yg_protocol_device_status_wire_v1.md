# 当前Type3兼容发现响应

核对日期：2026-10-21。Type3是GET_DEVICE_STATUS，不是历史HEARTBEAT。
空请求、单播、ACK_REQ或ACK_REQ|RETRY；响应固定16B，小端。

| 偏移 | 字段 | 当前APP值 |
| ---: | --- | --- |
| 0/2 | result/detail:u16 | 6 / 0，持久启动身份不可用 |
| 4 | role:u8 | 1，APP |
| 5 | boot_state:u8 | 0 |
| 6 | flags:u16 | 0，epoch_valid未置位 |
| 8 | boot_epoch:u64 | 0 |

当前状态适配器明确撤回持久epoch来源，不能根据Type3补造设备启动次数或授权Type56。
Loader不依赖这个APP兼容接口；角色/镜像确认状态由Type1 page1发现，
升级事务由Type53的主机随机nonce与session绑定，见[升级v3](yg_protocol_upgrade_wire_v3.md)。
Type3不写Flash、不清错、不确认试运行，也不触发停机或复位。
保留[历史codec向量](yg_protocol_device_status_wire_v1_vectors.json)用于编码/拒绝回归，
其中有效epoch用例不表示当前产品仍提供该能力。
