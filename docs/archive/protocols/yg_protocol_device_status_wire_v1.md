> 历史归档（2026-10-21）：重写前的历史/混合说明快照；当前同名文件按实际代码重建。 本文的“当前”指原始基线；现行入口见[当前说明](../../protocols/yg_protocol_device_status_wire_v1.md)。归档不新增执行授权。

# 设备启动身份发现 v1

日期：2026-09-28。ROOT冻结已分配Type3的最小发现接口；本轮仅代码，不构建、不测试。
Type3定义为GET_DEVICE_STATUS，替换历史HEARTBEAT候选；历史motor_protocol_v1不是当前schema契约。
不改1/2分页格式，不从GET_INFO.boot_id:u32补造u64启动代号，不扩BootEntryService。

## 1. 请求与16B回复

空请求，单播，公司ACK_REQ或ACK_REQ|RETRY；来源策略沿只读请求，不修改设备状态。
公司16B头/CRC、29位扩展CAN FD、1M/5M不变；回复同Type/seq，源/目标交换。

| 偏移 | 小端字段 | 含义 |
|---|---|---|
| 0～1 | result:u16 | 0为真实身份可用；6为当前数据不可用，沿用现行BUSY线路码 |
| 2～3 | detail:u16 | 首版0 |
| 4 | role:u8 | 1 APP，2 Loader，不是镜像role |
| 5 | boot_state:u8 | 0无有效交接；1已确认普通APP；2未确认trial APP；3Loader驻留 |
| 6～7 | flags:u16 | 仅bit0 epoch_valid；其他0 |
| 8～15 | boot_epoch:u64 | 真实非零Loader启动代号，全部64位 |

APP没有真实已验证交接时：result6/role1/state0/flags0/epoch0，仍返回16B，不丢弃回复。
字段错误/非空请求走公共错误回复，不能带伪造身份。
APP角色不得报告Loader阶段；有效epoch必须非零且与真实阶段一致。
主机用Type3读到完整boot，再发送56的8B expected_boot；不能只依据旧32位计数判断权限。
角色与启动状态用于发现，不意味着已停机、已保存或新镜像自检通过。

## 2. 接口与所有权

`yg_protocol_readonly_device_status_t`只包含boot_epoch/role/boot_state/epoch_valid。
`yg_protocol_readonly_encode_device_status`复用原readonly_payload模块，显式字段编码，不复制C布局。
`YgProtocolLink_BindDeviceStatusSource`借用APP所有者的稳定只读快照；NULL恢复真实APP角色但epoch不可用。
调用方保证快照生命周期，后台更新与Service串行；不能把栈上临时对象长期绑定。

E从D BootHwInfo.available、epoch、trial/confirmed提供实际值，不读取主机输入来建立启动身份。
Type3不触发试启动确认、停机、Flash、邮箱写入或复位。
Loader现有53已提供完整epoch，可继续53发现；不要求D增加重复发现路由。
本接口不把主机在线作为试启动确认前置，不猜测安装的Loader版本或身份。

## 3. 统一验证交接

黄金载荷见`yg_protocol_device_status_wire_v1_vectors.json`，本轮未执行。
覆盖高32位保持、APP/Loader角色区分、真实trial/confirmed、空/非法请求、无来源16B回复、
无效epoch不得进入56、快照生命周期以及发现不修改确认/停机状态。
产品接线和最终APP/Loader构建仍由ROOT/E/D分别交付，不将codec存在当验收完成。
