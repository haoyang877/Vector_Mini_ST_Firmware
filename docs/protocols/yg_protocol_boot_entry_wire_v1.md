# APP进入Loader载荷 v1

日期：2026-09-28。使用D固定提交`1109361d`的`shared/boot/loader_entry.h`，不复制一套交接类型。
本轮不构建、不测试。codec已实现；生产路由须同时接上业务所有者与真实物理TX完成。

## 1. 线路布局

公司16B头/CRC、CAN FD扩展ID、1M/5M不变。56仅主站2单播，ACK_REQ或ACK_REQ|RETRY。
请求恰好8B：expected_boot_epoch:u64小端且非零；source与sequence来自公司头。
这是当前Loader交给APP的真实启动代号，不能使用APP本地计数或公司GET_INFO的boot_id:u32补造。
回复恰好4B：result:u16、detail:u16，均小端，同Type/seq、源目标交换。

| APP共享结果 | 线路result/detail | 含义 |
|---|---|---|
| UNAVAILABLE | 2 / 0 | 无已安装且符合布局的Loader或无真实启动身份 |
| PENDING | 1 / 1 | 请求已受理，等待停机/资源归还/保存空闲/真实关相 |
| READY | 1 / 0 | 可以发送受控交接回复，尚未复位 |
| DENIED | 5 / 0 | 身份、状态或准入拒绝 |
| FAILED | 8 / 0 | 交接失败 |

长度错误3、字段错误4、未授权主站7、忙6沿用公共错误，保留4B结果布局。
READY不是APPLIED或新角色启动成功；主机必须重新查询并确认设备进入Loader。
内部ack_ticket:u64不进入payload，不能截断成seq16或硬件8bit标记。

## 2. 唯一所有者与完成条件

`BootEntryService.request`由APP生命周期所有者实现。同source/boot/sequence只关联原交接，
其他身份不能取走已开始的停机事务。返回PENDING时，不分配虚构的可复位权限。
READY必须带非零u64票据，其他结果票据为0。
通信把该票据关联到具体READY回复帧；物理完成再调用`BootEntryService.tx_complete`。
仅入队、FDCAN接收、队列空或其他帧发送完成都不符合条件。

所有者收到精确票据成功事件后，再复核原始身份、截止、输出关断、worker归还与保存空闲，
才能写ENTER邮箱和复位。发送失败或过期须撤销票据，保持功率禁止，不自动重新使能。
通信不访问寄存器、跳转、Flash或复位，不维护第二份停机状态。

## 3. 生产接线与限制

`yg_protocol_boot_decode/encode_reply`是窄载荷转换；READY编码会拒绝零票据，
未知共享结果或非READY携带票据也会拒绝，不能误报已受理。
没有实际业务回调和物理TX回执时，不注册56路由或声明该能力。
APP平台当前普通try_send接口仅表示提交，需要真实tracked提交与完成事件接通。
编解码存在不等于安装了Loader，也不等于能进入Loader。

黄金载荷见`yg_protocol_boot_entry_wire_v1_vectors.json`；本轮未执行。
统一验证须覆盖高32位boot、陈旧启动身份、不同来源窃取、重复请求、READY零票据、
正确帧与错误帧完成、发送失败、截止、关相/worker/保存复核及禁止自动重启。
