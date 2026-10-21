> 历史归档（2026-10-21）：当前载荷、nonce、40B回复及APP交接合并到v3；旧容器保留为历史。 本文的“当前”指原始基线；现行入口见[当前说明](../../protocols/yg_protocol_upgrade_wire_v3.md)。归档不新增执行授权。

# APP进入Loader载荷 v1

日期：2026-09-28。使用D固定提交`1109361d`的`shared/boot/loader_entry.h`，不复制一套交接类型。
本轮不构建、不测试。codec与link发送关联已实现；生产需接业务所有者与真实平台TX完成。

## 1. 线路布局

公司16B头/CRC、CAN FD扩展ID、1M/5M不变。56仅主站2单播，ACK_REQ或ACK_REQ|RETRY。
请求恰好8B：expected_boot_epoch:u64小端且非零；source与sequence来自公司头。
这是当前Loader交给APP的真实启动代号，不能使用APP本地计数或公司GET_INFO的boot_id:u32补造。
APP主机先通过[Type3发现接口](yg_protocol_device_status_wire_v1.md)读取完整真实epoch，再发56。
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
APP平台普通try_send接口仅表示提交；link使用ROOT提供的tracked提交与完成事件。
编解码存在不等于安装了Loader，也不等于能进入Loader。

`YgProtocolLink_BindBootService`只在RX启动前或停止RX且清空队列时调用。
启动顺序固定为Runtime_Init（纯软件）→APP绑定全部业务/快照→Runtime_Start（既有平台开启RX）。
未Start时Runtime_Service不处理收发或bus-off；重复Start幂等，不重配或清队列。
两回调完整且没有在途票据/待发回复才开放最后一条56路由；NULL撤销，缺一项拒绝。
描述表包含56不等于业务路由启用，未绑定时不处理该业务或声明能力。
READY回复入队时记录其精确tail位置；仅到对应head才调用tracked发送。
完整u64票据只保留一份旁记录，不给40帧普通队列每条扩大字段。
入队异常回传失败；提交成功转IN_FLIGHT，前台消费匹配票据的真实完成事件。
旧票据与其他帧的事件忽略；重复56忙6，不替换原票据或延长时限。
READY起100ms为绝对排队期限，未入队或平台一直拒绝提交时均会终止，不因队列进展续长。
PREPARED超时只撤销该56待发视图；QUEUED超时核对精确槽及READY字段后移除该帧，
最多移动39条后续普通帧，保持其顺序，不清空队列或修改普通丢帧计数。
随后向原所有者回传false一次，释放旁记录，不晚发过期READY。
IN_FLIGHT不走软件排队删除；持续消费平台终态，确保已提交票据被真实完成/失败释放。
有在途交接时Init或换绑定拒绝，不能清掉待完成身份。
平台须产生真实成功/失败/取消事件，APP所有者在最终完成时复核时限和静默资格。
ROOT平台实现固定在`6592f64f`：保存完整u64票据并读取确切硬件槽完成/取消位，
100ms未完成、取消或bus-off报告失败；完成记录消费前不复用发送槽，失败后待硬件撤销。
H复用该提交的三个platform文件，不另写硬件实现。Service每次先消费完成再处理收发，
即使APP所有者已超时也必须消费平台终态，避免悬挂记录阻止后续发送。

黄金载荷见`yg_protocol_boot_entry_wire_v1_vectors.json`；本轮未执行。
统一验证须覆盖高32位boot、陈旧启动身份、不同来源窃取、重复请求、READY零票据、
正确帧与错误帧完成、发送失败、截止、关相/worker/保存复核及禁止自动重启。

最终统一测试前，APP工程加`yg_protocol_boot.c`与`shared/boot`包含目录；
link native脚本加该C，平台stub按真实tracked接口补完成事件，不把send返回true当物理成功。
