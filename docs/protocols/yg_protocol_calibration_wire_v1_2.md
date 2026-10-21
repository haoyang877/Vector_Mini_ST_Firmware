# 当前单项维护载荷v1.2

核对日期：2026-10-21。此页集中维护请求、结果和已支持kind，不必从旧v1/v1.1提案拼接。
承载为公司16B头/CRC、扩展CAN FD、1M/5M；主站2单播单帧，ACK_REQ或ACK_REQ|RETRY。
source/sequence来自统一头，响应同Type/seq并交换地址。

| Type | 4B请求，小端 | 业务 |
| --- | --- | --- |
| 160 START | kind:u16，reserved=0:u16 | 启动一项任务 |
| 161 STATUS | 非零ticket:u32 | 查询原始任务 |
| 162 CANCEL | 非零ticket:u32 | 仅取消原始任务 |

| kind | 当前支持 |
| ---: | --- |
| 1/2 | LUT / 电角零位 |
| 3/4/5 | 齿槽/相电阻/摩擦已撤回，UNSUPPORTED/detail1 |
| 6/7/8 | 机械零位 / 维护SAVE / DEFAULTS |

kind0非法，未知非零kind不支持；线路kind不能强转内部枚举。
没有163分页结果、整套标定或隐式补跑前置任务。

## 所有合法业务响应为12B

| 偏移 | 字段 | 解释 |
| ---: | --- | --- |
| 0/2 | result/detail:u16 | 业务结果与原因 |
| 4 | ticket:u32 | 原任务身份，无关联时0 |
| 8 | kind:u16 | 线路任务种类，无关联时0 |
| 10 | phase:u8 | 0 NONE、1 IN_PROGRESS、2 COMPLETED、3 CANCEL_REQUESTED、4 CANCELLED、5 FAILED |
| 11 | effects:u8 | 0 UNKNOWN、1 UNCOMMITTED、2 COMMITTED |

START通常result1/phase1/非零ticket；STATUS result0表示查询成功，可同时phase5。
标定完成而effects1表示RAM结果未确认持久化；只有持久所有者确认才可effects2。
CANCEL可先返回result1/phase3，实际归还前不得报告phase4或开放工作区。
已结束ticket的取消不创建任务或恢复目标。未知ticket拒绝，无关联字段清零，不伪造值。

| result/detail | 原因 |
| --- | --- |
| 2/1 | 不支持kind |
| 4/2 | 无效/陈旧ticket |
| 5/3 | 状态或标定准入不允许 |
| 4/4 | 同source/seq但请求内容不同 |
| 4/5 | 陈旧START序号 |
| 4/6 | 字段或保留值非法 |
| 8/7 | 无更细原因的任务失败 |

长度3、忙6、未授权7、故障17；未绑定2/detail0。
START按source/seq/kind幂等；取消由任务唯一所有者核对身份并登记，不能先查ticket再调用无关全局STOP。
复位后旧ticket无效。通信不拥有第二份任务状态、工作区或功率控制。
维护SAVE的ticket不同于Type122参数revision。具体保存和取消边界见[标定指南](../guides/calibration_process.md)。
[v1.2向量](yg_protocol_calibration_wire_v1_2_vectors.json)覆盖当前范围；
v1/v1.1向量保留为兼容与退役kind拒绝参考，不声明旧任务可用。
