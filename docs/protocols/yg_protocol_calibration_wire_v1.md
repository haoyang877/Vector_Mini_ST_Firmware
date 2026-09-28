# 标定任务通信载荷 v1

日期：2026-09-28。ROOT批准最小160～162布局；本轮代码实现，不构建、不测试。
编号与线上字段不沿用旧Mode或内部枚举，不保留未使用的READ_RESULT/opaque数据入口。
163分页结果暂不注册；已定义标量结果通过120读取，保存通过122。

## 1. 不变边界与职责

公司16B头/CRC、CAN FD扩展ID、1M/5M、小端不变。
所有任务请求单帧、主站2单播、ACK_REQ或ACK_REQ|RETRY；回复同Type同seq、交换源/目标。
通信只解析、转发与编码，APP维护所有者控制准入、原始ticket、取消、结果和资源归还。
未绑定返回UNSUPPORTED，不把通信路由或候选kind存在当作标定能力。
通信不直接写模式、目标、标定有效位或Flash，不隐式开放其他功率模式。

## 2. 任务输入

| Type | 4B请求，小端 | 功能 |
|---|---|---|
| 160 CALIBRATION_START | 0～1:kind:u16；2～3:reserved=0 | 选择一项明确标定任务 |
| 161 CALIBRATION_STATUS | 0～3:ticket:u32非零 | 查询原始任务 |
| 162 CALIBRATION_CANCEL | 0～3:ticket:u32非零 | 只取消该原始任务 |

kind固定：1 ENCODER_LUT、2 ELECTRICAL_ZERO、3 COGGING。
0非法；其他非零kind解析后由业务明确UNSUPPORTED，不映射未知模式。
首版输入参数就是任务kind，算法使用已验证的产品配置；不接收无单位/范围的任意功率参数。
后续如确需外部算法参数，须另冻结字段、范围、单位及版本，不追加隐式尾字节。
source和sequence来自公司头，不再添加会话前缀。

APP显式映射：wire1→MOTOR_MAINTENANCE_ENCODER_LUT，wire2→ELECTRICAL_ZERO，wire3→COGGING。
内部SAVE及其他维护项不因此开放。不能强制转换MotorMaintenanceKind或ModeNow_TypeDef。

## 3. 所有合法业务请求的12B回复

| 偏移 | 字段 | 含义 |
|---|---|---|
| 0～1 | result:u16 | 公共0完成、1受理及2/3/4/5/6/7/8/11/17错误 |
| 2～3 | detail:u16 | 下节任务原因 |
| 4～7 | ticket:u32 | 真实原始任务身份，无任务为0 |
| 8～9 | kind:u16 | 本节固定编号，无任务为0 |
| 10 | phase:u8 | 固定阶段 |
| 11 | effects:u8 | 是否已持久提交，不与任务阶段混用 |

phase：0 NONE、1 IN_PROGRESS、2 COMPLETED、3 CANCEL_REQUESTED、4 CANCELLED、5 FAILED。
不提供没有真实来源的百分比进度。phase1只表示仍进行，包含受理后的排队和算法运行。
effects：0 UNKNOWN、1 UNCOMMITTED、2 COMMITTED。标定活动值完成与参数持久化必须分别确认。
effects2仅在持久所有者确认该结果已保存时产生，不能根据Mode切到Save_Param猜测。

START通常result1/phase1、非零ticket与实际kind；重试返回同一任务当时的真实进展。
STATUS查询成功result0，可同时phase5：查询成功不等于标定成功。
CANCEL仅提出取消时result1/phase3；资源归还前不能报phase4或开放新任务。
已结束的同ticket取消可返回原终态，不创建任务、不恢复目标。
未知或陈旧ticket拒绝，无关联任务时ticket/kind/phase/effects全0。
错误也保持12B、无伪造值；公司头非法/无法路由的帧由已有公共错误路径处理。

## 4. 错误、重试与身份

| result / detail | 含义 |
|---|---|
| 2 / 1 | 不支持task_kind |
| 4 / 2 | 无效或陈旧ticket |
| 5 / 3 | 运行状态或标定准入不允许 |
| 4 / 4 | START同source/seq但kind等请求内容不同 |
| 4 / 5 | 陈旧START序号 |
| 4 / 6 | 字段/保留值非法 |
| 8 / 7 | 任务执行失败且没有更细原因 |

长度错误3；忙6；未授权主站7；未绑定2/detail0；业务故障17。
任务最终失败也可在STATUS的phase5及真实原因中体现，不强行清错或再启动。

START所有者按完整source/seq/kind保留原始请求与ticket；同内容重传返回原ticket。
不能在任务结束后收到重发又创建第二个任务，同身份不同内容须拒绝。
只有原ticket可查询/取消，复位后旧ticket无效；输出与资源状态由维护所有者复核。
CANCEL在核对ticket到登记取消之间必须由同一所有者原子处理，不能先查后调全局STOP，
否则旧任务结束与新任务创建之间的竞态会错误停止新任务。
通信不维护第二份标定状态或大工作区，也不缓存ACCEPTED充当最终成功。

## 5. 接口与后续统一验证

`yg_protocol_job_decode`生成无指针值请求，失败不改输出。
`yg_protocol_job_reply_t`为专用status/token/kind/detail/phase/effects结果，
不重载通用参数reply的revision/value。`yg_protocol_job_encode_reply`显式编码12B，
受理/成功缺少真实ticket/kind/phase时拒绝，link转换为内部失败，不能伪报成功。
`YgProtocolLink_BindJobService`只复制handler/context，由APP组合层在初始化后绑定。
E负责APP任务映射、START幂等、ticket取消和真实进展；H负责codec/路由，不跨层控制硬件。

这是内部job回调类型的收口，旧contract测试的job_handler及job reply变量须在统一测试前
改为专用reply，不保留旧泛型包装。APP无旧生产job调用者；其他服务reply不变。
黄金载荷见`yg_protocol_calibration_wire_v1_vectors.json`，本轮未执行。
最终覆盖重复START原ticket、同seq异kind、未知kind、停止/故障取消、陈旧ticket、
取消资源未归还、查询失败终态、活动完成未保存与真实COMMITTED，不在测试中默认允许功率输出。
