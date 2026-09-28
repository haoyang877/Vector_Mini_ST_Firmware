# 参数与清故障线路接入 v1

日期：2026-09-28。基于参数目录v0.1，由本轮整体重构请求及ROOT裁决收口。
这是本轮新增线上定义及代码接入，不能把历史候选写成已经可用的业务。
本轮不测试、不构建；APP所有者接线与最终统一验证仍须完成。

## 1. 不变边界

公司16B头、2B CRC、小端、PF=0xEF、扩展CAN FD、1M/5M与101/108/110/116/124不变。
请求必须为单播，主控源地址2，ACK_REQ或ACK_REQ|RETRY；回复同Type同seq，交换源/目标。
参数ID只占payload一个字节，绝不能按本地C结构体的u16字段直接发送。
SN/BYTES没有真实源，当前只接标量i32/u32，明确UNSUPPORTED，不填造字符串或身份。

## 2. 107 CLEAR_FAULT

本轮ROOT冻结：请求为空，不含掩码。请求清除全部已恢复且允许复位的锁存故障，
回复沿用4B `result:u16, detail:u16`。实时故障未恢复必须拒绝或如实留存，剩余故障
由108/124查询。CLEAR不启动、不使能、不恢复旧目标，也不替代STOP。
通信仅提交值请求；故障源/恢复/取消旧请求及完成判定均由E所有者负责。

101 enable=1与107经adapter的`lifecycle_handler/lifecycle_context`注入APP映射函数。
未绑定返回UNSUPPORTED，不能直接操作PWM或假造成功；STOP/失能沿原停机服务。

## 3. 参数请求逐字节布局

| Type | 偏移与长度 | 解释 |
| --- | --- | --- |
| 120 READ | 0:id:u8；1～3:0 | 恰好4B；读取活动值，不读取Flash原始记录 |
| 121 WRITE | 0:id:u8；1:value_type:u8；2:value_len:u8；3:0；4～7:expected_revision:u32；8～11:value[4] | 恰好12B；type=1 i32或2 u32，len=4；仅修改活动值 |
| 122 SAVE | 0～3:expected_revision:u32 | 恰好4B；不隐含改参数或擦写动作 |

source_node/sequence来自公司头，由协议复制到内部请求，不能再在payload添加会话前缀。
内部value:i32保存原始32位位模式，u32值不能先作有符号范围裁剪再发送；所有者按type
解释并做单位/产品范围校验。本地结构体不参与线路序列化。

## 4. 回复

| 请求及结果 | 逐字节payload | 长度 |
| --- | --- | --- |
| READ成功 | 0～1:result=0；2～3:detail；4:id；5:真实value_type；6:4；7:实际access；8～11:活动revision；12～15:value[4] | 16B |
| WRITE完成 | 0～1:result=0；2～3:detail；4:id；5:实际value_type；6:4；7:0；8～11:新活动revision；12～15:实际应用值 | 16B |
| WRITE仅受理 | result=1:u16；detail:u16 | 4B；不能虚构新revision/已应用值 |
| SAVE仅受理/完成 | result:u16；detail:u16；已确认persisted_revision:u32 | 8B；result=1不表示Flash完成 |
| 任意明确拒绝/失败 | result:u16；detail:u16 | 4B；无伪造值或版本 |

access bit0=可读、bit1=当前可写、bit2=标定所有者写入；其余位必须零。
READ由所有者返回实际类型/权限，不根据候选目录推断已经实现写功能。
WRITE返回type必须与请求相同。READ不得以ACCEPTED冒充有值快照。
SAVE的OK只在实际持久记录写入并校验提交完成后产生。通信调用栈不擦Flash，不忙等。

## 5. 错误和修订语义

沿用线路结果0/1/2/3/4/5/6/7/8/11/17。参数特定错误使用固定detail原因：

| result | detail | 含义 |
| --- | --- | --- |
| 2 | 1 | UNSUPPORTED_ID |
| 5 | 2 | READ_ONLY |
| 4 | 3 | REVISION_CONFLICT |
| 8 | 4 | PERSIST_FAILED |
| 5 | 5 | WRONG_STATE |
| 4 | 6 | BAD_FIELD，包括类型/保留字节错误 |
| 4 | 7 | STALE_SEQUENCE |
| 4 | 8 | CONFLICTING_RETRY，同序号但操作/值不同 |

长度错误result=3，忙result=6，未授权主站result=7，范围错误result=11，故障result=17。
没有绑定该业务的UNSUPPORTED使用result=2/detail=0，和未知参数区分。

活动revision由E参数所有者单独持有，不是schema11。首次新WRITE完成后恰好递增一次；
验证失败/仅受理不递增；同seq重试不再次应用、不增加revision。
同(source,seq)还必须比较operation、id、type、value位模式和expected_revision，内容不同
必须拒绝。READ不改revision。新seq携带旧revision返回冲突，不能绕过并发校验。
计数将回绕时拒绝新变更，不允许自然回绕让旧请求重新有效。
SAVE重复请求关联同一持久作业；同序号可查询完成结果，不得重新擦写。
STOP撤销/故障取消后不能把旧作业结果充当新请求完成；此状态由所有者掌握。

## 6. 模块接口与范围

`yg_protocol_parameter_decode/encode_reply`负责字段；link绑定
`yg_protocol_parameter_service_t`，后台串行调用handler，不保留输入payload指针。
内部request携带源节点和序号，reply携带值、revision及value_type/access。
旧只读adapter仅在成功时提供真实类型和READ权限；WRITE/SAVE不假装已经实现。

黄金payload见`yg_protocol_parameter_wire_v1_vectors.json`。统一验证以后覆盖：字段非法、
u32高位保持、主站拒绝、未绑定、同seq重传冲突、版本冲突、停止取消和Flash保存失败。
参数ID/缩放以参数目录为准；当前无真实实现的ID明确不支持，不增加新模式或参数ABI。
