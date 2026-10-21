> 历史归档（2026-10-21）：当前载荷、nonce、40B回复及APP交接合并到v3；旧容器保留为历史。 本文的“当前”指原始基线；现行入口见[当前说明](../../protocols/yg_protocol_upgrade_wire_v3.md)。归档不新增执行授权。

# 公司统一帧升级载荷 v1

历史契约：本轮简化 Loader 使用[升级载荷 v2](yg_protocol_upgrade_wire_v2.md)。
下述持久 boot、SHA 与维护角色语义不适用于 v2；保留本文和向量用于旧实现比较。

日期：2026-09-28。ROOT已批准50～55布局及分片幂等澄清。本轮只实现代码，构建与测试后续统一进行。
本定义替代历史未实现的升级候选，不支持旧YI标准ID帧，也不声称当前APP/Loader已完整接线。

## 1. 总体边界

使用公司16B头、2B CRC16、29位扩展CAN FD，PF=0xEF，仲裁1 Mbit/s、数据5 Mbit/s。
Type位于公司头6～7，小端。升级不重新定义CRC、CAN ID、分片或物理DLC规则。
来源主站2，目标单播；请求ACK_REQ或ACK_REQ|RETRY，回复RESPONSE且交换源/目标。
source_node不能255；当前五轴3～7是部署范围，不是协议地址上限。

公共载荷接口在`shared/protocol/yg_protocol_upgrade_wire.c/.h`。
数值封装显式小端，96B镜像清单是原样大端容器；不能通过C结构体memcpy生成报文。
公共编解码不依赖APP或Loader类型，不访问Flash、输出、硬件或复位。
外层先完成CAN ID/公司头地址一致性、FD/BRS、CRC、零填充和分片重组，再调用载荷解码。
载荷解码只校验字段，不能替代主站授权、镜像目标、真实性策略、租约或存储权限。

## 2. 请求布局

boot_epoch是Loader实际持久分配的非零u64启动代号；session_id是本次boot内不重复的非零u32会话。
后续请求不能省略任何一个身份。STATUS空请求只发现启动状态，不续租也不授予写权限。

| Type | 小端数值偏移与原样数据 | 总长度 |
|---|---|---|
| 50 UPDATE_BEGIN | 0～7:expected_boot:u64；8～103:manifest[96]原样大端 | 104B |
| 51 UPDATE_WRITE | 0～7:boot:u64；8～11:session:u32；12～15:offset:u32；16～17:length:u16；18起:data[length] | 26～146B |
| 52 UPDATE_FINISH | 0～7:boot:u64；8～11:session:u32 | 12B |
| 53 UPDATE_STATUS发现 | 空；不带伪造会话 | 0B |
| 53 UPDATE_STATUS会话 | 0～7:boot:u64；8～11:session:u32 | 12B |
| 54 UPDATE_ACTIVATE | 0～7:boot:u64；8～11:session:u32 | 12B |
| 55 UPDATE_ABORT | 0～7:boot:u64；8～11:session:u32 | 12B |

WRITE的offset与length均8B对齐，length为8～128B，报文长度必须恰好匹配。
校验offset+length不回绕；镜像范围、顺序、已写数据比较与存储读回由Loader所有者继续验证。
BEGIN最多三片（46+46+12B）；128B WRITE最多四片（46+46+46+8B）。
24B及更小块可单帧；不为追求单帧另造第二种WRITE布局。

## 3. 统一40B回复

所有50～55业务回复使用相同布局，失败也带当前真实状态；物理CAN FD长度为64B，尾部零填充。
回复Type与本次请求相同，seq关联本次逻辑请求/分片尾片。未知Type由产品普通错误路径处理。

| 偏移 | 字段 | 语义 |
|---|---|---|
| 0～1 | result:u16 | 0完成，1受理，其余沿用公共错误码 |
| 2～3 | detail:u16 | 本节固定原因，不发送内部枚举 |
| 4～11 | boot_epoch:u64 | 当前实际Loader启动代号，不能回显未经校验的请求值 |
| 12～15 | session_id:u32 | 当前真实会话，无会话为0 |
| 16 | phase:u8 | 下表固定阶段 |
| 17 | flags:u8 | bit0待激活；bit1完整性校验限定；其余零 |
| 18～19 | max_block_bytes:u16 | 实际8～128B上限且8B对齐；首版128 |
| 20～23 | next_offset:u32 | 连续已读回的镜像字节数，不是总线接收量 |
| 24～27 | image_bytes:u32 | 清单中镜像长度，包含打包补齐的尾部 |
| 28～31 | image_token:u32 | 镜像身份，不替代会话 |
| 32～35 | image_version:u32 | 实际镜像版本 |
| 36～39 | image_role:u32 | 1普通、2受抑制维护；无镜像0 |

phase固定：0 IDLE、1 READY、2 ERASING、3 RECEIVING、4 VERIFYING、5 VERIFIED、6 COMMITTED、7 ERROR。
产品适配逐项映射自己的枚举，不能因为当前数值相同就直接转换或复制整个状态对象。
bit0仅在VERIFIED等待精确激活ACK时设置；首版始终设置bit1，SHA不表示签名认证。
next_offset/image_bytes必须8B对齐，next_offset不能大于image_bytes。
启动结果与持久确认由真实启动记录及APP身份查询提供，40B升级状态不等于APP已成功运行。

| result / detail | 固定含义 |
|---|---|
| 4 / 1 | 字段非法 |
| 5 / 2 | 启动代号不匹配 |
| 5 / 3 | 会话不匹配 |
| 5 / 4 | 当前阶段不允许操作 |
| 5 / 5 | 产品、硬件、布局或镜像身份不匹配 |
| 11 / 6 | 范围或顺序越界 |
| 8 / 7 | Flash编程、读回或记录提交失败 |
| 4 / 8 | 相同offset但数据不同 |
| 8 / 9 | 整包摘要、向量或内嵌描述符校验失败 |
| 2 / 10 | 不支持的真实性策略 |
| 5 / 11 | 功率输出未确认关闭或链路不可用 |
| 5 / 12 | 租约或激活回复完成时限已到 |
| 8 / 13 | 内部故障或无法获得真实状态 |

长度错误result3；未授权主站result7；忙result6；不支持业务result2/detail0。
适配器若无法从内部失败判断具体原因，返回诚实的通用拒绝/失败，不伪造Flash或摘要成功。
codec允许已定义公共结果0/1/2/3/4/5/6/7/8/11/17；未定义结果或保留标志拒绝编码。

## 4. 身份、重试与分片

公司12.1分片首seq=0，后续递增，逻辑消息输出seq等于尾片序号。
所以每个146B WRITE都可能为seq3；不同offset使用相同尾片seq合法。
升级不能沿用未分片参数的(source,seq)全局去重，不能增加隐藏request_id或改变分片ABI。

主机一次只允许一个完整升级逻辑请求在途，不交错不同块的分片。
整条重传使用原manifest或原offset/data，先完成一条回复再推进下一块。
幂等身份按实际业务定义：

- BEGIN：source、boot、完整manifest；同活动身份重复只返回原会话，不再次擦除。
- WRITE：source、boot、session、offset并精确比较data；同内容重试不重复编程，不同内容拒绝。
- FINISH/ACTIVATE/ABORT：source、boot、session、operation；重复不得重启校验、分配新ACK票据或延长激活时限。

主机依据boot/session/next_offset/phase识别结果，不能仅凭seq判断写入完成。
分片重复比较及超时由公共重组器负责，业务仅在整条重组成功后执行一次。
参数120～122的未分片序号去重规则不因此改变。

## 5. 完成语义与安全边界

BEGIN建立事务后回复ACCEPTED；整槽擦除由前台Poll有界推进，RECEIVING后才允许WRITE。
WRITE只有全部编程且读回一致才返回OK，next_offset仅包含持久确认字节。
FINISH启动完整摘要校验回复ACCEPTED；最终VERIFIED由STATUS确认，不能提前ACTIVATE。
ACTIVATE受理回复ACCEPTED，不表示已持久提交、复位或新APP成功。

Loader所有者给ACTIVATE分配非零u64内部ACK票据。产品发送端把完整票据关联到该具体回复帧；
ISR只收集真实发送事件，前台完成回调再推进提交和START。
入队成功、FDCAN接受、TX FIFO空、其他回复发送完成都不能替代该票据的物理成功事件。
重试不更换票据、不延长截止；发送失败、链路丢失、输出未关闭或超时不能复位。
ACK票据不放入payload，不截断为seq16、session32或8bit硬件标记。

镜像单槽失败留在Loader救援，不承诺回滚旧镜像。参数RAW11区不参与升级写入。
image_bytes包含打包阶段按8B补齐的尾部，SHA覆盖含padding的整份镜像。
未按该规则打包的镜像拒绝；设备不能自行补齐并改变摘要对象。
96B清单的保留/真实性策略按镜像契约校验，未知策略拒绝。SHA只检测完整性，不认证发布者。

56 ENTER_BOOT暂不注册：待APP/Loader实际handoff值接口、输出静默证明及精确回复完成接入后定义。
公共编解码存在不是升级能力；GET_CAPS只能在产品路由、存储、完成事件与启动路径实际接通后声明。

## 6. 接口与统一验证交接

`YgProtocolUpgrade_DecodeRequest`失败不改输出，成功借用message载荷；BEGIN/WRITE核心必须在返回前复制或消费。
`YgProtocolUpgrade_EncodeReply`显式编码40B；阶段/标志/长度非法时不写缓冲或written。
无队列、HAL、Flash、租约定时器或APP状态依赖，不增加大静态缓冲。
D拥有Loader路由、主站授权、枚举映射、具体TX票据及实际存储事务；ROOT/E拥有APP交接。

黄金payload见`yg_protocol_upgrade_wire_v1_vectors.json`。黄金向量用于后续统一测试，不代表当前已执行。
统一验证覆盖：boot高32位保持、错误长度/零身份/越界/对齐、原样BE清单、分片尾seq复用、
同offset异内容、旧boot/session拒绝、SHA含padding、单槽失败救援、精确票据/失败/截止与START禁止。
分别构建APP与Loader后检查共用源码唯一性、资源预算、链接布局及参数保护。
