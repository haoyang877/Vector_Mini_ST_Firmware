# 当前公司升级线路、交接与镜像v3

核对日期：2026-10-21。依据shared目标、upgrade codec、Loader与APP启动实现。
本页合并早期升级载荷/交接页中仍有效的字段，当前包不再依赖阅读旧容器规范。
公司帧版本1，扩展CAN FD+BRS、1M/5M、主站2单播；请求ACK_REQ或ACK_REQ|RETRY。
外层数值小端，96B镜像清单/描述符是大端容器，不能memcpy本地结构体。

## 目标、包与镜像

| 项目 | 当前G431目标 |
| --- | --- |
| product/hardware | 0x59473236 / 1，编译目标声明 |
| layout | 6，Loader16KiB；APP基址0x08004000、容量96KiB |
| parameter ABI/schema | 0x000E08F0 / 14；记录2288B，兼容读取12/13 |
| APP/Loader版本声明 | 1.5.7 / 2.1.0；实际以匹配源码生成的shared清单/描述符为准 |
| image profile | 0x53593549，descriptor版本3、96B |
| activation/attempt/confirm偏移 | 64..71 / 72..79 / 80..87 |
| 持久标记 | ACT3=0x41435433、TRY3=0x54525933、OKA3=0x4F4B4133；各后4B为反码 |

版本声明不是发布或新实机PASS。清单96B：CRC在48..51、52..95为0；
描述符52..63及88..95为0，包内64..87为FF。CRC32使用反射EDB88320、
初值/末异或FFFFFFFF，覆盖8B补齐镜像；描述符48..51归零、64..87归FF计算。
清单/描述符身份、长度、CRC及入口范围一致才可接受；仅普通APP角色1。
v2容器、旧SHA profile、旧布局不能线上混刷。打包入口为 `loader/tools/package_image.py`。
当前布局黄金输入见[layout6夹具](../../tests/fixtures/loader_image_layout6.json)；
旧layout5/v2夹具保留作历史互验和拒绝测试，不能直接部署。

## 50～55请求与身份

历史字段名boot_epoch/expected_boot在线上保留，但含义是主机随机非零u64事务nonce，
不是设备持久启动次数。每次新事务、复位或过期后使用新nonce，同事务重试保持。
发现回复boot=0合法，不能拿0发BEGIN，也不能用节点号/固定1伪造nonce。

| Type | 请求，小端数值 | 总长 |
| --- | --- | --- |
| 50 BEGIN | nonce:u64 + manifest[96]原样大端 | 104B |
| 51 WRITE | nonce:u64 + session:u32 + offset:u32 + length:u16 + data | 18+length，26～146B |
| 52 FINISH | nonce:u64 + session:u32 | 12B |
| 53 STATUS | 空发现，或nonce:u64+session:u32查询事务 | 0/12B |
| 54 ACTIVATE | nonce:u64 + session:u32 | 12B |
| 55 ABORT | nonce:u64 + session:u32 | 12B |

WRITE长度8～128B，offset/length均8B对齐且不溢出；连续顺序写入并精确读回。
BEGIN最多3片，128B WRITE最多4片；分片规则见[公共重组契约](yg_protocol_fragment_v0_2.md)。
主机保持每次一个完整逻辑请求在途；分片尾seq可在不同块重复，不能作为升级去重身份。
BEGIN按source/nonce/完整manifest幂等，WRITE再匹配session/offset/完整data；
重复FINISH/ACTIVATE/ABORT不重新分配提交票据或延长绝对时限。

## 统一40B回复

50～55合法业务回复同Type/关联seq，交换src/dst。物理CAN FD64B，尾部补零不计应用CRC。

| 偏移 | 字段 |
| ---: | --- |
| 0/2 | result/detail:u16 |
| 4 | nonce:u64，无会话发现可0 |
| 12 | session:u32，无会话0 |
| 16/17 | phase:u8 / flags:u8 |
| 18 | max_block_bytes:u16，当前128 |
| 20/24 | next_offset/image_bytes:u32，8B对齐；next不超过length |
| 28/32/36 | image_token/image_version/image_role:u32；无镜像0 |

phase：IDLE0、READY1、ERASING2、RECEIVING3、VERIFYING4、VERIFIED5、COMMITTED6、ERROR7。
flags bit0待激活、bit1 INTEGRITY_ONLY、bit2 HOST_NONCE，其余0；当前通常0x06或0x07。
40B响应不表示APP已启动或健康确认。`next_offset`只统计已持久写入并读回字节。

result使用0完成、1受理、2不支持、3长度、4字段、5拒绝、6忙、7未授权、8失败、11范围、17故障。
detail固定：1字段、2nonce不匹配、3session、4阶段、5目标身份、6范围/顺序、
7Flash/读回、8相同offset数据冲突、9CRC/向量/描述符、10真实性策略不支持、
11输出/链路、12超时、13内部失败。result/detail配对以codec和产品映射为准，
不可把内部枚举直接发出或把失败补成成功。

空STATUS是只读发现，也让Loader驻留救援；不擦除、不建session。
GET_INFO本身不关闭50ms自动跳转窗口。会话STATUS可续5s租约，空发现不续租。
FINISH先受理，由前台推进整包校验，STATUS见VERIFIED后才ACTIVATE。
ACTIVATE的具体响应关联完整u64内部TX票据，物理成功后复核整包/关相/链路，
才持久激活并复位；入队、FIFO空或其他帧完成不能替代。完成期限500ms。

## APP进入Loader：Type56

仅安装匹配Loader的配套APP开放。请求8B主机nonce，响应4B result/detail：
不可用2/0、等待停机1/1、READY1/0、拒绝5/0、失败8/0；其他长度/字段/权限错误沿公共码。
Type3当前没有持久epoch，不能用其0值授权56。
APP在停机、资源归还、保存空闲和数字关相满足后，将完整u64票据绑定具体READY响应。
只有真实发送完成且身份/时限/静默复核通过才写一次性ENTER邮箱并复位；失败保持停机。
READY排队与平台发送使用有界绝对时限，重试不换票据；不能因队列进展刷新时限。
主机随后重新发现同UID的Loader，不能把READY当作角色切换成功。

## GET_INFO与确认

Type1请求12B小端：session:u32、request_id:u32、page:u16、reserved=0:u16。
每页32B，无前置result；APP与Loader支持page0..2。

| 页 | 布局 |
| --- | --- |
| 0 | 0 UID[12]；12 product:u32；16 hardware:u16；18 boot_api:u16；20 app_version:u32；24 loader_version:u32；28 boot_id:u32=0 |
| 1 | 0 SN[16]；16 motor_type:u32；20 identity_valid:u8；21 node:u8；22 role:u8（APP1/Loader2）；23 boot_state:u8（未知0/已确认1/试运行2/Loader3）；24 image_crc32:u32；28 profile:u32 |
| 2 | 八个u32：MCU、layout、parameter ABI、schema、APP基址、APP容量、Loader容量、身份页基址 |

配套boot_api=3，独立APP为0且Loader版本为0；Loader仅对完整已激活APP报告版本/CRC。
版本major:u16/minor:u8/patch:u8，高位到低位；实际值来自shared目标与镜像。
SN独立生产页，不等于UID，不存在普通CAN写SN命令。

Type57 APP_CONFIRM：主站2单播ACK，8B version:u32+image_crc32:u32，4B result/detail响应。
精确匹配当前APP且连续健康至少1s、数字关相、保存空闲后一次写确认；相同身份可幂等重试。
试运行5s内不确认则停止喂IWDG，复位后留Loader；5s不是物理复位完成时长。
主机须再读同UID/page1的版本、CRC、APP角色和boot_state1才报告升级成功。
Type57不使能电机，不使用Type56交接TX票据。

## 兼容、失败与证据

单槽升级：擦除后没有旧镜像自动回滚，无跨复位续传。
完整传输未激活不允许冷启动，TRY只消费一次；已尝试未确认的镜像复位后驻留。
参数与身份页不在升级写范围；参数单份保存的掉电限制见[schema14](parameter_schema14_storage_v1.md)。
布局/ABI不兼容时首次安装需明确受控配套流程，不猜测跨版本迁移。
CRC/UID防误刷不提供签名认证或恶意重放防护。
[历史v3向量](yg_protocol_upgrade_wire_v3_vectors.json)和[当前目标夹具](../../tests/fixtures/loader_image_layout6.json)
有各自基线，向量不是实机通过证据。设备结果见[报告索引](../reports/README.md)。
