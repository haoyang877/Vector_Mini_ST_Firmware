# 公司升级线路与镜像 v3

日期：2026-09-29。公司帧版本仍为1，ISO扩展CAN FD/BRS，仲裁1Mbit/s、数据5Mbit/s，主站2。
50..56的请求/回复布局、nonce绑定、分片与结果码继承[升级载荷v2](yg_protocol_upgrade_wire_v2.md)，
但容器换成v3，不能发送v2镜像。Type57是本项目评审分配，部署前公司消息表须同步。

## 新镜像契约

| 项目 | 值 |
| --- | --- |
| profile | 0x53593549 |
| descriptor version/bytes | 3 / 96 |
| layout | 5；Flash/RAM分区未移动 |
| parameter ABI/schema | 0x000C10D0 / 12；4304B原记录不变 |
| manifest | 96B大端，CRC仍在48..51，其后全0 |
| descriptor | 原0..63身份字段保留，52..63及88..95全0 |
| activation / attempt / confirm | 64..71 / 72..79 / 80..87，下载包全FF |
| marker values | ACT3=0x41435433、TRY3=0x54525933、OKA3=0x4F4B4133；后4B为反码 |

CRC32：EDB88320反射、初值/末异或FFFFFFFF，覆盖8B对齐的完整镜像。描述符48..51归一化0，
64..87归一化FF。v2和旧SHA profile都拒绝。APP入口必须位于描述符之后的实际镜像范围。
单槽激活门槛、尝试消费与确认规则见[当前设计](../architecture/loader/loader_product_v3.md)。
50..55回复flags仍为0x06或0x07；不能仅靠flags推断v3，必须检查GET_INFO的boot_api/profile/layout。

空 Type 53 STATUS 是只读发现请求，同时让 Loader 留在救援状态，关闭自动跳转。
它不擦除、不创建 session；GET_INFO 本身不关闭 50ms 启动窗口。救援客户端可在复位前
反复发空 STATUS 捕获该窗口，随后严格检查实际 UID、配套版本和几何才发 BEGIN。

## GET_INFO（Type1）

请求仍为12B小端：session:u32、request_id:u32、page:u16、reserved:u16=0；必须单播并要求ACK。
APP与Loader支持page0..2，每页32B，无前置结果码。UID始终来自芯片，SN另存生产记录。

| 页 | 偏移与内容 |
| --- | --- |
| 0 | 0 UID[12]；12 product:u32；16 hardware:u16；18 boot_api:u16=3；20 app_version:u32；24 loader_version:u32；28 boot_id:u32=0 |
| 1 | 0 SN[16]；16 motor_type:u32；20 identity_valid:u8；21 CAN ID:u8；22 role:u8（APP1/Loader2）；23 boot_state:u8（未知0/已确认1/试运行2/Loader3）；24 image_crc32:u32；28 profile:u32 |
| 2 | 八个u32：MCU、layout、parameter ABI、schema、APP基址、APP容量、Loader容量、生产身份页基址 |

Loader只对完整且已激活镜像报告APP版本/CRC，否则为0。独立APP的boot_api和Loader版本为0，
不可经56升级；page1启动状态0。page1/profile和page2几何都是维护契约，普通GET_CAPS仍仅page0。
版本编码为major:u16/minor:u8/patch:u8（最高到最低位），1.0.0=0x00010000，来源是共同目标清单。
boot_id=0及旧Type3缺少持久epoch的UNAVAILABLE语义保留；nonce不是可信设备启动次数。

## APP_CONFIRM（Type57）

请求8B小端：version:u32、image_crc32:u32。仅主站2单播ACK请求，不支持分片或广播。
回复4B：result:u16、detail:u16；成功0，字段/长度错误4/3，未满足健康/关相/身份条件5，
未安装确认服务2或路由不支持。Type57不使用Type56的内部TX票据，不触发复位或电机使能。

APP需监督进展与本地通信健康连续观察至少1s、保持数字关相与参数保存空闲，才能一次写确认。
版本与CRC必须精确匹配当前APP；已确认的相同身份重复请求成功，但错误身份仍拒绝。
试运行5s内未确认停止喂IWDG；看门狗复位留Loader。客户端收到成功回复后必须再查询page1，
只有相同UID、APP角色、版本/CRC正确且boot_state=1才能报告升级成功。

## 黄金向量与兼容

- [v3升级向量](yg_protocol_upgrade_wire_v3_vectors.json)：BEGIN内清单与v3镜像完全一致。
- [v3镜像向量](../../tests/fixtures/loader_image_v3.json)：合成字节，不能烧录或当作实机证据。
- 旧[v2镜像向量](../../tests/fixtures/loader_image_v2.json)保留用于兼容历史和拒绝测试。
- SN/电机类型没有新增CAN写命令或参数ID；生产SWD写独立页，禁止用主站ID当作生产认证。
- 无签名认证；CRC与UID/目标检查用于完整性和防误刷，不能抵御拥有总线或SWD权限的攻击者。
