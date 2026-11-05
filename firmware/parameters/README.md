# 参数配置与记录

本目录提供配置值操作，不持有生命周期或电机运行状态。

| 文件 | 职责 |
| --- | --- |
| parameter_record.c/h | 编译期默认值装配、schema14 记录装载/导出和 CRC32 |
| parameter_values.c/h | 标量类型、权限、范围、缩放及配置合法性校验 |
| motor_axis_profile.c/h | AXS 轴记录创建、校验和关节控制包络解析 |

默认输入统一在 `firmware/config`；活动 `MotorConfiguration` 由 MotorAxis 持有。
启动时 MotorAxis 从 BSP Flash 读取记录并交给 Param_Download；其他版本或无效记录回退默认值。
schema14 固定为 2288 字节，字段布局和保留槽不因源码整理变动。

当前 APP 接入启动装载和配置校验；Keil调试命令已删除，没有在线修订号或保存队列。
MotorAxis保留停机保存实现，复用启动记录暂存区导出完整记录，实际写入与回读由BSP完成；
当前APP没有对外保存请求入口。
标量读写仍用于独立契约测试；通信参数命令未绑定服务，返回 UNSUPPORTED。
BspFlash_Write 负责固定区域的硬件写入，本目录不直接擦写 Flash。

见[参数所有权](../../docs/architecture/parameter_ownership_v1.md)。

<a id="storage"></a>
## 存储与兼容

当前 APP 只装载 schema14、2288 B 记录，参数 ABI 为 0x000E08F0。
节点是记录偏移 0 的 node_id，只接受 1～254 的有限精确整数；AT32F456 使用产品节点与 AXS2 修订 2 角色双向核对；其他板沿用独立地址。
节点与运行速度包络独立；AXS 只能进一步收窄应用限值，不能增加权限。

记录前 2240 B 为基本配置、编码器标定和轴配置，最后 48 B 为热保护/速度前馈扩展。
配置确认位于偏移 2136，CRC 位于 2140，schema 位于 2160，识别字位于 2164；
记录字段小端，CRC 覆盖整条记录并跳过 CRC 字段自身。encoder_calib_flag 允许 0x01/0x02/0x04，未知位拒绝。
启动不转换或回写旧记录；schema12/13、截断、CRC 错误或不合法内容均回退为未确认默认配置。

保存仍是单份记录，掉电可能使 CRC 失效；身份页不参与普通参数擦写，固件升级不写参数/身份区。
当前记录按两个 2 KiB 页擦除，参数分区本身没有缩小，旧版第三页残留不作为当前记录读取。
历史迁移工具只生成离线文件，不表示当前 APP 支持旧版本或在线保存。
实际字段和合法性以 parameter_record.c/h 为准，固定字节见
[存储夹具](../../tests/fixtures/protocol/parameter_schema14_vectors.json)。

AT32F456 产品记录继续使用 schema14 的固定长度和 CRC。AXS2 修订 2 中两个轮节点的行程填零，三个关节要求实际上下界；速度为电机轴单位。旧 APP 拒绝新修订，新 AT32F456 APP 须显式重新配参后才能启用旧设备。详见[五轴工装](../../docs/guides/at32f456_five_axes.md)。
