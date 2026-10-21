# 轴身份、应用包络与持久化

核对日期：2026-10-21。当前结构见[`axis_config.h`](../../firmware/motor/axis_config.h)，
装载/创建服务见[`motor_axis_profile.h`](../../firmware/parameters/motor_axis_profile.h)。
轴记录是32B独立版本记录，含身份、位置范围rad、最大速度rad/s与CRC32，保存在参数记录中。

| 记录 | 身份 | 当前解释 |
| --- | --- | --- |
| AXS1 | 8B名称roll/pitch | 保留已有独立增益，读取经过CRC和范围检查 |
| AXS2 | joint_type与config_revision | 修订1仅解析roll/pitch控制基线；修订0只登记身份，不能推断有可运行配置 |
| 全零/全擦除 | 未配置 | 按装载服务的未配置语义处理；损坏或未知布局不能冒充未配置 |

稳定轴编号：unknown0、roll1、pitch2、yaw3、右轮4、左轮5。
编号存在不表示yaw或轮子已获得AXS2位置控制配置。设备CAN节点是独立地址字段，
不从轴类型推导；速度包络也不按节点1/2隐式钳制，见[地址策略](../protocols/parameter_node_application_v2.md)。

启动/恢复时校验CRC，运行只使用已验证记录。实际位置须严格在配置范围内，
目标可等于边界；软件限制不保证惯性机构没有机械过冲。
数值轴profile装载可能覆盖位置增益/最大速度，通用参数写入因此拒绝21/22/23，
避免承诺不存在的持久化效果。机械零位与每台编码器LUT仍归设备配置，不因型号相同互相复制。

当前外层记录为[schema14](../protocols/parameter_schema14_storage_v1.md)，不能沿用历史schema9/10的固定地址修改法。
离线 `tools/bench/motor_axis_record.py` 只生成参数副本；使用前须核对输入布局和匹配符号，
其AXS1构造能力不等于支持任意schema重写，当前完整记录迁移使用版本化迁移工具。
历史roll/pitch范围、写板过程和哈希保存在[归档](../archive/guides/motor_axis_profiles.md)，不作为通用默认值。
