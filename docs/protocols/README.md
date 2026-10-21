# 当前协议与兼容数据

本索引按实际静态消息注册和shared目标核对。公司外部消息表的登记问题仍按各契约正文处理；
项目源码接线、公布能力与设备验收是不同事实。

| 主题 | 现行入口 |
| --- | --- |
| 全部已注册业务及设备版本/身份 | [设备发现与支持范围](yg_protocol_device_discovery_v1.md) |
| CAN FD承载、队列和处理上限 | [承载适配](yg_protocol_canfd_adapter_v0_1.md) |
| 纯分片规则 | [重组v0.2](yg_protocol_fragment_v0_2.md) |
| 101/107/108/110/116/124及45B字段 | [单轴命令与反馈](yg_protocol_motor_wire_v1.md) |
| 117零目标启动 | [普通启动v0.2](yg_protocol_ordinary_start_wire_v0_2.md) |
| 当前仅公布SPEED | [能力v2](yg_protocol_core_speed_caps_v2.md) |
| 八状态线上投影及STOP/CLEAR | [状态与恢复v1.2](yg_protocol_motor_state_v1_2.md) |
| Type3兼容不可用epoch | [兼容发现响应](yg_protocol_device_status_wire_v1.md) |
| 120/121/122请求、revision和detail | [参数线路](yg_protocol_parameter_wire_v1.md) |
| 实际参数ID、权限、来源和尺度 | [参数目录](yg_protocol_parameter_catalog_v0_1.md) |
| 地址与应用包络分离 | [装载政策v2](parameter_node_application_v2.md) |
| 当前写入格式与旧版迁移 | [schema14](parameter_schema14_storage_v1.md) |
| 仍支持的旧格式读取/扩展语义 | [schema12](parameter_schema12_loading_policy_v1.md)、[schema13](parameter_schema13_delivery_v1.md) |
| 160～162五项单项维护 | [维护v1.2](yg_protocol_calibration_wire_v1_2.md) |
| 50～55升级、56交接、57确认、v3容器 | [升级v3](yg_protocol_upgrade_wire_v3.md) |
| 数值编码实现约束 | [数值等价编码](numeric_encoding_v1.md) |
| 链路工具的有限验证范围 | [CAN FD冒烟流程](yg_protocol_link_smoke_test_v0_1.md) |

旧编号表、评审稿、退役启动计数/租约/多轴提案和旧版线路说明见[协议归档](../archive/protocols/README.md)。
旧JSON向量保留原路径，供现有离线工具和拒绝/兼容测试读取；历史向量中的能力不代表当前注册能力。
当前黄金数据包括schema14、core_speed_caps_v2、ordinary_start_wire_v0_2和calibration_wire_v1_2向量；
升级目标几何另以tests/fixtures/loader_image_layout6.json为准。原向量内容本轮不改写。
