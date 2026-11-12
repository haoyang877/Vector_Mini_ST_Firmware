# 固件参数的来源与生效时机

| 来源 | 典型字段 | 何时生效 |
| --- | --- | --- |
| 板级编译配置 | 采样电阻、放大倍数、通道、时钟；快慢环分频 | 工程通过 `BSP_BOARD` 选择实现。单位换算由板级资源决定。[配置目录](../../firmware/config/README.md) |
| 电机与应用默认值 | 极对数、R/L、磁链、PI、限流和速度包络 | `Param_Return_Default` 组装默认值。有效记录经 `Param_Download` 装载；AT32 产品轴还须匹配节点、AXS2 角色及电气模型。[参数装配](../../firmware/parameters/parameter_record.c) |
| 设备标定 | 转子 LUT、方向、电零位、机械零位、输出轴与 Hall 系数 | 在线标定在原会话关相发布；重启从有效记录恢复。电流偏置每次上电重新采集。[存储与兼容](../../firmware/parameters/README.md#storage) |
| 生产身份与运行数据 | SN、电机类型、芯片 UID；控制目标、PI 积分、动态反馈 | 身份页与芯片 UID 由设备信息查询上报。目标、积分和实时采样仅属于运行实例。[参数所有权](parameter_ownership_v1.md) |

电流 PI 的默认设计使用 `Kpd=Ld×ωc`、`Kpq=Lq×ωc`、`Kid=Kiq=R×ωc`。保存的 Ki 未预乘采样周期。有效记录可包含整定值，改编译默认值不会自动覆写已有记录。

维护一个参数时，应能指出其单位、来源、有效范围、活动对象、是否持久化和失效条件。源码中的标量读写接口与 CAN 端点的业务绑定分别核对。当前 120–122 的回应见[参数命令](../protocols/README.md#parameters)。
