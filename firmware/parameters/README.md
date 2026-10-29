# 参数配置与记录

本目录负责配置默认值、标量访问和存储记录转换，不执行电机控制或持有主状态。

| 文件 | 职责 |
| --- | --- |
| parameter_record.c/h | 装配默认配置，装载/导出schema14及兼容旧记录，校验CRC |
| parameter_values.c/h | 按参数ID检查类型、权限、范围及缩放，操作调用者显式传入的配置 |
| motor_axis_profile.c/h | 创建/校验AXS轴记录，解析关节控制包络 |
| application_profile_defaults.h | 应用包络、外环和可选前馈的默认输入 |
| device_profile_defaults.h | 单台设备地址和默认校准初值 |

硬件默认值在platform/config，电机默认值在motor；本目录没有复制一套运行状态。
主状态准入、去重、revision及保存票据归APP参数事务；实际字节读写归平台param_store。
四类参数的完整归属见[参数所有权](../../docs/architecture/parameter_ownership_v1.md)。
