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

当前 APP 只接入启动装载和配置校验，没有参数事务所有者、在线修订号或保存队列。
标量读写与记录导出函数仍用于独立契约测试；通信参数命令未绑定服务，返回 UNSUPPORTED。
BspFlash_Write 负责固定区域的硬件写入，本目录不直接擦写 Flash。

见[参数所有权](../../docs/architecture/parameter_ownership_v1.md)和
[存储格式](../../docs/protocols/parameter_schema14_storage_v1.md)。
