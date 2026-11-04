# 参数所有权与四类配置

更新：2026-11-03。当前 APP 接入启动参数装载与配置校验，Keil调试命令及在线参数事务已删除。
存储契约见[schema14](../../firmware/parameters/README.md#storage)。

| 类别 | 默认来源 | 活动对象 |
| --- | --- | --- |
| 硬件 | config/board_config.h | 编译期常量，不开放在线写入 |
| 电机 | config/motor_profile_config.h | MotorConfiguration.model：极对数、R/L、磁链、PI、标定电流和热参数 |
| 应用 | config/application_config.h | MotorConfiguration.application：电流/速度包络、外环、前馈和轴记录 |
| 设备 | config/application_config.h 与本次采样 | MotorConfiguration.device：节点、零偏、编码器方向/LUT/零位 |

路径相对 firmware。板型来自工程 BSP_BOARD，电机型号由 FOC_ACTIVE_MOTOR_PROFILE 独立选择。
当前无覆盖时默认 YQ-5708；更改编译默认值不覆盖已有有效 Flash 记录。

## 所有权与提交

MotorAxis 单份持有配置和运行对象。生命周期借用配置只读视图与运行状态，
控制算法显式接收配置及输出对象；通信仅调用绑定服务，不取得可写对象地址。
Watch 全局符号只用于观测，不构成跨模块写接口。

- 启动：BspFlash_Read → Param_Download；记录无效则 Param_Return_Default。
- 存储：只接受 schema14，固定 2288 字节；CRC、确认位和字段布局保持稳定。
- 零偏：旧记录值不提供本次启动资格，关相 IDLE 累计 2048 拍后发布三相和可用母线零偏。
- 编码器：LUT/零位单份存于 device.encoder；EncoderState 借用配置并持有反馈动态状态。
- 标定：任务写入候选；LUT 完成后先关相，前台复核原会话、STOP 和故障后发布。
- 运行状态：模式、目标、PI 积分、测量和观测历史不写入参数记录。

启动参数记录与停机保存复用同一份2288字节静态暂存区；LUT 标定独占另一份6144字节工作区。
没有在线修订号、保存票据或自动SAVE。MotorAxis_SaveCalibration仅在空闲关相且标定结果有效时
调用Param_Upload及BspFlash_Write，返回成功表示写入及回读完成。
当前协议参数命令仍返回UNSUPPORTED；ParameterValue_Read/Apply未绑定在线服务。
Keil保存命令已删除，保留的MotorAxis_SaveCalibration目前没有APP请求入口。

## 合法性与硬件边界

参数层校验类型、范围、轴记录、热参数和前馈参数；配置有效不等于已经确认或完成标定。
配置确认、实时采样、母线电压、编码器和故障条件共同参与生命周期准入。
当前 YQ 热默认值为空，因此关闭热调度后仍不能跳过配置校验。

固定区域实际字节读写归 BspFlash。写入要求前台和真实关相，由 BSP 协调采样暂停、
擦写及回读；失败保持关相并禁止不确定状态下重试。MotorAxis_SaveCalibration封装该调用，
不在外层临界区包裹CRC或Flash操作。
Loader 的参数读取与镜像产品保持独立，不能因 APP 内部结构整理改变共享存储 ABI。
