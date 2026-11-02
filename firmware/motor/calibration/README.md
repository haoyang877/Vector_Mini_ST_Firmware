# 编码器标定

只有`encoder_calibration.c/.h`：状态机、运动控制和私有计算函数在同一实现文件内。
没有独立core模块、控制适配模块或回调。对外保留6个任务接口，计算函数不再公开。

```c
EncoderCalibrationTask_StepLut(&control, &workspace);
EncoderCalibrationTask_StepElectricalZero(&control);
```

`control`由MotorAxis在调用时绑定配置、FOC、电机、速度PI、编码器、观测器和无感启动对象，
两个任务复用同一绑定；任务不保存绑定地址。电零位只访问配置、FOC、电机和编码器。
工作区由Axis独占，保持6144字节，待发布或取消前不得复用。

FOC执行集中在私有控制函数；状态机随后读取反馈推进阶段。对齐、稳速、采样、验证和停止
仍按原时序执行：首拍只建立会话、每拍清4个bin、构表最多8个bin；不跳过保护或验证。

`GetStep`、`IsReleased`只读；`Cancel`清理两个会话，不关相、不复位控制器、不改配置。
`PublishLut`只在关相前台、复核会话后发布。接口不写Flash，功率许可和状态迁移仍归生命周期。

变更与验证见[最小实现记录](../../../docs/plans/active/2026-10-30-calibration-control-boundary-v3.md)。
