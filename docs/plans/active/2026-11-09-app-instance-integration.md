# APP 实例化合并验收

目标为将 8539043c 合入已集成双板 Loader 的 b8bea79b，保留 AT32 ADC2 母线采集和轴配置。以两板普通 APP 与 LoaderApp 的编译、PR 和 release 回归验证软件。AT32 PITCH 节点 6 台架验证角度标定、正反向速度与位置闭环、固件升级及重启后身份和参数。使用各板原有镜像、目标分区和参数契约。回退以合并前目标提交和设备原始备份为基准。证据放在 outputs/instance-merge-20261109 及对应 harness 运行目录。
