# 标定与已退役维护功能的边界

2026-10-30：运行层保留编码器线性化、电气零位、机械零位，完成关相后回空闲。
通信160的kind 1、2、6由APP协议适配层映射到MotorStateMachine_RequestCalibration。
算法归motor/calibration，专用工作区归Axis，状态机不解析协议。

默认恢复、Flash保存、通用任务查询/按编号取消及旧维护模块继续退役。
旧Flash标定数据仍可读取，新标定结果只更新RAM；停止沿用110，不保留任务历史。
协议范围见[标定通信v1.6](../protocols/README.md#parameters)。
