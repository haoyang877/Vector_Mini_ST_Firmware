# 三类标定通信入口 v1

## 目标与边界

把已有编码器线性化、电气零位、机械零位接入通信入口。复用0x00A0（160）及
kind 1、2、6，适配层调用MotorLifecycle_RequestCalibration，完成继续由原运行层
通过COMPLETE回空闲。五态核心、自检占位、算法和关相/故障路径保持不变。

只恢复三类启动；不恢复通用任务历史、查询、取消、保存或自动重试。
12字节回复保留格式，非零token仅作为本次受理编号，phase=执行中、effects=未持久化。
主机通过现有电机状态确认标定态和回空闲，使用现有STOP取消，结果只保留RAM。

## 实施与验收

- 在APP协议适配层绑定已有job服务，显式映射kind到MotorCalibrationType。
- 未知/退役任务及161/162保持UNSUPPORTED；沿用解码层的长度、保留字和主站校验。
- 编号只在受理成功后递增；达到上限时拒绝新请求，不回绕、不启动电机。
- 测试实际报文解码→绑定适配器→真实生命周期→完成回空闲；覆盖三类标定、拒绝、
  故障、非法报文和状态机边界。补充版本化协议文档及黄金向量。
- 编译两板相关ARM对象；执行定向测试、格式、lint及PR profile，不接触硬件。

## 基线与回退

本轮前文件保存在outputs/calibration-communication-20261030/before，哈希见
baseline_hashes.json。只撤销本轮适配器、测试和文档补丁，保留工作树其他修改。

## 结果

三类标定已通过现有160路由接入APP协议适配层，新增handle_calibration_request明确映射
类型和受理结果。只增加4字节受理编号，没有任务历史、动态分配或新调度入口。
状态机及运行层源码与本轮基线哈希一致，自检仍占位。

已通过：两板真实协议解码/适配器/生命周期联测、两板Axis标定分派与完成测试、
13条回复黄金向量、非法字段/长度/主站/类型及错误结果映射、10项边界检查，
6个本轮源码/测试文件格式检查及lint。两板APP适配器、启动、生命周期、Axis、
job协议共10个ARM对象编译通过。

专项证据位于outputs/calibration-communication-20261030下的protocol、lifecycle、axis、
arm和task-files.diff；整仓PR输出为outputs/runs/calibration-communication-20261030。
整仓PR未通过：既有BSP测试格式、platform/devices包含路径和文档链接、旧测试构建及
测试清单漂移仍存在；本轮协议、生命周期、Axis和集成检查在整仓运行中均通过。
未烧录或运行真实电机，ARM对象编译不表示完整固件链接通过。
