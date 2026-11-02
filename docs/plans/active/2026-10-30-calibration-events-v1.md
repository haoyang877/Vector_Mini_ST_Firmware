# 五态标定与完成事件接入 v1

## 目标

按用户要求接通CALIBRATE与COMPLETE：空闲接受标定请求，执行已有标定算法，
完成后关闭输出并返回空闲。自检继续只保留枚举，不添加入口或实现。
新增命名明确表达标定类型、请求、执行、候选发布和完成，不恢复旧维护任务框架。

## 范围与基线

支持现有编码器线性化、电气零位和机械零位。LUT需要6144字节静态工作区及关相后
前台发布；结果仅更新RAM配置，不自动保存Flash，不恢复已移除的任务通信协议。
保留停止/故障优先、参数写入互斥和旧代际结果失效。

本轮开始时工作区保存在outputs/calibration-events-20261030/before，
对应baseline_hashes.json。只回退本轮补丁，不覆盖其他未提交修改。

## 验证

验证正常完成、拒绝非法类型/非空闲请求、停止和故障、迟到完成、重复完成、
LUT发布前等待与发布失败。编译两板相关APP及标定模块，运行原有控制/协议/采样测试、
格式、lint和PR profile；不连接硬件、不启动真实电机。

## 结果

已完成。五态纯迁移保持42行；自检只占位。

- MotorLifecycle_RequestCalibration受理三种明确类型；MotorAxis的
  run_active_calibration_cycle调用原有算法，finish_calibration通过COMPLETE返回空闲。
- 线性化在关相后由前台发布候选；停止、故障或原代际失效后不得发布。
- 两板生命周期、快环标定分派及原电流偏置测试通过；停止、故障、关相失败、
  迟到完成、发布等待/失败和配置修订都已有覆盖。状态快照与协议回归通过，
  线上标定/空闲/故障映射分别断言为4/5/8；边界检查10项通过。
- 两板的生命周期、Axis、应用前台、状态适配器及标定模块共12个ARM对象编译通过。
  Axis的BSS为9008字节，包含新增6144字节标定工作区；没有动态分配。
- 原观测器/标定热路径测试脚本缺少BSP_BOARD定义；保留原失败证据，并在独立输出目录
  显式补入两板宏，四个板型/阻尼组合的真实算法测试均通过，没有修改该脚本。
- 本轮11个源码/测试文件格式检查通过，lint零新增问题。整仓PR仍有非本轮错误：
  BSP测试格式、缺失platform/devices包含路径和文档链接、旧测试构建及测试清单漂移。

证据：outputs/calibration-events-20261030下的lifecycle-tests、axis-tests、status、
parameter-protocol、calibration-core-with-board、arm-final；完整PR输出为
outputs/runs/calibration-events-20261030及calibration-events-20261030-final。
没有烧录、真实电机运行或中断耗时测量；相关对象编译不代表整仓链接已通过。
