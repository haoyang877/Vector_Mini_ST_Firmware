# FOC 坐标结构体接口 v1

## 目标与范围

将 `firmware/motor/control/foc.h` 的 Clarke、Park 及 SVM 接口改为
`ABCVect`、`AlphaBetaVector`、`DqVector`，同步所有生产和台架调用。
FOC 内部坐标成员按向量组织，采样对象 `MotorMeasurement` 保持原有所有权和字段。
删除多输出标量参数，SVM 返回扇区，合并判区分支，保留时间公式以避免改变浮点融合结果。

## 保持不变

- 相序、等幅归一化、角度单位、Math_SinCos 和 PWM 极性。
- SVM 六扇区边界及各扇区浮点运算顺序，零向量仍归扇区 1。
- 电流和电压控制律、故障条件、PI 更新与 PWM 提交次序。
- 线路协议、持久化 ABI、测量所有权、保护及输出准入规则。

## 基线、验收和回退

基线为本次编辑前工作区，不将其他未提交工作当作本任务改动。
源码副本、原始差异和日志位于 `outputs/foc_axis_structs_v1/`。
修改前正余弦测试通过；既有 FOC 对照脚本缺少 BSP_BOARD/使用旧 BSP 返回类型，
上电零偏测试已有编码器重启标志断言失败。这些与坐标接口改动分别记录。

验收包括全调用搜索、25.2 万组坐标/SVM 逐位基线对照、三角函数测试、
PR profile、格式和 lint 检查，以及受影响目标编译。
资源与运行时间只记录可复现编译/主机证据，不推断硬件周期数。
回退只逆向本任务差异，禁止恢复整个工作区或覆盖已有改动。

## 结果

按用户命名要求，三相类型最终为 `ABCVect`，原 `AbcVector` 名称不保留别名。

- 生产与台架的 Clarke/Park/SVM 调用均改为向量接口，旧签名仅保留在冻结参考夹具。
- `coordinate_equivalence_final/test.log`：252032 组向量、六扇区及边界，五个函数与修改前输出逐位一致。
- `fast_math_final/test.log`：平方根和 1081407 组三角函数、Park 往返验证通过。
- 六个修改后的生产 C 文件分别以 BSP_BOARD=1/2 编译，补充 SEGGER RTT 头路径后均通过；
  闭环台架生产文件单独编译通过。不会据此宣称完整链接或硬件时序通过。
- lint 复测通过；本任务文件格式通过。全局格式仍有未修改的
  `test_bsp_function_contract.py` 问题。
- PR profile 已执行：`outputs/runs/foc-axis-structs-v1/summary.json`。
  架构、接口、集成测试通过；完整验证仍因在途迁移的旧 BSP 契约、缺失头文件、
  缺失参数宏、旧测试夹具与文档路径失败。新测试的导入排序问题已修复并复测。
- STM32 编译已有 13 个 PARAM_APP_JOINT_V1_* 未定义错误；AT32 APP 另外存在
  已删除 bus_current_feedback.h 的引用。未修改这些无关的参数或 BSP 行为。
- 主目录中的误改已全部撤回，只保留用户原有的四个未提交文件。
- 未连接硬件、未烧录。
