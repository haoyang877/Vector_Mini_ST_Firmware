# 三相电流向量 v1

## 目标

将 FocState 的三相测量字段合为 `ABCVect phase_current_a`，统一为 A/B/C 分量访问。
MotorCurrent_Update 接收只读 ABCVect 并整体赋值，Clarke 直接读取该成员。
同步保护、标定、遥测、台架、测试和当前 Watch 文档。

## 不变行为与验收

单位仍为 A，相序、采样次序、无效值传播、滤波、故障阈值与控制算法不变。
原三个 float 的顺序和所占空间不变；冻结历史参考结构不改写。
编辑前快照及日志存于 outputs/phase_current_vector_v1/。
验收包括调用/旧字段搜索、上电零偏与采样、母线测量、坐标变换、双板编译、
格式/lint 和 PR profile。只回退本次差异，不覆盖其他在途工作。

## 结果

生产FocState仅保留ABCVect成员，旧字段只存在于冻结历史参考夹具。
Clarke直接读取该成员；初始化、采样、台架注入使用整体赋值，保护和遥测保持A/B/C顺序。

- 两板的相电流精度与NaN传播、上电零偏、状态快照和母线测量测试通过。
- 252032组坐标/SVM验证、48000拍旧标量布局对照通过，控制和PI逐位一致，存储大小不变。
- 两板共14个受影响生产翻译单元编译通过；闭环台架编译和RTT实帧测试通过。
- 本任务源码格式和lint检查通过；PR profile已执行，详见
  `outputs/runs/phase-current-vector-v1/summary.json`，全量结果另受既有工程和测试问题影响。
- 未连接硬件或烧录。
