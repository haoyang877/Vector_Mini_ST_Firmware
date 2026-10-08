# 模块核心职责与删减清单

更新：2026-10-08。描述当前集成工作树，模块独立报告不是整机验收结果。

读代码先看三件事：输入是什么，谁拥有可变状态，结果交给谁。
纯计算只处理数据；任务适配处理采样和时序；主状态机唯一决定运行许可、关相与收尾。
不要求每个函数一个文件，也不增加通用注册框架。

## APP 核心工作

| 模块 / 入口 | 最核心的处理 | 状态归属与协作边界 |
| --- | --- | --- |
| composition / board_config | 初始化顺序、板卡选择、协议业务绑定、启动交接 | 组装已有对象，不计算控制公式 |
| runtime / motor_axis | 单轴对象装配、20 kHz 快拍、2 kHz 监督、后台调用顺序 | 持有唯一运行对象；显式借用给执行、维护与参数事务 |
| runtime / motor_parameter_transaction | 参数身份去重、修订号、保存票据、装载与实际保存协作 | 独占原有事务状态；不缓存对象指针，不另设生命周期 |
| runtime / motor_execution | 按已批准模式执行控制，连接外环和电流环 | 消费许可与显式对象；不接管启动、取消或存储 |
| runtime / motor_maintenance | 分发标定/辨识、确认功能前提、取消当前功能 | 任务结果交主状态机；大数组借用固定工作区 |
| runtime / maintenance_workspace | 单份 6 KiB 互斥借用与归还 | 不能在真实归还前复用；没有堆分配 |
| motor_state_machine | 接受/拒绝请求，维护唯一主状态、功率许可与收尾 | 故障、STOP、取消和资源释放保留；CLEAR 不启动 |
| motor/foc / PI 与控制计算 | 电流/速度误差到输出，限幅与积分跟踪 | 控制器状态显式传入；公式不解析通信或写 Flash |
| motor/foc / sensing、无感启动与观测 | 采样换算、启动推进、转子角度/速度估计 | 保留现有时序与控制数学；平台只提供采样/PWM 能力 |
| motor/position / position_cascade | 位置误差到速度，速度 PI 到电流，总电流限幅 | 私有 core 独占 PI/Iq；friction 独占摩擦状态；组合层负责轨迹/HOLD/积分协调 |
| motor/trajectory / position_smooth_trajectory | 有界规划、轨迹参考与连续性 | 自有规划状态，只输出参考值；不操作电机 |
| motor/feedback / angle_feedback | 原始角度校正、多圈展开、速度估计 | 编码器对象唯一；采样通过平台，结果供所有模式使用 |
| motor/calibration / encoder_calibration_core | 环形角差、原点/零位归约、有界 LUT 构建与残差验证 | 标准值类型和显式计算状态，无 FOC、全局运行对象或硬件依赖 |
| motor/calibration / encoder_calibration_task | 启动、稳定/跟踪时窗、逐拍采集、应用结果与退出 | 传入采样事实并消费核心结果；保留限流、故障、取消及工作区释放顺序 |
| motor/calibration / cogging_calibration、cogging_map | 双向样本归约、候选表与有效性/记录计算 | 值输入输出；不控制主状态 |
| motor/calibration / foc_cogging_calibration | 齿槽采集时序、保持控制、任务结果及发布候选 | 发布交运行模块；发布 RAM 不表示 Flash 保存成功 |
| motor/calibration / foc_cogging_runtime | 单份活动表、补偿使能/混合、运行保护事实 | 不依赖采集阶段；复用 motor/foc/cogging_compensation 计算 |
| motor/identification | 相电阻/摩擦辨识计算与对应任务推进 | 计算与 task 分工；返回结果，不直接持久化 |
| motor/protection | 根据测量与状态判定故障/额度 | 阈值与故障事实保留，快速关断通过既有平台和状态边界 |
| motor/math | 数学、角度、滤波计算 | 无硬件、APP、通信依赖，不建 catch-all utils |
| services/parameters | schema12 映射、纯标量类型/范围校验、轴配置解析 | 不拥有参数事务票据、功率许可或通信节点全局状态 |
| services/lifecycle、telemetry | 请求/状态值契约、停机服务与只读快照 | 服务不再建立另一套运行状态机 |
| communication/can | 有界接收队列、链路与端点调度 | 中断入队，前台处理业务；不在 CAN 中断擦写 Flash |
| communication/protocol | 线路校验、业务请求转换、结果编码 | 调用业务所有者；不因收到目标而自行启相 |
| platform/api 与 STM32 ports/BSP | 硬件能力和实际 I/O | HAL、寄存器、板卡细节集中在平台实现 |

Loader 核心为镜像边界、接收/写入、读回校验、提交/试启动与失败恢复；不承担电机控制。
上位机核心为操作、状态展示与升级计划；真实传输与 GUI 分开，演示不能记实机 PASS。
shared 只保留实际共用的 codec/CRC、镜像、启动邮箱与目标契约，不放运行状态。
产品协作见[软件架构说明](software_architecture.md)。

## 本批实际删除与移出

| 原内容 | 处理及依据 |
| --- | --- |
| 位置 speed_reference、feedback_current 影子缓存 | 删除重复状态；公开返回时值已由 PI.reference/output 持有，快照直接取该状态 |
| 位置完整清零后再次复位 PI | 删除重复复位；核心和摩擦分别一次复位自己的状态 |
| 摩擦输出里未被消费的 breakaway 字段 | 删除无调用字段；实际起步功能及其内部状态仍保留 |
| 编码器 task 内 LUT/零位计算体 | 移入 encoder_calibration_core；任务调用单份计算，删除旧内嵌实现 |
| 齿槽 task 内运行补偿、活动表和保护状态 | 移入 foc_cogging_runtime，删除 task 对它们的直接所有权 |
| 齿槽重复签名包装、临界区前随后被覆盖的读值 | 复用已有签名函数，删除无独立语义的包装和读值；临界区内重新核对保留 |
| 重复齿槽失败/取消/完成清理 | 单份本地收尾处理；各终态的故障结果和顺序保持 |
| Axis 内参数缓存/修订/保存实现和 I/O include | 移入参数事务所有者；Axis 只转发和调度，状态未复制、Watch 符号保留 |

“移出”减少的是一个模块的职责，不等于删除功能或保证整个 APP 更小。
所有变更均有真实生产调用；此次没有退役控制模式、保护、工作区、取消、保存或调试入口。
证据和回退见[本批计划](../plans/active/2026-10-08-module-core-decoupling-v2.md)。

## 尚需单独梳理

当前主状态机仍为九状态，本批未改跳转与功率条件。反馈/采样模块仍包含平台调用，
尚不能称全部为纯计算。参数持久层的应用角色/设备地址兼容映射仍保留。
这些问题需按具体调用链收口，不能因为目录看起来复杂而删除有效功能。
后续新增删除项必须记录实际调用搜索、替代位置、保持行为和匹配证据。
