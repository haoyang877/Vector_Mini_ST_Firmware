# Watch 全局观测对象决策 v1

## 目标与基线

用户要求将刚列出的重点运行变量全部改为非静态全局变量，方便 Keil Watch 直接查看。
基线为 83f39c6 加用户当前在途修改；s_context 已被用户去除 static。本次对 MotorAxis
全部文件级可变对象采用相同处理，并规范 s_context 定义的缩进和所有权说明。

## 范围与所有权

motor_axis.c 的 s_motor_state、s_speed_pi、s_rotor_encoder、s_flux_observer、
s_sensorless_startup、s_foc_state、s_config_revision、s_revision_work_epoch、
s_last_read、s_last_write、s_save_job，以及 motor_state_machine.c 的 s_context
均提供外部链接符号。它们各自仍只有一份定义，初始化、运行写入和并发交接由原模块负责。
其他生产模块继续使用既有命令、快照或显式对象参数，不新增跨模块全局写入调用链。

为兼容既有 Watch 表达式，保留 s_ 符号名称；这是用户调试要求下对 STYLE.md 的 g_ 命名
约定和原文件私有封装的明确例外，不作为新增模块的默认规则。其他模块、私有函数、常量
及函数局部存储不在本次所列观测对象范围。全局可见会降低链接层封装约束；调试写入不等于
正式控制受理，不用于绕过启相、故障或维护互斥检查。

## 行为边界

只改变对象的链接可见性及说明，名称、类型、字段布局、零初始化、volatile、控制数学、
保护门限、参数存储和协议均保持原值。外部可见性可能影响编译器优化、符号和镜像布局，
必须使用新构建的匹配 AXF；本次不声称机器码、时序或实机行为已经验收。

## 验收、回退与证据

仅重建主工程 normal 一次，检查全部 12 个对象在 AXF 中为全局对象符号，保存当前输入和
产物哈希；不执行全量回归、下载、复位或运动。T 的 wheel-regression 冻结候选保持独立。
回退时只将本次 MotorAxis 对象恢复 static；保留用户事先对 s_context 和其他文件的修改。
证据归 outputs/watch-global-objects-20260929/，用户调试/Watch 文档同步更新。

## 实际结果

主目标一次重建得到 0 错误、0 警告；AXF 符号表确认上述 12 个对象均为唯一的
STB_GLOBAL / STT_OBJECT。本次 HEX 与原冻结 HEX 哈希一致，AXF 的调试符号已更新。
仅编译及读取符号表，未访问硬件或替换 T 的独立冻结产物；实机验收状态不因此改变。
