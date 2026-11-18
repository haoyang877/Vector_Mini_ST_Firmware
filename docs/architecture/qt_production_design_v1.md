# Qt生产工具设计

核对日期：2026-11-17。适用于 `tools/bench/qt_production_desktop.py` 的v2.1.7源码。
本产品与Rust候选、C++/Qt1.4.4工具分别构建。操作见[生产指南](qt_production_design_v1.md)。

## 模块与所有权

| 模块 | 拥有的数据与职责 |
| --- | --- |
| [qt_production_desktop.py](../../tools/bench/qt_production_desktop.py) | 创建Qt应用、Windows子进程入口、离线界面自检。 |
| [qt_production_window.py](../../tools/bench/qt_production_window.py) | 主线程界面、当前设备显示、用户草稿、页面和操作请求；不直接持有SDK句柄。 |
| [production_controller.py](../../tools/bench/production_controller.py) | 一个在途任务、常驻CAN会话、取消和释放、子进程及退出收尾。 |
| [can_session.py](../../tools/bench/can_session.py) | 常驻CAN进程中的扫描、状态和参数会话；句柄生命周期由该会话管理。 |
| [production_backend.py](../../tools/bench/production_backend.py) | 检测、标定、速度与其他操作的执行、数据及进度；输入在访问设备前检查。 |
| [five_axis_jlink.py](../../tools/bench/five_axis_jlink.py) | SWD探针选择、识别、镜像/参数保护、烧录和版本回读。 |
| [production_flow.py](../../tools/bench/production_flow.py) | 按UID记录本次生产步骤，动作开始就使依赖的旧结果失效。 |
| [production_report.py](../../tools/bench/production_report.py) | 汇总证据和状态，导出JSON、CSV、HTML。 |

## 任务与SDK占用

速度会话依据CAPS位26识别输出坐标v2：新设备目标及反馈均为输出轴rad/s，
行程直接用输出角差检查；旧设备仍用电机坐标，行程角差除减速比。
速度上限参数保留电机轴单位，新设备的目标限额按减速比换算。
报告记录motion_coordinate与target_speed_limit_rad_s，CSV按对应会话坐标解释。

主线程通过控制器提交操作。扫描、参数和状态查询复用常驻CAN进程；
J-Link、升级、检测、运动等其他任务使用独立进程。控制器等待CAN释放确认后才交接占用。
手动动作可以请求取消自动查询，后续动作等待当前操作完成；不并行向同一设备发送两组任务。
界面线程处理进度和结果，不阻塞等待SDK。默认每1秒状态刷新不是新一轮检测或标定。

取消的含义由操作定义：运动/检测通过既有停止流程收尾，J-Link烧录不能被普通停止按钮中断。
关闭窗口先等待关相、任务和队列回收；不能把进程消失当作设备已停止。

## 身份和版本来源

界面用当前UID组织会话，重新识别及换板不能沿用旧设备的通过状态。
J-Link单支可自动选择，多支需指定序列号；失去所选探针后清除身份和版本绑定。
本地HEX/清单表示预期包；SWD的APP描述符与Loader镜像/清单核验表示本轮安装结果；
CAN身份回复表示实际通信结果。这三类来源分开显示，不能互相补造缺失的验收。

### J-Link 整包安装

整包页提供探针序列号列表及扫描入口。单支自动选择，多支等待用户选择；
扫描只枚举探针，不连接目标。每次识别、烧录、擦除重新枚举并检查选定序列号，
选定探针拔出时拒绝操作，不自动改用另一支。写入前继续核对已绑定的 MCU UID。
换探针、重新识别、擦除、换下一台及新烧录开始时，不沿用旧版本确认。

默认执行烧录后的 CAN 验收。仅接 J-Link 时可取消该选项，此时不打开 CAN，
仍完成 Flash 回读、APP 启动和参数保持校验。APP 版本从板端描述符双读并与本次镜像比较；
Loader 版本取与已回读一致的完整 Loader 镜像匹配的批次清单。
没有独立的 Loader 运行时版本读取结果，不将此来源写成 CAN 回读。
界面分别展示本地包版本、板端 SWD 安装版本及 CAN 验收状态。
未执行 CAN 时明确显示“CAN 通信未验证”，不计作完整生产验收或开放已验收入口。

## 生产步骤和报告

烧录、擦除、标定和硬件检测开始时，ProductionFlow按动作清除相关旧步骤。
重新烧录或擦除会清空该UID旧步骤；LUT标定使角度、零位及后续复检失效。
结果以本轮任务返回和证据记录为依据，不从设备曾经存在的标定标志推测其生产历史。
状态包含通过、失败、未测和未提供信息；报告保留取消与部分采样，不把缺失值显示成零或通过。
“归档并开始下一台”只是本次会话归档，下一台需重新识别和执行流程。

## 验证范围

现有 [Qt控件测试](../../tests/unit/test_qt_production.py)及
[J-Link流程测试](../../tests/unit/test_five_axis_jlink.py)验证界面与模拟后端边界；
`--offline-smoke`只验证无设备的页面构建与截图。
这些不能证明真实多探针、失联停机、Flash安全或电机精度。
回归应覆盖零/单/多探针、选择消失、芯片身份变化、无 CAN 完整烧录、描述符不符和旧版本失效。
多探针和仅 SWD 的现场行为仍需实机验收；当次检查、设备故障与质量状态记录在 outputs。
