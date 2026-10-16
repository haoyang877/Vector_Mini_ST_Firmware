# 核心维护与编码器标定模块

更新：2026-10-09。适用v1.3.0；维护只支持LUT、电零位、机械零位、SAVE及DEFAULTS。

## 一项任务一条调用链

```text
单项请求 → 主状态机准入 → MAINTENANCE
  → MotorAxis快拍 → MotorMaintenance_Step → 对应任务Step
  → 返回RUNNING / COMPLETED / FAILED
  → 主状态机关相、取消/归还 → STOPPING → IDLE
成功LUT/零位/DEFAULTS → 回到IDLE后独立SAVE → 前台提交与回读 → STOPPING → IDLE
```

没有自动全套标定计划。任务的内部阶段由任务自己推进，主状态机只处理准入与结果。
任务RAM完成、输出关闭、资源归还和Flash已保存分别确认，不能用一个ACK代替。
SAVE受理取决于本次停止、会话事件和IDLE条件；可以同监督拍衔接，不保证固定延后一拍。
完整条件见[主状态输入与跳转](motor_state_flow_simplification_v1.md)。

## 文件与状态所有权

| 文件 | 核心职责及状态 |
| --- | --- |
| [motor_maintenance.c](../../firmware/app/runtime/motor_maintenance.c) | 检查任务参数/电压/反馈前提，分发五种维护，调用取消与释放 |
| [encoder_calibration_task.c](../../firmware/motor/calibration/encoder_calibration_task.c) | 持有标定阶段、时间和窗口；适配控制与观测器；返回结果 |
| [encoder_calibration_core.c](../../firmware/motor/calibration/encoder_calibration_core.c) | Q15环形差/均值、采样累计、分批构表、插值及残差；状态由调用方传入 |
| [maintenance_workspace.c](../../firmware/app/runtime/maintenance_workspace.c) | 独占固定联合工作区，编码器采样与参数记录互斥，不使用heap |
| [motor_parameter_transaction.c](../../firmware/app/runtime/motor_parameter_transaction.c) | 活动revision、保存票据及真实提交结果，不接管功率许可 |

## 标定内部只做什么

| 任务 | 内部流程 | 结果 |
| --- | --- | --- |
| LUT（原Mode13） | 静态对齐 → 无感启动/稳定 → 找原点 → 正向采样 → 分批构表 → 正向残差验证 → 发布/卸流 | 写1024点LUT及线性化有效位，清电/机械零位有效位 |
| 电零位（原Mode15） | 已有效LUT → d轴对齐 → 有效角度平均 → 清控制状态 | 写电角零位和对应有效位 |
| 机械零位 | 读取当前健康编码器位置，重设机械参考 | 写机械零位，复位外环 |
| DEFAULTS | 调用参数所有者恢复默认配置 | RAM配置改变，随后独立保存 |
| SAVE | 快拍等待，前台借用参数记录 → 导出 → Flash写入/回读 | 真实成功后才报告持久完成 |

LUT所需无感启动与观测器保留。核心计算不包含FOC对象、HAL、Flash或启停动作。
标定的覆盖、残差、跟踪与超时条件保留，不为缩短流程绕过检查。
电零位依赖LUT，不会隐式先启动LUT。协议kind与内部枚举不同，必须显式映射。

## 取消与保存边界

STOP/FAULT撤销任务及派生保存意图；Cancel清任务阶段和借用引用。
只有确认任务已释放、前台认领已归还，主状态机才能允许复用工作区。
迟到结果可完成资源归还，不能恢复已取消任务或补造保存成功。
已经提交的Flash不能因取消回滚；新SAVE必须用明确的新请求身份。
维护SAVE与参数122的ticket/revision分别管理，不能混用。

已删除齿槽、摩擦、相电阻辨识以及整套计划；旧kind3～5返回不支持。
通信入口见[单项维护v1.2](../protocols/yg_protocol_calibration_wire_v1_2.md)，
计算边界见[编码器核心](encoder_calibration_core_v2.md)。
