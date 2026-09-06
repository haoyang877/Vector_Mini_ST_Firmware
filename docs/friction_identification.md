# 摩擦辨识（Mode 19）

## 功能与模型

Mode `19` 通过八个正反向稳态速度点，辨识电流域的库仑摩擦与粘性摩擦：

```text
omega > 0: Iq =  Ic_pos + B_pos * omega
omega < 0: Iq = -Ic_neg + B_neg * omega
```

`omega` 单位为 rad/s，`Ic` 单位为 A，`B` 单位为 A/(rad/s)。辨识结果只作为候选值保存；固件不会自动应用，也不会自动启用摩擦补偿。

## 分层实现

| 层 | 文件 | 职责 |
| --- | --- | --- |
| Core Service | `Firmware/Core/Services/Identification/friction_identification.*` | 无硬件依赖的状态机、采样、约束拟合与结果验收 |
| Product | `Firmware/Product/mechanical_load_profiles.*` | 速度点、稳态/超时、饱和和 RMSE 阈值 |
| Runtime | `Firmware/Runtime/MotorControl/friction_identification_runtime.*` | 速度环、编码器、电流反馈、安全停车和候选参数适配 |
| Port/Application | `Firmware/Ports/friction_identification_port.h`、`Firmware/Application/friction_identification_service.*` | 向 USB/CAN 提供只读结果和显式应用操作 |

Domain 不引用 Runtime、HAL、通信或 Product 类型。Mode 19 作为 `SERVICE_PROCEDURE_FRICTION_IDENTIFICATION` 进入主生命周期状态机，不把协议动作码泄漏到 Runtime。

## 运行条件与运动序列

启动前必须满足：Mode 0、Error 0、编码器在线且线性化/电角零位有效、速度限幅不低于 0.5 rev/s、速度 PI 与电流限幅有效，并确保正反向连续转动安全。

默认速度点为：

```text
+0.10, -0.10, +0.20, -0.20, +0.40, -0.40, +0.50, -0.50 rev/s
```

每个点先确认 0.75 s 稳态，再采集至少一整机械圈和 0.5 s 数据；之后停车并切换方向。持续达到电流限幅 90%、跟踪/采样/停车超时、样本无效或拟合残差超限都会安全停车并报告 `MOTOR_FAULT_FRICTION_IDENTIFICATION`。

## USB 操作

推荐脚本：

```powershell
python tools/friction_identification.py COM4 --output validation/friction/result.json
```

审核后应用到 RAM：

```powershell
python tools/friction_identification.py COM4 --apply
```

加 `--save` 会在应用后调用 Mode 9 保存。底层协议保持与主线一致：

| 命令 | 含义 |
| --- | --- |
| `w_mod=19` / `w_mod=0` | 启动 / 取消辨识 |
| `r_fst` | 状态、原因、点序号、进度、候选有效位 |
| `r_fdt` | 导出八个平均点 |
| `r_fcp` / `r_fcn` | 候选正/反向库仑摩擦电流 |
| `r_fvp` / `r_fvn` | 候选正/反向粘性系数 |
| `r_frp` / `r_frn` | 正/反向拟合 RMSE |
| `w_fap=1` | Mode 0 下显式应用有效候选到 RAM |
| `r_fva` | 当前已应用模型是否有效 |
| `r_acp` / `r_acn` | 已应用库仑参数 |
| `r_avp` / `r_avn` | 已应用粘性参数 |

状态值：0 idle、1 tracking、2 sampling、3 stopping、4 complete、5 failed。失败原因：0 none、1 cancelled、2 invalid config、3 tracking timeout、4 sample timeout、5 stop timeout、6 current saturation、7 invalid sample、8 fit rejected、9 safety fault。

## CAN 与持久化

CAN 仍通过 `CAN_SET_MODE (0x00)` 写入 19 启动。`0x58` 应用候选；`0x59..0x62` 读取状态、原因、候选参数、RMSE 和有效位，定义见 `Firmware/Communication/Protocol/can_protocol_v1.h`。

参数 schema 已升级为 v9，并在 v8 尾部追加四个摩擦系数和有效位。v8 双槽记录可原位迁移，v4-v8 原始记录仍可读取；旧 schema、分流电阻配置变化或编码器方向变化都会将已保存的摩擦模型置为无效，其他电机与编码器标定数据不受影响。
