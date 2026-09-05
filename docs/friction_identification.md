# 摩擦辨识

## 1. 适用范围

模式 `19` 在空载、允许连续正反转的电机上辨识等效库仑摩擦和粘性摩擦。结果使用电流域参数：

```text
omega > 0: Iq =  Ic_pos + B_pos * omega
omega < 0: Iq = -Ic_neg + B_neg * omega
```

其中 `omega` 为机械角速度（rad/s），`Ic` 单位为 A，`B` 单位为 A/(rad/s)。该功能只辨识并保存模型，不执行摩擦补偿。

稳态电流包含轴承、密封、铁耗、风阻和逆变器误差形成的等效阻力。若机构带有重力、弹簧或外部负载，这些负载也会进入结果；没有外部力矩测量时无法与方向不对称摩擦完全分离。

## 2. 运行条件和运动序列

启动前必须满足：

- 当前模式为 `Motor_Disable` 且无错误；
- TLE5012B 在线，编码器线性化和电零位均有效；
- 电流限制大于零，速度限制不低于 `0.5 rev/s`；
- 速度 PI 参数有效；
- 电机空载，并且正反两个方向均有足够的连续旋转空间。

固件依次运行以下八个速度点：

```text
+0.10, -0.10, +0.20, -0.20, +0.40, -0.40, +0.50, -0.50 rev/s
```

每个点先用 `0.75 s` 窗口的平均速度确认稳态，然后采集至少一个完整机械转和至少 `0.5 s` 数据。低速瞬时纹波不会清空稳定窗口或整圈样本；整圈平均速度必须落在目标容差内。采样使用 `Iq_filt` 和实际机械速度的在线均值。每个速度点之后先闭环减速至零，再进入下一点。完整机械转采样用于平均齿槽转矩等位置周期扰动。

以下情况会停止 PWM 并报告 `FrictionIdentification_Error`：

- 速度跟踪、采样或停车超时；
- `Iq` 给定持续达到电流限制的 90%；
- 传感器数据无效；
- 正、反向电流方向不符合空载摩擦假设；
- 拟合残差超过验收阈值。

母线过压/欠压、过流、过温和编码器掉线仍由原有全局保护处理。

## 3. 模块边界

实现分为两个独立层次：

| 层次 | 文件 | 职责与依赖 |
| --- | --- | --- |
| 辨识算法 | `Foc/friction_identification.h/.c` | 实例化状态机、采样和拟合；只依赖 C 标准库，通过 `Config`、`Input` 和结果结构交换数据 |
| FOC 适配 | `Foc/foc_friction_identification.h/.c` | 将速度环、编码器、电流、保护和参数存储映射到算法接口 |

算法层不引用 `MotorControl_TypeDef`、`FOC_TypeDef`、编码器、PI 控制器、全局错误处理、硬件配置或通信接口，也不读取任何全局变量。速度点和验收阈值由适配层组装成配置传入；算法输出的唯一控制量是目标机械速度。USB、CAN、任务调度和错误处理只调用 FOC 适配层，不访问算法实例。

这种边界允许在其他电机平台上复用算法层，只需重新实现一层输入采集、速度目标执行和安全停机适配。

## 4. USB 使用

推荐使用上位机脚本：

```powershell
python tools/friction_identification.py COM5
```

默认行为是运行、导出原始点、用 Python 独立复算并写出 JSON，不会应用参数。审核结果后，可显式应用到 RAM：

```powershell
python tools/friction_identification.py COM5 --apply
```

同时写入 Flash：

```powershell
python tools/friction_identification.py COM5 --apply --save
```

除非使用 `--yes`，脚本会在运动前要求输入 `RUN`，应用候选值前要求输入 `APPLY`。

底层 USB 参数如下：

| 参数 | 含义 |
| --- | --- |
| `w_mod=19` | 启动辨识 |
| `w_mod=0` | 取消辨识并停车 |
| `r_fst` | 状态、失败原因、点序号、进度、候选有效位 |
| `r_fdt` | 顺序导出八个原始平均点 |
| `r_fcp` / `r_fcn` | 候选正/反向库仑摩擦电流 |
| `r_fvp` / `r_fvn` | 候选正/反向粘性系数 |
| `r_frp` / `r_frn` | 正/反向拟合 RMSE |
| `w_fap=1` | 仅在 Disable 状态将有效候选应用到 RAM |
| `r_fva` | 当前 RAM/Flash 模型是否有效 |
| `r_acp` / `r_acn` | 当前已应用的库仑参数 |
| `r_avp` / `r_avn` | 当前已应用的粘性参数 |
| `w_mod=9` | 使用原有参数保存流程写入 Flash |

`r_fst` 的状态值：`0 idle`、`1 tracking`、`2 sampling`、`3 stopping`、`4 complete`、`5 failed`。

失败原因值：`0 none`、`1 cancelled`、`2 invalid config`、`3 tracking timeout`、`4 sampling timeout`、`5 stop timeout`、`6 current saturation`、`7 invalid sample`、`8 fit rejected`、`9 controller safety fault`。原因 `9` 应结合 `r_err` 读取具体的过压、欠压、过流、过温或编码器错误。

## 5. CAN 接口

CAN 仍通过 `CAN_SET_MODE (0x00)` 写入模式 `19` 启动。新增参数 ID：

| ID | 参数 |
| --- | --- |
| `0x58` | 应用有效候选模型，数据写 `1.0f` |
| `0x59` / `0x5A` | 状态 / 失败原因 |
| `0x5B` / `0x5C` | 候选正/反向库仑参数 |
| `0x5D` / `0x5E` | 候选正/反向粘性参数 |
| `0x5F` / `0x60` | 正/反向 RMSE |
| `0x61` | 候选有效位 |
| `0x62` | 已应用模型有效位 |

原始八点数据仅通过 USB 导出。CAN 应用参数后仍需显式进入模式 `9` 才会保存到 Flash。

## 6. Flash 兼容性

参数 schema 升级为 v9，在 v8 末尾追加四个摩擦系数和有效位。v4-v8 参数仍可读取；旧 schema 或电流采样电阻档位变化时，摩擦模型按无效处理，其他已有迁移逻辑保持不变。
