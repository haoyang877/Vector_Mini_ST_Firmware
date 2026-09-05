# 新硬件与新电机快速适配指南

本文说明 Vector Mini ST 固件中哪些内容属于配置、配置的生效优先级，以及如何在不修改控制算法的前提下适配新电机、新 PCB、新编码器或新 MCU。

## 1. 先判断需要改哪一层

| 变化范围 | 主要修改位置 | 不应修改 |
| --- | --- | --- |
| 同一硬件更换电机 | `Firmware/Product/vector_mini_st_profile.h`、`motor_profiles.*` | Domain 算法、STM32 Platform |
| 同一电机增加/移除阻尼器 | `mechanical_load_profiles.*`、`ACTIVE_MECHANICAL_LOAD_PROFILE` | 电流环、编码器算法 |
| 同一 PCB 调整限流、速度、位置增益 | USB/CAN 运行时参数；稳定后回填 MotorProfile | Domain、CubeMX 配置 |
| 更换采样电阻或运放增益 | `current_sense_profile.h`、`board_profile.c` | 电流换算算法 |
| 新 PCB、引脚或外设实例变化 | `Vector_Mini_ST.ioc`、`Firmware/Platform/<target>/`、BoardProfile | Domain、Application |
| 同协议、不同分辨率编码器 | `encoder_profiles.*` | 电机控制算法 |
| 不同编码器协议 | 新建 `RotorSensorPort` Adapter，并在 Composition 注入 | Domain 编码器模型 |
| 新 MCU | 新建 Platform 目录、CubeMX 工程和目标工程配置 | Application、Domain |
| Flash 容量或分区变化 | `parameter_store_flash.c`、Keil ROM 区域/Scatter、Bootloader 布局 | ParameterManager 算法 |

基本原则：硬件或产品变化应停留在 Product、Platform 和 Composition。如果为了换 PCB 或电机而修改 `Firmware/Domain/`，通常表示抽象边界仍不完整。

## 2. 配置数据如何进入运行系统

```text
编译期选择 ACTIVE_BOARD_PROFILE / ACTIVE_MOTOR_PROFILE / ACTIVE_MECHANICAL_LOAD_PROFILE
                         |
                         v
Product 构造只读 BoardProfile / MotorProfile / EncoderProfile
                         |
                         v
Composition 注入 Runtime 和 Application Service
                         |
                         v
ParameterSnapshot_LoadDefaults() 生成运行默认配置
                         |
             +-----------+-----------+
             |                       |
             v                       v
兼容 Flash 记录覆盖默认值       无兼容记录则继续使用默认值
             |
             v
USB/CAN 在 STANDBY 中写候选配置
             |
             v
20 kHz 安全点整对象应用
             |
             v
显式执行模式 9 后保存到 Flash A/B 槽
```

配置优先级为：

1. 与产品 ID、硬件 Profile ID、电机 Profile ID、参数 Schema 均兼容的 A/B Flash 记录；
2. 旧格式 ParameterSnapshot 兼容迁移；
3. Product Profile 的编译默认值。

更换 Profile ID 可以阻止新的 A/B 记录误用旧配置，但旧格式回退记录不带完整 Profile ID。新硬件或新电机首次量产烧录时，应擦除参数页，或者升级 Schema 并明确实现迁移策略。

## 3. 编译期配置清单

### 3.1 产品选择和默认参数

文件：`Firmware/Product/vector_mini_st_profile.h`

当前采用单一活动 Profile 的编译模式：

```c
#define ACTIVE_MOTOR_PROFILE MOTOR_PROFILE_HT8115_4
#define ACTIVE_BOARD_PROFILE BOARD_PROFILE_VECTOR_MINI_ST
#define ACTIVE_MECHANICAL_LOAD_PROFILE MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
```

也可以通过编译器宏覆盖这些选择。每个量产组合必须使用唯一且稳定的 Board/Motor Profile ID。

机械负载当前提供两个选择：

| 选择 | 编译器宏值 | 用途 |
| --- | --- | --- |
| 无阻尼器 | `MECHANICAL_LOAD_PROFILE_NO_DAMPER` | 原始轻载标定参数，位置摩擦前馈为 0 |
| 约 1.5Nm 阻尼环 | `MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM` | 高转矩观测器标定及 1.5/1.55A 摩擦前馈 |

Keil 的 `C/C++ > Define` 中设置：

```text
ACTIVE_MECHANICAL_LOAD_PROFILE=MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
```

移除阻尼器时只替换为 `MECHANICAL_LOAD_PROFILE_NO_DAMPER`，不修改 Runtime 或 Domain。

当前电机默认值：

| 参数 | 当前值 | 内部单位 | 说明 |
| --- | ---: | --- | --- |
| 极对数 | 21 | — | 不是磁极总数 |
| 相电阻 | 1.905 | Ω | 当前定义为 `3.81 × 0.5` |
| 观测器电阻系数 | 0.4199475 | — | 等效观测器电阻 `1.905 × 0.4199475 ≈ 0.800 Ω` |
| D/Q 轴电感 | 0.001635 / 0.001635 | H | 1.635 mH |
| 永磁磁链 | 0.0175025 | Wb | 17.5025 mWb |
| 默认标定电流 | 3 | A | 由 6 mΩ 电流采样档决定 |
| 默认运行限流 | 6 | A | 必须低于板级命令上限 |
| 默认速度限制 | 0.5（阻尼器）/ 6.2（无阻尼器） | rev/s | 由机械负载 Profile 选择 |
| 电机模型运行时范围 | R: 0.0001–5；L: 1 µH–5 mH；磁链: 0.01 mWb–1 Wb | SI 单位 | 由 MotorProfile 统一校验 |
| 电流环带宽 | `500 × 2π` | rad/s | 默认加载及在线修改 R/L 时用于推导 PI |
| 开环电压 | 1 | V | 仅用于开环/标定 |
| 开环电角速度 | 12 | rad/s | — |
| 速度加/减速度 | 50 / 50 | rev/s² | 加载后转成 rad/s² |
| 速度 Kp/Ki | 0.05 / 0.5 | 当前控制器定义 | 需要实机整定 |
| 位置加/减速度 | 0.125 / 0.125 | rev/s² | — |
| 位置最大速度 | 0.5（阻尼器）/ 0.125（无阻尼器） | rev/s | 由机械负载 Profile 选择 |
| 阻抗位置 Kp/Kd/Ki | 8 / 0.5 / 10 | A/rad 等 | 输出为 Iq |
| 位置积分限幅 | 5 | A | 还受板级电流上限限制 |
| 级联位置 Kp/Kd | 0.05 / 0.5 | 1/s、无量纲组合 | 位置外环输出速度 |

相电阻辨识默认值也属于 MotorProfile：测试电流 1/2/3 A、最低有效电流 0.5 A，斜坡 200 ms，稳定 200 ms，采样 100 ms，暂停 100 ms，总超时 3000 ms，电流/电压容差、滤波系数以及相间差异告警/故障阈值 3%/5%。测试母线窗口直接使用 BoardProfile 的欠压/过压阈值。

### 3.2 BoardProfile

文件：`Firmware/Product/board_profile.h`、`board_profile.c`、`current_sense_profile.h`

BoardProfile 是硬件电气特性的单一只读视图：

| 分组 | 配置字段 |
| --- | --- |
| 身份与时序 | `profile_id`、`control_frequency_hz` |
| 电流采样 | 采样电阻、默认/最小/最大 ADC 零偏、零偏标定样本数、A/LSB、命令上限、标定上限、过流阈值 |
| 母线采样 | V/LSB、欠压/过压阈值 |
| 温度 | 最大温度、NTC 串联电阻、标称电阻、Beta、标称温度 |
| 逆变器 | 死区时间、相电阻路径补偿 |
| 故障确认 | 过流、母线电压确认周期，温度采样分频 |
| 通信默认 | CAN 节点 ID、心跳超时 |

当前板级值：

| 项目 | 当前值 |
| --- | ---: |
| 控制/PWM 频率 | 20 kHz |
| ADC 参考与满量程 | 3.3 V / 4095 count |
| 电流运放增益 | 10 |
| 默认采样电阻 | 6 mΩ |
| 电流换算 | 约 0.01343 A/count |
| 母线分压 | 10 kΩ / 1 kΩ |
| 母线换算 | 约 0.008864 V/count |
| ADC 零偏默认/范围 | 2048 / 1948–2148 count |
| 零偏标定样本数 | 20000 个 20 kHz 样本，即 1 s |
| 命令/标定/过流阈值 | 10 / 10 / 18 A |
| 欠压/过压 | 10 / 30 V |
| 最高温度 | 100 °C |
| 逆变器模型死区 | 210 ns |
| NTC | 10 kΩ，Beta 3455 K，25 °C |
| 过流确认 | 5 个 20 kHz 周期 |
| 电压确认 | 10000 个 20 kHz 周期，即 0.5 s |
| 温度采样分频 | 20 |
| CAN 节点/心跳 | 0 / 500 ms |

`current_sense_profile.h` 当前只内置 2 mΩ 和 6 mΩ 两档。新采样电阻或新运放增益必须增加一个完整档位，不能只改 A/LSB；命令限流、标定限流、过流阈值、默认电流和功率路径补偿也要一起评审。

### 3.3 EncoderProfile

文件：`Firmware/Product/encoder_profiles.*`

| 字段 | 当前值 | 说明 |
| --- | ---: | --- |
| Profile ID | 1 | 当前未提供独立的 `ACTIVE_ENCODER_PROFILE` |
| 每圈计数 | 65536 | TLE5012B 转成无符号 Q15 环形计数 |
| 速度更新分频 | 10 | 20 kHz / 10 = 2 kHz |
| 速度采样周期 | 0.5 ms | 必须与分频一致 |
| 默认电零位/机械零位 | 0 / 0 | 标定后由 Flash 覆盖 |
| 默认标定标志 | 0 | 新设备必须标定 |
| 默认反向 | 0 | 安装方向必须实机确认 |

编码器线性化 LUT、方向、电零位、机械零位属于运行时持久化数据，不应写死在通用算法里。

### 3.4 控制与标定调参 Profile

文件：`Firmware/Product/control_tuning_profile.*`

该 Profile 包含两组参数：

- 无感启动与磁链观测器：对齐电流斜坡/保持时间、Id/Iq、电角速度目标、启动斜坡、锁定比例、角度交接、丢锁时间、最大观测速度、observer gamma、观测器电阻系数、最大校正步长、最小磁链和低通系数；
- 编码器标定：电角度线性化的对齐/加速时间、扫描电角速度和失锁超时，以及机械扫描速度、稳速判据、扫描圈数、验证圈数、每 LUT 桶最小样本数、每周期构建桶数、各阶段超时、RMS/峰值误差门限和停止减速/降流时间。

这些参数通常不通过现场协议开放。新电机只在惯量、摩擦、磁链或允许标定电流明显变化时才需要修改；修改后必须重新执行标定超时、停机距离和失锁保护测试。

机械负载相关差异单独放在 `mechanical_load_profiles.*`。1.5Nm 阻尼器标定使用
3.5A 对齐电流、3.5→4.5A 启动 Iq、5.5A 最低相电流限值、0.01 锁速滤波系数、
5s 锁定超时和 5 个机械采样圈；无阻尼器保留 2A 对齐、0.15→0.5A 启动 Iq、
原锁定时间和 10 个采样圈。阻尼器的电角零位对齐最低使用 4.5A。

磁链观测器使用：

```text
observer_Rs = phase_resistance_ohm × flux_observer_resistance_scale
```

`phase_resistance_ohm` 仍是电机真实相电阻，用于电流环 PI、辨识和其他电机模型；
`flux_observer_resistance_scale` 是 PWM 基波电压模型在目标工作点的等效修正，只属于
ControlTuningProfile。当前带约 1.5Nm 阻尼器实测：真实相电阻保持 `1.905 Ω`、系数
`0.4199475`（等效 `0.800 Ω`）时，Mode 13 连续三次完成锁定、闭环、LUT 构建、
验证和自动保存。不得把 `mrs` 临时改成 `0.800 Ω` 后作为量产电机参数保存。

### 3.5 控制周期

文件：`Firmware/Product/control_loop_config.h`

| 控制环 | 当前频率 | 相对 20 kHz 分频 |
| --- | ---: | ---: |
| 电流环 | 20 kHz | 1 |
| 速度环/速度估算 | 2 kHz | 10 |
| 位置阻抗/轨迹 | 1 kHz | 20 |
| 位置级联外环 | 5 kHz | 4 |

若修改 20 kHz 基频，必须同时核对：TIM1 实际频率、ADC 触发、`BoardProfile.control_frequency_hz`、DWT deadline、各控制分频和所有按控制周期计数的超时。所有子频率必须整除基频。

### 3.6 ProductManifest、参数 Schema 和镜像契约

文件：`product_manifest.*`、`parameter_schema.h`、`Bootloader/image_contract.h`

需要维护：产品 ID、MCU ID、硬件修订、硬件 Profile ID、电机 Profile ID、固件版本、构建号、量产标志、参数 Schema 和 Bootloader 契约版本。

以下情况通常需要增加参数 Schema：

- `ParameterSnapshot` 增删字段或改变字段含义/单位；
- 旧值无法安全迁移；
- 新硬件不能安全使用旧的校准或缩放数据。

仅调整某个产品的默认值时，不应随意增加 Schema；应使用新的 Board/Motor Profile ID 隔离记录。

## 4. 运行时可配置参数

运行时参数只允许在 `STANDBY` 写入。Application 校验后写候选配置，20 kHz 在安全点提交；除非显式执行保存服务，否则断电后不会保留。

| 参数 | USB 三字符 | 协议写入单位 | 当前范围 | 当前默认 |
| --- | --- | --- | --- | ---: |
| 极对数 | `pol` | 整数 | 2–30 | 21 |
| 标定电流 | `ica` | A | 0–板级标定上限 10 A | 3 |
| 电流限制 | `ilm` | A | 0–板级命令上限 10 A | 6 |
| 速度限制 | `slm` | rev/s | >0，且不超过 Profile 上限 | 0.5（当前阻尼器 Profile） |
| 速度加速度 | `sac` | rev/s² | 0–1000 | 50 |
| 速度减速度 | `sde` | rev/s² | 0–1000 | 50 |
| 速度 Kp | `s_p` | 控制器系数 | 0.01–2 | 0.05 |
| 速度 Ki | `s_i` | 控制器系数 | 0–2 | 0.5 |
| 位置加速度 | `pac` | rev/s² | >0–200 | 0.125 |
| 位置减速度 | `pde` | rev/s² | >0–200 | 0.125 |
| 位置最大速度 | `pms` | rev/s | >0、≤机械负载 Profile 上限且 ≤总速度限制 | 0.5（当前阻尼器 Profile） |
| 阻抗位置 Kp | `p_p` | A/rad | 0–50 | 8 |
| 阻抗位置 Kd | `p_d` | A/(rad/s) | 0–10 | 0.5 |
| 阻抗位置 Ki | `p_i` | A/(rad·s) | 0–10 | 10 |
| 位置积分限幅 | `p_l` | A | 0–10 | 5 |
| 级联位置 Kp | `c_p` | 1/s | 0–50 | 0.05 |
| 级联位置 Kd | `c_d` | 当前控制器定义 | 0–10 | 0.5 |
| 相电阻 | `mrs` | 写入为 Ω | Profile 当前范围 0.0001–5.0 Ω | 1.905 Ω |
| D 轴电感 | `mld` | 写入为 H | 1 µH–5 mH | 1.635 mH |
| Q 轴电感 | `mlq` | 写入为 H | 1 µH–5 mH | 1.635 mH |
| 磁链 | `mfx` | 写入为 Wb | 0.01 mWb–1 Wb | 17.5025 mWb |

其他通信/编码器配置（持久化情况见说明）：

| 项目 | USB 三字符 | 范围/说明 |
| --- | --- | --- |
| CAN 节点 ID | `cid` | 0–7 |
| CAN 波特率 | `cbr` | 100、125、200、250、500、1000、2000、2500、5000 kbit/s；当前不写入 ParameterSnapshot |
| CAN 心跳 | `chb` | 0 表示关闭，或 500–1000 ms |
| 编码器方向 | `erv` | 0/1；修改会使已有方向相关标定失效，应重新标定并保存 |
| 编码器 LUT/电零位/机械零位 | 通过服务生成 | 不应作为普通数值直接写入 |

### 4.1 当前运行时配置限制

必须注意以下实现事实：

1. 在线修改 R、Ld、Lq 时，Runtime 会使用活动 MotorProfile 的电流环带宽同步重算对应的 `d/q current Kp/Ki`。如果新电机需要不同带宽，仍应建立新的编译期 MotorProfile。
2. R/L/磁链的运行时上下限由 MotorProfile 提供；当前 HT8115-4 Profile 的相电阻范围是 0.0001–5.0 Ω，覆盖其 1.905 Ω 默认值。
3. USB 写 `mrs/mld/mlq/mfx` 使用 SI 单位 Ω/H/H/Wb，但 USB 读取文本分别显示 mΩ/µH/µH/mWb，读写单位并不对称。配置工具必须显式换算。
4. CAN 波特率虽然可运行时切换，但当前 `ParameterSnapshot` 不保存它，复位后回到接口默认 1000 kbit/s。
5. Board、Motor 和 MechanicalLoad Profile 均为编译期单选，不支持运行时切换硬件组合。
6. 开环电压、开环电角速度、初始电角度和位置误差窗口不写入 ParameterSnapshot；每次加载默认值或有效 Flash 记录时都会从活动 MotorProfile 重新应用。
7. `mechanical_load_profiles.c` 已提供可按 ID 查询的双配置表；Board、Encoder 和 ControlTuning 仍只构造一个活动对象。
8. Product Profile 已使用具名初始化器，新增字段时不会静默错位；仍应在首次上电前执行 Profile 参数审查和硬件验证。
9. `mrs` 必须保存实测物理相电阻；Mode 13 的速度观测偏差应通过
   `flux_observer_resistance_scale` 修正，不能通过伪造 `mrs` 或放宽锁定门限处理。

因此推荐：运行时协议用于实验整定速度、位置和限幅；新电机的 R/L/磁链、电流环带宽和安全上限应写入新的编译期 MotorProfile，然后恢复该 Profile 默认值再标定。

## 5. 同一硬件快速适配新电机

### 5.1 准备参数

至少准备：

- 极对数；
- 每相电阻，明确是相电阻还是线间电阻；
- Ld、Lq；
- 永磁磁链或可靠 Kv；
- 允许连续/峰值相电流；
- 最高机械速度；
- 推荐标定电流；
- 转动惯量、负载和允许加速度。

所有电气参数必须转换为固件内部 SI 单位：Ω、H、Wb、A、rad/s。接口速度使用 rev/s，进入固件后乘 `2π`。

### 5.2 创建量产 MotorProfile

1. 在 `vector_mini_st_profile.h` 增加唯一的 `MOTOR_PROFILE_<name>` ID。
2. 增加对应 `#elif ACTIVE_MOTOR_PROFILE == ...` 默认参数块。
3. 核对 `motor_profiles.c` 中的安全上限；新电机需要不同上限时，将上限也纳入按 Profile 选择的配置，而不是修改 Domain。
4. 用编译器宏或头文件选择新的 `ACTIVE_MOTOR_PROFILE`。
5. 确认 ProductManifest 中的 `motor_profile_id` 已变化。
6. 擦除旧参数页，或进入 Standby 后执行恢复默认值服务并保存。

### 5.3 USB 快速试验顺序

以下报文末尾必须带 `\r\n`：

```text
\w_mod=0          进入 Standby
\w_pol=新极对数
\w_ica=低风险标定电流A
\w_ilm=低风险运行限流A
\w_slm=低风险速度上限rev/s
\w_sac=低风险加速度rev/s2
\w_sde=低风险减速度rev/s2
\w_mod=9          保存参数
```

若临时写电机模型，单位必须如下：

```text
\w_mrs=0.250      0.250 Ω，不是 0.250 mΩ
\w_mld=0.000500   500 µH
\w_mlq=0.000500   500 µH
\w_mfx=0.012000   12 mWb
```

这组在线写入会按活动 MotorProfile 的带宽同步重算电流环 PI，适合低风险台架试配。若新电机的目标带宽不同，用于正式上电控制前仍应生成新的编译期 MotorProfile、恢复默认值并重新标定。

### 5.4 标定与首次运行

1. 空载或机械安全状态上电，等待自动模式 11 电流零偏标定完成并回到模式 0。
2. 核对三相零电流读数、母线电压和温度。
3. 用极低电流/电压检查相序和编码器方向；必要时在 Standby 设置 `erv=0/1`。
4. 执行模式 5 或 13 完成编码器线性化。
5. 执行模式 15 完成电零位标定。
6. 需要位置坐标原点时执行模式 7。
7. 从低限流电流模式开始，再依次验证速度和位置控制。
8. 最后执行模式 9 保存，并断电重启验证参数和标定数据读回。

若选择模式 13，新电机首次适配还要完成观测器电阻系数确认：

1. 保持 `mrs` 为实测相电阻，使用编码器 RTT 同时记录实际电角速度、观测器电角速度
   和锁速低通值；
2. 只在限压、限流、可自由旋转且可立即停机的台架上调整
   `flux_observer_resistance_scale`；该值是编译期 Product 调参，不开放现场协议；
3. 以目标匀速段的观测器/编码器速度比接近 1、能进入 handoff/closed-loop、无持续
   相位发散为通过条件；不能只看 Mode 13 是否超时；
4. 至少连续执行两次 Mode 13，随后执行 Mode 15，硬件复位后确认 `mrs`、LUT 和电角
   零位均恢复，最后再做低速闭环回归。

## 6. 快速适配新 PCB 或新硬件修订

### 6.1 Product 配置

1. 新增 `BOARD_PROFILE_<name>` 和唯一 Profile ID。
2. 设置电流采样电阻、运放增益、ADC 参考、母线分压、NTC 和保护阈值。
3. 设置功率级死区模型、最大电流/电压/温度和故障确认周期。
4. 更新 `PRODUCT_HARDWARE_REVISION` 和 ProductManifest。
5. 检查新硬件是否允许沿用旧的编码器标定与参数 Schema。

### 6.2 CubeMX 和 Platform

在 `Vector_Mini_ST.ioc` 中确认并重新生成：

- TIM1 三相互补 PWM、CH4 ADC 触发、中心对齐、频率、极性、死区和 Break；
- ADC1/ADC2 注入通道、Rank 顺序、采样时间和 TIM1 触发边沿；
- 编码器 SPI 模式、频率、数据宽度、CS/MOSI 管脚；
- FDCAN 时钟、采样点、过滤器容量和可支持波特率；
- USB、1 kHz TIM7、LED/RGB 资源；
- 中断优先级，保证 20 kHz 控制优先于通信和后台任务。

当前 Platform 中存在以下固定映射，新 PCB 必须逐项检查：

| 文件 | 当前固定内容 |
| --- | --- |
| `measurement_adc12.c` | ADC2 JDR1/2/3 = Ia/Ib/Ic，JDR4 = Vbus；ADC1 JDR1 = 温度 |
| `power_stage_tim1.c` | TIM1 CH1/2/3 及互补输出 |
| `rotor_sensor_tle5012b.c` | SPI2、TLE5012B SSC、PB15 MOSI 模式切换、板载 CS |
| `can_fdcan1_transport.c` | FDCAN1 和当前时钟下的分频计算 |
| `board_runtime.c` | ADC1/2、TIM1 CH4 和 TIM7 启动顺序 |
| `indicator_stm32g431.c` | LED GPIO、TIM2 DMA RGB |

如果只是同 MCU 的新 PCB，可以增加 `Firmware/Platform/Stm32G431/<board>` Adapter 并由 Composition 选择；如果 MCU 或 HAL 句柄体系变化，应建立新的 `Firmware/Platform/<mcu>/`，不要在原 Adapter 中堆积大量板型条件分支。

### 6.3 Flash 和链接布局

当前应用 ROM 为 `0x08000000–0x0801BFFF`，参数 A/B 槽为：

```text
Slot 0: 0x0801C000，8 KiB
Slot 1: 0x0801E000，8 KiB
```

Flash 型号或 Bootloader 布局变化时，必须同时修改：

- Keil Target ROM 区域和生成的 Scatter；
- `parameter_store_flash.c` 页大小、槽大小和地址；
- Bootloader 镜像槽、邮箱和回滚区域；
- 链接后镜像越界检查。

严禁只修改 Flash Adapter 地址而不修改链接区域。

## 7. 适配新编码器

当前量产实现只支持板载 TLE5012B。

同协议、仅分辨率或速度估算频率不同：

1. 新增 Encoder Profile ID；
2. 修改每圈计数、速度分频和采样周期；
3. 默认清除标定标志、零位和 LUT；
4. 重新执行线性化与电零位标定。

不同协议或外设：

1. 新建实现 `RotorSensorPort` 的 Platform Adapter；
2. Adapter 负责 SPI/ABI/CRC/状态位和原始角度归一化；
3. 在 Composition 中选择并注入该 Port；
4. Domain Encoder 继续只处理标准化计数、方向、LUT、机械/电角度和速度；
5. 添加断连、CRC、跨零点、方向和最大速度测试。

## 8. 仍散落在 Profile 之外的硬编码配置

以下数值目前会影响硬件或电机适配，修改产品时必须纳入评审：

| 位置 | 当前内容 | 建议归属 |
| --- | --- | --- |
| `rotor_sensor_tle5012b.c` | SPI 等待上限、PB15 寄存器位和 AF5 | Platform 配置 |
| `parameter_store_flash.c` | Flash 页面和槽地址 | Product memory layout |
| `interface_can.c` | 默认 1000 kbit/s、允许的波特率和心跳范围 | Communication/Product Profile |
| CubeMX TIM1 | 当前 DeadTime 配置为 0，而 BoardProfile 模型为 210 ns | 必须确认由定时器还是门驱动器提供实际死区 |

协议 ID、队列容量、CRC 多项式、数学常量等通常属于协议或算法不变量，不应为了换电机随意修改。

## 9. 必做验证门禁

### 9.1 构建和静态检查

```powershell
pwsh -NoProfile -File tools/verify_architecture.ps1
```

- Keil 全量 Rebuild 必须 0 error、0 warning；
- 检查所有工程文件存在、IncludePath 正确；
- 检查应用镜像不覆盖参数槽或 Bootloader 区域；
- 新 Platform 之外不得出现 HAL/寄存器依赖。

### 9.2 无电机或限能量台架

- 上电、复位、故障和通信超时时 PWM 始终关闭；
- 三相高低桥极性、互补关系、死区和 Break 实测正确；
- ADC Rank 与 Ia/Ib/Ic/Vbus/温度对应正确；
- 零电流 ADC 位于允许窗口；
- 电流、母线电压和温度用外部仪表校准；
- 过压、欠压、过流、过温注入能进入 FAULTED 并关闭功率。

### 9.3 电机验证

- 极对数、相序、编码器方向和电角方向一致；
- 低电流模式下 Id/Iq 方向正确且无明显直流偏置；
- R/L/磁链单位正确，电流 PI 与目标带宽相符；
- 标定过程中电流、速度、持续时间均低于硬件限制；
- 电流 → 速度 → 位置逐级升能量验证；
- 模式切换、停机和故障过程无非预期扭矩脉冲；
- 20 kHz DWT 最大周期低于 deadline 且超限计数为 0。

### 9.4 参数持久化

- 新 Profile 首次烧录不加载旧板/旧电机参数；
- 保存后复位，所有参数和编码器标定可正确恢复；
- A/B 槽写入、擦除、校验和提交期间分别断电，至少保留一个有效旧记录；
- 恢复默认值后不会自动启动电机；
- Schema 迁移和不兼容记录均回退到安全默认值并留下诊断记录。

## 10. 最短执行清单

### 只换电机

```text
新 Motor Profile ID
→ 填 R/L/磁链/极对数/电流/速度/带宽
→ 低安全上限
→ 全量构建
→ 擦除旧参数或恢复默认值
→ 零偏/方向/线性化/电零位标定
→ 电流/速度/位置逐级验证
→ 保存并重启验证
```

### 只换 PCB

```text
新 Board Profile ID 和硬件修订
→ 电流/电压/温度换算与保护阈值
→ CubeMX PWM/ADC/SPI/FDCAN/中断
→ Platform Adapter 映射
→ Flash/链接布局
→ 无电机波形与保护测试
→ 低能量电机测试
→ 掉电与持久化测试
```

### 换 MCU 或编码器协议

```text
保留 Application/Domain
→ 新 Platform Adapter
→ 新 CubeMX/Keil 目标
→ Composition 注入新 Port
→ 更新 ProductManifest 和内存布局
→ 完整主机、目标、保护和协议回归
```
