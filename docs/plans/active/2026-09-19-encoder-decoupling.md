# Encoder 解耦：传感器通道 / SPI 传输 / 角度输出 v2.0

日期：2026-09-19。状态：**阶段 A 已完成（本轮提交）；阶段 B/C/D 待办**。Q1/Q2/Q3/Q6/Q8 按推荐执行；Q4 已确认"保留"。

范围：`firmware/platform/stm32g4/bsp/encoder.{c,h}`（428 + 120 行）的解耦。
目标三层：**硬件的归硬件、通信（传感器协议）的归通信、输出角度的归输出角度**；
并为本项目后续更换传感器（当前 TLE5012B，未来 MT6701 / MT6535 等）提供
**"传感器功能通道 + 每型号 driver"** 的可替换结构。

---

## 1. 现状

### 1.1 一个 BSP 模块里混了三层

| 层 | 现有实现 | 位置 |
| --- | --- | --- |
| 硬件/传输 | `SPI_WaitFlag`/`SPI_WaitBSYClear`/`SPI_Reg_ReadRx16`/`SPI_Reg_TxRx16`（SPI2 寄存器自旋，超时 340）、`SPI2_MOSI_HiZ`/`RestoreAF`（GPIOB MODER/AFR）、`BRD_ENC_CS_ENABLE/DISABLE`、`brd_enc_spi`/`BRD_ENC_SPI`、`HAL_SPI_Init`+`Error_Handler`、`Encoder_BeginSample` 的总线就绪判定 | encoder.c + encoder.h 宏 |
| 通信/协议 | TLE5012B SSC：请求帧 `0x8021`、读相位（释放 MOSI + 2×NOP + 交换 0）、`(word & 0x7FFF) << 1` 解码、`Encoder_ReadStatus` 分类、坏帧 streak/错误计数、TLE 诊断字段 | encoder.c |
| 角度输出与校验 | 方向取反、LUT 线性化（1024 点插值）、电零位→θe、多圈 `shadow_q15`→θm、2 kHz 滑动窗速度估计、`Encoder_IsOnline`、标定读写（电零位/机械零位/反向） | encoder.c |

### 1.2 反向依赖与死路径

- BSP 吃 motor 状态：`Encoder_Update/CompleteSample(MotorControl_TypeDef*, …)`；
- BSP 调 services 策略：首拍多圈解析 `MotorAxisProfile_InitialEncoderOffsetQ15()`
  （定义在 `services/parameters/motor_axis_profile.{c,h}`）；
- `encoder.h` 暴露寄存器/引脚宏与 `main.h`，被 motor/services/communication 多处包含（债 5 条）；
- **死路径**：`Encoder_ReadStatus` 中 `CRC_MISMATCH`、`MAGNET_*`、`OVERSPEED`、
  `TLE_RESET/SYSTEM/INTERFACE_ERROR`、`TLE_INVALID_ANGLE` 从不产生；`tle5012_safety_word`
  恒 0、CRC 计数恒 0（安全字/CRC 未实现）。

### 1.3 调用面（决定阶段 C 改动量）

- 实例：`Encoder_TypeDef OnBoard_Encoder`（定义 `app/motor_state.c`，extern `motor_state.h`）。
- 经 getter：`foc_mode_dispatch.c`、`foc_run.c`、`foc_run_state.c`、`foc_task.c`、`motor_state.c`、
  `rtt_telemetry.c`、`interface_can.c`、`foc_cogging_calibration.c`、`foc_friction_identification.c`、
  `foc_errhandle.c`。
- **字段直读**：`bad_frame_streak`（`foc_run_state.c` ×2）、`calib_flag`（`foc_run.c`、friction）、
  `reverse`（`interface_can.c` ×3）、`theta_mech`/`vel_mech`（`rtt_telemetry.c`）、
  `has_valid_sample`（夹具桩）。
- 持久化：`services/parameters/foc_param.c` 读写 `electrical_zero_q15`/`mechanical_zero_q15`/
  `linearization_lut_q15`/`calib_flag`/`reverse`。
- 电零位标定入口：`Encoder_SetElectricalZero` / `Encoder_SetElectricalZeroQ15` **当前无调用方**，
  原因是 mode 15（`Calib_EleAngelOffset`）电零位标定随 FOC-Calibration 功能删除而移除
  （标定流程见 `docs/guides/calibration_process.md`：对齐后把平均 `linearized_q15` 写入
  `electrical_zero_q15` 并置位标志）。这两个 API 属于待重建的标定路径，**保留**，
  阶段 C 迁移时签名与语义不变。`Encoder_GetCountInCPR_Ratio` 同样保留（遥测/调试备用）。
- 20 kHz 时序：`foc_task.c` 先 `Encoder_BeginSample()`，后 `Encoder_CompleteSample(...)`；
  两次调用之间禁止访问同一总线；`Encoder_Update()` 为同步回退。
- 夹具：`test_encoder_sample_overlap.py` 桩掉寄存器与 CS 宏，切片 `Encoder_BeginSample` +
  `Encoder_ReadTle5012BFrame`（256 种 SR 组合、同帧/回退/两类失败/CS+HiZ 清理）——已等价于
  传输层契约测试。另有 8 个夹具以函数桩/结构体桩使用 encoder API。
- 债：style 豁免 3 个文件（`encoder.c`/`encoder.h`/overlap 夹具）；`encoder.h` 接口债 17 条；
  架构债 5 条指向 `encoder.h`。

---

## 2. 新增需求：传感器可替换

当前 TLE5012B 未来可能换成 MT6701 / MT6535 等。因此不能把 TLE 的帧格式、位宽、CRC 规则
写进角度层或传输层，而要形成**传感器功能通道**：同一组接口，每个型号一个 driver。

---

## 3. 目标架构

```text
[角度层]      motor/position/angle_feedback.{c,h}
                方向 / LUT / 电零位 / θe·θm / 多圈 / 速度估计 / 在线判定
                （与传感器型号无关；只消费归一化样本）
                     ^ EncoderSensorSample{status, angle_q15, frame/safety(诊断), crc_ok}
[传感器通道]  platform/api/encoder_sensor.h        ← 功能通道契约（每型号一份实现）
              ports/motor/encoder_tle5012b.c       ← 现状型号
              ports/motor/encoder_mt6701.c         ← 未来
              ports/motor/encoder_mt6535.c         ← 未来
              ports/motor/encoder_sensor_stm32g4.c ← 板级选择/分发（见 §5）
                     ^ 原始 16 位帧
[SPI 传输]    platform/api/encoder_spi.h
              ports/motor/encoder_spi_stm32g4.c    ← SPI2 收发/CS/MOSI 方向/超时/初始化
```

### 3.1 传输契约（`platform/api/encoder_spi.h`）

```c
void encoder_spi_init(void);                       /* 上电初始化 SPI/引脚；失败走致命错误路径 */
bool encoder_spi_read_begin(uint16_t request_frame);   /* 异步：片选+请求帧；忙/未使能返回 false */
bool encoder_spi_read_complete(bool begin_ok, uint16_t request_frame, uint16_t read_frame,
                               uint16_t *word);    /* 读回响应→切读相位→取数据帧→释放片选 */
```
契约：不暴露 SPI 句柄/寄存器/引脚；`read_complete` 无论成败都释放片选与 MOSI 方向；
`begin_ok=false` 时内部补发请求帧（同步回退）；两次调用之间禁止同一总线上的其它访问。

### 3.2 传感器通道契约（`platform/api/encoder_sensor.h`）

```c
typedef enum
{
    ENCODER_SENSOR_OK = 0,
    ENCODER_SENSOR_BUS_TIMEOUT,   /* 总线超时 */
    ENCODER_SENSOR_FRAME_ERROR,   /* 帧/校验失败（该型号支持 CRC/安全字时） */
    ENCODER_SENSOR_SENSOR_FAULT,  /* 传感器自报故障（磁场/复位/接口） */
    ENCODER_SENSOR_NO_SAMPLE      /* 本轮没有新样本 */
} EncoderSensorStatus;

typedef struct
{
    EncoderSensorStatus status;
    uint16_t angle_q15;   /* 单圈角度，Q15（driver 负责把该型号位宽归一化到 0..65535） */
    uint16_t frame_word;  /* 原始数据帧；仅诊断 */
    uint16_t safety_word; /* 状态/安全字；无该字段的型号填 0 */
    bool crc_ok;          /* 无校验的型号恒 true */
} EncoderSensorSample;

void encoder_sensor_init(void);                        /* 上电初始化（driver 内部可调 encoder_spi_init） */
bool encoder_sensor_begin(void);                       /* 异步发起一次采样；不支持则返回 false */
EncoderSensorStatus encoder_sensor_complete(bool started, EncoderSensorSample *out);
EncoderSensorType encoder_sensor_type(void);           /* 型号标识，用于遥测/诊断 */
```
要点：
- 角度层只见 `EncoderSensorSample`，不见帧格式/位宽/CRC 规则；
- 每型号 driver 实现同一组符号；`TLE5012B` 的请求帧、读相位、`(word&0x7FFF)<<1` 与
  安全字/CRC 决策都在 driver 内；
- 既有 `Encoder_ReadStatus` 保留为**角度层对外的兼容状态**（遥测/报告在用），由
  `EncoderSensorStatus` 映射而来，数值语义不变。

### 3.3 角度层窄接口（阶段 C）

`Encoder_TypeDef` 定义迁到 `motor/position/angle_feedback.h`，并为字段直读提供窄接口：
```c
uint8_t  AngleFeedback_CalibFlags(const AngleFeedback *);      /* 取代 ->calib_flag */
uint16_t AngleFeedback_BadFrameStreak(const AngleFeedback *);  /* 取代 ->bad_frame_streak */
uint8_t  AngleFeedback_Reverse(const AngleFeedback *);         /* 取代 ->reverse */
float    AngleFeedback_MecPos(const AngleFeedback *);          /* 取代 ->theta_mech */
float    AngleFeedback_MecVel(const AngleFeedback *);          /* 取代 ->vel_mech */
```

---

## 4. 驱动选择机制（三方案）

| 方案 | 做法 | 优点 | 代价 |
| --- | --- | --- | --- |
| 1 链接期 | 每个工程/板型只把一个 driver `.c` 编进工程；换型号=改工程条目 + 新 driver 文件 | 零运行时代价、无 vtable、镜像最小 | 换型号需要动两个 Keil 工程 |
| **2 编译期分发（推荐）** | `hw_conf.h`（板级）定义 `ENCODER_SENSOR_TYPE`；`ports/motor/encoder_sensor_stm32g4.c` 用 `#if` 只编译并转发到选中 driver（各 driver 用 `xxx_sensor_*` 内部符号） | 换型号=改一个板级宏；与现有 `CURRENT_SENSE_SHUNT_MILLIOHM` 板级 profile 同例；通道契约不变 | 需保证宏与实现一致（加 `#error` 兜底） |
| 3 运行期表 | 函数指针表 + 运行期选择 | 运行期可换 | 20 kHz 间接调用、未用驱动进镜像；违反"固定控制流不用回调注册表" |

推荐方案 2；若你更倾向"零多余代码"，方案 1 同样满足"功能通道 + 每型号 driver"。

---

## 5. 阶段划分与验收

| 阶段 | 改动 | 验收 | 风险 |
| --- | --- | --- | --- |
| **A 传输层** | 新增 `encoder_spi.h` + `ports/motor/encoder_spi_stm32g4.c`；`encoder.c` 只经契约访问总线；`encoder.h` 删 `brd_enc_spi`/`BRD_ENC_SPI`/CS 宏并补 17 条中文契约（豁免全清）；overlap 夹具改为"端口寄存器级 + 协议 seam 级"两个子用例；双工程登记；债务基线清理 | 原生套件全 PASS（断言等价改写）；format/lint/architecture/interfaces 0 new；Keil 双目标 0/0 | 低（表达式/时序逐位保持，调用方零改动） |
| **B 传感器通道** | 新增 `encoder_sensor.h` + `encoder_tle5012b.c`（含干净的解码函数）；`encoder.c` 只消费 `EncoderSensorSample`；新增通道级向量测试（请求帧/解码/状态映射） | 向量 PASS；overlap 夹具改为通道 seam；`Encoder_ReadStatus` 映射等价 | 中（死路径决策见 Q3） |
| **C 角度层归位** | `AngleFeedback`（或保名 `Encoder_TypeDef`）迁 motor；字段直读改窄接口；`foc_param.c`/`interface_can.c`/`foc_run*.c`/`rtt_telemetry.c`/`foc_task.c` 同步；首拍轴向策略从 BSP 移出 | 调用方等价编译/行为；架构债 5 条清零；夹具同步 | **中-高**（签名与持久化字段面广） |
| **D 在线检查** | 现仅"valid + 坏帧<100"；是否新增冻结/跳变检查见 Q7 | checker 向量 | 低（若仅归位） |

阶段 B 完成后，**新增 MT6701/MT6535 只需**：加一个 driver 文件 + 板级宏选择 + 该型号的
解码向量；角度层与传输层零改动。

---

## 6. 必须逐位保持的不变量

1. 解码（现状 TLE）：`raw_q15 = (uint16_t)((word & 0x7FFF) << 1)`。
2. 传输时序：片选→请求帧→（读回响应）→释放 MOSI→2×NOP→读帧→等 BSY→释放片选；
   任一步失败也必须释放片选；失败映射 `ENCODER_READ_SPI_TIMEOUT`。
3. 总线就绪判定 `(CR1&SPE)==0 || (SR&(TXE|OVR|BSY|RXNE))!=TXE`；超时常数 340；
   TxRx 前自动置 SPE 并排空 OVR。
4. 角度数学：方向取反 `0 - raw`；LUT 插值 `a + ((b-a)*frac)>>6`；
   电角度 `(uint16_t)((linearized - electrical_zero) * pole_pairs)`；机械角用 `shadow_q15`。
5. 多圈：delta 回绕 ±32768；首拍按 `MotorAxisProfile_InitialEncoderOffsetQ15` 数值不变。
6. 速度：16 样本窗、`SPEED_LOOP_DIVIDER`、零死区 8、`vel_elec = vel_mech * pole_pairs`。
7. 离线/标定：`has_valid_sample && bad_frame_streak < 100`；标定位标志与持久化字段不变。
8. 无分配、无阻塞、ISR 安全；Q15/单位语义与调用顺序不变。

---

## 7. 验证计划

- 夹具再划分（保留 `test_encoder_sample_overlap.py` 名与清单不变）：
  1. 端口级（寄存器桩）：256 种 SR 组合、CS/HiZ 顺序、两类失败、CS 必释放；
  2. 通道级（传输 seam 桩）：请求/读帧取值、同帧与回退、失败→状态映射；
  3. 角度级（阶段 C）：方向/LUT/电零位/多圈回绕/速度窗/离线阈值向量；
  4. 检查级（阶段 D）：有效性/坏帧向量。
- 门禁：`format --check`、`lint`、`architecture`、`interfaces`、`project-layout`、`docs`。
- 构建：Keil 双目标 0 Error / 0 Warning（`KEIL_UV4`）。
- 回归：`tests/run.py --cc <zig>` 全量。

---

## 8. 协调与不做

- 本项即 [foc_run 归位与瘦身](2026-09-19-foc-run-placement.md) 的 **H3**（最高风险项）；
  [E 生命周期对接准备](2026-09-19-e-framework-interface-prep.md) 将其列为编码器特性回移前置。
- 阶段 A 不触碰并行会话在改的文件；阶段 C 触达 `interface_can.c`/`foc_run*.c`/
  `rtt_telemetry.c`/`foc_param.c` 前先确认并行改动收敛。
- 不做：不改 20 kHz 调度与 Q15/单位语义；不改参数持久化 ABI；不删 TLE 诊断字段；
  不实现新传感器（MT6701/MT6535）本体——本轮只保证"加 driver 即可扩展"的结构。

## 9. 待评审确认

- **Q1 传输契约粒度**：`init/read_begin/read_complete` 三函数（推荐）还是同步+异步两形态？
- **Q2 通道契约范围**：`encoder_sensor.h` 采用上面的"状态+归一化角度+诊断字"（推荐），
  还是希望 driver 直接返回 SI 角（rad）？
- **Q3 安全字/CRC 死路径**：本轮保持"声明不实现"（严格行为保持，推荐），还是本轮实现
  TLE CRC/安全字校验（属功能改动，需要数据手册向量与台架证据）？
- **Q4 无调用方 API**：✅ 已确认**保留**：`Encoder_SetElectricalZero`/`SetElectricalZeroQ15`
  是 mode 15 电零位标定的落点（该标定随 FOC-Calibration 删除，待重建），
  `GetCountInCPR_Ratio` 保留备用；阶段 C 迁移时签名与语义不变。
- **Q5 类型归属与命名**：阶段 C 先保名 `Encoder_TypeDef` 迁 motor（零签名改动，推荐），
  还是同时改名 `AngleFeedback_TypeDef`？
- **Q6 驱动选择**：方案 2 编译期分发（推荐，与 shunt profile 同例）还是方案 1 链接期？
- **Q7 检查层范围**：本轮只归位现有在线判定；冻结/跳变等新增检查另立计划（推荐）？
- **Q8 推进粒度**：A 完成并提交后评审一次再做 B/C/D（推荐），还是一次性完成？
- **Q9 传感器归一化位宽**：driver 统一输出 Q15（推荐）还是保留各型号原始位宽 + 缩放参数？
- **Q10 未来型号资料**：MT6701/MT6535 的位宽/CRC/时序在你手里有数据手册吗？
  （决定 driver 骨架与向量，本轮可先不动。）

## 10. 验证证据

阶段 A（2026-09-19）：

- 新增 `platform/api/encoder_spi.h` + `ports/motor/encoder_spi_stm32g4.c`；`encoder.c` 只经传输
  契约访问总线（寄存器自旋、CS、MOSI 方向、超时常数全部下沉端口，表达式逐位保持）；
  `encoder.h` 删除 `brd_enc_spi`/`BRD_ENC_SPI`/CS 宏并补齐 17 条中文接口契约（接口债清零）。
- 夹具 `test_encoder_sample_overlap.py` 重做为两个子用例：**传输端口**（寄存器桩，2048 例：
  总线就绪判定、CS/MOSI 顺序、两类传输失败、资源释放）与**协议编排**（传输 seam 桩，
  1024 例：请求帧/读帧取值、异步与回退路径、角度解码、超时状态）。
- 原生套件 17/17 PASS；`format` / `lint` / `architecture` / `interfaces` / `project-layout`
  0 new；Keil 双目标 0 Error / 0 Warning（主工程 Code=82208）。
- 顺带修复：`test_outer_loop_runtime.py` 读 `encoder.h` 未指定编码（新增中文注释后 GBK 解码
  失败）；两个 Keil 工程的端口组登记（`power_stage` 重复、`motor_sensing` 丢失）——修复后
  布局检查 0 错误、双目标可链接。
- 待办：阶段 B（驱动通道 + TLE driver）、C（角度层归位）、D（检查层）；Q3/Q7/Q9/Q10 仍待确认。
