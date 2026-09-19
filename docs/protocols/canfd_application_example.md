# CAN FD 电机位置控制应用示例

本文是公司协议 v1.2.3 的离线 C99 示例。它演示一条主控到 3 号电机的位置命令、
一条电机到主控的状态反馈，以及同一套打包、封装、解包和拒收流程。示例不访问 CAN
适配器，不使能电机，不代表现有固件已经接入这些函数。

源代码：[company_canfd_position.c](../../host_app/examples/company_canfd_position.c)

离线测试：[test_company_canfd_example.py](../../tests/unit/test_company_canfd_example.py)

## 1. 应用层对象

主控发送位置目标：

```text
source=0x02，destination=0x03
type=105 SET_POSITION
frame_sequence=2
command_sequence=2
lease_tag=0x1234
position=1000，单位 0.001 rad，即 1.000 rad
```

电机反馈快照：

```text
source=0x03，destination=0x02
type=124 MOTION_FEEDBACK
frame_sequence=10
position=950，单位 0.001 rad
speed=-1200，单位 0.001 rad/s
Iq=300，单位 mA
```

这些 C 结构体是内存中的业务对象，**不是线上布局**。所有多字节字段由显式小端函数
写入 `uint8_t data[]`，不使用 `memcpy(struct)` 或 `#pragma pack`。

## 2. 打包和封装

```text
MotorMessage
  → 校验 type、方向和地址
  → 编码小端业务 Payload
  → 写入公司 16 B 帧头
  → 计算 header CRC8
  → 计算公司 CRC16
  → 选择 CAN FD 物理数据区长度并补 00
  → 组合 29 位 CAN ID、IDE、FDF、BRS、DLC
  → 交给 comm_hw/CAN FD 驱动发送
```

位置命令的 CAN ID：

```text
Priority=2, PF=0xEF, DA=0x03, SA=0x02
CAN ID = 0x08EF0302
```

完整 32 字节 CAN FD 数据区：

```text
5A A5 01 00 02 03 69 00 02 00 08 00 00 00 00 D1
69 02 34 12 E8 03 00 00 D2 E1 00 00 00 00 00 00
```

字段分段：

```text
5A A5                         magic
01                            version
00                            flags
02 03                         src=02, dst=03
69 00                         type=105
02 00                         frame_sequence=2
08 00 00 00                   payload_len=8
00                            reserved
D1                            header CRC8
69 02 34 12 E8 03 00 00      SET_POSITION payload
D2 E1                         company CRC16 = 0xE1D2
00 00 00 00 00 00             FD padding
```

CAN FD 控制器的物理 CRC17/CRC21 不放入 `data[]`，由控制器自动生成。公司 CRC16
覆盖公司头和业务 Payload；两层校验不能互相替代。

反馈帧：

```text
CAN ID = 0x0CEF0203
5A A5 01 00 03 02 7C 00 0A 00 08 00 00 00 00 A9
B6 03 00 00 50 FB 2C 01 34 7B 00 00 00 00 00 00
```

其中 Payload 为：

```text
position = B6 03 00 00 = 950
speed    = 50 FB       = -1200
Iq       = 2C 01       = 300
```

## 3. 接收和解包

接收顺序必须是：

```text
CAN 控制器确认 IDE/FDF/BRS/DLC 和硬件 CRC
  → 检查 29 位 ID、PF、源地址、目的地址和优先级
  → 检查公司 Magic、version、flags、保留位
  → 根据固定布局检查 len，不能先信任未验证长度
  → 验证 header CRC8
  → 验证公司 CRC16
  → 检查 CRC 后填充全为 00
  → 检查 type 与业务 Payload
  → 检查 CAN ID 地址与公司头 src/dst 一致
  → 转换为 MotorMessage
  → 交给 Command Router
```

`unpack_message()` 在所有校验通过前不写出目标对象。真实设备接收后还必须由业务层
检查控制权、lease、模式、使能状态、目标范围、序号新旧和运动看门狗；解包成功不等于
电机已经执行。

## 4. 离线运行

```powershell
python -m unittest tests.unit.test_company_canfd_example
```

或运行项目 PR 验证：

```powershell
uv run python tools/run.py verify --profile pr
```

示例会输出：

```text
CONTROL ID=08EF0302 IDE=1 FDF=1 BRS=1 DLC=13 LEN=32 DATA=...
DECODE_CONTROL position_mrad=1000 lease=1234 command_seq=2
FEEDBACK ID=0CEF0203 IDE=1 FDF=1 BRS=1 DLC=13 LEN=32 DATA=...
DECODE_FEEDBACK position_mrad=950 speed_mrad_s=-1200 iq_ma=300
PASS offline codec checks; no hardware accessed
```

测试还会使用独立 Python 参考编码器逐字节核对两条黄金帧，并注入数据位、长度、地址、
BRS、Payload 类型和负数边界错误。构建日志和运行日志写入 `outputs/`，不作为协议版本
或硬件验收证据。

本示例使用当前基础协议的 200 Hz/单轴消息格式；它没有把五轴双向 1 kHz 同步建议
偷偷加入正式命令表。多轴 1 kHz 需要另行登记同步消息、反馈周期和总线调度能力。
