# yg_protocol CAN FD 硬件连通性冒烟测试 v0.1

## 目的

`tools/bench/yg_protocol_link_test.py` 只验证一块板卡的 CAN FD 物理链路、扩展 ID、FD+BRS、
协议头 CRC、整帧 CRC 和只读 GET_CAPS 响应。它不发送电机控制、参数写入、Flash 擦写或升级命令。

固件端点在 `FDCAN1` 的 RX 中断接收帧，在 2 kHz `CAN_Service` 中推进路由，并返回 GET_CAPS。
固定位时序为仲裁段 1 Mbit/s、数据段 5 Mbit/s。请求使用 source=主机 Node ID、destination=板卡
Node ID、priority=2；响应使用反向地址、priority=3。

## 台架前置条件

- 板卡已刷入包含 `yg_protocol_link` 的固件，并提供该固件 SHA-256。
- CAN 收发器、终端电阻和适配器均已确认支持 1 Mbit/s / 5 Mbit/s CAN FD BRS。
- 仅给板卡逻辑电源，电机功率级关闭或物理断开；测试过程中禁止转动电机。
- 适配器 SDK 根目录包含 `.local/canfd-sdk/ControlCANFD.dll`，只使用 channel 0。
- Node ID、板卡标签和现场场景已记录，避免与其他主站并发发送。

## 命令

```powershell
.\.venv\Scripts\python.exe tools/bench/yg_protocol_link_test.py `
  --adapter-root D:\Work\Code\motor_ctrl_app `
  --node-id 7 `
  --source-id 1 `
  --board vector-mini-st-01 `
  --firmware-sha256 <64 位 SHA-256> `
  --no-motor-power `
  --out outputs\bench\yg-link-<时间戳>
```

成功时 `run.json` 的 `success` 为 `true`，其中包含适配器 DLL SHA-256、请求和响应原始字节、
响应能力字段及固定波特率。响应超时、ID/地址不符、CRC 错误或 payload 长度错误均失败。

## 请求/响应黄金形态

以板卡 Node ID=7、主机 source=1、sequence=1 为例：

```text
请求 ID：08EF0701
请求物理长度：32
请求应用数据：5A A5 01 20 01 07 02 00 01 00 0C 00 00 00 00 <CRC8>
              44 33 22 11 01 00 00 00 00 00 00 00 <CRC16> 00 00
```

请求 payload 是 `session_id=0x11223344`、`request_id=1`、`page=0`、保留字段为 0，全部小端。
响应为 `message_type=2`、`flags` 含 RESPONSE、source=7、destination=1、sequence=1，能力
payload 长度 16 字节。具体 CRC 和响应原始帧以 `run.json` 为准。

该冒烟结果只证明只读通信链路和端点闭环；它不证明控制命令、反馈周期、5 轴同步或 1 kHz
时序已经验收。
