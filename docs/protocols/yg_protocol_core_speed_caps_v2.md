# 核心 SPEED 能力声明 v2

2026-10-29增补：当前能力由[开环扩展v0.3](yg_protocol_simplification_design_v1.md#正式开环控制扩展-v03-2026-10-29)
追加mode9/10，supported_modes为0x0604，features仍为0x1D00。下文保留v2的历史
SPEED语义；主机速度准入不变，新增黄金向量与旧向量共同保留。

日期：2026-10-16。覆盖旧发现记录中“supported_modes=0”的当前实现状态；
保留历史提案的原结论作为历史，不将实现能力声明当作当前候选实机验收。

## 线路与已实现范围

GET_CAPS page0仍为16B、小端，字段偏移不变。沿用motor_protocol_v1中已分配的
features bit8=速度控制、supported_modes bit2=mode2。APP已有117零目标启动、116同模式
目标更新及110停机链路，本版仅公布这一条核心速度路径：features=0x00001D00，
supported_modes=0x0004；CAN FD/分片/合并反馈的bit10/11/12保持。
其他电流/位置/Vq候选入口没有因此获得能力声明，MIT与低侧制动仍不开放。

页0黄金payload：`00 1d 00 00 04 00 40 00 2e 00 00 00 00 00 00 00`。
结构化向量见[黄金向量](yg_protocol_core_speed_caps_v2_vectors.json)。

## 主机准入与兼容

当前会话身份与能力页均有效，且SPEED功能位、mode2位和必需CAN FD/分片/反馈位齐全，
才允许核心速度START/SET；仍须经过原有IDLE/RUNNING、反馈、台架及序号条件。
未知、缺失、清零或过期能力一律不猜支持，也不回退到旧线协议。
GET、STOP与合法配套升级不以SPEED支持为条件；STOP仍走已有真实结果确认。

旧APP报告0时，新主机保持可读、可停机、可升级，但速度按钮禁用；更新到对应APP后再用。
旧主机若忽略能力，不构成本版的主机兼容保证。字段、Type、长度、模式编号和参数ABI不变。
本文件不增加运行租约、启动身份、自动重连、功率动作或硬件验收结论。
