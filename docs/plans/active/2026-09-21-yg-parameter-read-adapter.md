# 参数只读适配器（yg_protocol P 任务）v0.1

日期：2026-09-21。状态：离线实现与 native 测试完成；未接入运行入口，未做硬件验证。

上游依据：[yg_protocol 核心接口契约 v0.1](../../protocols/yg_protocol_core_interfaces_v0_1.md)、
[模块开发交接单](2026-09-21-yg-protocol-module-handoff.md)（P 任务）、
[电机通信协议评审稿 §8.4](../../protocols/motor_protocol_v1.md)。
旧 CAN 参数仅作为字段来源参考，不复用其编号或编码。

## 1. 目标与边界

为 `yg_protocol_parameter_service_t.handler` 提供第一批只读处理器：把协议评审稿最小注册表中
**能明确映射到现有参数结构**的少量参数暴露为活动只读副本，业务服务与通信头解耦。

允许新增文件（本计划范围）：

- `firmware/services/parameters/parameter_read_service.h/.c`
- `firmware/communication/protocol/yg_protocol_parameter_adapter.h/.c`
- `tests/unit/yg_protocol_parameter_adapter_test.c`
- `tests/unit/native/test_yg_protocol_parameter_adapter.py`

不改任何既有参数结构、schema、ID、Flash 布局、公共 `yg_protocol_*` 头或 `tests/run.py`。
`2026-09-20-parameter-descriptor-model.md` 仅是设计稿，本轮不实施描述符表或全量参数重构。

## 2. 分层与所有权

```text
yg_protocol_parameter_request_t
        │  yg_protocol_parameter_adapter_handle（通信适配器）
        ▼
ParameterRead_Get（业务纯函数，不依赖 yg_protocol/CAN/HAL）
        │
        ▼
调用方提供的 ParameterReadSource（活动参数只读副本 + 所有者提供的 config_revision）
```

- 业务服务不读取任何可变全局量（`MotorControl` 等）、不访问 Flash、不分配、不阻塞。
- 只读副本由组合层在参数上传/快照完成后显式填充；`snapshot_ready=false` 时任何读取返回
  `NOT_READY`（处理器映射为 `BUSY`），不用旧值或零值冒充有效读数。
- `config_revision` 必须由参数所有者提供，禁止用 `PARAM_SCHEMA_VERSION` 冒充活动配置版本。

## 3. 实现的参数与单位

参数 ID 取自评审稿 §8.4 最小注册表；只登记能明确映射的项：

| ID | 名称 | value_type | 线上单位 | 来源字段 | 缩放 |
| --- | --- | --- | --- | --- | --- |
| 0 | 活动参数 revision | 2 (u32) | 版本计数 | `ParameterReadSource.config_revision` | 无 |
| 1 | 最大速度 | 1 (i32) | 0.001 rad/s | `values.speed_limit` (rad/s) | ×1000 |
| 2 | 最大 Iq | 1 (i32) | mA | `values.current_limit` (A) | ×1000 |

转换使用毫米/毫安整数，四舍五入到最近毫单位；非有限、非正或缩放后超过 `2e9` 一律返回
`INVALID_VALUE`，不饱和、不静默截断。旧 CAN 的速度族用百分之一（centi），本处理器按评审稿
使用千分之一（milli），两者不得混用。

## 4. 尚未实现（明确缺口）

- 评审稿参数 3（最大加速度）在 `InterfaceParam_TypeDef` 中有 `speedAcc`/`posAcc` 两个候选，
  语义不唯一，故不实现。
- 参数 4/5（位置下限/上限）、6（使能位置窗口）、7（目标超时策略）、8（受控停止最长时间）
  在现有 `InterfaceParam_TypeDef` 中没有对应字段，不实现。
- 评审稿 §8.7/§8.9 的扩展参数 ID（20～25）依赖阻抗/原点配置对象，不在现有参数结构内。
- 内部 `yg_protocol_service_reply_t` 没有 `value_type` 字段；READ 的 `value_type` 由参数 ID
  唯一决定，需由上层按 ID 或后续接口扩展推导，本轮不扩张公共头。
- WRITE/SAVE/RESTORE_DEFAULTS 一律 `UNSUPPORTED`，不构造暂存事务或假受理。

## 5. 验收

- 业务服务：READ 正确单位、未知 ID、未就绪、负/NaN/Inf/越界、输入与失败输出不残留。
- 处理器：READ 成功值/revision 映射；未知 ID、写操作、未就绪、非法值映射；操作后只读副本
  逐字节不变；handler 为 NULL 返回 `UNSUPPORTED`。
- 命令：`tests/unit/native/test_yg_protocol_parameter_adapter.py --cc <zig> --out outputs/...`。
- 样式：`uv run python tools/run.py format --check` 与 `lint`。

## 6. 回滚

仅新增文件；删除新增文件与本节索引条目即可回滚。未改任何运行入口、ABI、Flash 或工程输入。
