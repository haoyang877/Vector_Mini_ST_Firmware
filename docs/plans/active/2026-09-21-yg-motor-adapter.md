# yg_protocol 电机 STOP/DISABLE 适配器 v0.1

日期：2026-09-21。状态：**离线适配器第一步已实现并通过独立原生测试；未接入 CAN 运行入口，
未做真实所有者接线**。基线：`8982922acaaaab9ebec0ff3a4b8020ebd4e47422`。

## 1. 目标与边界

为 yg_protocol 电机服务的通信适配器提供可注入的 STOP/DISABLE 业务边界：

- 通信适配器实现 `yg_protocol_motor_adapter_t`，把线格式请求映射到业务窄接口与内部结果。
- 业务侧只发出停机请求并跟踪所有者确认；不直接写 `MotorControl`，不新增第二套状态机。
- 本批不接入 CAN 入口，不使能，不改造 Flash/参数/工程输入，不接触硬件。

## 2. 现有唯一控制所有者分析

- `firmware/app/foc_run_state.c` 是当前唯一运行状态所有者：静态持有 `AppLifecycle lifecycle`，
  是 `AppLifecycle_Step` 的唯一调用者，并在 2 kHz 慢拍执行动作与功率级记录。
- `firmware/services/lifecycle/app_lifecycle.c` 是平台无关纯核心：只产出动作位与拒绝原因，
  本身不访问硬件，也不保证门极已关断（`APP_ACTION_DISABLE_POWER` 只是请求）。
- `firmware/communication/can/can_binding_commands.c` 是旧线格式直写路径（直接改
  `MotorControl.ModeNow/iqRef/...`），不是目标架构的控制所有者，本批不改动。

结论：通信侧不得自行驱动 `AppLifecycle_Step`。新增的
`motor_stop_service` 只向所有者暴露“请求停机 + 查询是否已关断”两个窄端口；真实所有者
（`FocRunState`）当前没有这个窄请求入口，故本批以显式注入回调作为 test seam，真实接线
**未完成**，由集成人员在其 2 kHz 拍内实现该端口。

## 3. 文件与接口

| 文件 | 作用 |
| --- | --- |
| `firmware/services/lifecycle/motor_stop_service.{h,c}` | 平台无关 STOP/DISABLE 业务接口：幂等请求、异步 token、确认通道 |
| `firmware/communication/protocol/yg_protocol_motor_adapter.{h,c}` | `yg_protocol_motor_service_t.handler` 适配器与结果映射 |
| `tests/unit/yg_protocol_motor_adapter_test.c` | 离线原生夹具（生命周期替身 + 脚本化所有者） |
| `tests/unit/native/test_yg_protocol_motor_adapter.py` | 独立测试入口（`--cc --out`） |

### 3.1 所有者窄端口

`motor_stop_owner_port_t` 只有两个同步、非阻塞回调：

- `request_stop(owner_context)`：受理或拒绝停机；返回 `ACCEPTED/DENIED`。
- `is_power_disabled(owner_context)`：**唯一停机确认证据**；只有明确为真才算关断。

两个回调缺一即拒绝 `MotorStopService_Init`，`RequestStop` 只能返回 `UNAVAILABLE`，
杜绝“没有确认接口却报告 OK”。

### 3.2 幂等与异步

- `MotorStopService_RequestStop` 在 `IDLE` 时调用一次 `request_stop` 并分配非零 token；
  之后处于 `PENDING`，重复请求只轮询确认，不重复下达。
- `PENDING` 且确认成立才升级 `CONFIRMED` 并返回 `OK`；否则保持 `ACCEPTED`。
- `CONFIRMED` 后重复 STOP/DISABLE 只有在所有者仍确认关断时返回 `OK`；若输出后来重新使能，服务回到 `IDLE` 并重新请求停机。
- 所有者拒绝返回 `DENIED`，服务保持 `IDLE`，修正后可重新请求。

### 3.3 处理器映射

| 业务结果 | 内部状态 | 说明 |
| --- | --- | --- |
| `OK` | `YG_PROTOCOL_SERVICE_OK` | 已确认功率输出禁止 |
| `ACCEPTED` | `YG_PROTOCOL_SERVICE_ACCEPTED` | 异步等待确认，携带 token |
| `DENIED` | `YG_PROTOCOL_SERVICE_DENIED` | 所有者拒绝 |
| `UNAVAILABLE` | `YG_PROTOCOL_SERVICE_UNSUPPORTED` | 未接线或缺少确认通道 |
| `FAILED` | `YG_PROTOCOL_SERVICE_FAILED` | API 误用/内部状态非法 |

`reply` 在任何早退路径都被稳定初始化；`ENABLE/SET_MODE/SET_TARGET` 一律
`UNSUPPORTED`。请求只借用，不保留指针；`detail` 携带业务结果供诊断，不是线上字段。

## 4. 数据所有权与并发

- `motor_stop_service_t`、`yg_protocol_motor_adapter_t` 由组合层持有；端口与上下文只借用。
- 请求只在调用期只读引用，处理器不存储、不复制、不排队。
- 服务为单上下文串行设计；所有者端口回调在调用方上下文同步执行。通信与监督**不得**
  各自对同一 `AppLifecycle` 实例并发调用 `Step`；唯一所有者仍在 2 kHz 拍内串行驱动。

## 5. 离线验证

命令：

```text
.venv/Scripts/python.exe tests/unit/native/test_yg_protocol_motor_adapter.py \
  --cc <zig> --out outputs/tests/yg_protocol_motor_adapter
```

覆盖：早退与稳定初始化、API 误用、缺确认通道、生命周期所有者确认停机与幂等重复、
生命周期 FATAL 拒绝、异步 `ACCEPTED`+token→确认 `OK`、拒绝后重试、无法确认时永不 `OK`。

## 6. 尚未实现 / 缺口

1. **真实接线未完成**：`FocRunState` 尚无“窄停机请求 + 关断确认”入口；需在其 2 kHz 拍内
   实现 `motor_stop_owner_port_t`，并保证只由它调用 `AppLifecycle_Step`。
2. **ENABLE/MODE/TARGET 契约缺失**：需要来源授权（不能从 CAN ID 推断）、控制会话关联、
   `lease_ms`/`execute_at_us` 语义、模式许可与限位、目标单位与快照原子性。
3. **token 查询契约缺失**：当前用重复 STOP/DISABLE 轮询确认；若采用一次性 `ACCEPTED` 回执，
   需要独立的查询操作与超时语义，本批未定义。
4. **watchdog/lease**：未做推测实现；心跳租约、命令超时与断链自动停机属后续独立设计。
5. **接线与工程登记**：`tests/run.py`、native 构建清单、Keil 工程与 `-I
   firmware/services/lifecycle` 由集成人员统一登记。
6. **无硬件验收**：本批不刷写、不转动、不接入 CAN 入口，不声明实机行为。

## 7. 兼容与回滚

- 不修改任何既有头、生命周期核心、FOC、硬件或测试总入口；新增文件不影响旧运行路径。
- 回滚只需删除本批新增文件；无持久化、协议或 ABI 痕迹。

## 8. 依据

- [核心接口契约 v0.1](../../protocols/yg_protocol_core_interfaces_v0_1.md)
- [模块开发交接单 v0.1](2026-09-21-yg-protocol-module-handoff.md)
