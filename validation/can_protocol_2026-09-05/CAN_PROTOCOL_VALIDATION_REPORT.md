# CAN 通信协议实机验证报告

## 1. 结论

CAN Protocol V1 的全部 95 个已定义命令 ID 均已覆盖：42 个 `SET`、52 个 `GET` 和 1 个 `APPLY`。完整自动化测试产生 106 条判定记录，结果为 **106 PASS / 0 FAIL**。

最终设备状态：节点 ID 0、经典 CAN 1 Mbit/s、模式 0、故障码 0、编码器在线、摩擦模型有效。最终附加 100 次连续读取为 100/100 成功，CANalyst-II 接收和发送错误计数均为 0。

## 2. 测试对象与环境

- 分支：`codex/production-firmware-refactor`
- 基础提交：`d8a93d69f8741abfe82a9e0b811362fad2d93cba`
- 测试镜像包含当前工作区尚未提交的 CAN 修订。
- 编译器：Keil ARMCC 5.06 update 7 build 960
- 构建结果：0 Error、0 Warning
- 下载器：J-Link，序列号 602722271
- CAN 适配器：CANalyst-II / ControlCAN，通道 0
- 总线：经典 CAN，标准 11-bit ID，1 Mbit/s，4 字节大端序 float32
- 实测母线电压遥测：27.81346 V
- HEX SHA-256：`F2FB5B7E6E1D614BCDC603B9CF8B2B03E277EEE08E31443ED7C13122DDC6A2F5`

## 3. 覆盖与结果

| 测试组 | 记录数 | 结果 | 内容 |
|---|---:|---:|---|
| GET | 51 | 51 PASS | 所有已实现 GET 命令返回有限 float32 |
| SET/GET | 23 | 23 PASS | 参数修改、读回比对、恢复原值 |
| CONTROL | 4 | 4 PASS | 模式 0、0 A 电流、0 r/s 转速、当前位置保持 |
| CONFIG | 4 | 4 PASS | 节点 0→1→0、波特率 1000→500→1000 kbit/s |
| IGNORED | 13 | 13 PASS | 只读遥测对应 SET 和占位 SET 按设计无响应、无写入 |
| NOT_IMPLEMENTED | 1 | 1 PASS | `GET_COGGING` 按当前实现无响应 |
| GUARDED | 1 | 1 PASS | `APPLY_FRICTION_MODEL=0` 不应用候选模型且无响应 |
| NEGATIVE | 6 | 6 PASS | 错节点、错误 DLC、远程帧、扩展帧、未知 ID、NaN 均被拒绝 |
| SAFETY | 1 | 1 PASS | 心跳超时使模式 1 自动退回模式 0 |
| STRESS | 1 | 1 PASS | `GET_MODE` 200/200，平均 0.98 ms，最大 1.71 ms |
| BUS | 1 | 1 PASS | 适配器状态可读，RX error=0、TX error=0 |
| **合计** | **106** | **106 PASS** | **0 FAIL** |

## 4. 关键功能结果

### 控制命令

- `SET_CURRENT=0`：进入模式 1，设定值读回 0 A，随后禁用。
- `SET_SPEED=0`：进入模式 2，设定值读回 0 r/s，随后禁用。
- `SET_POSITION=当前位置`：进入模式 3，位置设定值一致，随后禁用。
- `SET_MODE=0`：模式读回 0。
- 复位后首次电流控制会先执行模式 11 电流零偏校准；测试等待该流程完成后重新下发命令，模式 1 正常进入。

### 配置命令

- 节点 ID 从 0 切换到 1 后，新节点立即响应，旧节点过滤器不再响应；随后恢复节点 0。
- 波特率从 1 Mbit/s 实际切换到 500 kbit/s并通信成功，随后实际切回 1 Mbit/s。
- 心跳配置写入 600 ms 可读回并恢复为 500 ms。

### 心跳保护

设备确认进入模式 1 后停止 CAN 通信 650 ms，超过配置的 500 ms 超时，设备自动退回模式 0。恢复通信后的第一帧会在路由 GET 命令前清除 CAN 断连故障，因此随后观测为 `mode=0, error=0`，符合当前恢复逻辑。

### 参数保护和恢复

所有可写调参项均采用“读取原值 → 写测试值 → GET 读回 → 恢复原值”。极对数和编码器方向修改会按设计使运行时编码器标定失效，因此在控制模式测试前通过硬件复位重新加载已保存标定数据。测试没有执行参数保存动作，临时测试值未写入 Flash。

## 5. 协议占位与限制

- `GET_COGGING (0x27)` 当前没有响应实现；测试按“已知未实现”通过，不能解释为齿槽数据读取功能已完成。
- 13 个只读遥测 SET/占位 SET 按当前路由设计被忽略，没有 ACK。
- 当前摩擦候选模型 `candidate_valid=0`，因此没有发送 `APPLY_FRICTION_MODEL=1`，避免覆盖已经有效的活动模型；命令 ID 和保护分支已验证，但“有效候选模型应用成功”需要在完成一次摩擦辨识后单独验证。
- 本报告验证 CAN 帧、命令路由、参数读写及安全交互；`SET_MODE` 中模式 4～19 对应的各电机业务流程不在本轮重复执行，它们不是不同的 CAN 帧协议。
- 温度保护此前按实机调试要求跳过，`GET_TEMP` 的通信响应正常，但本报告不判定温度数值的物理准确性。

## 6. 产物

- `can_protocol_results.csv`：全部逐项结果，适合表格筛选。
- `can_protocol_results.json`：机器可读的完整结果和统计。
- `keil_build.log`：Keil 构建记录。
- `jlink_flash.log`：J-Link 下载和校验记录。
- `final_smoke_100.txt`：最终 100 次连续通信及总线错误计数。
- `../../tools/can_protocol_full_test.ps1`：可重复执行的完整协议测试脚本。

