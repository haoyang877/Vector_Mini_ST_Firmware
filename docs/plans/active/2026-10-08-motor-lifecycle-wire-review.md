# 电机生命周期线路接入评审记录

日期：2026-10-08。状态：**项目编号已确认，APP 协议接线已实现；仍不宣告新功率能力**。

## 目标与基线

将 H 独立交付的四模式启动/切换、明确租约与整计划标定接到 CAN FD，同时保持旧 `0x0065/0x006B/0x0074/0x0075`、旧单项 `0x00A0～0x00A2`、45 B 反馈和参数 schema12 不变。源树 `codex/motor-lifecycle-flow` 的 base 为 `7f44bd083902081b00d1761648c158c34350d098`，业务源提交 `7bf4a99258fbaae5adbf77e2a1c3be7376332ec6`，审阅时 HEAD 为 `8bbc90fbecfc8eeb2c58012c836b2a2e5c41c3ff`。本轮不修改 GUI、主状态机或 Keil 工程，不运行硬件。

具体线路见[项目线路 v1.4](../../protocols/yg_protocol_motor_lifecycle_wire_v1_4_proposal.md)：仅新增 `0x0076 RUN_SESSION` 和 `0x00A6 CALIBRATION_PLAN` 两个 CMD ID，均已登记和注册到 APP 路由；**这不是公司外部消息表已同步的声明**。旧请求保持原字节和含义。整计划 START 已补足 20 B，其中最后 8 B 为启动 epoch；STATUS/CANCEL 仍为 12 B。

## 接线前必须解决的三项事实

1. `firmware/app/composition/yg_protocol_status_adapter.c` 的 `refresh_boot_source()` 当前明确把启动 epoch 标为不可用。新运行请求如果没有可信启动身份，跨复位的旧帧无法辨别。不能把 `boot_id=0`、MCU UID 或主站 nonce 假装为 epoch。建议 `RUN_SESSION` 要求真实非零 epoch；在状态源实现前保持新能力位为 0、请求返回不支持。
2. `firmware/app/motor_state_machine/motor_state_machine.c` 的 `request_identified_run()` 在实际 `RequestEnable/RequestModeSwitch` 可能拒绝前就设置 `remote_source_valid`。这与“被拒绝的请求不改变所有者状态”目标不一致；应由主状态机负责人修正并添加拒绝后状态不变测试。H 协议适配器不直接改主状态机。
3. `RequestIdentifiedRun()` 的重试身份只检查来源、seq 和模式；同一身份但不同 `timeout_ms` 仍可能刷新租约。H 适配器必须在调用前按完整请求内容识别重复和冲突，并为 RENEW 保存独立的最近序号/结果；旧 SPEED117 的重试缓存不能复用。整计划的 `RequestCalibration()` 没有来源和 seq 参数，H 也必须维护**独立**的父计划重试记录。

## 已确定的边界

- `0x0076` 和 `0x00A6` 按项目内编号使用；外部公司表随后同步，不修改旧命令含义。
- 新 RUN_SESSION 与 CALIBRATION_PLAN START 都核对 `YgProtocolLink_BindDeviceStatusSource` 提供的有效非零 epoch。当前独立 APP 的该源无效，新 START 回复“不支持”，能力位仍为 0。旧 117 和旧单项 160～162 没有 epoch 字段，仍是调试兼容路径，不能据此声称有跨复位保护。
- 查询、取消和 STOP 不依赖 epoch；STOP 始终进入原停机路径。主站 STOP、已接受的失能和成功清故障会撤销新命令缓存并推进主站水位；其他节点 STOP 也停机，但不写主站水位。主状态机拒绝后来源绑定修复由负责该模块的工作树完成。

## 实施与验收顺序

1. 编号和字段冻结后，按候选 v1.4 增加编解码、两条路由及显式线值→业务值映射；拒绝未知模式、未实现的电感/磁链、Nm、motor_type 自动配置。
2. 绑定真实 `RequestIdentifiedRun/RefreshControlLease/RequestCalibration/ReadCalibration/CancelCalibration`，只复制值，不暴露 HAL 或可写电机对象。补原始身份去重与取消/STOP 顺序检查。
3. 加版本化黄金向量和少量针对性测试；跑 `verify --profile pr`。能力默认保持 0，只有组合镜像、关相、超时、重复与故障验收通过后再启用对应 bit。不能用旧输出证明新镜像。

本工作树已修改线路 ABI 和路由，因此需主站使用 v1.4 字段及项目内编号；不宣称硬件已验证。完整请求/回复向量已保存为[JSON](../../protocols/yg_protocol_motor_lifecycle_wire_v1_4_vectors.json)。原生链路夹具覆盖无效 epoch、重复/冲突、续租去重、STOP/失能/清故障后旧帧、非主站 STOP 和计划查询/取消。Keil release 构建在本机缺少 UV4，LoaderApp 的 Flash 影响尚未量测。
