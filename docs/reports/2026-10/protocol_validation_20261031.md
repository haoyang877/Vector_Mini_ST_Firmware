# CAN FD应用协议实测 2026-10-31

当前AT32F456 Commissioning镜像的62项检查通过，1000次连续查询无超时。
结论仅覆盖本报告列出的应答、拒绝及台架停止语义，不表示已移除的业务能力重新可用。
未修改固件、未复位或烧录；上轮无感标定结果保留在RAM。

## 基线与结果

AT32F456＋YQ-5708（8:1），节点1，USB-CAN FD适配器，仲裁1M、数据5M、BRS。
用户已确认电机固定、输出轴自由，电流3A、速度50rad/s上限；本轮仅零Id/零Iq启相，
进一步收紧至0.5A、5rad/s、0.1rad累计行程、4s时限、200ms私有租约。
镜像SHA256为`378a6de158968c147c60b89d14605b768db721e4e89e15d436c9bdf82a7223ba`。

| 检查 | 实测结果 | 判断 |
| --- | --- | --- |
| 身份、能力、设备与电机状态 | 节点/身份正确，六类只读查询字段及无效哨兵符合契约 | 通过 |
| 1000次查询，10Hz | 1000/1000；P99 3.403ms，最大5.462ms，平均2.213ms | 通过：P99≤20ms、超时100ms |
| 28项报文/采集边界 | CRC、填充、版本、地址、权限、长度、旧编号、忙状态、重复与冲突序号均按约定处理 | 通过 |
| 公共命令错误及不支持能力 | 长度/字段错误正确拒绝；已移除参数和标定功能明确返回UNSUPPORTED | 通过拒绝语义 |
| 116和124 | 116返回UNSUPPORTED，124的请求关联、独立帧序号、CRC和45字节字段正确 | 通过当前契约 |
| 运行中普通STOP | 首答`01000100`，即ACCEPTED/detail=1；主机3.509ms内读到关相 | 通过 |
| KEEPALIVE和停止续租 | 跨多个租期运行574.7ms；随后仅查询，距最后一次续租状态读数202.5ms时已关相，reason=4 | 通过 |
| 过期RUN重复与冲突请求 | 原请求返回缓存的ACCEPTED、运行编号不变；冲突载荷返回4，不重新启相 | 通过 |
| 适配器关闭重开 | 关闭通道、不发STOP；重开读到租约停止，后续五次查询均未自动恢复 | 通过 |
| 最终状态 | 数字关相、故障0、动力限值无效；1024项RAM LUT及电零位14162不变 | 通过 |

延迟为包含USB、主机调度、日志和查询的实测往返时间；关相时刻为主机观察上界，
并非示波器测得的PWM关闭时间。适配器重开时约395ms才重新获得状态，不能据此推算精确停机时刻。
公共STOP结束后私有台架状态记为外部状态中断reason=7，电机故障码仍为0；这不是租约超时。

## 首轮脚本误判与重测

首轮脚本错误地要求STOP首答立即为OK；收到合法ACCEPTED后中止后续启相，随后安全关相。
原始`exercise.json`中的FAIL保留，不改写成通过。只修正主机判据为“OK或ACCEPTED，并复核实际关相”，
单独重测10项停止/续租/重连场景全部通过。固件未改，连续1000次查询未重复使用新的结果覆盖。
汇总的62项来自28项边界、24项只读/公共接口和10项最终生命周期验证；首轮误判未计为通过。

## 明确限制

- 普通APP通用通信看门狗能力为0；本次200ms失联保护来自Commissioning私有台架租约。
- 116目标执行、120/121参数读写和122保存目前不支持；测试通过表示拒绝符合契约。
- 108/124的十二个测量有效位均为0，测量值是无效哨兵，不能当实时电流、速度或温度使用。
- 101 enable=1低侧制动候选、161标定查询和162取消目前不支持；117仅测错误请求拒绝，未测带载启动。
- 没有物理拔线、线路干扰、CAN总线错误注入、Loader升级或复位测试。

当前语义来源为[核心协议v1.8](../../protocols/README.md#implementation)、
[电机状态v1.7](../../protocols/README.md#feedback)及
[统一台架协议v1](../../protocols/README.md#implementation)。

## 原始证据

- [逐项可展开报告及延迟图](../../../outputs/protocol-validation-20261031/report.html)
- [最终汇总](../../../outputs/protocol-validation-20261031/summary.json)
- [28项边界原始结果](../../../outputs/protocol-validation-20261031/boundaries/boundaries.json)
- [首轮结果（保留主机误判）](../../../outputs/protocol-validation-20261031/exercise.json)
- [停止/续租/重连重测](../../../outputs/protocol-validation-20261031/lifecycle/exercise.json)
- [逐帧原始报文](../../../outputs/protocol-validation-20261031/exercise_frames.jsonl)
- [生命周期原始报文](../../../outputs/protocol-validation-20261031/lifecycle/exercise_frames.jsonl)
- [全部证据SHA256](../../../outputs/protocol-validation-20261031/evidence_sha256.json)

本轮仅增加计划、报告及忽略目录下批次脚本，没有新增生产模块。
最终quick检查通过，包含格式、lint、工程布局和离线测试，见
[检查记录](../../../outputs/runs/protocol-validation-20261031/summary.json)。
