# 116目标与108/124实时反馈实测 2026-10-31

现有接口已经接通：117启动原控制模式，116直接更新原目标，108/124返回原测量对象的实时快照。
未增加生产模块、目标邮箱或控制算法，只新增一个`MotorLifecycle_SetTarget`入口，扩充已有`ReadStatus`。
实机验证了正负Iq目标、反馈、请求拒绝和停止；短时速度目标已提交，但尚未证明低速跟踪达标。

## 实机结果

AT32F456＋YQ-5708（8:1），节点1，CAN FD 1M/5M。沿用用户批准的3A、50rad/s上限，
本批次实际目标仅±0.05A和0.25rad/s；追加5rad累计行程、3s时限、200ms租约。
15项接口检查通过，结果不等同于速度控制性能验收。

| 场景 | 实测结果 |
| --- | --- |
| 108实时状态连续查询 | 1000/1000，P99 2.329ms，最大3.759ms |
| 117电流模式启动 | 返回ACCEPTED，随后实际RUNNING；由原台架监护，run_kind=7 |
| 116 +50mA | 124关联正确、目标50mA；108实测Iq均值50.7mA |
| 116 −50mA | 124关联正确、目标−50mA；108实测Iq均值−49.5mA |
| 同序号同内容 | 返回原提交结果，不重复提交 |
| 同序号冲突 / 旧序号 | 分别拒绝为4 / 5 |
| 20A越限 / 跨模式目标 | 分别拒绝为11 / 5，原目标不变 |
| 未启动、已停止后的目标 | 拒绝为5，不启动、不恢复旧目标 |
| 116速度目标0.25rad/s | 108持续返回目标250mrad/s；约0.3s样本实际均值0.85mrad/s、范围−1～1mrad/s |
| 普通接口运行失去续租 | reason=4并实际关相，使用原200ms私有租约 |
| 最终 | 关相、故障0、动力限值无效；1024项LUT及电零位14162与下载前一致 |

速度样本表明接口提交和测量有效，不证明目标速度已经实现。当前未调整速度PI、摩擦补偿或观测器参数，
也未将此短记录用于20秒速度稳定性验收。MCU温度在本板本次采集中无有效来源，正确保持无效；
母线电流、电压、编码器位置和速度均按有效位报告。Vd/Vq是FOC估算电压，电机温度始终无效。

追加1rad/s短时指令，目标保持1000mrad/s，实测速度均值4.36mrad/s、范围−1～32mrad/s，
电机轴位置跨度5mrad，实测Iq最大131mA，故障0。仍未建立目标速度跟踪，原因未定位，不能宣称速度环已跑通。
此记录单独保存在[追加速度结果](../../../outputs/motor-live-protocol-20261031/speed_probe/result.json)，
没有覆盖0.25rad/s或之前的失败/限制。

## 实现与兼容

- [运行所有者](../../../firmware/app/runtime/motor_lifecycle.c)校验同模式、停止/故障、现有包络后直接提交；
  状态读取与目标提交都用短临界区，缩放和CAN发送在临界区之外。
- [APP协议适配](../../../firmware/app/composition/app_protocol.c)负责单位、结果和最后一次成功请求的重试处理；
  16位序号回绕和STOP后旧请求均有离线回归。
- [协议编码](../../../firmware/communication/yg_protocol.c)继续共用原45字节布局。
  124由116触发，不另设周期广播器，不把反馈当成机械到位确认。
- [当时的台架监护（现已退役）](../../guides/unified_motor_bench.md)允许受限的原117启动，
  116目标不被私有RUN参考覆盖；普通APP没有新增私有租约或通用失联看门狗。

语义、单位、有效位和兼容说明见[协议v1.9](../../protocols/README.md#control)，
对应[黄金向量](../../../tests/fixtures/protocol/yg_protocol_motor_live_v1_9_vectors.json)由真实编码与适配测试核对。
位置和速度继续使用电机轴坐标，没有隐式换算8:1。

## 构建、回读与数据保留

新Commissioning镜像SHA256为`4cdc9ac03294aff63d94ab3e82d9e85fc28917ad6e1b6046f4c960a2c7a9a33a`。
AT32普通版、Commissioning、STM32普通版均零错误零警告；原工作树PR检查通过，
包含140项单元、16项集成及41组原生检查，参见[回归结果](../../../outputs/runs/motor-live-protocol-final/summary.json)。
新增用例覆盖六种目标模式、范围、故障/停止、序号回绕、缺测、缩放越界及台架对117的监护。
独立冻结源码的[release检查](../../../outputs/motor-live-protocol-20261031/release_source/outputs/runs/motor-live-protocol-frozen-verified/summary.json)
通过，含真实历史FOC参考回归与Keil重建；冻结验证仓库只在outputs内，未提交用户工作树。
最终重建的Commissioning HEX与已下载文件逐字节一致，格式、lint和文档检查通过。

下载前保存完整Flash和2054字节编码器标定；新镜像按HEX各数据段回读一致，参数Flash区逐字节不变。
首次调试器恢复命令会隐式复位，后续启动覆盖RAM，因此未通过运行后校验；其日志保留。
最终在已确认关相的状态用对齐SRAM字写入恢复，字节回读与CAN读出的1024项LUT、标志和零位均一致。
没有把恢复失败当作完成，也没有重做标定或保存Flash参数。

## 波形与原始证据

- [交互展开逐项结果及波形](../../../outputs/motor-live-protocol-20261031/report.html)
- [电流与速度波形](../../../outputs/motor-live-protocol-20261031/live_waveform.png)
- [原始CSV](../../../outputs/motor-live-protocol-20261031/waveform.csv)、[全部CAN帧](../../../outputs/motor-live-protocol-20261031/frames.jsonl)
- [逐项结果JSON](../../../outputs/motor-live-protocol-20261031/result.json)、[下载与标定恢复验证](../../../outputs/motor-live-protocol-20261031/flash/verification.json)
- [追加速度原始CSV](../../../outputs/motor-live-protocol-20261031/speed_probe/waveform.csv)、[最终状态与LUT校验](../../../outputs/motor-live-protocol-20261031/final_state.json)

与上一轮“116不支持”的结果分别保存；旧报告仍表示当时镜像行为，不回写历史记录。
