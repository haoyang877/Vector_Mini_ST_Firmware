# 齿槽控制状态所有权收口 v1

日期：2026-09-29。状态：本批源码与离线验收完成，完整门禁仍有既有失败。
基线：快环边界提交 `f8c344b`。

## 目标、调用者与不变项

用户要求删除重构旧全局变量。当前 app/motor/services/communication/common/shared/loader
公开头的可写 extern 仅剩齿槽四个对象：TorqueGuard、TorqueTelemetryState、
TorqueTelemetry、FocCoggingCompensationRequest。常量配置不是可写状态；其他模块私有
状态维持明确所有者，不为消除 static 制造大型上下文。

四对象实际消费者为齿槽适配器、真实电流任务、native齿槽检查与历史SWD操作说明。
仓库内没有继续写它们的生产台架脚本。删除旧符号与写变量入口，不提供兼容别名。
补偿请求通过有界值接口受理，由现有稳定配置借用前台消费；台架租约原子更新，
跳闸事实及遥测通过快照读取。新的数据接口只服务既有行为和离线测试，不开放新CAN编号。

保持 schema11、表格式、CAN/升级v2、控制数学、保护阈值、运行频率、32B邮箱和分区。
租约仍按20kHz倒计时，超速或无效测速优先于到期，跳闸由原结果协议停机；关闭保护
不能撤回已经挂起的STOP。补偿仍前台完整校验、200ms渐入渐出，快环不做CRC。
遥测仍冻结同一快环36B帧，未消费的帧不覆盖；改为显式请求/读取即消费，不允许改内部状态。

## 实施和验收

1. 私有化四对象，收回所有可写extern；请求与读取接口具备中文契约，不返回内部地址。
2. 用短临界区提交租约和遥测请求/读取；不在临界区做CRC、Flash、分配或等待。
3. 迁移真实native调用，覆盖非法租约不改变保护、租约到期/超速、挂起STOP不丢失、
   遥测重复请求/空指针/消费、补偿请求由稳定前台消费。原核心特征继续通过。
4. 更新实际操作文档，记录SWD变量入口删除；受影响文件格式/lint/接口不得新增失败。
5. 新源码运行PR及两种APP构建、包校验，记录ROM/RAM及资源变化。当前全局既有
   门禁失败仍单列；不增加豁免或以旧镜像证明新代码。

不烧录、不复位、不写设备参数、不命令电机。台架租约不是生产失联保护，也不允许绕过
主状态机使能。完成后合入干净重构主线，保留当前证据，再归档本轮临时worktree。

## 基线、回退与证据

基线普通APP ROM88,284 B/RAM31,568 B，LoaderApp ROM89,940 B/RAM31,616 B；
当前包89,984 B，余4,224 B。基线PR为26组native和19项integration通过，unit42/43，
format17/lint601/interfaces34/docs10。快环边界计划记录完整对照。
按本批提交revert回退，不恢复旧符号作为兼容，不reset/clean其他工作树；保留忽略数据。
新证据放 `outputs/cogging-state-owner-20260929/` 与本批独立 `outputs/runs/`。

## 实际结果

- 四个旧符号及可写extern全部删除，私有状态归齿槽适配器；前台请求与快照使用短临界区。
  APP/motor/services/communication/common/shared/loader公开头仅剩一个外部const配置声明，
  没有可写extern对象；不包含vendor/CubeMX域，也不宣称消灭模块私有静态状态。
- 补偿开关只登记值，稳定配置前台消费；保护非法值保持旧租约，关闭不撤回挂起STOP；
  遥测无覆盖、空指针不消费、读取后调用方副本与下一帧独立。真实核心/适配器测试通过。
- PR：26组native、19项integration PASS，unit42/43（历史outputs断链）；architecture5known/0new、
  布局/hygiene PASS。format16（减少1）、lint601、接口34、docs10，本批文件无新增问题。
- 普通APP ROM88,292 B / RAM31,568 B，LoaderApp ROM89,940 B / RAM31,616 B；
  相对最初fc6c876c分别减少24 B，RAM未增加。两工程全量构建0 Error / 0 Warning。
- 当前升级包89,984 B，槽余4,224 B；真实包通过Loader runtime原生模型验证。
  最坏栈、实机周期、物理输出与掉电未验收，没有设备写入或电机命令。
- 证据：`outputs/runs/cogging-state-owner-candidate-20260929/summary.json` 与
  `outputs/cogging-state-owner-20260929/`；全局门禁FAIL和开发版本0不标为发布通过。

当前齿槽指南已替换旧SWD字段、旧CAN开关和历史地址操作说明；历史实机报告独立保留。
