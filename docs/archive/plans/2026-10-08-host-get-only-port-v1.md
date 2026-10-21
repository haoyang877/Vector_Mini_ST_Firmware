> 历史归档（2026-10-21）：历史基线、被后续范围替代或阶段已停止；归档不等于全部完成。 本文的“当前”指原始基线；现行入口见[当前说明](../../plans/README.md)。归档不新增执行授权。

# YG_MC_HOST 显式 CAN FD 只读端口 v1

日期：2026-10-08。独立分支 `codex/host-lifecycle`，基线 `5aa54e46`。

## 目标与不变行为

把已验收的只读帧回读服务接到用户显式选择的 ControlCANFD 端口，按顺序查询
GET_INFO 页 0..2、GET_CAPS、GET_DEVICE_STATUS 和 GET_MOTOR_STATE。
七工作区、离线演示、升级本地校验与 `prepared_only` 导出保持原行为。
没有默认端口、自动重连、功率动作、STOP 发送、参数写入、升级发送或 SWD 执行。
现有 Python 升级客户端保持独立，GUI 只复用它的统一帧编解码和端口封装。

## 边界与验收

- 仅点击连接后启动辅助进程；用户明确给出 Python 3、SDK DLL、单播节点。
  缺文件、独占失败、启动超时均停留在未连接状态。
- 辅助进程只接受 Type 1/2/3/108，输入必须为三整数的完整 JSON 对象。
  每次只发一条 GET；回复严格核对扩展 FD/BRS、无 RTR/error/ESI、CAN ID、
  序号、类型、长度、CRC 与补零。错误或 250 ms 超时即断开，清除旧回读。
- 不同 UID 或启动编号变化后重新读取全部相关快照；SN 为生产 ASCII 字符串，
  STOP 回执 detail 必须为零，后续反馈不能证明持续关相。
- Qt 首页/工作台只显示实际回读的 SI 字段和来源；没有设备波形时不以演示曲线代替。
  所有功率和写入入口维持关闭。
- 假端口/假 DLL 与 Qt 离线交互测试、格式和 lint、仓库 PR profile 通过；
  Windows 预览八页无布局退化。上述验证不接触设备，也不构成实机通过。

## 回退与待办

回退仅撤销本批 `host_app` 和本计划；保留前一只读边界提交、Loader、固件、
共享 ABI 和旧打包输出。现场验证需另行授权、指定独占无动力台架与当前设备，
核对厂家 SDK 的帧标志、断线及身份行为。H 线路与启动编号提供方冻结后，
再独立评估参数、升级和功率动作的设备执行；本批不预设这些命令布局。

## 当前证据

Qt/MinGW 构建成功；host CTest 2/2 PASS，假端口/假 DLL Python 测试 3/3 PASS。
PR profile PASS，见 `outputs/runs/host-get-only-accepted-20261008/summary.json`，
其中 `hardware_contacted=false`。Windows 预览正常退出，连接页及七工作区截图位于
`outputs/host_app/preview-get-only/`。当前测试覆盖显式连接门禁、GET 白名单、
帧标志和关联、独占打开失败、UID/epoch 清旧值与 STOP 瞬时语义；尚无实机通过结论，
也未制作含 Python 与厂家 DLL 的新版独立安装包。
