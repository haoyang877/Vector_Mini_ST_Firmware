# 电机状态反馈契约 v1.1

日期：2026-10-08。用户决定：合并初始化为 INIT，线上状态用 3，0 保留退役值。
本版补充[单轴线路契约 v1](yg_protocol_motor_wire_v1.md)的状态语义，
不改变统一帧版本 `0x01`、Type／长度／CRC、45B payload、字段偏移或参数 schema12。
实施记录见[八状态计划 v3](../plans/active/2026-10-08-motor-state-flow-simplification-v3.md)。

## 当前状态映射

| APP 主状态 | 108／124 的 `motor_state` | 语义 |
| --- | ---: | --- |
| INIT | 3 | 合并旧 BOOT 与 SELF_TEST；初始化／基础自检，输出关闭，不表示可启动 |
| IDLE | 5 | 原 READY，输出关闭并待命；参数未确认或标定不完整仍可待命 |
| STARTING | 2 | 控制准备，等待外环并在启相前重新核对许可 |
| RUNNING | 6 | 控制运行，零速度／位置保持仍可启相；不承诺机械运动 |
| MAINTENANCE | 4 或 5 | mode 7／8 的标定投影为 4，其余任务沿用 5；任务种类和结果单独读取 |
| STOPPING | 7 | 请求关闭输出并等待真实资源归还；STOP ACK 不能替代静默证明 |
| FAULT | 8 | 可恢复故障锁存，明确 CLEAR＋源恢复＋资源释放后重新 INIT |
| FATAL | 9 | 不可恢复锁存，仅外部复位；普通 CLEAR／连接恢复不能解除 |

0 不再由本版 APP 发布，保留为旧版 BOOT 退役值，不分配新含义。
1 的升级状态仍归 Loader，本版没有增加 APP 升级主状态。
已有业务故障快照继续优先投影为 8；FATAL 优先投影为 9。
保留 CURRENT／SPEED／POSITION／VQ 及维护 mode 原编号，未改变命令载荷或任务 ABI。

## STOP 与身份语义

INIT／IDLE 只有实际关相、无待归还任务／前台／工作区及无故障时，STOP 才原地确认。
仍撤销待启动和目标、禁止派生 SAVE、推进代际使旧事件失效，保留来源和 16 位 STOP
序号屏障。静默 INIT STOP 不重启自检期限或 4 拍连续健康窗口；非静默仍走 STOPPING。
故障优先于 STOP，FATAL 不降级。首次零点有效性、1.5 s 初次期限和 CLEAR 后 500 ms
普通期限保持，监督停摆不在此保证内。

原任务完成与 SAVE 成功继续用独立身份和终态，仍沿用退出再保存的路径。
STARTED 只表示原启动身份曾达到 RUNNING 且数字启相确认，不表示当前输出或转子运动。
STOP 和 CLEAR 均不恢复旧目标，不回滚已经提交 Flash 的效果。

## 兼容与黄金向量

旧主机已识别 3 为 selftest，可继续解析长度／偏移／模式，但显示名称应改为初始化／自检。
主机若以“必须先收到 boot=0”作为允许查询或其他流程的前提，须改为等待状态 3／5
和当前健康／启动限制；不要等待本版不再发布的 0。旧设备反馈的 0 可显示旧版初始化，
不能解释成新含义。内部 Watch 值不等于线上编号，必须使用当前符号文件。

黄金 payload 见[机器可读向量](yg_protocol_motor_state_v1_1_vectors.json)。
它们基于既有 45B 编码黄金输入，仅改变偏移 43 的状态字节，用于布局测试；
不是某状态下实际设备测量或启相证据。其余 44 字节逐字节保持。

`run_state_owner_test.c` 编译真实 `protocol_state`，核对八状态及两类标定映射，
非法内部状态投影 FATAL；`yg_protocol_contract_test.c` 使用真实编码器检查全部
45B 状态 payload，再恢复原 RUNNING 向量完成 CAN FD pack／unpack 和 CRC 检查。
本版离线源码验证不替代当前设备的固件和主机联调。
