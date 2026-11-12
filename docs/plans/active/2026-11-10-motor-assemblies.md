# 同镜像五轴电机装配适配

目标为 AT32F456 五轴共用 APP。左右轮与 ROLL 使用直驱 HT8115-4，PITCH 为用户确认的 YQ-5740；YAW 沿用 YQ-5708 预设。两轴均为 40:1。

基线为重构工作树的五轴预设及已有 40:1 修改。热默认值依赖减速比，输出传感器依赖角色判断，工装另按节点判断热参数。把热输入归入型号，把减速比和 Hall 归入轴装配，用同一预设生成记录和启动装载。工装可按实物型号核对节点。

验收覆盖五节点的生成和回读、型号错配、缺少热输入、机械换算和联合标定。沿用 schema14 和 AXS2 修订 2，保留较窄的已存限值。执行 PR 离线验证，实机先按用户报出的轴做只读检查。回退本次预设与工装变更时保留原工作树的 40:1 修改。

## 当前验证

- 五节点工装回读通过。0x06 匹配 YQ-5740，输出轴 0.5 rad/s 对应电机轴 20 rad/s。错配型号、保留节点、缺少必需热输入均拒绝输出文件。原有无型号参数的 HT 调用通过。
- PR 结果保存在 `outputs/runs/motor-assemblies-5740-pr/summary.json`。AT32 独立 APP、LoaderApp 均为零错误和零警告；从源码副本生成规范工程，校验初装包的镜像和哈希，清单位于 `outputs/motor-assemblies/at32-5740-bundle/summary.json`。
- 用户确认本批电机均已脱开限位、可连续旋转。首台 YQ-5740 PITCH 的新板经 SWD 双次读取，256 KiB Flash 全部为擦除态。CAN 通道 0 查询未收到回复，尚无板上 APP 或本机标定结果。

## PITCH 首板静止验收

本阶段执行静止检查；现场随后说明电机功率始终接通。使用常规 schema14、AXS2 修订 2，节点 0x06，电机轴限速 0.5 rad/s。热字段暂沿用旧 PITCH 台架记录中的 3 A / 3 A、3600 s、0.8 / 0.7 / 1 / 0.5；型号确认位与标定标志均为零。未写入旧电机的 LUT、零位或 Hall 系数，本轮未验证绕组热特性。

按板级 Loader 的关相电平配置六个桥臂引脚，通过现有维护工具的关相检查后初装。Loader v2.2.0 和 APP v1.6.0 均已写入，写入参数后全片回读相符。主站按本机 UID、版本、镜像 CRC 及目标布局匹配后确认试运行。节点连续六次回复 IDLE、模式 0、故障位 0。实测证据分别位于 `outputs/motor-assemblies/pitch-install/provisioned.json` 与 `can-verified.json`。上电标定和转动尚待测试。

## 首轮三项测试

用户指定联合角度标定、CAN 升级、速度闭环的顺序。PITCH 改用常规已确认修订 2 记录，电机轴速度限值 4 rad/s，运行限流 3 A，标定限流 1.6 A。参数区完整回读匹配，启动为空闲、零故障。

第 1 轮标定被受理，约 5.8 s 后报协议故障 0x1000，SWD 核实内部故障为 SENSORLESS（13）。采到的电机轴位置在 0.606..0.757 rad 内往复，母线 30.53..30.55 V。STOP 后六路输出读回关闭，未取得有效标定。随后节点无应答，SWD 目标连接亦失败，探针仍有 3.18..3.20 V 参考。详见 `outputs/motor-assemblies/pitch-test/first-calibration-trace.json`。等待目标重新上线后继续诊断。

## YQ-5740 PITCH 三项实机结果

重新上电后，在节点 0x06 核对同一 MCU UID 和原记录。观测器在正向稳速时约为 +336 电 rad/s，电机编码器却为 −24 rad/s。将这块板记录中的 `encoder_reverse` 从 0 改为 1，回读整个参数区，只修改方向位及基础记录 CRC。

改向后两轮都完成转子 LUT 和电零位。输出轴建表拒绝两轮数据，每轮 2048 对样本。九点滤波后的 Hall 均值再次取整，两轮分别出现 3 和 2 个非正角度增量。旧代码离线回放均返回映射错误；保留均值的小数位后，两轮均通过原有范围、覆盖、单调性及辨圈误差检查。变动仅涉及标定时建表，采样结构和 Flash 布局沿用原值。

| 顺序 | 实测结果 | 证据（相对于 `outputs/motor-assemblies/pitch-test/`） |
| --- | --- | --- |
| 1. 联合角度标定 | 转子 LUT 和电零位有效，电零位 30573 Q15；40:1 输出轴 Hall LUT 有效。上位机在收尾收到一次有效位全零的测量帧，脚本判失败。随后独立回读 Flash，通过基础和扩展 CRC、绑定、方向和整表单调性检查，连续查询确认 IDLE、零故障、六路关断。未设置工装机械零点。 | `calibration-trace.json`、`calibrated-verified.json`、`calibrated-record.bin` |
| 2. CAN FD 升级 | 通过 Loader 重装并确认同一 APP v1.6.0，镜像 CRC32 `b299b17f`。升级前后全片读回比较，APP 槽外各字节相同，4372 B 本机标定记录与步骤 1 一致。 | `can-upgrade/summary.json` |
| 3. 速度闭环 | 电机轴依次给定 +4、−4、0 rad/s，各保持 3 s，最后 12 个样本平均为 +4.00225、−3.98267、+0.002 rad/s。运行限流 3 A，标定限流 1.6 A。STOP 后 IDLE、零故障且六路关断，随后五帧转速绝对值均小于 0.01 rad/s。 | `speed-summary.json`、`speed-trace.json`、`settled.json` |

本机标定记录 SHA-256 为 `15715d3671f30c6e04d18e5a217a4e1294af7dafd1bf1ce689f67d20f3cf8e1c`。后续换机时根据该台装配核实方向并逐台标定。

工作树 PR 验证通过，记录在 `outputs/runs/pitch-hall-fractional-pr/summary.json`。冻结源码副本的 release 验证通过，记录在 `outputs/pitch-5740-verification/outputs/runs/pitch-joint-final/summary.json`。冻结源码重建的 AT32 CAN 镜像与实机测试包 SHA-256 相同。
