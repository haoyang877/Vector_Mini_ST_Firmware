<!-- v0.6.6：外部参考兼容200Hz接收抖动与5ms插补，实际写板结果见本轮实机证据。 -->
# 软件版本号与查询

2026-10-21：当前候选为 **v0.6.6/build0**，CAN `0x6F` 回复 `00 06 06 00`。
v0.6.6进一步兼容200Hz接收抖动和5ms插补；保留v0.6.5的mode3连续测速反馈。
实机已安装v0.6.6，本轮运动因供电异常中断，状态见[本轮报告](../reports/2026-10-21-external-reference-200hz-v066.md)。
此前v0.6.5离线阶段与v0.6.4部署记录保留，不能替代本轮v0.6.6的完整运动验收。
下面 v0.6.3、v0.6.1 及更早记录仅描述当时的构建与验证。
本次配置及旧 Loader 部署阻断项见 [经典 CAN 部署](../protocols/classic_can_deployment_v063.md)。

文档 v1.5，2026-10-20；当前 `codex/ovp-36v` APP源码：**v0.6.1，build 0**。
v0.5.3先裁剪mode 18并压缩正弦表，v0.6.0再集成绕组热保护和热参数配置；
保留mode 3、速度前馈、摩擦斜率配置及CAN Loader。APP链接到0x08004000，Loader仍为v1.0.0。
此前100轮验收对应冻结v0.5.0，不能作为本次v0.6.1镜像的实机耐久或电机性能验收。
v0.6.1恢复7a2d235的mode 3控制并保留现代参数接口，增加默认关闭的温度保护通信开关；
Roll实测范围及参数见[本轮报告](../reports/2026-10-20-roll-mode3-historical-tuning-v1.md)。
合并与验证范围见[整合记录](../reports/2026-10-14-loader-merge-ovp36.md)，历史v0.4.1归档仍保留。
v0.5.2已验证旧APP状态初装、CAN升级及复位后的标定数据保留，见[本轮记录](../reports/2026-10-14-maintenance-v110-app-v052.md)。
历史 `roll-pitch-v0.1` 标签仍指向原节点。其他工作树的 APP/Loader 版本独立维护。

## 修改与读取

版本唯一配置位于 `firmware/app/firmware_version.h`：
`FIRMWARE_VERSION_MAJOR/MINOR/PATCH/BUILD`。每项为0～255的十进制整数，
修改后重新编译、下载才影响板端；版本由源码编入 Flash，不写入电机参数区。

- RTT 文本通道0在启动时输出 `Vector Mini ST APP v0.6.1 (build 0)`；通道1
  的原有 LKS 示波器格式不变。错过启动文本可读取以下符号或用 CAN 查询。
- Keil/J-Link：只读符号 `g_firmware_software_version_string` 为 `v0.6.1`；
  `g_firmware_software_version` 为 `0x00060100`。地址由本次 AXF/map 决定，
  不能沿用旧 AXF 的地址。
- CAN：标准帧ID=`(node_id << 8) | 0x6F`，发送4字节 `00 00 00 00`；
  同一ID回复4字节 `00 06 01 00`，依次为主、次、修订、构建号。
  Pitch节点4示例ID为 `0x46F`，Roll为 `0x36F`，Yaw为 `0x56F`。
  四字节必须作为大端 uint32 或分别按字节读，**不是float32**。

主机 `can_parameter_protocol.decode(0x6F, payload)` 返回无符号版本整数。
现有 `Client.read()` 是配对 SET/GET 参数接口，不支持这个独立只读查询；
手动发送上述只读帧并解码即可。黄金向量：`tests/vectors/software_version_v1.json`。

## 兼容性

新增0x6F为只读命令，无对应写命令，不启动电机、不保存参数、不修改控制增益。
线路长度、节点寻址和既有参数编码保持不变；协议版本0x67仍返回2。
不改变 Flash 参数 schema10、PSP1 两套 PI、轴配置、LUT、电零位或机械零位。
旧固件忽略此查询、可能超时；不得将超时解释为版本0或自动触发烧录/标定。

版本标记是功能版本，精确构建仍须结合源码、目标和工件SHA256。
2026-10-10 当前Pitch节点4已下载并完整回读核验v0.2.2，参数区和新运动范围保留，最终停机/无故障/关相；未执行本版自动运动试验。证据：outputs/pitch_v022_download_after_range_20261010/download_result.json。其他板端版本以各自实际回读为准。

本次v0.2.1冻结用户认可的V7停靠逻辑；新增版本镜像的硬件验证单独记录。

2026-10-10 经用户授权通过 J-Link 对当前右轮节点2执行下载校验，板内镜像
已与冻结v0.2.0一致，下载接口成功返回0。返回值不表示实际写入字节数，不能
据此断言未重复擦写。完整 Flash 及复位后回读一致，
版本数字/字符串、配置和个体标定均核对通过，最终mode0、故障0、三相关闭，
未启动电机。证据：`outputs/software_version_update_20261010/download_result.json`。
随后更新左轮节点1为v0.2.0：下载回读及复位核验通过，既有参数和个体标定保留，
启动按生产参数服务迁移DMP1/PVF1/PSP1尾记录，两组速度PI均为0.06/0.6。
完整复位后Flash与离线执行生产迁移函数的预测逐字节相同，最终停机、无故障、
三相关闭。证据：`outputs/software_version_update_20261010_02/download_result.json`。
随后更新Roll节点3并保存纯速度PI0.06/0.6、位置内速度PI0.5/1.0；原有范围、
滤波、摩擦前馈和个体标定保持。下载、参数保存及复位后完整回读通过，最终
停机、无故障、三相关闭。证据：`outputs/software_version_update_20261010_03/download_result.json`。
Pitch/Yaw未在本轮更新。该构建包含当前未提交修改，不是正式发布认证。

2026-10-12 v0.3.0新增FSL1独立斜率尾记录；当前构建未下载，原v0.2.2实机记录保留其时间边界。兼容见friction_slew_v1.md。
