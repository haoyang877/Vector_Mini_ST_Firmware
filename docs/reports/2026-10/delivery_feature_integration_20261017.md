# 交付功能集成与新固件包记录

日期：2026-10-17。源码`codex/system-refactor@3016431374828eacdadc2f2a9d47d64647d79509`，
来源固定为已提交交付`5665ff87bf3939da980f25798b0110d651a1dc67`。
本页记录离线集成，不把旧OVP实机结果当作新架构实机PASS。

## 实际完成

- 热保护纯核接入实际三相快采样、独立采样监督和统一故障/CLEAR。动态额度在速度和位置PI抗饱和及最终d/q总向量处生效；电压和标定不能绕过热降额。STOP/CLEAR/DEFAULTS保留热历史。
- 速度整形/惯量前馈由现有2kHz外环拥有，默认关闭，原归一化PI增益口径保持；前馈与PI共用原应用/热预算。
- 复用已交付观测器局部运算、SPI两段重叠及标定准备期分拍清表；不恢复旧任务框架、CCM、RTT或旧协议。
- 参数schema13尾追加48B，保留schema12前缀并兼容读取；新增配置经原121/122事务及实际存储保存。离线清标定工具保留设备/模型/应用及身份页，只清角度LUT/零位/资格并撤销配置确认。
- 确定修复CAN bus-off后BUSY状态直接Start导致全局Error_Handler的错误：先Stop，再由READY启动，失败返回不可用。真实生产C反例修复前失败、修复后通过；尚不能据此认定历史无RX的唯一原因。
- Keil配套APP构建后由同一AXF派生调试AXF/HEX/BIN及未激活CAN包；重复Build严格验证并复用，失败撤销旧成功派生物。下载后仍从真实Loader校验启动，不强改PC/SP/VTOR。
- 主机v1.4.4使用同一schema13/ABI契约，保留不匹配拒绝，基础界面未扩张。

未纳入来源未提交的摩擦旋钮实验，未恢复此前按用户要求删除的复杂轨迹、摩擦辨识或旧线协议。

## 本轮离线证据

集成PR通过；随后以同一提交的干净检出执行release通过（包括真实Keil normal构建）。
独立Loader和配套APP另外构建成功，均0错误0警告。原始证据保留在忽略输出目录，
不以历史日志替代本轮：`outputs/runs/delivery-feature-integration-20261017-pr2/summary.json`、
干净检出的`outputs/runs/core-v140-release-20261017/summary.json`及包内`release-verification.json`。

| 产品 | Flash镜像/加载区字节 | 静态RAM字节 | 说明 |
| --- | ---: | ---: | --- |
| normal APP | 77944 | 18528 | 独立零地址目标，不用于覆盖已安装Loader |
| 配套APP v1.4.0 | 80360 | 18576 | APP槽94208B，余13848B |
| Loader v1.0.0 | 15108 | 4952 | Loader槽20480B |

真实Keil重复Build再次调用AfterBuild并返回0错误0警告，CRC保持`94c23f15`、token保持`0x26101701`。
严格复用不是重新链接，也不是CAN升级或APP健康确认。

## 冻结文件和未测项

固件包位于`outputs/delivery/core-v140-20261017/`；`summary.json`列两产品工件hash，
`freeze.json`列来源、主机及写域。整包HEX仅含Loader和APP，逐地址解码确认不覆盖参数/身份，
SHA256为`46c637717a7b9b9c324ee4192a2f3d294cb81ef7a90257764115fde2e56df713`。
APP-only调试HEX为`1794b5189d8e984830fb2c02404f1c640353413bb8ac423e5dfc920004251998`。
CAN升级使用独立未激活`app-package/image.bin`，SHA256为
`86d51c44e064612dbfa46949f9a53d4638bf78209d8759388a90fd3c2a10437a`。
两类镜像不得互换；完整列表以包内清单为准。

新包已交T按用户授权执行下载、清标定、通信、LUT/零位、速度、状态切换和升级。
发包时上述实机项目均尚未通过，本轮热模型IRQ耗时、Keil APP-only真实复位跳转也等待T结果。
不能沿用昨天447镜像/PC或旧B1交付的通信结果；T需重新固定当前UID、供电及输出关闭事实。
硬件仍只由T持有，ROOT/D/F不并发连接。

首次实际接触停在下载前：T确认UID匹配，CPU仍运行旧`0x08004000`入口，TIM1三相使能为0，
但六路门极GPIO为模拟模式，不能凭IDR=0确认驱动输入被拉低。T按AGENTS要求停止并关闭探针，
未HALT/GO/复位、未读备份、未写Flash、未开CAN，未擦标定。此预检不构成新APP运行结果。
已请用户物理断开电机母线并确认停转；得到该确认前不自动恢复试验。
实际记录为`outputs/bench/core-v140-host144-20261017/attempt-01-fresh-baseline/baseline.json`。
