# 统一电机采集代码

适用AT32F456＋YQ-5708（8:1）。2026-10-31已开始离线回归与关相实测，
已下载并回读校验测试镜像，后续按3A/50rad/s截止执行了0.3A电流和0.5rad/s速度测试。
Id结果满足建议值，Iq上升时间与速度跟踪失败；已关相，详见[续测记录](../reports/2026-10/unified_motor_continued_20261031.md)。
随后通过原标定入口连续三次完成无感启动和观测器闭环，见[无感实测](../reports/2026-10/sensorless_bringup_20261031.md)。
标定启动段用于观察过程，不能代替标定后稳定运行的独立角度精度测量。
执行范围及结果见[测试记录](../reports/2026-10/unified_motor_validation_20261031.md)；
建议指标仍以[v1.1](../plans/active/2026-10-31-unified-motor-test-cases-v1.md)为准。

## 代码入口

| 入口 | 职责 |
| --- | --- |
| [motor_bench.c](../../firmware/app/runtime/motor_bench.c) | 运行限值、单次Id/Iq短脉冲/速度给定、128点采集与冻结读回 |
| [unified_motor_test.py](../../tools/bench/unified_motor_test.py) | CAN FD请求、原始数据、指标计算、HTML/PNG报告 |
| [协议v1](../protocols/unified_motor_bench_v1.md) | 测试命令0x7F00及字段定义 |

只在`ESC-APP-AT32F456-Commissioning.uvprojx`启用`APP_ENABLE_MOTOR_BENCH=1`。
普通APP默认关闭，不增加测试命令。FOC、观测器、速度环和标定沿用现有实现；
标定继续调用原0xA0入口，新增模块不另建控制器。运行前必须显式设置限值，复位后没有测试运行许可。

## 当前能记录什么

| 用例 | 已接入的数据与计算 | 后续需补齐的范围 |
| --- | --- | --- |
| TC01 | 三相原码/A值、电角度、静态噪声和旋转基波残差 | 长时统计、多窗口/档位/重复次数；外部物理PWM纹波 |
| TC02 | 辅助ADC sin/cos原码、XY图、幅值/相差/基波残差 | 四个静止位置、双向重复和采样时序核验 |
| TC03 | Id/Iq给定与反馈、误差/纹波 | 零参考、多电流档和长时多窗口；锁轴条件延期 |
| TC04 | 板端触发Id及≤5ms Iq脉冲，上升/稳定时间、超调、串扰 | 各档正负/下降沿及重复测量；锁轴条件延期 |
| TC05 | 板端时间戳的主机轮询趋势、空载稳态误差 | 多转速档、切换/停止响应和持续振荡判定；加载延期 |
| TC06 | 原始/修正角度、LUT、内部参考误差、电角度/Id/Iq及阶段图 | 独立角度参考、重启持久化和重复性；输出轴角度标定未实现 |
| TC07 | 1000次只读请求、延迟统计、若干非法字段的拒绝 | 普通启停生命周期、CRC/地址/丢帧/重复请求注入 |

这是一套待联调的采集代码，不是已经完整实现并验证v1.1的自动验收套件。
模板`pending_scope`明确记录缺项，汇总不会把这些项目显示成整项PASS；
`PASS`子项仅表示实际记录满足其已实现判据。离线回归、关相采样和带电控制必须分别记录验证范围。

## 离线准备

在仓库根目录使用已安装的开发环境：

```powershell
python tools/build/build_firmware.py --target r40-app-commissioning --uv4 <Keil的UV4.exe路径>
python tools/bench/unified_motor_test.py template --output outputs/unified-motor/plan.json
```

生成模板不会打开CAN。`reviewed=false`，电流/速度/行程/时限和镜像身份均未填写；
因此模板不能直接启动电机。`test_speed_rad_s`是工况速度，和较高的速度保护截止值分别填写。
所有角速度及累计行程均按电机轴，不是减速后的输出轴；仅绘图分析使用8:1换算。
采集时限须包含预热与Iq回零后的5ms尾段，机械行程和线缆允许范围须另行确认。

现有适配器入口为`ControlCanFd`，复用现有公司帧编解码；USB不作为第二套实现。
显式执行时，主机还要求匹配的本地镜像SHA256、GET_INFO page0的32字节身份、
DLL路径、节点地址、新输出目录和显式`run --execute`。这些是运行参数，不构成固件已烧录或读回校验的证明。
标定算法原有电流/速度不被测试模块缩小；测试限值过低会触发停止，不会偷偷改标定行为。

## 结果查看

一次实际采集将保存`plan.json`、`manifest.json`、`raw/*.csv`、`raw/*.json`、
`raw/frames.jsonl`、`plots/*.png`、`results.json`和`report.html`。
报告可离线重新生成：

```powershell
python tools/bench/unified_motor_test.py report --input outputs/<实际批次目录>
```

波形使用板端20kHz tick，快环缓冲停止后读取；慢速趋势不是均匀采样，不用于高频纹波或FFT。
sin/cos读取最新完成的辅助DMA半帧，没有独立辅助采样时间戳，不能据此宣称同步采样。
原始数据不足、周期不足、硬件缺失和未执行均单独展示，不生成模拟实测图。
通信错误终止本批次，不自动重发动力请求；停止后仍需分别确认数字关相与机械静止。

离线用例见[主机与报告回归](../../tests/unit/test_unified_motor_bench.py)和
[固件内存回归](../../tests/unit/native/test_unified_motor_bench.py)，均已执行。
固件内存回归使用外设替身，不等同于真实电流环或速度环测试。

## 普通接口实测补充

[116/108/124实测](../reports/2026-10/motor_live_protocol_20261031.md)已通过正负50mA目标及实时反馈验证。
配置LIMITS后，可用原117启动；INFO的run_kind=7表示普通接口运行，沿用KEEPALIVE和原截止。
116不会被台架参考覆盖，也不会启动或切换模式。普通APP仍不具有该私有租约。
