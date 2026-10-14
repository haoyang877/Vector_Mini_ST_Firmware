# 热参数通信配置 v1 离线验证

范围：`codex/winding-thermal-protection`工作树候选，APP v0.3.0 build 3。仅验证逻辑与普通构建；没有连接探针、下载固件、启动电机或合入开发分支。

## 最少标定与实现

电机标定仅保留连续电流与升温时间常数。单相持续电流自动取连续电流÷√2，冷却常数取max(120 s, 升温常数)。峰值电流属于运行限制，80%进入、70%退出、100%停机、50%恢复属于默认动作策略。

七组CAN参数0x70～0x7D支持读写，0x7E查询扩展版本1；线路数据为4字节大端float32，热门限用百分数。停机且三相高阻时接受配置，通信上下文预计算系数，下一关相快拍应用。参数改变不清热量或故障，持续电流降低时按平方比例放大已有H，增大时保留H。

mode 9保存MTH1/v1独立40字节尾记录，schema10及旧2292字节前缀保持，总长2332字节。旧空尾部采用默认值，坏CRC/版本/参数关系拒绝参数准入；个体标定及原有尾记录偏移保持。

## 结果

- Python：55项单元、36项集成测试全部通过。
- 原生C：20组全部通过，包含真实热模型、生产CAN收发/分派、参数上传/下载、位置/速度控制、限流与关相路径。
- 热配置专项：黄金帧、停机写入、NaN/Inf/越界及错误门限顺序拒绝、单槽保护、派生参数、热历史/锁存保留及可配置恢复门限通过。
- MTH1专项：黄金CRC、存取一致、320位逐位破坏拒绝、旧全0/全FF尾部、未知版本及坏参数拒绝通过。PVF1/PSP1原黄金记录仍通过，CRC helper复用没有改变原计算契约。
- Keil普通目标：0错误、0警告。Code 91532、RO 5368、RW 436、ZI 27532字节；ROM 97336、RAM 27968字节。
- 工程输入检查：普通/HIL工程均无缺失源文件或头文件；没有构建或运行HIL硬件目标。用户原有Keil工程与选项文件SHA256在本轮构建前后保持。

新测试通过Ruff检查，修改C片段使用根Clang格式配置。该旧分支没有统一verify/format/lint命令，已运行可用完整离线回归及普通构建，不宣称正式PR/release认证。

## 证据与复现

当前证据：`outputs/thermal_can_20261014/summary.json`、`source_manifest.json`、`keil_build.log`及`verified_regression/`。summary记录源码清单哈希、AXF/HEX/map哈希、版本、测试组、布局及原用户Keil文件保持结果；旧失败迭代不作为本次通过证据。

```text
python tests/run.py --cc <C99编译器> --out outputs/thermal_configuration_regression
python tools/run.py test_motor_thermal_configuration --cc <C99编译器> --out outputs/thermal_configuration
python tools/run.py build_firmware --target normal
```

逻辑测试不证明实际绕组安全温升、默认冷却规则的物理保守性、CAN电气链路或最坏快环耗时。3 A及65.9849 s仍是候选配置，8 A / 10 s仍只作为参考参数来源，不能作为已验证安全时间。

详细参数及帧：[通信协议](../protocols/motor_winding_thermal_configuration_v1.md)。实现决策：[配置计划](../plans/2026-10-14-winding-thermal-can-configuration-v1.md)。
