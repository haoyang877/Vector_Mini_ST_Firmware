# 虚拟平台：静态启动与最小速度闭环

复用真实APP、状态机、控制、标定、参数和通信源码，以虚拟端口替代STM32实现。
第一、二阶段提供静态外设；第三阶段增加平均逆变器和PMSM电气/机械模型。
仅验证给定模型上的软件行为，不代表真实电机、传感器标定或硬件测试通过。
不更改现有平台API和Keil工程，不链接HAL、寄存器替身或真实CAN驱动。

## 结构与入口

- `virtual_platform.h`：台架专用接口，不由APP包含。
- `virtual_platform.c`：原平台API实现、采样编码、虚拟时间与有界CAN队列。
- `inverter_model.c/.h`：平均三相电压，不包含PWM开关细节。
- `motor_model.c/.h`：PMSM dq方程、机械惯量和粘性阻尼。
- `tests/virtual_bench/main.c`：原APP启动、数字关相、发送完成/超时的离线检查。
- `tests/virtual_bench/speed.c`：独立模型定律检查、原命令入口速度闭环与STOP。
- `tools/build/virtual_bench.py`：从normal工程读取可移植C文件并直接编译。

沿用`App_Init/Process/FastTick/SupervisorTick`。一个进程只初始化一次；
模型先用前一拍输出推进50us，再CAN接收、20kHz快环、到期2kHz监督、前台。
默认10个积分子步，使用中点法；无抢占、无MCU耗时证据。
默认虚拟UID不是实机UID；空白参数区让真实参数事务回退默认配置，
配置未确认与未标定的启动限制仍保留。速度场景在初始化前装载有效字节记录，
仍由真实参数解码装载；理想编码器全零LUT和零位是合成夹具前提，不是实测标定。
Flash替身只在进程内保存，不模拟掉电。
电流输入为三相不同静态零点，原固件执行完整1秒上电校正。
LED/RGB为空实现；Loader端口明确不支持。静态场景启相仍失败；连接物理模型后
启相才使逆变器电压作用于电机。模型真值只给台架，不提供给APP。

## 运行

先按根AGENTS安装开发依赖。命令中的编译器使用本机实际路径：

```powershell
uv run python tools/run.py format --check
uv run python tools/run.py lint
uv run python tools/run.py virtual_bench --cc <zig路径> --out outputs/virtual_bench/startup
uv run python tools/run.py virtual_bench --cc <zig路径> --scenario speed --substeps 10 --out outputs/virtual_bench/speed10
uv run python tools/run.py virtual_bench --cc <zig路径> --scenario speed --substeps 20 --out outputs/virtual_bench/speed20
```

输出包含编译日志、运行日志、原生程序与summary.json的源码/头文件哈希。
速度场景另输出speed.csv和parameter_seed.bin及产物哈希。
静态场景运行1.6秒虚拟时间：前0.5秒仍INIT，完成校正和稳定窗口后IDLE，
无故障、无目标电流、从未启相；另检查64位发送票据、完成消费和100ms超时取消。
`state=2`表示内部IDLE，`mode=4`表示内部NONE，均不等同线上编号。

## 速度场景与边界

原RequestIdentifiedEnable启动SPEED，原SubmitControl调速，原RequestIdentifiedStop停机；
本场景不经过CAN编码链，不声称验证线上控制通信。
目标10/20/30/20/0rpm各2秒；末0.5秒比较独立模型真值与反馈，
均值误差<=max(2rpm,5%)，标准差<=2rpm，始终无故障或ADC饱和。
随后重新给20rpm运行1秒，直接STOP，50ms后IDLE、目标撤销、输出关闭、电流衰减。
转子保留惯性，STOP关相不等于转子瞬间静止。

对象R=1.905Ω、Ld=Lq=1.635mH、磁链0.0175025Wb、21极对；
J=0.002kg·m²、B=0.002N·m/(rad/s)为演示用合成机械参数，非实测辨识。
电气输入直接引用当前HT8115_4型号宏；硬件使用当前6mΩ/增益10/12位ADC/四次累加。
normal工程显式选型宏由构建命令继承。控制器保留默认30rpm上限、6A及全部默认增益；
夹具只确认配置和理想编码器标定。对象参数初始化后独立，不由运行控制对象反向同步。
PWM极性沿用当前SVM：相电压=Vbus*(三相比较均值-本相比较值)。
相电流经现有电路换算反编码，12位量化后累加四次；编码器量化为15位后左移。
累计机械角采用double，避免多圈后小积分步长被float舍入吞掉。
关相电流按R/L衰减；不模拟二极管回馈、母线动态、死区、开关损耗、磁饱和和热物理。
原固件软件热保护仍执行，温度输入保持固定25°C。
