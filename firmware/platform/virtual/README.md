# 虚拟平台：静态外设接入

当前只完成第一、二阶段：复用真实APP、状态机、控制、标定、参数和通信源码，
以虚拟端口替代STM32实现。没有逆变器或电机动力学，不能验证运动或标定精度。
不更改现有平台API和Keil工程，不链接HAL、寄存器替身或真实CAN驱动。

## 结构与入口

- `virtual_platform.h`：台架专用接口，不由APP包含。
- `virtual_platform.c`：原平台API实现、静态输入、虚拟时间与有界CAN队列。
- `tests/virtual_bench/main.c`：原APP启动、数字关相、发送完成/超时的离线检查。
- `tools/build/virtual_bench.py`：从normal工程读取可移植C文件并直接编译。

沿用`App_Init/Process/FastTick/SupervisorTick`。一个进程只初始化一次；
同刻先CAN接收、20kHz快环、到期2kHz监督、前台。无抢占、无MCU耗时证据。
默认虚拟UID不是实机UID；空白参数区让真实参数事务回退默认配置，
配置未确认与未标定的启动限制仍保留。Flash替身只在进程内保存，不模拟掉电。
电流输入为三相不同静态零点，原固件执行完整1秒上电校正。
LED/RGB为空实现；Loader端口明确不支持。启相会锁存台架失败并结束推进，
不通过运动成功结果掩盖缺失的物理模型。

## 运行

先按根AGENTS安装开发依赖。命令中的编译器使用本机实际路径：

```powershell
uv run python tools/run.py format --check
uv run python tools/run.py lint
uv run python tools/run.py virtual_bench --cc <zig路径> --out outputs/virtual_bench/startup
```

输出包含编译日志、运行日志、原生程序与summary.json的源码/头文件哈希。
本批场景运行1.6秒虚拟时间：前0.5秒仍INIT，完成校正和稳定窗口后IDLE，
无故障、无目标电流、从未启相；另检查64位发送票据、完成消费和100ms超时取消。
`state=2`表示内部IDLE，`mode=4`表示内部NONE，均不等同线上编号。

后续在同目录新增逆变器、电机模型，并明确采样点与PWM锁存时刻；
对象真实参数与控制器配置分开，模型真值只能由台架读取。
当前不新增模型占位文件、插件注册器或通用运行框架。
