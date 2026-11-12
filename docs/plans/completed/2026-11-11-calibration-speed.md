# 五轴标定输出轴限速

基线为工装 v1.2.0。PITCH 初装运行限速改为 30 rad/s，五轴标定增加独立的 40 rad/s 输出轴限速。保留原扫圈目标、减速比、加减速度、参数格式和旧记录中的较低运行限制。用直驱和减速轴正反转边界测试、原生参数回读及 Windows 整包自检验收。需要回退时使用原 v1.2.0 工装及其配套固件和参数备份。


工装 v1.3.0、参数 v1.1.0、AT32 APP v1.6.3 随同一个 Windows 包交付。原生回归覆盖两个方向及 1:1、40:1 的阈值，PR 和冻结源码的 release 验证均通过，见 `outputs/runs/commission-v130-final/summary.json` 和 `outputs/b130/outputs/runs/commission-v130-verified/summary.json`。解压 EXE 校验了全部 24 份清单文件、五轴预设、PITCH 30 rad/s 交互边界，并从 v1.1.0 已保存的参数重绘 1,024 点曲线。此次未连接硬件。ZIP SHA-256 为 `9ea125a8e6e39957d82b83ab7b3ad6cca0aafa1434518c488cd10c612d37e864`。
