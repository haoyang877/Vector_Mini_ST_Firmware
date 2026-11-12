# AT32 电机烧录配参与标定工装

基线为已有五轴空板安装器和固件 Type 160/161 标定服务。目标是一个 Windows 入口完成初装、按实物选轴和方向、执行标定并回读保存结果，同时支持已配板重标。沿用固件和存储 ABI、UID 绑定、空板写入边界与原安装器的默认行为。验收覆盖实际 FULL 包、五轴记录、同向与反向装配、直驱与带 Hall 的任务顺序、票据错误和中断停机。按 UID 保留安装快照和标定前后的参数。工具可独立撤回。

## v1.0.0 交付

Windows x64 工装、参数生成器、CAN 接口 DLL、Loader v2.2.0、APP v1.6.2、FULL HEX、CAN 镜像包与中文说明已合成一个 ZIP。[操作说明](../../guides/motor_commission.md)。重打包入口为 `tools/run.py package_motor_commission --bundle <工厂目录> --can-dll <SDK DLL>`，打包机使用项目 dev、maintenance 依赖和 PyInstaller 6.16.0。

交付件位于 `outputs/deliveries/YG-Motor-Commission-v1.0.0-win64.zip`，SHA-256 为 `140f5ae080546675e43c17584d626663baf89fd346d35aa7dd4cf51ff6092d40`。实际解压后在无开发工具 PATH 的中文带空格目录验证 19 项清单、版本号、交互启动与退出、五轴两方向生成、自检及损坏固件拒绝。模拟工装覆盖五个轴的任务顺序、票据保存、关相、错误路径和初装中断恢复。本版本的新 EXE 未在另一台空板执行带电烧录与运动标定。

工作树 release 检查见 `outputs/runs/motor-commission-v1-delivery/summary.json`，离线检查通过，因源码工作树有未提交变更未生成正式固件发布清单。交付沿用原始 debug 固件分类和构建身份。

## v1.1.0 同向参数交付

根据工装要求取消反向参数生成。随包提供五个轴的 v1.0.0 同向初装参数，旧反向编码器记录在开始重标定前拒绝。Windows EXE 文件名、PE 文件版本和命令行版本一致。FULL HEX 保留原始 Loader v2.2.0、APP v1.6.2 和构建日期。实物操作与参数单位见[工装说明](../../guides/motor_commission.md)。

交付件为 `outputs/deliveries/YG-Motor-Commission-v1.1.0-win64.zip`，SHA-256 为 `9c88979884cdec216d48c024490e2b6b986912d150b3748e725bd63ff0549f8a`。解压运行 EXE 自检，核对 24 项文件摘要、五个轴的参数内容及 CRC、主程序和原生生成器拒绝反向输入。模拟台架验证反向板在任何运动命令前终止。本次未操作实机。

## v1.2.0 标定曲线与结果交付

初装和自定义限速自动采用各轴随包热参数。每次成功标定显示 UID、保存状态、方向、电气零位、CRC32 和补偿峰峰值，在独立运行目录中保存前后参数、结果 JSON、单圈补偿 PNG 和 CSV。旧版已保存的标定记录可按摘要离线导出曲线。[操作说明](../../guides/motor_commission.md)。

交付包为 `outputs/deliveries/YG-Motor-Commission-v1.2.0-win64.zip`，SHA-256 为 `7d2561cac228933a253f5185779550e97e648cafd48407c97c01b272dba3b91a`。通过解压 EXE 自检、交互限速纠错与自动热参数验证。使用 v1.1.0 的 PITCH 实机 Flash 回读验证图与 2×1,024 个 CSV 数据点及原有参数摘要。模拟已完成会话回放覆盖结果显示及全部五份文件；本版未另行驱动电机。PR 通过，见 `outputs/runs/commission-v120/summary.json`。
