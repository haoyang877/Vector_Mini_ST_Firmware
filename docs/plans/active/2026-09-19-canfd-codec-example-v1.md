# 公司 CAN FD 打包解包示例计划 v1.0

日期：2026-09-19。范围：离线 C99 教学示例，不改 APP、Loader 或协议 ABI。

## 目标与依据

演示已有 v1.2.3 的 SET_POSITION（105）和 MOTION_FEEDBACK（124）：
业务对象、小端 Payload、公司 16 B 头与 CRC、29 位 CAN ID、DLC、接收校验与对象还原。
源地址 02、目的地址 03，位置单位 0.001 rad。

已检查 templates/c_module、firmware/communication/protocol、现有 Python 参考编码器
和 tests/run.py。实际固件命令与公司设计稿不同；示例放 host_app/examples，
不把设计稿直接接入现有硬件路径。公司 CRC 在现有固件中没有可复用的同契约 C 实现，
以现有 tools/analysis/protocol_vectors_reference.py 作为独立黄金向量依据。

## 实施与验收

1. 一个独立 C 文件，显式小端编码，无 packed struct、HAL、动态分配或硬件访问。
2. 两个消息的完整循环；反馈来自明确的模拟测量，不把目标伪装为真实位置。
3. 原生检查覆盖黄金向量、负数、长度、CRC、地址、类型与填充；接收失败不修改输出。
4. 文档说明真实驱动需在业务层检查控制权、模式、使能、序号、限位与超时。
5. 运行格式、lint 和 PR 验证，结果记录于本文；产物进入 outputs。

兼容性：仅演示已有单轴 200 Hz 基础格式，不引入新命令，不代表已实现五轴双向
1 kHz 同步控制；该功能仍由独立建议稿规定。撤销示例、测试和文档即可回滚，固件不受影响。

## 验证结果

待本次执行完成后记录。
