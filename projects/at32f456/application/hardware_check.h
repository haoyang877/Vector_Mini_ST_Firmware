#ifndef AT32_HARDWARE_CHECK_H
#define AT32_HARDWARE_CHECK_H

#include "motor_axis.h"
#include "yg_protocol.h"

/** @brief 单板检查前台唯一写入开关，ADC 只在一次检查期间发布快照。 */
extern volatile bool g_hardware_check_capture;

/** @brief 在 CAN 开放前绑定本轴检查入口。
 * @param axis 板级持有的轴，寿命覆盖协议端点。
 * @return 前台协议回调，不启动采样或运动。
 */
ProtocolHardwareCheck HardwareCheck_Bind(MotorAxis *axis);

/** @brief 发布已完成的 ADC、SPI 与同拍功率状态，单生产者不等待。
 * @param axis 本板的轴，只能在 ADC 完成事件尾部调用。
 */
void HardwareCheck_CaptureFast(const MotorAxis *axis);

/** @brief 前台消费快照、推进检查并在本轮拥有的驱动异常时请求停机。
 * @param axis 绑定的同一根轴，不在中断调用。
 */
void HardwareCheck_Process(MotorAxis *axis);

#endif
