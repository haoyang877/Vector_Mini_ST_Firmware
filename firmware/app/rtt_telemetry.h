#ifndef RTT_TELEMETRY_H
#define RTT_TELEMETRY_H
#include "motor_types.h"
/**
 * @brief 发布既有四通道 RTT 值帧。
 * @param mode 本拍运行模式。
 * @param position_rad 机械位置，rad。
 * @param speed_rad_s 机械角速度，rad/s。
 * @param iq_reference_a iq_a 参考，A。
 * @param iq_feedback_a iq_a 测量，A。
 * @note 20 kHz 上下文调用，非阻塞，不读取电机内部对象。
 */
void RTT_Sampling(MotorControlMode mode,
                  float position_rad,
                  float speed_rad_s,
                  float iq_reference_a,
                  float iq_feedback_a);
#endif
