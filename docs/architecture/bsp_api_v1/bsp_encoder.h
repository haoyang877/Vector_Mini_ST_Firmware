/**
 * @file bsp_encoder.h
 * @brief 两板共用的编码器采样接口，统一使用65536计数表示一圈。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 发起本拍角度读取，返回时读回传输已经启动。
     * @note 与ReadAngle成对调用；两板均不返回状态，不检查忙、错误或超时，不屏蔽中断。
     */
    void BspEncoder_Begin(void);

    /**
     * @brief 等待已发起的传输完成，释放片选并直接返回角度。
     * @return 0..65535对应一圈；未做方向、零位或LUT修正。
     * @note 必须先Begin；只等待硬件完成，不检查CRC或状态位，不设置超时。
     *       不做方向、零位或LUT修正。
     */
    uint16_t BspEncoder_ReadAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
