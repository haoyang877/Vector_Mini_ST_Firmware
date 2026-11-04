/**
 * @file bsp_encoder.h
 * @brief 两板共用的编码器采样接口，可选择16位或21位单圈计数。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 角度输出的计数位宽；不改变SPI帧或传感器的真实分辨率。 */
    typedef enum
    {
        BSP_ENCODER_16BIT = 16, /**< 0..65535对应一圈，现有控制环使用此格式。 */
        BSP_ENCODER_21BIT = 21  /**< 0..2097151对应一圈。 */
    } BspEncoderResolution;

    /**
     * @brief 发起本拍角度读取，返回时读回传输已经启动。
     * @note 与ReadAngle成对调用；两板均不返回状态，不检查忙、错误或超时，不屏蔽中断。
     */
    void BspEncoder_Begin(void);

    /**
     * @brief 等待已发起的传输完成，释放片选并直接返回角度。
     * @param resolution 选择BSP_ENCODER_16BIT或BSP_ENCODER_21BIT输出计数格式。
     * @return 16位时为0..65535，21位时为0..2097151，均对应一圈。
     * @note 必须先Begin；只等待硬件完成，不检查CRC或状态位，不设置超时。
     *       MT6835提供真实21位；TLE5012B的15位原始值按所选格式左移补零。
     *       不做方向、零位或LUT修正；现有控制算法应显式选择16位。
     */
    uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
