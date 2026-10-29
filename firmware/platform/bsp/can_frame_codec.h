#ifndef CAN_FRAME_CODEC_H
#define CAN_FRAME_CODEC_H
#include "bsp_can.h"
/**
 * @brief 校验现行扩展FD/BRS发送帧并编码DLC。
 * @param frame 输入帧，不保留指针。
 * @param dlc 输出编码，失败时保持原值。
 * @return 合法且编码成功返回true。
 */
bool CanFrame_EncodeTx(const BspCanFrame *frame, uint8_t *dlc);
/**
 * @brief 将硬件DLC转换为物理字节数。
 * @param dlc 未移位的编码0..15。
 * @param length 输出字节数，失败时保持原值。
 * @return 输入和输出指针有效返回true。
 */
bool CanFrame_DecodeLength(uint32_t dlc, uint8_t *length);
#endif
