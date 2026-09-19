#ifndef CAN_PARAMETER_WIRE_H
#define CAN_PARAMETER_WIRE_H
#include <stdint.h>

/* 线路约定层：命令/参数注册表、数值编码等级、定点转换与 ID/长度规则。
 * 本文件只描述“线上字节长什么样”，不含传输机制、命令派发或电机耦合。 */

typedef enum
{
    CAN_SET_MODE = 0x00,
    CAN_GET_MODE = 0x01,
    CAN_SET_CURRENT = 0x02,
    CAN_GET_CURRENT_SET = 0x03,
    CAN_SET_SPEED = 0x04,
    CAN_GET_SPEED_SET = 0x05,
    CAN_SET_POS = 0x06,
    CAN_GET_POS_SET = 0x07,
    /***********************************/
    CAN_SET_NODE_ID = 0x08,
    CAN_GET_NODE_ID = 0x09,
    CAN_SET_POLEPARIS = 0x0A,
    CAN_GET_POLEPARIS = 0x0B,
    CAN_SET_ENCODER_STATE = 0x0C,
    CAN_GET_ENCODER_STATE = 0x0D,
    CAN_SET_CURRENT_CAL = 0x0E,
    CAN_GET_CURRENT_CAL = 0x0F,
    CAN_SET_CURRENT_LIMIT = 0x10,
    CAN_GET_CURRENT_LIMIT = 0x11,
    CAN_SET_SPEED_LIMIT = 0x12,
    CAN_GET_SPEED_LIMIT = 0x13,
    CAN_SET_SPEED_ACC = 0x14,
    CAN_GET_SPEED_ACC = 0x15,
    CAN_SET_SPEED_DEC = 0x16,
    CAN_GET_SPEED_DEC = 0x17,
    CAN_SET_SPEED_KP = 0x18,
    CAN_GET_SPEED_KP = 0x19,
    CAN_SET_SPEED_KI = 0x1A,
    CAN_GET_SPEED_KI = 0x1B,
    CAN_SET_POS_ACC = 0x1C,
    CAN_GET_POS_ACC = 0x1D,
    CAN_SET_POS_DEC = 0x1E,
    CAN_GET_POS_DEC = 0x1F,
    CAN_SET_POS_MAXSPEED = 0x20,
    CAN_GET_POS_MAXSPEED = 0x21,
    CAN_SET_POS_KP = 0x22,
    CAN_GET_POS_KP = 0x23,
    CAN_SET_POS_KD = 0x24,
    CAN_GET_POS_KD = 0x25,
    CAN_SET_COGGING = 0x26,
    CAN_GET_COGGING = 0x27,
    CAN_SET_CAN_BR = 0x28,
    CAN_GET_CAN_BR = 0x29,
    CAN_SET_CAN_HB = 0x2A,
    CAN_GET_CAN_HB = 0x2B,
    /***********************************/
    CAN_SET_VBUS = 0x2C,
    CAN_GET_VBUS = 0x2D,
    CAN_SET_IBUS = 0x2E,
    CAN_GET_IBUS = 0x2F,
    CAN_SET_IA = 0x30,
    CAN_GET_IA = 0x31,
    CAN_SET_IB = 0x32,
    CAN_GET_IB = 0x33,
    CAN_SET_IC = 0x34,
    CAN_GET_IC = 0x35,
    CAN_SET_ID = 0x36,
    CAN_GET_ID = 0x37,
    CAN_SET_IQ = 0x38,
    CAN_GET_IQ = 0x39,
    CAN_SET_SPEED2_FILT = 0x3E,
    CAN_GET_SPEED2_FILT = 0x3F,
    CAN_SET_POS2_FILT = 0x40,
    CAN_GET_POS2_FILT = 0x41,
    CAN_SET_TEMP = 0x42,
    CAN_GET_TEMP = 0x43,
    CAN_SET_RS = 0x44,
    CAN_GET_RS = 0x45,
    CAN_SET_LD = 0x46,
    CAN_GET_LD = 0x47,
    CAN_SET_LQ = 0x48,
    CAN_GET_LQ = 0x49,
    CAN_SET_FLUX = 0x4A,
    CAN_GET_FLUX = 0x4B,
    CAN_SET_ERROR = 0x4C,
    CAN_GET_ERROR = 0x4D,
    CAN_SET_ENCODER_REVERSE = 0x4E,
    CAN_GET_ENCODER_REVERSE = 0x4F,
    CAN_SET_POS_KI = 0x50,
    CAN_GET_POS_KI = 0x51,
    CAN_SET_POS_INTEGRAL_LIMIT = 0x52,
    CAN_GET_POS_INTEGRAL_LIMIT = 0x53,
    CAN_SET_CASCADE_POS_KP = 0x54,
    CAN_GET_CASCADE_POS_KP = 0x55,
    CAN_SET_CASCADE_POS_KD = 0x56,
    CAN_GET_CASCADE_POS_KD = 0x57,
    CAN_APPLY_FRICTION_MODEL = 0x58,
    CAN_GET_FRICTION_STATE = 0x59,
    CAN_GET_FRICTION_REASON = 0x5A,
    CAN_GET_FRICTION_COULOMB_POS = 0x5B,
    CAN_GET_FRICTION_COULOMB_NEG = 0x5C,
    CAN_GET_FRICTION_VISCOUS_POS = 0x5D,
    CAN_GET_FRICTION_VISCOUS_NEG = 0x5E,
    CAN_GET_FRICTION_RMSE_POS = 0x5F,
    CAN_GET_FRICTION_RMSE_NEG = 0x60,
    CAN_GET_FRICTION_CANDIDATE_VALID = 0x61,
    CAN_GET_FRICTION_MODEL_VALID = 0x62,
    CAN_SET_STATUS_STREAM = 0x64,
    CAN_GET_STATUS_STREAM = 0x65,
    CAN_GET_PROTOCOL_REVISION = 0x67,
    CAN_GET_COGGING_STATE = 0x68,
    CAN_GET_COGGING_REASON = 0x69,
    CAN_GET_COGGING_PROGRESS = 0x6A,
    CAN_GET_COGGING_POINT = 0x6B,      /* float32 请求索引，float32 有符号 Q15 回复 */
    CAN_GET_COGGING_FULL_SCALE = 0x6C, /* 32768 对应的安培满量程 */
    CAN_GET_COGGING_VALID = 0x6D,
    CAN_GET_TEMPERATURE_SOURCE = 0x6E, /* float32：1 = MCU 内核 */
    CAN_GET_TEMPERATURE_VALID = 0x6F,  /* float32：0/1 */
} CAN_PARAM_ID;

/** @brief CAN 参数在线路上的数值编码等级。 */
typedef enum
{
    CAN_VALUE_FLOAT32,
    CAN_VALUE_MILLI_I32,
    CAN_VALUE_CENTI_I32,
    CAN_VALUE_MILLI_I16,
} CanValueEncoding;

/**
 * @brief 返回写命令参数的线路编码等级。
 * @param param_id 参数 ID。
 * @return 电流族返回 int16 毫安，位置返回 int32 毫弧度，速度族返回 int32 百分一；
 *         其余参数沿用遗留 float32。
 */
CanValueEncoding CanParamWire_CommandEncoding(CAN_PARAM_ID param_id);
/**
 * @brief 返回回复参数的线路编码等级。
 * @param param_id 回复参数 ID。
 * @return 电流族与摩擦结果返回 int16 毫安，位置返回 int32 毫弧度，速度族返回
 *         int32 百分一；其余参数沿用遗留 float32。
 */
CanValueEncoding CanParamWire_ReplyEncoding(CAN_PARAM_ID param_id);
/**
 * @brief 把有限 SI 值转换为保留最小值哨兵的 int32 毫单位。
 * @param value 待转换的 SI 值，允许非有限值。
 * @return 截断并饱和到 2^31 边界的线路值；非有限值返回 INT32_MIN 哨兵。
 */
int32_t CanParamWire_Milli32(float value);
/**
 * @brief 把有限 SI 值转换为保留最小值哨兵的 int32 百分一单位。
 * @param value 待转换的 SI 值，单位 rad/s 或 rad/s²，允许非有限值。
 * @return 截断并饱和到 2^31 边界的线路值；非有限值返回 INT32_MIN 哨兵。
 */
int32_t CanParamWire_Centi32(float value);
/**
 * @brief 把有限安培值转换为保留最小值哨兵的 int16 毫安。
 * @param value 待转换的安培值，允许非有限值。
 * @return 截断并饱和到 ±32767 的线路值；非有限值返回 INT16_MIN 哨兵。
 */
int16_t CanParamWire_Milli16(float value);
/**
 * @brief 把有限 SI 值转换为保留最小值哨兵的 int16 百分一单位。
 * @param value 待转换的 SI 值（状态帧的温度与母线电压），允许非有限值。
 * @return 截断并饱和到 ±32767 的线路值；非有限值返回 INT16_MIN 哨兵。
 */
int16_t CanParamWire_Centi16(float value);
/**
 * @brief 按线上编址规则组装标准帧标识符。
 * @param node 节点身份，调用方保证取值 0..7。
 * @param param_id 参数 ID，只使用低 8 位。
 * @return 11 位可寻址的标识符 `node << 8 | param_id`。
 */
uint16_t CanParamWire_Identifier(uint8_t node, CAN_PARAM_ID param_id);
/**
 * @brief 返回该编码等级在线路上的负载字节数。
 * @param encoding 数值编码等级。
 * @return int16 毫安编码返回 2，其余编码返回 4。
 */
uint8_t CanParamWire_Length(CanValueEncoding encoding);
#endif
