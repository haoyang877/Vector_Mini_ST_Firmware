#ifndef COGGING_MAP_H
#define COGGING_MAP_H
#include <stdbool.h>
#include <stdint.h>

/* 齿槽表记录：数据格式、身份签名、完整性校验、查表与成表。
 * 只处理数据，不包含标定过程、电机控制或硬件访问。 */

#define COGGING_MAP_POINTS 1024U
#define COGGING_MAP_POINTS_LOG2 10U
/* 编码器整圈为 16 位：高 10 位选表点，低 6 位在相邻点之间线性插值。 */
#define COGGING_MAP_INDEX_SHIFT (16U - COGGING_MAP_POINTS_LOG2)
#define COGGING_MAP_INDEX_MASK ((1U << COGGING_MAP_INDEX_SHIFT) - 1U)
#define COGGING_MAP_SUBSTEPS (1U << COGGING_MAP_INDEX_SHIFT)
#define COGGING_MAP_MAGIC 0x31475143U /* CQG1：有符号满量程 Q15、1024 个机械位置 */

/* 表索引 i 对应经过方向设置与线性化的单圈编码器角 i * 2π/1024，
 * 不依赖用户机械零位；正值表示需要正 Iq 保持。
 * CRC 覆盖 magic、encoder_signature、full_scale_a，再覆盖 iq_q15（小端目标）。 */
typedef struct
{
    uint32_t magic, encoder_signature;
    float full_scale_a;
    uint32_t crc32;
    int16_t iq_q15[COGGING_MAP_POINTS];
} CoggingMapRecord;

extern CoggingMapRecord CoggingMap;

/**
 * @brief 由编码器方向、电零位、极对数和线性化 LUT 生成表身份签名。
 * @param reverse 编码器方向配置（0/1）。
 * @param electrical_zero Q15 电零位。
 * @param pole_pairs 电机极对数。
 * @param encoder_lut 1024 点线性化 LUT，所有权归调用方。
 * @return 用于表匹配的 CRC32 签名。
 * @note 前台调用；LUT 长度固定 1024，禁止传入更短的缓冲区。
 */
uint32_t Cogging_EncoderSignature(uint8_t reverse,
                                  uint16_t electrical_zero,
                                  int32_t pole_pairs,
                                  const int16_t *encoder_lut);

/**
 * @brief 按 16 位线性化机械角在 1024 点表上跨圈线性插值，输出等效保持电流。
 * @param record 已发布的完整表；只读，调用方保证表已通过 CoggingMap_Valid。
 * @param position_q15 有方向、线性化后的单圈机械角，整圈映射 0..65535。
 * @return 该角度的等效保持电流，单位 A，使用表内保存的满量程换算。
 * @note 20 kHz 电流环热路径：不做参数校验、不访问硬件、不写全局。
 */
float CoggingMap_LookupA(const CoggingMapRecord *record, uint16_t position_q15);

/**
 * @brief 校验记录的 magic、身份签名、满量程与 CRC 是否一致。
 * @param record 待校验记录，允许为 NULL。
 * @param signature 当前编码器身份签名。
 * @return 全部一致返回 true；否则返回 false。
 * @note 包含整表 CRC 复算，禁止在 20 kHz 电流环调用。
 */
bool CoggingMap_Valid(const CoggingMapRecord *record, uint32_t signature);

/**
 * @brief 由标定暂存表生成发布记录：去除整圈均值、Q15 钳位并写入 CRC。
 * @param fresh_q15 标定暂存的 1024 点有符号 Q15 表，只读。
 * @param full_scale_a 采样满量程，单位 A，必须为正的有限值。
 * @param signature 当前编码器身份签名。
 * @param record 输出记录；仅在返回 true 时被完整覆盖。
 * @return 参数合法且全部转换值在 int16 范围内返回 true；否则返回 false。
 * @note 仅前台、PWM 关闭后调用；写入期间先把 magic 清零，避免读到半成品记录。
 */
bool CoggingMap_Build(const int16_t *fresh_q15,
                      float full_scale_a,
                      uint32_t signature,
                      CoggingMapRecord *record);
#endif
