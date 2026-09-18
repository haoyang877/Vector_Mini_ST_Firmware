#ifndef COGGING_COMPENSATION_H
#define COGGING_COMPENSATION_H
#include <stdbool.h>
#include <stdint.h>

#include "cogging_calibration.h"

/* 齿槽转矩补偿纯核心：准入、查表、渐变与限幅。
 * 边界约定：
 * - 只接收原始类型事实，不读硬件全局、不操作 PWM、不做存储与协议编解码。
 * - 准入在前台一次性完成（含整表 CRC 复核）；20 kHz 的 Update 只比较身份缓存。
 * - 台账字段名与布局保持既有约定，供 CAN 0x26/0x27、RTT 与 J-Link 继续观察。 */

/* 约 200 ms 达到满补偿的渐变速率，乘以每拍周期得到每拍步长。 */
#define COGGING_COMPENSATION_BLEND_RATE_PER_S 5.0f
/* 查表输出保护上限：标定表实测约 ±0.19 A，异常表不得主导力矩指令。 */
#define COGGING_COMPENSATION_LIMIT_A 1.0f

/** 补偿控制与遥测台账：request 由主机或前台写入，enabled 表示已准入。 */
typedef struct
{
    uint32_t request, enabled, rejected;
    float blend, table_a, applied_a, total_a;
} CoggingCompensationControl;

extern volatile CoggingCompensationControl CoggingCompensation;

/** 准入时锁定的硬件身份快照；运行中任一字段变化立即退出补偿。 */
typedef struct
{
    uint8_t reverse;
    uint16_t electrical_zero_q15;
    int32_t pole_pairs;
    uint32_t table_crc;
} CoggingCompensationIdentity;

/** 准入事实集合：只用于前台一次性判定，table_valid 含整表 CRC 复核。 */
typedef struct
{
    bool mode_accepts_request;
    bool error_clear;
    bool sensorless_off;
    bool table_valid;
    bool scale_matches;
    CoggingCompensationIdentity identity;
} CoggingCompensationAdmitInfo;

/** 每拍事实与输入量：由适配层采集，核心不感知其硬件来源。 */
typedef struct
{
    bool mode_is_current;
    bool error_clear;
    bool sensorless_off;
    bool encoder_usable;
    float command_a;
    float limit_a;
    float tick_s;
    uint16_t position_q15;
    CoggingCompensationIdentity identity;
} CoggingCompensationTick;

/**
 * @brief 尝试准入运行补偿：校验模式、故障、无感、表有效与量程匹配，并快照硬件身份。
 * @param control 补偿台账；函数写入 request/enabled/rejected。
 * @param info 准入事实集合，由适配层采集；本函数不读硬件全局。
 * @return 全部准入条件成立返回 true；任一不满足返回 false 并置 rejected=1。
 * @note 仅前台调用，包含整表 CRC 复核，禁止放入 20 kHz 电流环。
 */
bool CoggingCompensation_Admit(volatile CoggingCompensationControl *control,
                               const CoggingCompensationAdmitInfo *info);

/**
 * @brief 平滑关断补偿：清除 request/enabled，保留准入身份让叠加量在 200 ms 内渐出。
 * @param control 补偿台账。
 * @note 前台或 20 kHz 上下文均可调用；不改变已锁定身份。
 */
void CoggingCompensation_Disable(volatile CoggingCompensationControl *control);

/**
 * @brief 仅归零渐变系数，保留 request/enabled，用于停机预开启或立即关断的组合。
 * @param control 补偿台账。
 * @note 单独调用只清渐变；与 Disable 组合即“立即关断”语义。
 */
void CoggingCompensation_ZeroBlend(volatile CoggingCompensationControl *control);

/**
 * @brief 补偿单拍：资格判定、查表插值、±1 A 钳位、200 ms 渐变与总指令限幅。
 * @param control 补偿台账。
 * @param record 已发布表；仅在被准入且身份一致时读取。
 * @param tick 本拍事实与输入量，由适配层采集。
 * @return 叠加补偿后的总 Iq 指令，单位 A；不合格时为用户指令的直接限幅结果。
 * @note 仅电流环（20 kHz）上下文调用；身份复核只比较缓存 CRC 字段，不重算整表。
 */
float CoggingCompensation_Update(volatile CoggingCompensationControl *control,
                                 const CoggingMapRecord *record,
                                 const CoggingCompensationTick *tick);
#endif
