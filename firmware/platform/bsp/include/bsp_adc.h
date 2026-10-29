/**
 * @file bsp_adc.h
 * @brief 相干快帧、辅助原码/物理量及受控零偏提交。
 * @note MCU采集原码，BSP统一单位和有效性；应用拥有标定过程与保护。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 快帧内独立功能组的有效位，不表示采样是否同拍。 */
    typedef enum
    {
        BSP_ADC_FAST_PHASE_CURRENT_VALID = 1U << 0,
        BSP_ADC_FAST_BUS_VOLTAGE_VALID = 1U << 1,
        BSP_ADC_FAST_BUS_CURRENT_VALID = 1U << 2
    } BspAdcFastValid;

    /** @brief 同一PWM周期的逻辑快帧；OK表示取得帧，各组须检查valid_mask。 */
    typedef struct
    {
        float phase_current_a[3]; /**< 已扣零偏的A/B/C相电流，A；无效时三个字段均为NAN。 */
        float bus_voltage_v;      /**< 同一PWM采样周期的母线电压，V；无效时为NAN。 */
        float bus_current_a;      /**< 同一PWM采样周期的独立母线电流，A；无效时为NAN。 */
        uint32_t valid_mask;      /**< BspAdcFastValid位组合；禁用或未就绪组不置位。 */
        uint32_t sequence;        /**< 硬件快帧序号，读取不推进，零及回绕合法。 */
        uint32_t sampled_us;      /**< 原发布点时间，us，不宣称精确采样保持时刻。 */
    } BspAdcFastSample;

    /** @brief 同一PWM周期的电流原码；三相与母线电流独立使能，均不要求已有零偏。 */
    typedef struct
    {
        float phase_counts[3];    /**< 三相等效12位单次计数；本组无效时均为NAN。 */
        float bus_current_counts; /**< 母线电流等效12位计数；无效时为NAN。 */
        uint32_t valid_mask; /**< 仅使用PHASE_CURRENT_VALID和BUS_CURRENT_VALID，表示原码有效。 */
        uint32_t sequence;   /**< 对应硬件快帧序号，读取不推进。 */
    } BspAdcRawCurrent;

    /** @brief 全部逻辑模拟信号；不是引脚号，不作为协议或持久记录编号。 */
    typedef enum
    {
        BSP_ADC_PHASE_CURRENT_A,       /**< A相电流，A。 */
        BSP_ADC_PHASE_CURRENT_B,       /**< B相电流，A。 */
        BSP_ADC_PHASE_CURRENT_C,       /**< C相电流，A。 */
        BSP_ADC_BUS_VOLTAGE,           /**< 母线正极相对负极的电压，V。 */
        BSP_ADC_BUS_CURRENT,           /**< 独立传感器母线电流，A；属于快采样。 */
        BSP_ADC_MOTOR_TEMPERATURE,     /**< 电机温度，摄氏度。 */
        BSP_ADC_MCU_TEMPERATURE,       /**< 已校准MCU温度，摄氏度。 */
        BSP_ADC_ANALOG_SUPPLY_VOLTAGE, /**< 内部参考推算的MCU模拟供电VDDA，V。 */
        BSP_ADC_HARDWARE_VERSION,      /**< 版本采样脚相对ADC地的电压，V，不是版本号。 */
        BSP_ADC_CHANNEL_COUNT          /**< 通道数量/非法读取哨兵。 */
    } BspAdcChannel;

    /** @brief 单通道物理量快照；需要同拍电流/电压时使用ReadFast。 */
    typedef struct
    {
        float value;         /**< 单位由BspAdcChannel确定。 */
        uint32_t sequence;   /**< 本通道发布序号。 */
        uint32_t sampled_ms; /**< 原发布点时间，ms，不由读取动作刷新。 */
    } BspAdcValue;

    /** @brief 原码与可选同帧参考原码；不施加零偏、增益或温度校准。 */
    typedef struct
    {
        float counts;           /**< 等效12位ADC计数，0..4095，保留过采样小数。 */
        float reference_counts; /**< 同帧内部VREF计数，仅reference_valid时有效，否则为NAN。 */
        bool reference_valid;   /**< MCU温度必须携带同帧VREF；其它通道一般为false。 */
        uint32_t sequence;      /**< 原始转换发布序号。 */
        uint32_t sampled_ms;    /**< 原发布点时间，ms。 */
    } BspAdcRawValue;

    /**
     * @brief 读取同一PWM采样周期的电流/电压快照。
     * @param sample 仅OK时整体更新；
     *   三相电流、母线电流、母线电压分别检查valid_mask，不保存指针。
     * @return 新鲜完整帧为BSP_OK；空指针为INVALID_ARGUMENT；无帧为NOT_READY；超龄为STALE；
     *   硬件故障为IO_ERROR；全部快组禁用为NOT_SUPPORTED。
     * @note 快环或前台有界读取，不等待转换，不推进序号；失败不修改输出。
     * @note 复制允许采样中断抢占；版本冲突最多重试一次，再以短临界区取得一致副本。
     * @note 三组独立使能；仅电流组需要各自零偏。
     *   缺零偏只清本组有效位并填NAN，不屏蔽其它量。
     * @note 已使能的母线电流必须来自相同PWM周期，禁止把旧辅助缓存复制为当拍快数据。
     * @note 跨ADC结果由BSP按同一周期收齐后发布，未齐不发布新快帧；未启用组不参与收齐条件。
     */
    BspResult BspAdc_ReadFast(BspAdcFastSample *sample);

    /**
     * @brief 读取同一PWM周期的三相及独立母线电流原码。
     * @param sample 仅OK时整体更新；valid_mask只使用三相/母线电流位，不保存指针。
     * @return 取得原码帧为BSP_OK；空指针为INVALID_ARGUMENT；无帧为NOT_READY；超龄为STALE；
     *   硬件故障为IO_ERROR；两个电流组均禁用为NOT_SUPPORTED。
     * @note 关相标定或诊断上下文有界读取，不要求已有零偏，不启动转换。
     * @note 三相与母线电流原码独立有效；对应组未启用时填NAN并清位，不用另一个组的数值替代。
     * @note 原码有效位不要求物理零偏校准，因此可与ReadFast的物理有效位不同；不得据此直接启相。
     */
    BspResult BspAdc_ReadRawCurrent(BspAdcRawCurrent *sample);

    /**
     * @brief 读取指定逻辑信号的标定/诊断原码。
     * @param channel 三相电流、母线电流和母线电压均读快帧缓存；其它读各自辅助缓存。
     * @param sample 仅OK时整体更新；MCU温度含同帧VREF，VDDA通道counts为VREF原码，不保存指针。
     * @return 成功为BSP_OK；非法通道/COUNT/空指针为INVALID_ARGUMENT；未提供为NOT_SUPPORTED；
     *   无帧为NOT_READY；超龄为STALE；采集故障为IO_ERROR。
     * @note 快环、监督或前台均可有界读取，缺物理校准不妨碍原码读取。
     * @note 只做原码域归一化，不读取零偏参与运算，也不执行电流、电压或温度物理换算。
     * @note 逐通道调用不保证同拍；电流标定用ReadRawCurrent并检查组有效位。
     * @note 0和4095为合法原码；轨限与稳定样本规则归应用，不能伪造原码或读取不存在的通道。
     */
    BspResult BspAdc_ReadRawChannel(BspAdcChannel channel, BspAdcRawValue *sample);

    /**
     * @brief 原子提交已经计算并通过应用校验的三相零偏。
     * @param offsets_counts A/B/C有限零偏值，等效12位计数，不保存指针。
     * @return 完成为BSP_OK；参数非法为INVALID_ARGUMENT；实际未关相为REJECTED；
     *   未初始化为NOT_READY；未启用为NOT_SUPPORTED。
     * @note 允许原关相快ISR或前台的唯一标定所有者调用，有界不等待；
     *   在短临界区内检查关相并整体提交。
     * @note 成功后同一拍重新读取的物理电流即可使用；样本序号不推进，已返回快照不原地改变。
     * @note 应用只在提交成功后置标定完成；失败保留旧偏置，不保存Flash、不计算标定或放宽保护。
     */
    BspResult BspAdc_SetPhaseOffsets(const float offsets_counts[3]);

    /**
     * @brief 提交已计算的独立母线电流零偏。
     * @param offset_counts 有限的等效12位单次ADC零偏计数。
     * @return 成功为BSP_OK；非有限值为INVALID_ARGUMENT；未关相为REJECTED；未初始化为NOT_READY；
     *   无独立传感器/未启用为NOT_SUPPORTED。
     * @note 允许原关相监督ISR或前台唯一标定所有者调用；检查和提交处于同一短临界区。
     * @note 应用保留128连续样本、稳定窗口等校验；不影响估算电流、其它通道或持久记录。
     */
    BspResult BspAdc_SetBusCurrentOffset(float offset_counts);

    /**
     * @brief 读取指定逻辑信号的有效物理量。
     * @param channel 任一有效逻辑信号；电流/母线电压使用同一快帧缓存。
     * @param sample 仅OK时整体更新，不保存指针。
     * @return 成功为BSP_OK；非法通道/COUNT/空指针为INVALID_ARGUMENT；未提供为NOT_SUPPORTED；
     *   无帧为NOT_READY；本通道缺校准为NOT_CALIBRATED；超龄为STALE；故障为IO_ERROR。
     * @note 快环、监督或前台均可有界读取，不启动转换或刷新时间；同拍控制使用ReadFast。
     * @note 快通道只换算所选通道，物理值、序号与时间戳来自同一快帧副本。
     * @note 母线电流和三相电流独立使用各自零偏；母线电压不受电流标定资格影响。
     * @note 缺参考校准时MCU温度/VDDA不能伪造物理值，原始TS/VREF仍通过ReadRawChannel提供。
     */
    BspResult BspAdc_ReadChannel(BspAdcChannel channel, BspAdcValue *sample);

    /**
     * @brief 在原监督温度处理点推进慢速辅助采集。
     * @return 有新完成值为BSP_OK；正常等待为NOT_READY；无辅助通道为NOT_SUPPORTED；
     *   采集故障为IO_ERROR。
     * @note 仅监督任务唯一调用点，每拍至多一次，有界不等待；只负责温度、VDDA及版本等慢通道。
     * @note 母线电流属于PWM同步快采样，不由本接口触发、推进或降低采样频率。
     * @note 温度保持原读取/换算/发起下一次转换顺序；普通读取不启动硬件。
     */
    BspResult BspAdc_UpdateAuxiliary(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
