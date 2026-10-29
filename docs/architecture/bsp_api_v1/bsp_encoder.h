/**
 * @file bsp_encoder.h
 * @brief 编码器事务、单圈样本及只读诊断。
 * @note 采集事务由BSP持有；器件协议解码与SPI传输各自独立。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 保留现有型号诊断数值，数值2不复用。 */
    typedef enum
    {
        BSP_ENCODER_NONE = 0,
        BSP_ENCODER_TLE5012B = 1,
        BSP_ENCODER_MT6835 = 3
    } BspEncoderType;

#define BSP_ENCODER_CONFIG_WORDS 6U

    /** @brief 成功时发布的控制样本，不包含器件帧和寄存器。 */
    typedef struct
    {
        uint16_t angle_counts; /**< 未做方向/零位/LUT修正的65536计数域，0..65535。 */

        uint32_t sequence; /**< 仅成功发布推进，零和回绕合法。 */
    } BspEncoderSample;

    /** @brief 本次诊断快照中实际取得的原始字，不表示协议校验通过。 */
    typedef enum
    {
        BSP_ENCODER_DIAG_FRAME_VALID = 1U << 0,
        BSP_ENCODER_DIAG_SAFETY_VALID = 1U << 1
    } BspEncoderDiagnosticValid;

    /** @brief 最近一次采样事务的诊断快照，与控制样本独立读取。 */
    typedef struct
    {
        BspEncoderType type;       /**< 器件型号，用于解释诊断字。 */
        BspResult sample_result;   /**< 本次Complete结果，不是读取快照的结果。 */
        uint32_t attempt_sequence; /**< 每次有所有权的Complete推进，成功失败均计数。 */
        uint32_t sample_sequence;  /**< 仅sample_result为OK时对应控制样本，否则不可使用。 */
        uint32_t valid_mask;       /**< 原始字有效位；未取得的字清位且填0。 */
        uint16_t frame_word;       /**< 保留原诊断表示，具体含义见EXECUTION.md。 */
        uint16_t safety_word;      /**< 保留原安全/状态字；0本身不能证明有效或校验通过。 */
        bool crc_checked;          /**< 本次确实执行CRC校验；通过与否由sample_result表示。 */
    } BspEncoderDiagnostics;

    /**
     * @brief 预约并开始本拍编码器事务。
     * @return 取得事务为BSP_OK；已有所有者为BUSY；未启动为NOT_READY；未提供为NOT_SUPPORTED；
     *   启动失败为对应错误。
     * @note 唯一快环调用者，不重入；同步回退也先预约，在Complete原预算内执行。
     * @note 只有OK取得所有权；失败收回本次资源，调用方不得对失败Begin调用Complete。
     */
    BspResult BspEncoder_Begin(void);

    /**
     * @brief 将本拍事务推进到响应阶段。
     * @return 推进为BSP_OK；无本拍事务为NOT_READY；重复/非法阶段为FRAME_ERROR；
     *   传输失败返回对应错误。
     * @note 同一调用者每拍至多一次，不重发下一拍命令；保留原计算与传输重叠。
     * @note Begin成功后，无论本函数成败都须Complete收尾；不清除本拍已发生的错误。
     */
    BspResult BspEncoder_BeginReadback(void);

    /**
     * @brief 结束本拍事务，校验解码并收回总线。
     * @param sample 仅BSP_OK时整体更新的输出，不保存指针。
     * @return 成功为BSP_OK；无事务为NOT_READY；空指针为INVALID_ARGUMENT；
     *   超时/CRC/帧/器件错误分别返回TIMEOUT/CRC_ERROR/FRAME_ERROR/SENSOR_FAULT。
     * @note 成功Begin的同一调用者执行一次，保留有界等待；失败输出不变，不发布旧角度。
     * @note 即使输出为空也收尾自己的CS/DMA及总线方向，不收尾其它所有者。
     * @note 保持旧65536计数域及截断精度，不直接改成MT6835的18位角度换算。
     * @note 收尾同时发布本次独立诊断快照；空输出也收尾并记录INVALID_ARGUMENT。
     *   无本拍所有权不覆盖诊断。失败不能沿用旧诊断字冒充本次读取。
     */
    BspResult BspEncoder_Complete(BspEncoderSample *sample);

    /**
     * @brief 读取编译选定的真实器件型号。
     * @return 当前型号；功能未提供/未启用返回BSP_ENCODER_NONE。
     * @note 前台/ISR只读，不探测、不访问SPI或重配器件。
     */
    BspEncoderType BspEncoder_GetType(void);

    /**
     * @brief 读取最近一次已完成采样事务的诊断快照。
     * @param diagnostics 仅读取成功时整体更新的输出，不保存指针。
     * @return 取得快照为BSP_OK；尚无事务为NOT_READY；空指针为INVALID_ARGUMENT；
     *   未提供为NOT_SUPPORTED。本次采样是否成功查看sample_result。
     * @note 前台/ISR有界原子读取，不访问SPI，不推进事务，也不额外读取安全字。
     * @note 先核对type和valid_mask再解释原始字；需与控制样本配对时核对sample_sequence。
     */
    BspResult BspEncoder_ReadDiagnostics(BspEncoderDiagnostics *diagnostics);

    /**
     * @brief 只读刷新六个现有器件诊断字。
     * @return 全量成功为BSP_OK；总线占用为BUSY；未关相为REJECTED；未初始化为NOT_READY；
     *   未提供为NOT_SUPPORTED；其它为对应错误。
     * @note 仅关相前台，可有界阻塞；不写寄存器，索引含义沿用各器件。
     * @note 本轮未成功读取的字标为无效，不把旧值当新值；BUSY不破坏活动采样或原缓存。
     * @note 六字槽仅是现有器件的兼容诊断布局，不是通用寄存器地址；型号/索引见EXECUTION.md。
     *   取得总线后使旧六字缓存失效，再发布本轮结果；不改变采样诊断快照。
     */
    BspResult BspEncoder_CaptureConfig(void);

    /**
     * @brief 读取最近一次已执行的诊断捕获中的有效字。
     * @param index 0..BSP_ENCODER_CONFIG_WORDS-1。
     * @param word 仅BSP_OK时更新的输出，不保存指针。
     * @return 有效字为BSP_OK；空指针/越界为INVALID_ARGUMENT；无本轮有效字为NOT_READY；
     *   未提供为NOT_SUPPORTED。
     * @note 只读缓存，不访问总线；与CaptureConfig由同一前台所有者串行调用。
     */
    BspResult BspEncoder_ReadConfigWord(uint8_t index, uint16_t *word);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
