#ifndef R40_ADC_H
#define R40_ADC_H

#include <stdbool.h>
#include <stdint.h>

/* 仅平台内部的辅助采样快照；NTC仍保留原始计数。 */
typedef struct
{
    uint16_t bus_current_counts;
    uint16_t motor_temperature_counts;
    uint32_t sequence;
} R40AuxSamples;

typedef struct
{
    uint16_t mcu_temperature_counts;
    uint16_t reference_counts;
    uint32_t sequence;
} R40TemperatureSamples;

/**
 * @brief 配置关闭触发的ADC与DMA，校准并等待就绪。
 * @return 两组ADC就绪返回true，失败返回false且保持功率关闭。
 * @note 前台初始化一次，不启动计数器；仅显式输出轴诊断目标配置PA5/PA6模拟输入。
 */
bool R40_AdcInit(void);

/**
 * @brief 先准备DMA与中断，再开放ADC外部触发。
 * @return 初始化完成且首次启动返回true，否则false。
 * @note 在R40_TimingStart之前调用，保持六路功率输出关闭。
 */
bool R40_AdcStart(void);

/**
 * @brief 消费同一次抢占序列的四个过采样结果。
 * @return 完整且无错误的新序列返回true，未就绪或故障返回false。
 * @note ADC中断上下文调用；成功后才能派发当前拍的控制任务，本函数不调用APP。
 */
bool R40_AdcComplete(void);

/**
 * @brief 在每个PWM采样触发点检查上一周期是否有新样本。
 * @note TMR1通道1中断调用，缺帧锁存故障并关相，不自动恢复。
 */
void R40_AdcCarrierTick(void);

/**
 * @brief 读取辅助DMA快照，拒绝未完成、过期或故障的数据。
 * @param out 成功时写入快照，失败保持原值。
 * @return 取得不超过2ms的新鲜快照返回true，否则false。
 * @note 前台或监督上下文调用，不保存指针。
 */
bool R40_AdcReadAux(R40AuxSamples *out);

/**
 * @brief 读取ADC1普通序列的内部温度和参考电压快照。
 * @param out 成功时写入快照，失败保持原值。
 * @return 取得不超过2ms的新鲜快照返回true，否则false。
 * @note 与ADC1相电流抢占序列独立，不触发转换、不保存指针。
 */
bool R40_AdcReadTemperature(R40TemperatureSamples *out);

/**
 * @brief 读取采样通道是否未锁存故障。
 * @return 已启动且未故障返回true，否则false。
 */
bool R40_AdcHealthy(void);

#endif
