/**
 * @file bsp_internal.h
 * @brief BSP内部跨文件声明；按功能分段，不对应用公开。
 * @note 不放函数实现、可变对象定义或板级配置值；MCU专用状态留在对应实现。
 */
#ifndef BSP_INTERNAL_H
#define BSP_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_can.h"
#include "bsp_led.h"
#include "bsp_config.h"

/* ADC缓存与生命周期 */
/** @brief 辅助温度采样原码；12位单次域，sequence自然回绕。 */
typedef struct
{
    uint16_t motor_temperature_counts; /**< 电机NTC原码；目标启用时有效。 */
    uint32_t sequence;                 /**< 完整辅助帧序号，重复读取不推进。 */
    uint32_t sampled_ms;               /**< 与原码一起发布的采集时刻，ms。 */
} AdcAuxSample;

/** @brief MCU内部温度/VREF的12位原码；换算和标定资格由温度端口负责。 */
typedef struct
{
    uint16_t mcu_temperature_counts; /**< MCU温度通道原码。 */
    uint16_t reference_counts;       /**< 内部参考电压通道原码。 */
    uint32_t sequence;               /**< 完整帧序号，零和回绕后的零均是合法序号。 */
    uint32_t sampled_ms;             /**< 与原码一起发布的采集时刻，ms。 */
} AdcTemperatureSample;

/**
 * @brief 配置或绑定本板ADC资源并执行既有校准序列，不启相。
 * @return 本板初始化序列完成返回true，检测到初始化失败返回false。
 * @note 启动前台调用一次；厂商外设准备及错误检测范围保持各板既有实现。
 */
bool Adc_Init(void);
/**
 * @brief 准备采样中断及触发通道，不启动电机功率输出。
 * @return 本板启动序列完成返回true，本板前置条件不满足返回false。
 * @note 前台调用；监督定时器的启动仍由板级装配按原顺序负责。
 */
bool Adc_Start(void);
/**
 * @brief 确认当前快帧完成，先执行本板原有错误检查与原码发布。
 * @return 可以派发本拍APP快任务返回true，否则false。
 * @note 仅最高优先级且不可抢占的ADC完成上下文调用；本入口不再屏蔽中断。
 * @note 清硬件完成位的时点保持本板原有IRQ顺序；前台不得直接调用本入口。
 */
bool Adc_CompleteFast(void);
/**
 * @brief 读取本板采样已启动且未锁存既有采集故障的状态。
 * @return 已启动且本板采集故障未锁存返回true，否则false。
 * @note 只读；不代表新增的丢帧、模拟精度或逐通道健康检测。
 */
bool Adc_IsHealthy(void);
/**
 * @brief 读取独立辅助通道的完整、新鲜快照。
 * @param out 成功时写入的调用方快照；失败不修改，不保留指针。
 * @return 本板具备且取得有效快照返回true；无该硬件、过期或失败返回false。
 * @note 前台或监督任务调用；AT32保持2ms年龄约束，STM32当前无独立辅助端口。
 */
bool Adc_ReadAux(AdcAuxSample *out);
/**
 * @brief 读取本板MCU温度/VREF的完整原码，不执行温度换算。
 * @param out 成功时写入的调用方快照；失败保持原内容，不保留指针。
 * @return 有可消费原码返回true，无样本、禁用、过期或失败返回false。
 * @note 软件触发板只消费完成帧；自动采样板允许重复读取新鲜帧，调用方按sequence去重。
 */
bool Adc_ReadTemperature(AdcTemperatureSample *out);

/* ADC硬件采集与存储恢复 */
/* 后端只维护硬件采集状态；公共层负责逻辑通道、能力与输入校验。 */
/**
 * @brief 配置并校准本MCU采集硬件，不开放功率输出。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuAdc_Init(void);
/**
 * @brief 按既有顺序启动采集触发和完成通知。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuAdc_Start(void);
/**
 * @brief 确认本拍ADC事件并完成本MCU原有错误检查。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuAdc_CompleteFast(void);
/**
 * @brief 按物理结果槽读取四次累加原码，故障返回零。
 * @param rank 物理结果槽编号。
 * @return 对应计数或序号，单位见函数说明。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
uint16_t McuAdc_ReadRank(unsigned rank);
/**
 * @brief 消费软件触发温度完成帧，自动触发MCU返回false。
 * @param out 调用方输出缓冲；失败不发布有效结果。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；只填写原码，sequence和sampled_ms置零，完整序号由发布函数分配。
 */
bool McuAdc_PollTemperature(AdcTemperatureSample *out);
/**
 * @brief 推进本MCU软件触发温度采样，自动触发MCU不动作。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuAdc_ContinueTemperature(void);
/* 中断发布完整帧，公共层统一保管序号、时间戳、新鲜度与读快照。 */
/**
 * @brief 在完成中断中发布完整辅助帧及采集时间。
 * @param sample 本次完整样本，只在调用期间借用；忽略输入sequence及sampled_ms。
 * @param now_ms 采集完成时刻，单位ms，允许自然回绕。
 * @return 本次发布的序号；未启用时返回0且不发布，已启用时回绕后的0仍是合法序号。
 * @note 唯一辅助采集发布者调用；只有发布才推进序号，可将返回值用于同帧正余弦数据。
 */
uint32_t AdcFrames_PublishAux(const AdcAuxSample *sample, uint32_t now_ms);
/**
 * @brief 发布完整温度帧及采集时间。
 * @param sample 本次完整样本，只在调用期间借用；忽略输入sequence及sampled_ms。
 * @param now_ms 采集完成时刻，单位ms，允许自然回绕。
 * @return 本次发布的序号；未启用时返回0且不发布，已启用时回绕后的0仍是合法序号。
 * @note 唯一温度采集发布者调用；只有发布才推进序号，失效和读取不推进。
 */
uint32_t AdcFrames_PublishTemperature(const AdcTemperatureSample *sample, uint32_t now_ms);
/**
 * @brief 使辅助和温度帧失效，保留序号；仅存储暂停使用。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void AdcFrames_Invalidate(void);
/**
 * @brief 读取刚收齐的同周期母线电流原码，调用方已检查硬件能力。
 * @return 等效12位单次原码；无独立传感器时不可使用返回值。
 * @note BSP私有缓存读取，不启动ADC。
 */
uint16_t McuAdc_ReadBusCurrent(void);
/**
 * @brief 暂停本MCU实际采样，并使原快照失效。
 * @return 独占并暂停完成为true，失败为false。
 * @note 仅Flash事务在关相且屏蔽中断时调用。
 */
bool McuAdc_PauseForStorage(void);
/**
 * @brief 恢复暂停前的实际采样配置，不启相。
 * @return 恢复完成为true，故障或无事务为false。
 * @note 仅Flash事务调用；失败保持故障，不自行重试。
 */
bool McuAdc_ResumeAfterStorage(void);

/* 温度与参考电压换算 */
/** @brief 同帧温度与VDDA换算；两项结果独立，无效物理量为NAN。 */
typedef struct
{
    float celsius;                /**< MCU结温，摄氏度。 */
    float vdda_v;                 /**< 模拟供电电压，V。 */
    BspResult temperature_result; /**< OK或NOT_CALIBRATED，沿用温度保护资格。 */
    BspResult voltage_result;     /**< OK、NOT_CALIBRATED或IO_ERROR。 */
} AdcTemperatureValue;

/**
 * @brief 用同一完整原码快照换算温度和VDDA，只计算一次供电电压。
 * @param sample 借用的完整原码快照，不修改或保留。
 * @param value 温度与电压的独立换算结果，必须非空。
 * @note 监督上下文调用；不取样、不去重、不触发下一转换，不改变快照序号和时间。
 */
void McuAdc_ConvertTemperature(const AdcTemperatureSample *sample, AdcTemperatureValue *value);
/**
 * @brief 换算当前板实际接线的电机NTC原码。
 * @param counts NTC的12位原码。
 * @param celsius 仅成功更新的温度，摄氏度。
 * @return 成功为OK，开短路或越界为SENSOR_FAULT。
 * @note 仅支持电机NTC的板实现，使用既有分压和B值算法。
 */
BspResult McuAdc_MotorTemperature(uint16_t counts, float *celsius);

/* PWM内部初始化与硬件操作 */
/* MCU只操作定时器；窗口属于应用，公共BSP负责边界校验与结果归类。 */
/**
 * @brief 在生命周期原位置初始化PWM共用状态及本MCU定时器。
 * @note BSP私有启动入口，不启相；只在本次启动首次调用时执行。
 */
void Pwm_Init(void);
/**
 * @brief 只配置PWM硬件，保持功率输出关闭。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuPwm_Init(void);
/**
 * @brief 读取本板比较值换算周期，单位为定时器计数。
 * @return 初始化后的固定周期计数。
 * @note 仅在PWM初始化完成后读取一次；运行中不改变PWM周期。
 */
float McuPwm_PeriodCounts(void);
/**
 * @brief 核对提交期限并按原顺序写三相比较预装载，不启相。
 * @param a A相比较计数，调用方保证范围。
 * @param b B相比较计数，调用方保证范围。
 * @param c C相比较计数，调用方保证范围。
 * @return 完整提交为true；超时为false且不写任何比较值，由共用BSP关相并锁存。
 * @note 调用上下文保证不可抢占；完全关相时屏蔽影子更新后预载，结束恢复原控制位。
 * @note 定时器停转或剩余计数大于原保护预算时可提交；不强制更新或重置计数器。
 */
bool McuPwm_CommitThree(uint16_t a, uint16_t b, uint16_t c);
/**
 * @brief 核对提交期限并写一个逻辑相的比较预装载，不改变其他相。
 * @param phase 逻辑相编号。
 * @param counts 比较预装载计数。
 * @return 完整提交为true；超时或相号非法为false且不写比较值。
 * @note 相号由共用BSP先验证；完全关相预载不受运行期限限制，调用上下文保证不可抢占。
 */
bool McuPwm_CommitPhase(BspPwmPhase phase, uint16_t counts);
/**
 * @brief 按既有顺序启用六路输出，调用方已验证许可。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuPwm_EnableOutputs(void);
/**
 * @brief 按既有顺序关闭六路输出，保留采样触发。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuPwm_DisableOutputs(void);
/**
 * @brief 读取相通道和主输出状态，一致性由调用上下文保证。
 * @param any_channel 输出是否存在已启用相通道。
 * @param all_channels 输出是否所有相通道都已启用。
 * @param master 输出主输出使能状态。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuPwm_ReadOutputs(bool *any_channel, bool *all_channels, bool *master);
/**
 * @brief 关闭读回仍异常时进入本MCU既有致命停留。
 * @note MCU实现屏蔽中断后不再返回或喂看门狗，不重试启相。
 */
void McuPwm_Fatal(void);

/* CAN内部初始化、传输与IRQ */
typedef void (*BspCanReceiveHandler)(void);
/* 后端只持有控制器及物理槽身份；软件票据和截止时间由共用CAN拥有。 */
/**
 * @brief 配置CAN硬件，不发送帧；失败沿用本MCU既有处理。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_Init(void);
/**
 * @brief 启动已配置控制器并沿用原接收中断开放时点。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuCan_Start(void);
/**
 * @brief 读取控制器能否接受发送，不改变状态。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_IsAvailable(void);
/**
 * @brief 判断本MCU的总线故障是否需要恢复。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_NeedsRecovery(void);
/**
 * @brief 执行本MCU原有有界恢复，不改变协议或票据。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_Recover(void);
/**
 * @brief 按既有优先级开放接收通知。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuCan_EnableReceiveInterrupt(void);
/**
 * @brief 完成接收事件的硬件确认和有界回调。
 * @param handler 应用注入的有界接收回调，可为空。
 * @param events MCU接收事件位，必须在HAL清标志后仍保留溢出事实。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuCan_OnReceiveInterrupt(BspCanReceiveHandler handler, uint32_t events);
/**
 * @brief 非阻塞接收并解包一个硬件帧。
 * @param frame 完整CAN帧，仅在调用期间借用。
 * @return 取得为BSP_OK；空FIFO为NOT_READY；被消费坏帧为FRAME_ERROR；访问失败为IO_ERROR。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
BspResult McuCan_Receive(BspCanFrame *frame);
/**
 * @brief 向硬件提交一帧，返回本次物理槽身份。
 * @param frame 完整CAN帧，仅在调用期间借用。
 * @param dlc 硬件长度编码，范围0..15。
 * @param token 成功时输出本次物理发送槽身份。
 * @return 满足条件或操作成功返回true，否则false。
 * @note 前台唯一发送者调用，不持共享全局锁；后端自行隔离CAN硬件，不保存输入指针。
 */
bool McuCan_Submit(const BspCanFrame *frame, uint8_t dlc, uint32_t *token);
/**
 * @brief 读取指定物理槽是否已释放。
 * @param token 物理发送槽身份，不是上层事务票据。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_SlotIdle(uint32_t token);
/**
 * @brief 读取指定槽的取消事件。
 * @param token 物理发送槽身份，不是上层事务票据。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_TxCancelled(uint32_t token);
/**
 * @brief 读取指定槽的发送完成事件。
 * @param token 物理发送槽身份，不是上层事务票据。
 * @return 满足条件或操作成功返回true，否则false。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
bool McuCan_TxCompleted(uint32_t token);
/**
 * @brief 读取PTB终态与提交身份的匹配情况。
 * @param token 物理发送槽身份，不是上层事务票据。
 * @param matches 输出终态是否属于本次提交。
 * @param sent 输出本次提交是否成功发送。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuCan_ReadTerminal(uint32_t token, bool *matches, bool *sent);
/**
 * @brief 请求撤销指定硬件槽，完成前不可复用。
 * @param token 物理发送槽身份，不是上层事务票据。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
void McuCan_Abort(uint32_t token);
/**
 * @brief 初始化已启用的控制器，禁用功能不访问硬件。
 * @return 初始化完成或功能关闭为true，硬件失败为false。
 * @note 仅BSP启动调用，不开放应用接收。
 */
bool Can_Init(void);
/**
 * @brief 在既有向量处完成硬件接收收尾和有界应用事件。
 * @param handler 目标唯一接收者，轮询目标不调用。
 * @param events HAL已确认的事件位，AT32传0并直接读取硬件。
 * @note BSP或专用底层台架IRQ调用，不解析协议。
 */
void Can_OnReceiveInterrupt(BspCanReceiveHandler handler, uint32_t events);
/**
 * @brief 消费一次接收溢出事件，不弹出幸存帧。
 * @return 存在未报告溢出为true，否则false。
 * @note 调用方已进入临界区；合并尚未报告的重复事件。
 */
bool McuCan_TakeRxOverflow(void);
/**
 * @brief 查询控制器实际错误等级。
 * @return 按BUS_OFF、ERROR、WARNING、READY优先返回。
 * @note 控制器已经初始化并启动；只读不清错误。
 */
BspCanState McuCan_GetState(void);

/* LED初始化、节拍与输出 */
/** @brief 建立指示器初态；BSP启动调用，不改变电机输出。 */
void Led_Init(void);
/** @brief 每个原监督拍推进分频；BSP在应用事件后有界调用。 */
void Led_SupervisorTick(void);
/** @brief 准备实际LED输出并保持关闭；只在启动调用。 */
void McuLed_Init(void);
/**
 * @brief 写独立状态灯的逻辑亮灭。
 * @param channel 当前应用选择的码类型。
 * @param on 本显示拍是否点亮。
 * @note 只用于状态灯和颜色输出独立的MCU，不操作灯带。
 */
void McuLed_WriteCode(BspLedCodeChannel channel, bool on);
/**
 * @brief 提交实际颜色和亮度，不推进波形。
 * @param color 已验证的目标颜色。
 * @param brightness 0..100的有效亮度。
 * @note 缓冲和DMA由MCU持有，繁忙时不得改写仍在传输的缓冲。
 */
void McuLed_WriteColor(BspLedColor color, uint8_t brightness);
/** @brief 推进共享GPIO的软件亮度调制；仅AT32原1ms时基调用。 */
void McuLed_Tick1ms(void);

/* Flash初始化 */
/**
 * @brief 在芯片准备完成后建立固定存储区的访问资格。
 * @return 本次首次初始化成功为true，重复调用或已锁存故障为false。
 * @note BSP生命周期或专用底层台架调用；不擦写记录。
 */
bool Flash_Init(void);

/* 编码器初始化；器件协议和SPI契约位于devices。 */
/**
 * @brief 初始化编码器器件及SPI，保持关相，不开放应用事件。
 * @return 硬件初始化成功为true，失败为false。
 * @note BSP生命周期或专用底层台架调用一次；非公开功能接口。
 */
bool Encoder_Init(void);

/* AT32启动编排；测试入口与完整APP复用同一安全顺序。 */
/** @brief 准备门极低电平、时钟及停止的定时器；失败时禁止继续启动。
 * @return 首次初始化成功返回true，否则false。
 */
bool McuSystem_Init(void);
/** @brief 按原顺序开放时基和采样载波，保持六路输出关闭。
 * @return 首次启动成功返回true，否则false。
 */
bool McuSystem_Start(void);
/** @brief 在切换系统时钟之前将门极引脚准备为低电平。 */
void McuPwm_PreparePins(void);
/** @brief 定时器输出已确认关闭后，将门极引脚接入复用功能。 */
void McuPwm_ConnectPins(void);

#endif /* BSP_INTERNAL_H */
