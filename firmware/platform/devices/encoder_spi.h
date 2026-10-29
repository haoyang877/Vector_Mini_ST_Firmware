#ifndef ENCODER_SPI_H
#define ENCODER_SPI_H

#include <stdbool.h>
#include <stdint.h>

/* 编码器驱动与本板SPI之间的私有边界，控制层只包含bsp_encoder.h。
 * 请求均为MSB优先的两字节；响应按线上顺序返回，不解释角度/状态/CRC。
 * 本板实现固定物理模式：AT32四线，STM32单线换向。事务只允许一位所有者。
 * 采样请求的response_bytes：MT6835为4，TLE5012B为2；诊断分别为1和2。
 * 数据缓冲由调用者提供且至少容纳response_bytes字节，失败时内容不可使用。
 * 请求对象在调用期间有效即可；不可在一帧内切换请求。 */
typedef struct
{
    uint16_t command;
    uint8_t response_bytes;
} EncoderSpiRequest;

typedef enum
{
    ENCODER_SPI_READ_OK,
    ENCODER_SPI_READ_BUSY,
    ENCODER_SPI_READ_ERROR
} EncoderSpiReadStatus;

/* 0/1兼容公共接口的同步/请求阶段；失败标记仅在本板传输实现内解释。 */
#define ENCODER_SPI_READ_SYNC 0U
#define ENCODER_SPI_READ_COMMAND_PENDING 1U
#define ENCODER_SPI_READ_RESPONSE_PENDING 2U
#define ENCODER_SPI_READ_FAILED 3U

/**
 * @brief 按本板原有策略初始化SPI、引脚与可选DMA。
 * @return 本次执行初始化返回true；AT32重复初始化直接返回false。
 * @note 仅上电调用；致命错误沿用本板关相/停机路径。
 */
bool EncoderSpi_Init(void);

/**
 * @brief 非阻塞发起采样请求；不解释请求内容。
 * @param request 本板支持的采样请求和响应长度。
 * @return 成功发起返回true；总线不可用返回false并保留同步回退。
 * @note 快环调用；必须用同一请求调用Complete收尾。
 */
bool EncoderSpi_Begin(const EncoderSpiRequest *request);

/**
 * @brief 有界收回请求并启动响应，保留原有计算与传输重叠位置。
 * @param started 本拍Begin返回值，只允许推进一次。
 * @return 不透明事务阶段，必须原样交给Complete。
 * @note 失败也必须收尾；不在此释放其他调用者持有的事务。
 */
uint8_t EncoderSpi_BeginReadback(bool started);

/**
 * @brief 完成采样响应并收尾；阶段0按本板原有规则同步回退。
 * @param sample_stage 本拍Begin或BeginReadback返回的阶段。
 * @param request 与Begin一致的请求；响应长度满足本板采样约定。
 * @param data 响应字节缓冲，容量至少为request中的response_bytes。
 * @return 完整传输成功返回true；超时或总线错误返回false。
 * @note 仅快环；负责本次事务的CS、DMA或MOSI收尾，禁止抢占诊断事务。
 */
bool EncoderSpi_Complete(uint8_t sample_stage, const EncoderSpiRequest *request, uint8_t *data);

/**
 * @brief 同步读取一次诊断响应；不缓存或解释寄存器内容。
 * @param request 本板支持的诊断请求和响应长度。
 * @param data 响应字节缓冲，容量至少为request中的response_bytes。
 * @return 成功、未开始或传输失败；失败不改写输出。
 * @note 仅初始化/关相诊断调用；禁止从快环调用或与快环并发。
 */
EncoderSpiReadStatus EncoderSpi_Read(const EncoderSpiRequest *request, uint8_t *data);

#endif
